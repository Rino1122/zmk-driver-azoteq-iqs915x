/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Azoteq IQS9150/IQS9151 トラックパッドドライバ
 *
 * IQS5xxドライバをベースに、IQS915xシリーズのレジスタマップに対応。
 * 主な変更点:
 *   - リトルエンディアンのバイトオーダー
 *   - 異なるレジスタアドレス体系 (0x1000〜)
 *   - 16bitレジスタによるジェスチャーイベント読み取り
 *   - RDYラインの立ち下がりエッジ対応（DTS設定で極性変更可能）
 *   - デフォルトではI2C STOPで通信ウィンドウが閉じるため、
 *     1回のRDY期間中に1つのI2Cトランザクションのみ実行する
 *     ステートマシン方式で動作する。
 */

#define DT_DRV_COMPAT azoteq_iqs915x

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "iqs915x.h"

LOG_MODULE_REGISTER(iqs915x, CONFIG_INPUT_LOG_LEVEL);

/* ============================================================
 * I2C通信関数
 *
 * IQS9150はリトルエンディアン（IQS5xxはビッグエンディアン）。
 * レジスタアドレスは16bit幅で、アドレス自体はビッグエンディアンで送信される。
 * データ部分がリトルエンディアン（LSB first）。
 *
 * 重要: IQS9150のデフォルト設定ではI2C STOPで通信ウィンドウが閉じる。
 * そのため、1回のRDYサイクル中に実行できるI2Cトランザクションは1つのみ。
 * ============================================================ */

// 16bitレジスタを読み取る（リトルエンディアン: LSB first）
static int iqs915x_read_reg16(const struct device *dev, uint16_t reg,
                              uint16_t *val) {
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[2];
  // レジスタアドレスはビッグエンディアンで送信
  uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
  int ret;

  ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf,
                          sizeof(buf));
  if (ret < 0) {
    return ret;
  }

  // データはリトルエンディアン: buf[0]=LSB, buf[1]=MSB
  *val = (buf[1] << 8) | buf[0];
  return 0;
}

// 16bitレジスタに書き込む（リトルエンディアン: LSB first）
static int iqs915x_write_reg16(const struct device *dev, uint16_t reg,
                               uint16_t val) {
  const struct iqs915x_config *config = dev->config;
  // アドレスはビッグエンディアン, データはリトルエンディアン
  uint8_t buf[4] = {reg >> 8, reg & 0xFF, val & 0xFF, val >> 8};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

// 8bitレジスタに書き込む
static int iqs915x_write_reg8(const struct device *dev, uint16_t reg,
                              uint8_t val) {
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

/* ============================================================
 * トラックパッドデータの一括読み取り
 *
 * REL_X(0x1014)〜TRACKPAD_FLAGS(0x1022)の16バイトを1回の
 * I2Cトランザクションで読み取る。これにより通信ウィンドウの
 * 制約を守りつつ、必要なデータをすべて取得できる。
 *
 * メモリレイアウト (16 bytes):
 *   0x1014: REL_X (2 bytes, signed)
 *   0x1016: REL_Y (2 bytes, signed)
 *   0x1018: GESTURE_X (2 bytes) - 未使用
 *   0x101A: GESTURE_Y (2 bytes) - 未使用
 *   0x101C: SINGLE_FINGER_GESTURES (2 bytes)
 *   0x101E: TWO_FINGER_GESTURES (2 bytes)
 *   0x1020: INFO_FLAGS (2 bytes)
 *   0x1022: TRACKPAD_FLAGS (2 bytes)
 * ============================================================ */

// 一括読み取りデータ構造体
struct iqs915x_bulk_data {
  int16_t rel_x;
  int16_t rel_y;
  uint16_t gesture_x;  // 未使用だがメモリレイアウトに含まれる
  uint16_t gesture_y;  // 未使用だがメモリレイアウトに含まれる
  uint16_t gesture_sf; // Single Finger Gestures
  uint16_t gesture_tf; // Two Finger Gestures
  uint16_t info_flags;
  uint16_t trackpad_flags;
};

// REL_X(0x1014)からTRACKPAD_FLAGS(0x1022)まで16バイトを一括読み取り
static int iqs915x_read_bulk(const struct device *dev,
                             struct iqs915x_bulk_data *data) {
  const struct iqs915x_config *config = dev->config;
  // 読み取り開始アドレス: 0x1014 (REL_X)
  uint8_t reg_buf[2] = {0x10, 0x14};
  uint8_t buf[16];
  int ret;

  ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf,
                          sizeof(buf));
  if (ret < 0) {
    return ret;
  }

  // リトルエンディアンでデコード
  data->rel_x = (int16_t)((buf[1] << 8) | buf[0]);
  data->rel_y = (int16_t)((buf[3] << 8) | buf[2]);
  data->gesture_x = (buf[5] << 8) | buf[4];
  data->gesture_y = (buf[7] << 8) | buf[6];
  data->gesture_sf = (buf[9] << 8) | buf[8];
  data->gesture_tf = (buf[11] << 8) | buf[10];
  data->info_flags = (buf[13] << 8) | buf[12];
  data->trackpad_flags = (buf[15] << 8) | buf[14];

  return 0;
}

/* ============================================================
 * ボタンリリース遅延処理
 * タップジェスチャーはボタンの押下と解放を短時間で報告する必要がある。
 * 100msのディレイ後に自動的にボタンを解放する。
 * ============================================================ */
static void iqs915x_button_release_work_handler(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, button_release_work);

  for (int i = 0; i < 3; i++) {
    if (data->buttons_pressed & BIT(i)) {
      LOG_INF("Releasing synthetic button %d", i);
      input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
      // ビットをクリア
      // 注意: 潜在的な競合状態の可能性あり
      data->buttons_pressed &= ~BIT(i);
    }
  }
}

/* ============================================================
 * 初期化ステートマシン
 *
 * IQS9150はI2C STOPで通信ウィンドウが閉じるため、
 * 1回のRDYサイクルで1つのレジスタ書き込みのみ実行する。
 * RDYが発火するたびに1ステップずつ初期化を進める。
 * ============================================================ */
static void iqs915x_init_step_handler(const struct device *dev) {
  struct iqs915x_data *data = dev->data;
  const struct iqs915x_config *config = dev->config;
  int ret;

  switch (data->init_step) {
  case INIT_ACK_RESET:
    // リセットフラグをクリア（Activeモード + ACK Reset）
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL,
                              IQS915X_MODE_ACTIVE | IQS915X_ACK_RESET);
    if (ret < 0) {
      LOG_ERR("Failed to ACK reset: %d", ret);
      return;
    }
    LOG_INF("Init: ACK reset sent");
    data->init_step = INIT_CONFIG_SETTINGS;
    break;

  case INIT_CONFIG_SETTINGS:
    // イベントモード・ジェスチャーイベント有効化
    // TERMINATE_COMMSは設定しない（I2C STOP自動終了を使用）
    ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS,
                              IQS915X_EVENT_MODE | IQS915X_GESTURE_EVENT);
    if (ret < 0) {
      LOG_ERR("Failed to configure event mode: %d", ret);
      return;
    }
    LOG_INF("Init: Config settings written");
    data->init_step = INIT_SINGLE_FINGER_GESTURES;
    break;

  case INIT_SINGLE_FINGER_GESTURES: {
    // 1本指ジェスチャーの有効化設定
    uint16_t gestures = 0;
    gestures |= config->one_finger_tap ? IQS915X_SINGLE_TAP : 0;
    gestures |= config->press_and_hold ? IQS915X_PRESS_AND_HOLD : 0;
    ret = iqs915x_write_reg16(dev, IQS915X_SINGLE_FINGER_GESTURES_ENABLE,
                              gestures);
    if (ret < 0) {
      LOG_ERR("Failed to configure single finger gestures: %d", ret);
      return;
    }
    LOG_INF("Init: Single finger gestures configured");
    data->init_step = INIT_HOLD_TIME;
    break;
  }

  case INIT_HOLD_TIME:
    // プレス＆ホールドの判定時間
    ret = iqs915x_write_reg16(dev, IQS915X_HOLD_TIME,
                              config->press_and_hold_time);
    if (ret < 0) {
      LOG_ERR("Failed to configure hold time: %d", ret);
      return;
    }
    LOG_INF("Init: Hold time configured");
    data->init_step = INIT_TWO_FINGER_GESTURES;
    break;

  case INIT_TWO_FINGER_GESTURES: {
    // 2本指ジェスチャーの有効化設定
    uint16_t gestures = 0;
    gestures |= config->two_finger_tap ? IQS915X_TWO_FINGER_TAP : 0;
    gestures |= config->scroll ? IQS915X_SCROLL : 0;
    ret =
        iqs915x_write_reg16(dev, IQS915X_TWO_FINGER_GESTURES_ENABLE, gestures);
    if (ret < 0) {
      LOG_ERR("Failed to configure two finger gestures: %d", ret);
      return;
    }
    LOG_INF("Init: Two finger gestures configured");
    data->init_step = INIT_TRACKPAD_SETTINGS;
    break;
  }

  case INIT_TRACKPAD_SETTINGS: {
    // 軸設定（FlipX/Y, SwitchXY）
    uint8_t settings = 0;
    settings |= config->flip_x ? IQS915X_FLIP_X : 0;
    settings |= config->flip_y ? IQS915X_FLIP_Y : 0;
    settings |= config->switch_xy ? IQS915X_SWITCH_XY_AXIS : 0;
    ret = iqs915x_write_reg8(dev, IQS915X_TRACKPAD_SETTINGS, settings);
    if (ret < 0) {
      LOG_ERR("Failed to configure trackpad settings: %d", ret);
      return;
    }
    LOG_INF("Init: Trackpad settings configured");
    data->init_step = INIT_COMPLETE;
    data->initialized = true;
    data->work_state = WORK_READ_DATA;
    LOG_INF("IQS915x initialization complete");
    break;
  }

  case INIT_COMPLETE:
    // ここには到達しないはず
    break;
  }
}

/* ============================================================
 * メインワークハンドラ（ステートマシン方式）
 *
 * RDY割り込みで呼び出される。
 * IQS9150はI2C STOPで通信ウィンドウが閉じるため、
 * 1回のRDYにつき1つのI2Cトランザクションのみ実行する。
 *
 * 初期化中:
 *   iqs915x_init_step_handler() に処理を委譲し、
 *   1ステップずつレジスタを設定する。
 *
 * 通常動作:
 *   WORK_READ_DATA → 一括読み取りでデータ取得＋リセット判定
 *   → リセット検知時は WORK_ACK_RESET に遷移
 *   WORK_ACK_RESET → ACK書き込み → WORK_READ_DATA に戻る
 * ============================================================ */
static void iqs915x_work_handler(struct k_work *work) {
  struct iqs915x_data *data = CONTAINER_OF(work, struct iqs915x_data, work);
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  int ret;

  // 初期化中はステップハンドラに委譲
  if (!data->initialized) {
    iqs915x_init_step_handler(dev);
    return;
  }

  // 通常動作のステートマシン
  switch (data->work_state) {

  case WORK_READ_INFO_FLAGS:
    // このステートは現在未使用（WORK_READ_DATAで一括取得するため）
    // 将来の拡張用に残す
    data->work_state = WORK_READ_DATA;
    break;

  case WORK_ACK_RESET: {
    // リセットフラグをクリア（Activeモード + ACK Reset）
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL,
                              IQS915X_MODE_ACTIVE | IQS915X_ACK_RESET);
    if (ret < 0) {
      LOG_ERR("Failed to write ACK reset: %d", ret);
      return;
    }
    LOG_INF("ACK reset sent");
    // ACK後は再初期化を実行（デバイスがリセットされた場合、設定も消えるため）
    data->init_step = INIT_CONFIG_SETTINGS;
    data->initialized = false;
    break;
  }

  case WORK_READ_DATA: {
    // REL_X〜TRACKPAD_FLAGSまでを1回のI2Cトランザクションで一括読み取り
    struct iqs915x_bulk_data bulk;
    ret = iqs915x_read_bulk(dev, &bulk);
    if (ret < 0) {
      LOG_ERR("Failed to read bulk data: %d", ret);
      return;
    }

    // リセット検知
    if (bulk.info_flags & IQS915X_SHOW_RESET) {
      LOG_INF("Device reset detected");
      // 次のRDYでACKを書き込む
      data->work_state = WORK_ACK_RESET;
      return;
    }

    // --- 以下、通常のトラックパッド処理 ---

    bool tp_movement = (bulk.trackpad_flags & IQS915X_TP_MOVEMENT) != 0;
    bool scroll = (bulk.gesture_tf & IQS915X_SCROLL) != 0;

    if (!scroll) {
      // スクロール中でなければアキュムレータをクリア
      data->scroll_x_acc = 0;
      data->scroll_y_acc = 0;
    }

    // タップジェスチャーの判定
    uint16_t button_code;
    bool button_pressed = false;
    if (bulk.gesture_sf & IQS915X_SINGLE_TAP) {
      button_pressed = true;
      button_code = INPUT_BTN_0;
    } else if (bulk.gesture_tf & IQS915X_TWO_FINGER_TAP) {
      button_pressed = true;
      button_code = INPUT_BTN_1;
    }

    // プレス＆ホールドの状態遷移
    bool hold_became_active =
        (bulk.gesture_sf & IQS915X_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released =
        !(bulk.gesture_sf & IQS915X_PRESS_AND_HOLD) && data->active_hold;

    // 移動とジェスチャーの処理
    if (hold_became_active) {
      LOG_INF("Hold became active");
      input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
      data->active_hold = true;
    } else if (hold_released) {
      LOG_INF("Hold became inactive");
      input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      data->active_hold = false;
    } else if (button_pressed) {
      // 保留中のリリースをキャンセル
      k_work_cancel_delayable(&data->button_release_work);

      // ボタンを即座に押下
      input_report_key(dev, button_code, 1, true, K_FOREVER);
      data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);

      // 100ms後にリリースをスケジュール
      k_work_schedule(&data->button_release_work, K_MSEC(100));
    } else if (scroll) {
      // TODO: このdivisorをDTS設定で公開する
      int16_t scroll_div = 32;

      // 一度に一つのスクロール方向のみ有効
      if (bulk.rel_x != 0) {
        // デフォルトではX軸は「ナチュラル」
        if (!config->natural_scroll_x) {
          bulk.rel_x *= -1;
        }
        data->scroll_x_acc += bulk.rel_x;
        if (abs(data->scroll_x_acc) >= scroll_div) {
          input_report_rel(dev, INPUT_REL_HWHEEL,
                           data->scroll_x_acc / scroll_div, true, K_FOREVER);
          data->scroll_x_acc %= scroll_div;
        }
        return;
      }
      if (bulk.rel_y != 0) {
        if (config->natural_scroll_y) {
          bulk.rel_y *= -1;
        }
        data->scroll_y_acc += bulk.rel_y;
        if (abs(data->scroll_y_acc) >= scroll_div) {
          input_report_rel(dev, INPUT_REL_WHEEL,
                           data->scroll_y_acc / scroll_div, true, K_FOREVER);
          data->scroll_y_acc %= scroll_div;
        }
        return;
      }
    } else if (tp_movement) {
      if (bulk.rel_x != 0 || bulk.rel_y != 0) {
        input_report_rel(dev, INPUT_REL_X, bulk.rel_x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, bulk.rel_y, true, K_FOREVER);
      }
    }
    break;
  }
  }
}

/* ============================================================
 * RDY GPIO割り込みハンドラ
 * ============================================================ */
static void iqs915x_rdy_handler(const struct device *port,
                                struct gpio_callback *cb,
                                gpio_port_pins_t pins) {
  struct iqs915x_data *data = CONTAINER_OF(cb, struct iqs915x_data, rdy_cb);

  k_work_submit(&data->work);
}

/* ============================================================
 * デバイス初期化関数
 * Zephyrのデバイスモデルから呼び出される。
 *
 * GPIO・I2Cの初期設定のみ行い、デバイスのレジスタ設定は
 * RDY割り込みによるステートマシンで段階的に実行する。
 * ============================================================ */
static int iqs915x_init(const struct device *dev) {
  const struct iqs915x_config *config = dev->config;
  struct iqs915x_data *data = dev->data;
  int ret;

  if (!i2c_is_ready_dt(&config->i2c)) {
    LOG_ERR("I2C device not ready");
    return -ENODEV;
  }

  data->dev = dev;
  data->init_step = INIT_ACK_RESET;
  data->work_state = WORK_READ_DATA;
  data->initialized = false;

  k_work_init(&data->work, iqs915x_work_handler);
  k_work_init_delayable(&data->button_release_work,
                        iqs915x_button_release_work_handler);

  // リセットGPIOの設定（オプショナル）
  if (config->reset_gpio.port) {
    if (!gpio_is_ready_dt(&config->reset_gpio)) {
      LOG_ERR("Reset GPIO not ready");
      return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
      LOG_ERR("Failed to configure reset GPIO: %d", ret);
      return ret;
    }

    // デバイスをリセット
    gpio_pin_set_dt(&config->reset_gpio, 1);
    k_msleep(1);
    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_msleep(10);
  }

  // RDY GPIOの設定
  if (!gpio_is_ready_dt(&config->rdy_gpio)) {
    LOG_ERR("RDY GPIO not ready");
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
  if (ret < 0) {
    LOG_ERR("Failed to configure RDY GPIO: %d", ret);
    return ret;
  }

  gpio_init_callback(&data->rdy_cb, iqs915x_rdy_handler,
                     BIT(config->rdy_gpio.pin));
  ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
  if (ret < 0) {
    LOG_ERR("Failed to add RDY callback: %d", ret);
    return ret;
  }

  // RDYの割り込み設定
  // DTSのgpio-flagsで極性を設定可能（GPIO_INT_EDGE_TO_ACTIVE）
  ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure RDY interrupt: %d", ret);
    return ret;
  }

  LOG_INF("IQS915x driver loaded, waiting for first RDY...");

  return 0;
}

/* ============================================================
 * デバイスインスタンスマクロ
 * DTSで定義された各IQS915xノードに対してインスタンスを生成する。
 * ============================================================ */
#define IQS915X_INIT(n)                                                        \
  static struct iqs915x_data iqs915x_data_##n;                                 \
  static const struct iqs915x_config iqs915x_config_##n = {                    \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                          \
      .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                         \
      .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),             \
      .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                       \
      .press_and_hold = DT_INST_PROP(n, press_and_hold),                       \
      .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                       \
      .scroll = DT_INST_PROP(n, scroll),                                       \
      .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                   \
      .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                   \
      .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),     \
      .switch_xy = DT_INST_PROP(n, switch_xy),                                 \
      .flip_x = DT_INST_PROP(n, flip_x),                                       \
      .flip_y = DT_INST_PROP(n, flip_y),                                       \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, iqs915x_init, NULL, &iqs915x_data_##n,              \
                        &iqs915x_config_##n, POST_KERNEL,                      \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS915X_INIT)
