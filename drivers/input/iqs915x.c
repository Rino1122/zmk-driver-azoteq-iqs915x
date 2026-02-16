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

// 8bitレジスタを読み取る
static int iqs915x_read_reg8(const struct device *dev, uint16_t reg,
                             uint8_t *val) {
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};

  return i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), val, 1);
}

// 8bitレジスタに書き込む
static int iqs915x_write_reg8(const struct device *dev, uint16_t reg,
                              uint8_t val) {
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

// 通信ウィンドウを終了する
// Config Settings (0x11BE) の Terminate Comms (Bit 6) が1の場合に必要。
static int iqs915x_end_comm_window(const struct device *dev) {
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[3] = {IQS915X_END_COMM_WINDOW >> 8,
                    IQS915X_END_COMM_WINDOW & 0xFF, 0x00};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
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
 * メインワークハンドラ
 * RDY割り込みで呼び出され、レジスタを読み取ってZMKに入力イベントを報告する。
 * ============================================================ */
static void iqs915x_work_handler(struct k_work *work) {
  struct iqs915x_data *data = CONTAINER_OF(work, struct iqs915x_data, work);
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  uint16_t info_flags, trackpad_flags, gesture_events_sf, gesture_events_tf;
  int ret;

  // Info Flags を読み取る（リセット検知等）
  ret = iqs915x_read_reg16(dev, IQS915X_INFO_FLAGS, &info_flags);
  if (ret < 0) {
    LOG_ERR("Failed to read info flags: %d", ret);
    goto end_comm;
  }

  // リセット検知の処理
  if (info_flags & IQS915X_SHOW_RESET) {
    LOG_INF("Device reset detected");
    // リセットフラグをクリア
    iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, IQS915X_ACK_RESET);
    goto end_comm;
  }

  // Trackpad Flags を読み取る（指の数、移動検知）
  ret = iqs915x_read_reg16(dev, IQS915X_TRACKPAD_FLAGS, &trackpad_flags);
  if (ret < 0) {
    LOG_ERR("Failed to read trackpad flags: %d", ret);
    goto end_comm;
  }

  // Single Finger Gesture イベントを読み取る
  ret = iqs915x_read_reg16(dev, IQS915X_SINGLE_FINGER_GESTURES,
                           &gesture_events_sf);
  if (ret < 0) {
    LOG_ERR("Failed to read single finger gesture events: %d", ret);
    goto end_comm;
  }

  // Two Finger Gesture イベントを読み取る
  ret =
      iqs915x_read_reg16(dev, IQS915X_TWO_FINGER_GESTURES, &gesture_events_tf);
  if (ret < 0) {
    LOG_ERR("Failed to read two finger gesture events: %d", ret);
    goto end_comm;
  }

  // 移動とスクロールの状態を判定
  bool tp_movement = (trackpad_flags & IQS915X_TP_MOVEMENT) != 0;
  bool scroll = (gesture_events_tf & IQS915X_SCROLL) != 0;

  if (!scroll) {
    // スクロール中でなければアキュムレータをクリア
    data->scroll_x_acc = 0;
    data->scroll_y_acc = 0;
  }

  // タップジェスチャーの判定
  uint16_t button_code;
  bool button_pressed = false;
  if (gesture_events_sf & IQS915X_SINGLE_TAP) {
    button_pressed = true;
    button_code = INPUT_BTN_0;
  } else if (gesture_events_tf & IQS915X_TWO_FINGER_TAP) {
    button_pressed = true;
    button_code = INPUT_BTN_1;
  }

  // プレス＆ホールドの状態遷移
  bool hold_became_active =
      (gesture_events_sf & IQS915X_PRESS_AND_HOLD) && !data->active_hold;
  bool hold_released =
      !(gesture_events_sf & IQS915X_PRESS_AND_HOLD) && data->active_hold;

  // 移動量またはスクロール時に相対座標を読み取る
  int16_t rel_x, rel_y;
  if (tp_movement || scroll) {
    ret = iqs915x_read_reg16(dev, IQS915X_REL_X, (uint16_t *)&rel_x);
    if (ret < 0) {
      LOG_ERR("Failed to read relative X: %d", ret);
      goto end_comm;
    }

    ret = iqs915x_read_reg16(dev, IQS915X_REL_Y, (uint16_t *)&rel_y);
    if (ret < 0) {
      LOG_ERR("Failed to read relative Y: %d", ret);
      goto end_comm;
    }
  }

  // 移動とジェスチャーの処理
  // 各分岐の最後のレポートはsync=trueで送信し、
  // 入力サブシステムが順序通り処理するようにする。
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
    if (rel_x != 0) {
      // デフォルトではX軸は「ナチュラル」
      if (!config->natural_scroll_x) {
        rel_x *= -1;
      }
      data->scroll_x_acc += rel_x;
      if (abs(data->scroll_x_acc) >= scroll_div) {
        input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div,
                         true, K_FOREVER);
        data->scroll_x_acc %= scroll_div;
      }
      goto end_comm;
    }
    if (rel_y != 0) {
      if (config->natural_scroll_y) {
        rel_y *= -1;
      }
      data->scroll_y_acc += rel_y;
      if (abs(data->scroll_y_acc) >= scroll_div) {
        input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div,
                         true, K_FOREVER);
        data->scroll_y_acc %= scroll_div;
      }
      goto end_comm;
    }
  } else if (tp_movement) {
    if (rel_x != 0 || rel_y != 0) {
      input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
      input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
    }
  }

end_comm:
  // 通信ウィンドウを終了
  iqs915x_end_comm_window(dev);
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
 * デバイス初期設定
 * 初回起動時にIQS9150のレジスタを設定する。
 * ============================================================ */
static int iqs915x_setup_device(const struct device *dev) {
  const struct iqs915x_config *config = dev->config;
  int ret;

  // イベントモードを有効化し、ジェスチャーイベントも有効化する。
  // 通信ウィンドウの手動終了も有効化（0xEEEE書き込みで終了）。
  ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS,
                            IQS915X_EVENT_MODE | IQS915X_GESTURE_EVENT |
                                IQS915X_TERMINATE_COMMS);
  if (ret < 0) {
    LOG_ERR("Failed to configure event mode: %d", ret);
    return ret;
  }

  // Single Finger Gesture の有効化設定
  uint16_t single_finger_gestures = 0;
  single_finger_gestures |= config->one_finger_tap ? IQS915X_SINGLE_TAP : 0;
  single_finger_gestures |= config->press_and_hold ? IQS915X_PRESS_AND_HOLD : 0;
  ret = iqs915x_write_reg16(dev, IQS915X_SINGLE_FINGER_GESTURES_ENABLE,
                            single_finger_gestures);
  if (ret < 0) {
    LOG_ERR("Failed to configure single finger gestures: %d", ret);
    return ret;
  }

  // プレス＆ホールドの判定時間を設定
  ret =
      iqs915x_write_reg16(dev, IQS915X_HOLD_TIME, config->press_and_hold_time);
  if (ret < 0) {
    LOG_ERR("Failed to configure hold time: %d", ret);
    return ret;
  }

  // Two Finger Gesture の有効化設定
  uint16_t two_finger_gestures = 0;
  two_finger_gestures |= config->two_finger_tap ? IQS915X_TWO_FINGER_TAP : 0;
  two_finger_gestures |= config->scroll ? IQS915X_SCROLL : 0;
  ret = iqs915x_write_reg16(dev, IQS915X_TWO_FINGER_GESTURES_ENABLE,
                            two_finger_gestures);
  if (ret < 0) {
    LOG_ERR("Failed to configure two finger gestures: %d", ret);
    return ret;
  }

  // 軸設定（FlipX/Y, SwitchXY）
  uint8_t trackpad_settings = 0;
  trackpad_settings |= config->flip_x ? IQS915X_FLIP_X : 0;
  trackpad_settings |= config->flip_y ? IQS915X_FLIP_Y : 0;
  trackpad_settings |= config->switch_xy ? IQS915X_SWITCH_XY_AXIS : 0;
  ret = iqs915x_write_reg8(dev, IQS915X_TRACKPAD_SETTINGS, trackpad_settings);
  if (ret < 0) {
    LOG_ERR("Failed to configure trackpad settings: %d", ret);
    return ret;
  }

  // 通信ウィンドウを終了
  ret = iqs915x_end_comm_window(dev);
  if (ret < 0) {
    LOG_ERR("Failed to end comm window during initialization: %d", ret);
    return ret;
  }

  return 0;
}

/* ============================================================
 * デバイス初期化関数
 * Zephyrのデバイスモデルから呼び出される。
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
  // IQS9150はRDYがLOWになった時にデータ準備完了を通知する。
  // ただし、DTSのgpio-flagsで極性を設定可能なので、
  // ここではDTSの設定に従う（GPIO_INT_EDGE_TO_ACTIVE）。
  ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Failed to configure RDY interrupt: %d", ret);
    return ret;
  }

  // デバイスの準備完了を待機
  k_msleep(100);

  // デバイス設定の実行
  ret = iqs915x_setup_device(dev);
  if (ret < 0) {
    LOG_ERR("Failed to setup device: %d", ret);
    return ret;
  }

  data->initialized = true;
  LOG_INF("IQS915x trackpad initialized");

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
