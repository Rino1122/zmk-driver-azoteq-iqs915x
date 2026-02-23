/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Azoteq IQS9150/IQS9151 トラックパッドドライバ
 *
 * IQS915xのI2C特性:
 *   - リトルエンディアンのバイトオーダー
 *   - デフォルトではI2C STOPで通信ウィンドウが閉じる
 *   - RESTART方式の読み取り（i2c_write_read）は非対応
 *     → レジスタアドレスなしのraw読み取り（i2c_read）を使用
 *   - ストリーミング出力はREL_X (0x1014) から開始、16バイト
 *   - 書き込みは通常のi2c_write（アドレス+データ）で動作
 *   - 1回のRDY期間中に1つのI2Cトランザクションのみ実行可能
 */

#define DT_DRV_COMPAT azoteq_iqs915x

#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "iqs915x.h"

LOG_MODULE_REGISTER(iqs915x, CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL);

/* ============================================================
 * I2C通信関数
 *
 * 書き込み: 通常のi2c_write（レジスタアドレス+データ）
 * 読み取り: i2c_read（レジスタアドレスなし、ストリーミング出力）
 *
 * IQS9150はRESTART方式の読み取り（i2c_write_read）に非対応。
 * 代わりにRDYごとにストリーミングデータをraw読み取りする。
 * ストリーミング出力: REL_X(0x1014)〜TRACKPAD_FLAGS(0x1022) の16バイト
 * ============================================================ */

// 16bitレジスタに書き込む（リトルエンディアン: LSB first）
static int iqs915x_write_reg16(const struct device *dev, uint16_t reg,
                               uint16_t val) {
  const struct iqs915x_config *config = dev->config;
  // アドレスもリトルエンディアン
  uint8_t buf[4] = {reg & 0xFF, reg >> 8, val & 0xFF, val >> 8};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

// 8bitレジスタに書き込む
static int iqs915x_write_reg8(const struct device *dev, uint16_t reg,
                              uint8_t val) {
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[3] = {reg & 0xFF, reg >> 8, val};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

// バイトブロックを指定アドレスに書き込む（init-data用）
// bufにはデータのみが含まれ、アドレスは本関数が付与する
static int iqs915x_write_block(const struct device *dev, uint16_t reg,
                               const uint8_t *data, uint16_t len) {
  const struct iqs915x_config *config = dev->config;
  // アドレス(2バイト) + データを連結したバッファを作成
  uint8_t buf[IQS915X_INIT_WRITE_CHUNK_SIZE + 2];

  if (len > IQS915X_INIT_WRITE_CHUNK_SIZE) {
    return -EINVAL;
  }

  buf[0] = reg & 0xFF;
  buf[1] = reg >> 8;
  memcpy(&buf[2], data, len);

  return i2c_write_dt(&config->i2c, buf, len + 2);
}

/* ============================================================
 * ストリーミングデータの読み取り
 *
 * IQS9150はRDY信号後にレジスタアドレスなしでI2C読み取りを行うと、
 * REL_X(0x1014)から16バイトのストリーミングデータを返す。
 *
 * メモリレイアウト (16 bytes, リトルエンディアン):
 *   [0-1]:   REL_X (signed int16)
 *   [2-3]:   REL_Y (signed int16)
 *   [4-5]:   GESTURE_X (uint16)
 *   [6-7]:   GESTURE_Y (uint16)
 *   [8-9]:   SINGLE_FINGER_GESTURES (uint16)
 *   [10-11]: TWO_FINGER_GESTURES (uint16)
 *   [12-13]: INFO_FLAGS (uint16)
 *   [14-15]: TRACKPAD_FLAGS (uint16)
 * ============================================================ */

// ストリーミングデータ構造体
struct iqs915x_stream_data {
  int16_t rel_x;
  int16_t rel_y;
  uint16_t gesture_x;
  uint16_t gesture_y;
  uint16_t gesture_sf; // Single Finger Gestures
  uint16_t gesture_tf; // Two Finger Gestures
  uint16_t info_flags;
  uint16_t trackpad_flags;
};

// ストリーミングデータを読み取る（レジスタアドレスなし）
static int iqs915x_read_stream(const struct device *dev,
                               struct iqs915x_stream_data *data) {
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[16];
  int ret;

  // レジスタアドレスなしでraw読み取り
  ret = i2c_read_dt(&config->i2c, buf, sizeof(buf));
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
 * ============================================================ */
static void iqs915x_button_release_work_handler(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, button_release_work);

  for (int i = 0; i < 3; i++) {
    if (data->buttons_pressed & BIT(i)) {
      input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
      data->buttons_pressed &= ~BIT(i);
    }
  }
}

/* ============================================================
 * 初期化ステートマシン
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
      return; // 次のRDYでリトライ
    }
    LOG_DBG("Init: ACK reset sent");
    data->init_step = INIT_WRITE_INIT_DATA;
    data->init_data_offset = 0;
    break;

  case INIT_WRITE_INIT_DATA: {
    // init-dataが未設定の場合はスキップ
    if (!config->init_data || config->init_data_len == 0) {
      LOG_DBG("Init: No init-data, skipping NVM write");
      data->init_step = INIT_CONFIG_SETTINGS;
      break;
    }

    uint16_t offset = data->init_data_offset;
    uint16_t total = config->init_data_len;

    if (offset < IQS915X_INIT_DATA_MAIN_SIZE) {
      // メイン領域 (0x115C〜0x15EB) の書き込み
      uint16_t remaining = IQS915X_INIT_DATA_MAIN_SIZE - offset;
      uint16_t chunk = MIN(remaining, IQS915X_INIT_WRITE_CHUNK_SIZE);
      uint16_t addr = IQS915X_INIT_DATA_BASE_ADDR + offset;

      ret = iqs915x_write_block(dev, addr, &config->init_data[offset], chunk);
      if (ret < 0) {
        LOG_ERR("Failed to write init-data at 0x%04x: %d", addr, ret);
        return; // 次のRDYでリトライ
      }
      LOG_DBG("Init: Wrote %d bytes at 0x%04x (%d/%d)", chunk, addr,
              offset + chunk, total);
      data->init_data_offset = offset + chunk;
    } else if (offset < total) {
      // エンジニアリング領域 (0x2000〜0x2005) の書き込み
      uint16_t eng_offset = offset - IQS915X_INIT_DATA_MAIN_SIZE;
      uint16_t remaining = IQS915X_INIT_DATA_ENG_SIZE - eng_offset;
      uint16_t chunk = MIN(remaining, IQS915X_INIT_WRITE_CHUNK_SIZE);
      uint16_t addr = IQS915X_INIT_DATA_ENG_ADDR + eng_offset;

      ret = iqs915x_write_block(dev, addr, &config->init_data[offset], chunk);
      if (ret < 0) {
        LOG_ERR("Failed to write init-data at 0x%04x: %d", addr, ret);
        return;
      }
      LOG_DBG("Init: Wrote %d eng bytes at 0x%04x (%d/%d)", chunk, addr,
              offset + chunk, total);
      data->init_data_offset = offset + chunk;
    }

    // 全データ書き込み完了判定
    if (data->init_data_offset >= total) {
      LOG_INF("Init: All init-data written (%d bytes)", total);
      data->init_step = INIT_CONFIG_SETTINGS;
    }
    // 未完了の場合は同じステップに留まり、次のRDYで続きを書き込む
    break;
  }

  case INIT_CONFIG_SETTINGS:
    // ジェスチャーエンジン有効化（イベントモード）
    // 省電力のためイベントモードを使用（IQS915X_EVENT_MODE）
    // カーソル移動(TP_EVENT)とジェスチャー(GESTURE_EVENT)の両方を有効化
    ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS,
                              IQS915X_EVENT_MODE | IQS915X_GESTURE_EVENT |
                                  IQS915X_TP_EVENT);
    if (ret < 0) {
      LOG_ERR("Failed to configure settings: %d", ret);
      return;
    }
    LOG_DBG("Init: Config settings written (event mode + tp event)");
    data->init_step = INIT_SINGLE_FINGER_GESTURES;
    break;

  case INIT_SINGLE_FINGER_GESTURES: {
    uint16_t gestures = 0;
    gestures |= config->one_finger_tap ? IQS915X_SINGLE_TAP : 0;
    gestures |= config->press_and_hold ? IQS915X_PRESS_AND_HOLD : 0;
    ret = iqs915x_write_reg16(dev, IQS915X_SINGLE_FINGER_GESTURES_ENABLE,
                              gestures);
    if (ret < 0) {
      LOG_ERR("Failed to configure single finger gestures: %d", ret);
      return;
    }
    LOG_DBG("Init: Single finger gestures configured");
    data->init_step = INIT_HOLD_TIME;
    break;
  }

  case INIT_HOLD_TIME:
    ret = iqs915x_write_reg16(dev, IQS915X_HOLD_TIME,
                              config->press_and_hold_time);
    if (ret < 0) {
      LOG_ERR("Failed to configure hold time: %d", ret);
      return;
    }
    LOG_DBG("Init: Hold time configured");
    data->init_step = INIT_TWO_FINGER_GESTURES;
    break;

  case INIT_TWO_FINGER_GESTURES: {
    uint16_t gestures = 0;
    gestures |= config->two_finger_tap ? IQS915X_TWO_FINGER_TAP : 0;
    gestures |= config->scroll ? IQS915X_SCROLL : 0;
    ret =
        iqs915x_write_reg16(dev, IQS915X_TWO_FINGER_GESTURES_ENABLE, gestures);
    if (ret < 0) {
      LOG_ERR("Failed to configure two finger gestures: %d", ret);
      return;
    }
    LOG_DBG("Init: Two finger gestures configured");
    data->init_step = INIT_TRACKPAD_SETTINGS;
    break;
  }

  case INIT_TRACKPAD_SETTINGS: {
    uint8_t settings = 0;
    settings |= config->flip_x ? IQS915X_FLIP_X : 0;
    settings |= config->flip_y ? IQS915X_FLIP_Y : 0;
    settings |= config->switch_xy ? IQS915X_SWITCH_XY_AXIS : 0;
    ret = iqs915x_write_reg8(dev, IQS915X_TRACKPAD_SETTINGS, settings);
    if (ret < 0) {
      LOG_ERR("Failed to configure trackpad settings: %d", ret);
      return;
    }
    LOG_DBG("Init: Trackpad settings configured");
    data->init_step = INIT_TAP_TIME;
    break;
  }

  case INIT_TAP_TIME:
    // DTSでtap-timeが指定されている場合のみ書き込み
    if (config->tap_time > 0) {
      ret = iqs915x_write_reg16(dev, IQS915X_TAP_TIME, config->tap_time);
      if (ret < 0) {
        LOG_ERR("Failed to configure tap time: %d", ret);
        return;
      }
      LOG_DBG("Init: Tap time set to %d ms", config->tap_time);
    }
    data->init_step = INIT_ACTIVE_REPORT_RATE;
    break;

  case INIT_ACTIVE_REPORT_RATE:
    // DTSでreport-rate-msが指定されている場合のみ書き込み
    if (config->report_rate_ms > 0) {
      ret = iqs915x_write_reg16(dev, IQS915X_ACTIVE_MODE_REPORT_RATE,
                                config->report_rate_ms);
      if (ret < 0) {
        LOG_ERR("Failed to configure active report rate: %d", ret);
        return;
      }
      LOG_DBG("Init: Active report rate set to %d ms", config->report_rate_ms);
    }
    data->init_step = INIT_IDLE_TOUCH_REPORT_RATE;
    break;

  case INIT_IDLE_TOUCH_REPORT_RATE:
    if (config->report_rate_ms > 0) {
      // テスト: Idle-Touchモードに入った場合でもサンプリングレートを維持させる
      ret = iqs915x_write_reg16(dev, IQS915X_IDLE_TOUCH_REPORT_RATE,
                                config->report_rate_ms);
      if (ret < 0) {
        LOG_ERR("Failed to configure idle-touch report rate: %d", ret);
        return;
      }
      LOG_DBG("Init: Idle-Touch report rate set to %d ms",
              config->report_rate_ms);
    }
    data->init_step = INIT_VERIFY_RESET;
    break;

  case INIT_VERIFY_RESET: {
    // 最終ACKの前に一度読み取りを行う
    // デバイスの状態を最新にしてフラグを確認する
    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0) {
      LOG_ERR("Failed to verify reset: %d", ret);
      return;
    }
    LOG_DBG("Init: Verify reset flags: 0x%04x", stream.info_flags);

    if (stream.info_flags & IQS915X_SHOW_RESET) {
      // まだSHOW_RESETが残っている場合はACKを送信するステートへ
      data->init_step = INIT_FINAL_ACK_RESET;
    } else {
      // SHOW_RESETがクリアされていれば初期化完了
      data->init_step = INIT_COMPLETE;
      data->initialized = true;
      data->work_state = WORK_READ_DATA;
      LOG_INF("IQS915x initialization complete");
    }
    break;
  }

  case INIT_FINAL_ACK_RESET:
    // リセットフラグをクリア
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL,
                              IQS915X_MODE_ACTIVE | IQS915X_ACK_RESET);
    if (ret < 0) {
      LOG_ERR("Failed to final ACK reset: %d", ret);
      return;
    }
    LOG_DBG("Init: Final ACK reset sent");
    // 次の通信ウィンドウで状態が更新されたか必ず確認する
    data->init_step = INIT_VERIFY_RESET;
    break;

  case INIT_COMPLETE:
    break;
  }
}

/* ============================================================
 * メインスレッド
 *
 * RDY割り込みでセマフォが解放され、ストリーミングデータをraw読み取りする。
 * IQS9150はRESTART方式の読み取りに非対応のため、
 * i2c_read（レジスタアドレスなし）で16バイトを一括読み取りする。
 * ============================================================ */
static void iqs915x_thread_main(void *p1, void *p2, void *p3) {
  struct iqs915x_data *data = p1;
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  int ret;

  while (true) {
    if (!data->initialized) {
      // 初期化中はRDYが来なくてもタイムアウト(20ms)で強制的に次のステップへ進み、
      // マスターからのI2C通信（クロックストレッチ）によってICをウェイクアップさせる。
      // これにより、初期化中にICが低消費電力モードに入ったり、イベントモードに
      // 移行してRDYがアサートされなくなることによる初期化ストップを防ぐ。
      k_sem_take(&data->rdy_sem, K_MSEC(20));
      iqs915x_init_step_handler(dev);
      continue;
    }

    // 通常動作時はRDY割り込みを待機
    k_sem_take(&data->rdy_sem, K_FOREVER);

    // ストリーミングデータをraw読み取り
    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0) {
      LOG_ERR("Failed to read stream: %d", ret);
      continue;
    }

    // IQS915xのランタイムリセット検出
    // NVM非搭載のため、リセット時は全設定が失われる → 再初期化が必須
    //
    // 注意: 初期化完了直後の最初の読み取りではSHOW_RESETを無視する。
    // 初期化シーケンスは書き込みのみ（ストリーミング読み取りなし）のため、
    // INIT_ACK_RESETで送ったフラグクリアが反映されず、SHOW_RESETが残留する。
    if (stream.info_flags & IQS915X_SHOW_RESET) {
      LOG_WRN(
          "IQS915x runtime reset detected (flags=0x%04x), re-initializing...",
          stream.info_flags);
      data->initialized = false;
      data->init_step = INIT_ACK_RESET;
      data->init_data_offset = 0;
      data->active_hold = false;
      data->buttons_pressed = 0;
      continue;
    }

    // トラックパッドデータの処理
    bool tp_movement = (stream.trackpad_flags & IQS915X_TP_MOVEMENT) != 0;
    bool scroll = (stream.gesture_tf & IQS915X_SCROLL) != 0;

    // 診断: ジェスチャーフラグが非ゼロのときにログ出力
    if (stream.gesture_sf || stream.gesture_tf) {
      LOG_DBG("gesture: sf=0x%04x tf=0x%04x gx=%d gy=%d rx=%d ry=%d tp=0x%04x",
              stream.gesture_sf, stream.gesture_tf, (int16_t)stream.gesture_x,
              (int16_t)stream.gesture_y, stream.rel_x, stream.rel_y,
              stream.trackpad_flags);
    }

    if (!scroll) {
      data->scroll_x_acc = 0;
      data->scroll_y_acc = 0;
    }

    // タップジェスチャー判定
    uint16_t button_code;
    bool button_pressed = false;
    if (stream.gesture_sf & IQS915X_SINGLE_TAP) {
      button_pressed = true;
      button_code = INPUT_BTN_0;
    } else if (stream.gesture_tf & IQS915X_TWO_FINGER_TAP) {
      button_pressed = true;
      button_code = INPUT_BTN_1;
    }

    // プレス＆ホールドの状態遷移
    bool hold_became_active =
        (stream.gesture_sf & IQS915X_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released =
        !(stream.gesture_sf & IQS915X_PRESS_AND_HOLD) && data->active_hold;

    // 移動とジェスチャーの処理
    if (hold_became_active) {
      input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
      data->active_hold = true;
    } else if (hold_released) {
      input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      data->active_hold = false;
    } else if (button_pressed) {
      k_work_cancel_delayable(&data->button_release_work);
      input_report_key(dev, button_code, 1, true, K_FOREVER);
      data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);
      k_work_schedule(&data->button_release_work, K_MSEC(100));
    } else if (scroll) {
      // スクロール: GESTURE_X/Yをスクロールデルタとして使用
      // (REL_X/REL_Yはスクロール中はゼロ)
      int16_t gx = (int16_t)stream.gesture_x;
      int16_t gy = (int16_t)stream.gesture_y;
      int16_t scroll_div = config->scroll_divisor;
      if (gx != 0) {
        if (!config->natural_scroll_x) {
          gx *= -1;
        }
        data->scroll_x_acc += gx;
        if (abs(data->scroll_x_acc) >= scroll_div) {
          input_report_rel(dev, INPUT_REL_HWHEEL,
                           data->scroll_x_acc / scroll_div, true, K_FOREVER);
          data->scroll_x_acc %= scroll_div;
        }
      }
      if (gy != 0) {
        if (config->natural_scroll_y) {
          gy *= -1;
        }
        data->scroll_y_acc += gy;
        if (abs(data->scroll_y_acc) >= scroll_div) {
          input_report_rel(dev, INPUT_REL_WHEEL,
                           data->scroll_y_acc / scroll_div, true, K_FOREVER);
          data->scroll_y_acc %= scroll_div;
        }
      }
    } else if (tp_movement) {
      if (stream.rel_x != 0 || stream.rel_y != 0) {
        LOG_DBG("tp_movement: rel_x=%d, rel_y=%d", stream.rel_x, stream.rel_y);
        input_report_rel(dev, INPUT_REL_X, stream.rel_x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, stream.rel_y, true, K_FOREVER);
      }
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

  LOG_DBG("rdy_handler called");
  k_sem_give(&data->rdy_sem);
}

/* ============================================================
 * デバイス初期化関数
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

  k_sem_init(&data->rdy_sem, 0, 1);
  k_work_init_delayable(&data->button_release_work,
                        iqs915x_button_release_work_handler);

  // 専用スレッドの起動 (優先度: 高め K_PRIO_COOP(2))
  k_thread_create(&data->thread, data->thread_stack,
                  K_KERNEL_STACK_SIZEOF(data->thread_stack),
                  iqs915x_thread_main, data, NULL, NULL, K_PRIO_COOP(2), 0,
                  K_NO_WAIT);
  k_thread_name_set(&data->thread, "iqs915x");

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
 * ============================================================ */
#define IQS915X_INIT(n)                                                        \
  static const uint8_t iqs915x_init_data_##n[] =                               \
      DT_INST_PROP_OR(n, azoteq_init_data, {0});                               \
  static struct iqs915x_data iqs915x_data_##n;                                 \
  static const struct iqs915x_config iqs915x_config_##n = {                    \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                          \
      .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                         \
      .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),             \
      .init_data = iqs915x_init_data_##n,                                      \
      .init_data_len =                                                         \
          COND_CODE_1(DT_INST_NODE_HAS_PROP(n, azoteq_init_data),              \
                      (DT_INST_PROP_LEN(n, azoteq_init_data)), (0)),           \
      .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                       \
      .press_and_hold = DT_INST_PROP(n, press_and_hold),                       \
      .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                       \
      .scroll = DT_INST_PROP(n, scroll),                                       \
      .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                   \
      .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                   \
      .scroll_divisor = DT_INST_PROP(n, scroll_divisor),                       \
      .tap_time = DT_INST_PROP_OR(n, tap_time, 0),                             \
      .report_rate_ms = DT_INST_PROP_OR(n, report_rate_ms, 0),                 \
      .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),     \
      .switch_xy = DT_INST_PROP(n, switch_xy),                                 \
      .flip_x = DT_INST_PROP(n, flip_x),                                       \
      .flip_y = DT_INST_PROP(n, flip_y),                                       \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, iqs915x_init, NULL, &iqs915x_data_##n,              \
                        &iqs915x_config_##n, POST_KERNEL,                      \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS915X_INIT)
