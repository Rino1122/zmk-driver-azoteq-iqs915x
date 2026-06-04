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
 *   - ストリーミング出力はREL_X (0x1014) から開始、
 *     本ドライバではFINGER2_X/Yまで含めて28バイトを読む
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

#include <iqs915x.h>
#include "iqs915x_regs.h"

LOG_MODULE_REGISTER(iqs915x, CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL);

#define GESTURE_POINTER_SUPPRESS_TAIL_TICKS 1

/* ============================================================
 * I2C通信関数
 *
 * 書き込み: 通常のi2c_write（アドレス+データ）
 * 読み取り: i2c_write_read（スレーブアドレス+Write -> レジスタアドレス ->
 * Repeated START -> スレーブアドレス+Read -> 読み出し -> STOP/NACK）
 *   Zephyrのi2c_write_read_dtでデータシート要件の読み取りシーケンスが完結する。
 * ストリーミング出力: 0x1014〜0x102B の24バイト
 * ============================================================ */

// 16bitレジスタに書き込む（リトルエンディアン: LSB first）
static int iqs915x_write_reg16(const struct device *dev, uint16_t reg,
                               uint16_t val)
{
  const struct iqs915x_config *config = dev->config;
  // アドレスもリトルエンディアン
  uint8_t buf[4] = {reg & 0xFF, reg >> 8, val & 0xFF, val >> 8};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

// 8bitレジスタに書き込む
static int iqs915x_write_reg8(const struct device *dev, uint16_t reg,
                              uint8_t val)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t buf[3] = {reg & 0xFF, reg >> 8, val};

  return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

// 16bitレジスタを読み込む
static int iqs915x_read_reg16(const struct device *dev, uint16_t reg,
                              uint16_t *val)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_addr[2] = {reg & 0xFF, reg >> 8};
  uint8_t buf[2];
  int ret = i2c_write_read_dt(&config->i2c, reg_addr, 2, buf, sizeof(buf));
  if (ret < 0)
  {
    return ret;
  }
  *val = (buf[1] << 8) | buf[0];
  return 0;
}

// 8bitレジスタを読み込む
static int iqs915x_read_reg8(const struct device *dev, uint16_t reg,
                             uint8_t *val)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_addr[2] = {reg & 0xFF, reg >> 8};
  return i2c_write_read_dt(&config->i2c, reg_addr, 2, val, 1);
}

// 16bitレジスタを読み出し、値が異なる場合のみ書き込む
// 戻り値: 0=変更なし, 1=書き込み発生, 負値=エラー
static int iqs915x_update_reg16(const struct device *dev, uint16_t reg,
                                uint16_t val)
{
  uint16_t current_val = 0;
  int ret = iqs915x_read_reg16(dev, reg, &current_val);
  if (ret < 0)
    return ret;
  if (current_val == val)
    return 0;
  ret = iqs915x_write_reg16(dev, reg, val);
  if (ret < 0)
    return ret;
  return 1; // 書き込みが発生したことを示す
}

// 8bitレジスタを読み出し、値が異なる場合のみ書き込む
// 戻り値: 0=変更なし, 1=書き込み発生, 負値=エラー
static int iqs915x_update_reg8(const struct device *dev, uint16_t reg,
                               uint8_t val)
{
  uint8_t current_val = 0;
  int ret = iqs915x_read_reg8(dev, reg, &current_val);
  if (ret < 0)
    return ret;
  if (current_val == val)
    return 0;
  ret = iqs915x_write_reg8(dev, reg, val);
  if (ret < 0)
    return ret;
  return 1; // 書き込みが発生したことを示す
}

// 16bitレジスタの特定ビットを変更する（リード・モディファイ・ライト）
// 戻り値: 0=変更なし, 1=書き込み発生, 負値=エラー
static int iqs915x_modify_reg16(const struct device *dev, uint16_t reg,
                                uint16_t clear_mask, uint16_t set_mask)
{
  uint16_t current_val = 0;
  int ret = iqs915x_read_reg16(dev, reg, &current_val);
  if (ret < 0)
    return ret;
  uint16_t new_val = (current_val & ~clear_mask) | set_mask;
  if (current_val == new_val)
    return 0;
  ret = iqs915x_write_reg16(dev, reg, new_val);
  if (ret < 0)
    return ret;
  return 1; // 書き込みが発生したことを示す
}

// 8bitレジスタの特定ビットを変更する
// 戻り値: 0=変更なし, 1=書き込み発生, 負値=エラー
static int iqs915x_modify_reg8(const struct device *dev, uint16_t reg,
                               uint8_t clear_mask, uint8_t set_mask)
{
  uint8_t current_val = 0;
  int ret = iqs915x_read_reg8(dev, reg, &current_val);
  if (ret < 0)
    return ret;
  uint8_t new_val = (current_val & ~clear_mask) | set_mask;
  if (current_val == new_val)
    return 0;
  ret = iqs915x_write_reg8(dev, reg, new_val);
  if (ret < 0)
    return ret;
  return 1; // 書き込みが発生したことを示す
}

// バイトブロックを指定アドレスに書き込む（init-data用）
// bufにはデータのみが含まれ、アドレスは本関数が付与する
static int iqs915x_write_block(const struct device *dev, uint16_t reg,
                               const uint8_t *data, uint16_t len)
{
  const struct iqs915x_config *config = dev->config;
  // アドレス(2バイト) + データを連結したバッファを作成
  uint8_t buf[IQS915X_INIT_WRITE_CHUNK_SIZE + 2];

  if (len > IQS915X_INIT_WRITE_CHUNK_SIZE)
  {
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
 * REL_X(0x1014)から28バイトのストリーミングデータを返す。
 *
 * メモリレイアウト (28 bytes, リトルエンディアン):
 *   [0-1]:   REL_X (signed int16)
 *   [2-3]:   REL_Y (signed int16)
 *   [4-5]:   GESTURE_X (uint16)
 *   [6-7]:   GESTURE_Y (uint16)
 *   [8-9]:   SINGLE_FINGER_GESTURES (uint16)
 *   [10-11]: TWO_FINGER_GESTURES (uint16)
 *   [12-13]: INFO_FLAGS (uint16)
 *   [14-15]: TRACKPAD_FLAGS (uint16)
 *   [16-17]: FINGER1_X (uint16)
 *   [18-19]: FINGER1_Y (uint16)
 *   [20-21]: FINGER1_STRENGTH (uint16)
 *   [22-23]: FINGER1_AREA (uint16)
 *   [24-25]: FINGER2_X (uint16)
 *   [26-27]: FINGER2_Y (uint16)
 * ============================================================ */

// ストリーミングデータ構造体
struct iqs915x_stream_data
{
  int16_t rel_x;
  int16_t rel_y;
  uint16_t gesture_x;
  uint16_t gesture_y;
  uint16_t gesture_sf; // Single Finger Gestures
  uint16_t gesture_tf; // Two Finger Gestures
  uint16_t info_flags;
  uint16_t trackpad_flags;
  uint16_t abs_x;
  uint16_t abs_y;
  uint16_t finger2_x;
  uint16_t finger2_y;
};

// ストリーミングデータを読み取る
static int iqs915x_read_stream(const struct device *dev,
                               struct iqs915x_stream_data *data)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_addr[2] = {IQS915X_REL_X & 0xFF, IQS915X_REL_X >> 8};
  uint8_t buf[28];
  int ret;

  // アドレスを指定してRESTARTで読み取る
  ret = i2c_write_read_dt(&config->i2c, reg_addr, 2, buf, sizeof(buf));
  if (ret < 0)
  {
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
  data->abs_x = (buf[17] << 8) | buf[16];
  data->abs_y = (buf[19] << 8) | buf[18];
  data->finger2_x = (buf[25] << 8) | buf[24];
  data->finger2_y = (buf[27] << 8) | buf[26];

  return 0;
}

static void iqs915x_reset_absolute_tracking(struct iqs915x_data *data)
{
  data->last_abs_x = 0;
  data->last_abs_y = 0;
  data->last_abs_valid = false;
}

static void iqs915x_reset_two_finger_session(struct iqs915x_data *data)
{
  memset(&data->two_finger, 0, sizeof(data->two_finger));
  memset(&data->pinch_motion_history, 0, sizeof(data->pinch_motion_history));
  memset(&data->pinch_inertia_state, 0, sizeof(data->pinch_inertia_state));
}

static void iqs915x_reset_runtime_gesture_state(struct iqs915x_data *data)
{
  memset(&data->finger_tracker, 0, sizeof(data->finger_tracker));
  memset(&data->scroll_motion_history, 0, sizeof(data->scroll_motion_history));
  memset(&data->scroll_inertia_state, 0, sizeof(data->scroll_inertia_state));
  iqs915x_reset_two_finger_session(data);
}

static void iqs915x_update_finger_state(const struct iqs915x_config *config,
                                        struct iqs915x_data *data,
                                        const struct iqs915x_stream_data *stream)
{
  struct iqs915x_finger_tracker *tracker = &data->finger_tracker;
  struct iqs915x_two_finger_session *two_finger = &data->two_finger;
  uint8_t raw_count = stream->trackpad_flags & IQS915X_NUM_FINGERS_MASK;
  uint8_t debounce_frames =
      config->finger_debounce_frames > 0 ? config->finger_debounce_frames : 1;
  uint8_t stable_before = tracker->stable_count;

  tracker->current_count = raw_count;
  tracker->previous_count = stable_before;

  if (raw_count == stable_before)
  {
    tracker->pending_count = raw_count;
    tracker->debounce_count = 0;
  }
  else if (raw_count != tracker->pending_count)
  {
    tracker->pending_count = raw_count;
    tracker->debounce_count = 1;
  }
  else if (tracker->debounce_count < UINT8_MAX)
  {
    tracker->debounce_count++;
  }

  if (tracker->pending_count != stable_before &&
      tracker->debounce_count >= debounce_frames)
  {
    tracker->stable_count = tracker->pending_count;
  }

  tracker->tail_suppressed =
      stable_before == 2 && tracker->stable_count == 1;

  if (tracker->stable_count != 2 || raw_count < 2)
  {
    iqs915x_reset_two_finger_session(data);
    return;
  }

  int32_t centroid_x = ((int32_t)stream->abs_x + (int32_t)stream->finger2_x) / 2;
  int32_t centroid_y = ((int32_t)stream->abs_y + (int32_t)stream->finger2_y) / 2;
  int32_t finger_dx = (int32_t)stream->abs_x - (int32_t)stream->finger2_x;
  int32_t finger_dy = (int32_t)stream->abs_y - (int32_t)stream->finger2_y;
  int32_t distance = abs(finger_dx) + abs(finger_dy);

  if (!two_finger->active)
  {
    two_finger->active = true;
    two_finger->mode = IQS915X_2F_MODE_NONE;
    two_finger->centroid_last_x = centroid_x;
    two_finger->centroid_last_y = centroid_y;
    two_finger->distance_last = distance;
    return;
  }

  two_finger->centroid_dx = centroid_x - two_finger->centroid_last_x;
  two_finger->centroid_dy = centroid_y - two_finger->centroid_last_y;
  two_finger->distance_delta = distance - two_finger->distance_last;
  two_finger->centroid_last_x = centroid_x;
  two_finger->centroid_last_y = centroid_y;
  two_finger->distance_last = distance;
}

/* ============================================================
 * Power mode control
 *
 * Manual Control有効時は、ホストがSystem ControlのMode Selectで
 * Active/LP2を切り替える。イベントモード中はRDY待ちのない期間も
 * あるため、power API由来のモード変更はForce Comms前提で
 * i2c_writeを発行する。power APIの遷移ではTP Reseedは行わない。
 * ============================================================ */

static int iqs915x_write_power_mode(const struct device *dev, uint16_t mode)
{
  int ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, mode);

  if (ret < 0)
  {
    LOG_ERR("Failed to set power mode 0x%04x: %d", mode, ret);
    return ret;
  }

  return 0;
}

/* ============================================================
 * ボタンリリース遅延処理
 * ============================================================ */
static void iqs915x_button_release_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, button_release_work);

  for (int i = 0; i < 3; i++)
  {
    if (data->buttons_pressed & BIT(i))
    {
      // ドラッグ中（active_hold）の場合は左クリック(i=0)の離上をスキップ
      if (i == 0 && data->active_hold)
      {
        continue;
      }
      input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
      data->buttons_pressed &= ~BIT(i);
    }
  }
}

/* ============================================================
 * スクロール慣性処理
 *
 * 指が離れた後、最後のスクロール速度に基づいて減衰しながら
 * スクロール信号を送り続ける。タイマーの各ティックで速度に
 * 減衰率（friction）を掛けて減速し、閾値未満になったら停止する。
 * ============================================================ */

#define IQS915X_SCROLL_INERTIA_FP_BITS 8
#define IQS915X_SCROLL_INERTIA_FP_SCALE BIT(IQS915X_SCROLL_INERTIA_FP_BITS)

static void iqs915x_reset_motion_history(struct iqs915x_motion_history *history)
{
  memset(history, 0, sizeof(*history));
}

static void iqs915x_stop_scroll_inertia(struct iqs915x_data *data)
{
  k_work_cancel_delayable(&data->scroll_inertia_work);
  memset(&data->scroll_inertia_state, 0, sizeof(data->scroll_inertia_state));
  data->scroll_x_acc = 0;
  data->scroll_y_acc = 0;
}

static void iqs915x_reset_scroll_inertia(struct iqs915x_data *data)
{
  iqs915x_stop_scroll_inertia(data);
  iqs915x_reset_motion_history(&data->scroll_motion_history);
}

static void iqs915x_record_scroll_sample(struct iqs915x_data *data, int64_t now_ms,
                                         int16_t delta_x, int16_t delta_y)
{
  struct iqs915x_motion_history *history = &data->scroll_motion_history;
  struct iqs915x_motion_sample *sample = &history->samples[history->head];

  sample->ms = now_ms;
  sample->x = delta_x;
  sample->y = delta_y;
  history->head = (history->head + 1) % ARRAY_SIZE(history->samples);
  if (history->count < ARRAY_SIZE(history->samples))
  {
    history->count++;
  }
}

static bool iqs915x_calc_scroll_inertia_seed(const struct iqs915x_config *config,
                                             struct iqs915x_data *data,
                                             int64_t now_ms, int32_t *seed_x_fp,
                                             int32_t *seed_y_fp)
{
  const struct iqs915x_inertia_profile *profile = &config->scroll_inertia;
  const struct iqs915x_motion_history *history = &data->scroll_motion_history;
  int64_t newest_ms;
  int32_t sum_x = 0;
  int32_t sum_y = 0;
  uint8_t used = 0;

  *seed_x_fp = 0;
  *seed_y_fp = 0;

  if (!profile->enabled || history->count == 0)
  {
    return false;
  }

  newest_ms = history->samples[(history->head + ARRAY_SIZE(history->samples) - 1) %
                               ARRAY_SIZE(history->samples)]
                  .ms;
  if ((now_ms - newest_ms) > profile->stale_gap_ms)
  {
    return false;
  }

  for (uint8_t offset = 0; offset < history->count; offset++)
  {
    uint8_t idx =
        (history->head + ARRAY_SIZE(history->samples) - 1 - offset) %
        ARRAY_SIZE(history->samples);
    const struct iqs915x_motion_sample *sample = &history->samples[idx];

    if ((now_ms - sample->ms) > profile->recent_window_ms)
    {
      break;
    }

    sum_x += sample->x;
    sum_y += sample->y;
    used++;
  }

  if (used < profile->min_samples)
  {
    return false;
  }

  sum_x /= used;
  sum_y /= used;

  if (abs(sum_x) < profile->min_avg_speed && abs(sum_y) < profile->min_avg_speed)
  {
    return false;
  }

  *seed_x_fp = sum_x * IQS915X_SCROLL_INERTIA_FP_SCALE;
  *seed_y_fp = sum_y * IQS915X_SCROLL_INERTIA_FP_SCALE;
  return true;
}

static void iqs915x_scroll_inertia_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, scroll_inertia_work);
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  const struct iqs915x_inertia_profile *profile = &config->scroll_inertia;
  struct iqs915x_inertia_state *state = &data->scroll_inertia_state;
  int32_t step_x;
  int32_t step_y;
  int16_t scroll_div = config->scroll_divisor;

  if (!state->active || !profile->enabled)
  {
    return;
  }

  state->velocity_x_fp =
      (state->velocity_x_fp * profile->decay_x1000) / 1000;
  state->velocity_y_fp =
      (state->velocity_y_fp * profile->decay_x1000) / 1000;

  if (abs(state->velocity_x_fp) <
          (profile->min_avg_speed * IQS915X_SCROLL_INERTIA_FP_SCALE) &&
      abs(state->velocity_y_fp) <
          (profile->min_avg_speed * IQS915X_SCROLL_INERTIA_FP_SCALE))
  {
    iqs915x_stop_scroll_inertia(data);
    LOG_DBG("Scroll inertia stopped (velocity below threshold)");
    return;
  }

  state->accum_x_fp += state->velocity_x_fp;
  state->accum_y_fp += state->velocity_y_fp;
  step_x = state->accum_x_fp / IQS915X_SCROLL_INERTIA_FP_SCALE;
  step_y = state->accum_y_fp / IQS915X_SCROLL_INERTIA_FP_SCALE;
  state->accum_x_fp -= step_x * IQS915X_SCROLL_INERTIA_FP_SCALE;
  state->accum_y_fp -= step_y * IQS915X_SCROLL_INERTIA_FP_SCALE;

  if (step_x != 0)
  {
    data->scroll_x_acc += step_x;
    if (abs(data->scroll_x_acc) >= scroll_div)
    {
      input_report_rel(dev, INPUT_REL_HWHEEL,
                       data->scroll_x_acc / scroll_div, true, K_FOREVER);
      data->scroll_x_acc %= scroll_div;
    }
  }

  if (step_y != 0)
  {
    data->scroll_y_acc += step_y;
    if (abs(data->scroll_y_acc) >= scroll_div)
    {
      input_report_rel(dev, INPUT_REL_WHEEL,
                       data->scroll_y_acc / scroll_div, true, K_FOREVER);
      data->scroll_y_acc %= scroll_div;
    }
  }

  // 次のティックをスケジュール
  k_work_schedule(&data->scroll_inertia_work,
                  K_MSEC(profile->interval_ms));
}

// スクロール慣性を打ち切る（新しい操作が入った場合に呼ばれる）
static void iqs915x_cancel_scroll_inertia(struct iqs915x_data *data)
{
  if (data->scroll_inertia_state.active)
  {
    iqs915x_stop_scroll_inertia(data);
    LOG_DBG("Scroll inertia cancelled by new input");
  }
}

/* ============================================================
 * 初期化ステートマシン
 * ============================================================ */
static void iqs915x_init_step_handler(const struct device *dev)
{
  struct iqs915x_data *data = dev->data;
  const struct iqs915x_config *config = dev->config;
  int ret;

  switch (data->init_step)
  {
  case INIT_CHECK_SHOW_RESET:
  {
    // 起動直後まずSHOW_RESETを読み出して確認する
    uint16_t info_flags = 0;
    ret = iqs915x_read_reg16(dev, IQS915X_INFO_FLAGS, &info_flags);
    if (ret < 0)
    {
      LOG_ERR("Failed to read Info Flags: %d", ret);
      return; // 次のRDYでリトライ
    }

    if (info_flags == 0xEEEE)
    {
      // ICがまだビジー状態。次のRDYサイクルで再試行する
      LOG_DBG("Init: IC busy (0xEEEE), waiting...");
      break;
    }

    // INFO_FLAGS bit7 = SHOW_RESET。電源投入直後のリセット時にセットされる
    if (info_flags & IQS915X_SHOW_RESET)
    {
      LOG_INF("Init: SHOW_RESET is set (0x%04x). Proceed to write init-data.",
              info_flags);
      data->init_step = INIT_WRITE_INIT_DATA;
      data->init_data_offset = 0;
    }
    else
    {
      // warm bootなどでSHOW_RESETが立っていない場合は、
      // ソフトウェアリセットで既知の初期化シーケンスへ戻す
      LOG_INF("Init: SHOW_RESET is not set (0x%04x). Requesting software reset.",
              info_flags);
      data->init_step = INIT_SOFTWARE_RESET;
    }
    break;
  }

  case INIT_SOFTWARE_RESET:
    LOG_DBG("Init: Sending Software Reset (0x0200 to System Control)");
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, IQS915X_SW_RESET);
    if (ret < 0)
    {
      LOG_ERR("Failed to send SW Reset: %d", ret);
      return;
    }
    data->init_step = INIT_WAIT_SOFTWARE_RESET;
    data->wait_count = 0;
    break;

  case INIT_WAIT_SOFTWARE_RESET:
  {
    data->wait_count++;
    if (data->wait_count <= 10)
    {
      LOG_DBG("Init: Pausing for SW Reset (%d/10)", data->wait_count);
      break;
    }

    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0)
    {
      LOG_ERR("Failed to read during SW Reset wait: %d", ret);
      return;
    }

    if (stream.info_flags == 0xEEEE)
    {
      LOG_DBG("Init: SW Reset IC busy (0xEEEE)");
    }
    else if (stream.info_flags & IQS915X_SHOW_RESET)
    {
      LOG_INF("Init: SW Reset complete, SHOW_RESET is set");
      data->init_step = INIT_WRITE_INIT_DATA;
      data->init_data_offset = 0;
    }
    else
    {
      LOG_DBG("Init: Waiting for SHOW_RESET (flags=0x%04x)", stream.info_flags);
    }
    break;
  }

  case INIT_ACK_RESET:
  {
    uint16_t sys_ctrl =
        IQS915X_ACK_RESET | IQS915X_REATI_TP | IQS915X_REATI_ALP;
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, sys_ctrl);
    if (ret < 0)
    {
      LOG_ERR("Failed to send initial ACK reset: %d", ret);
      return;
    }
    LOG_DBG("Init: Sent initial Ack Reset and Re-ATI");
    data->init_step = INIT_WAIT_REATI;
    data->wait_count = 0;
    break;
  }

  case INIT_WRITE_INIT_DATA:
  {
    if (!config->init_data || config->init_data_len == 0)
    {
      // init-dataは必須。YAMLバインディングでrequired: trueとしているため
      // 通常はビルド時にエラーとなるが、実行時の安全策としても確認する。
      LOG_ERR("Init: init-data is required but not set. Halting.");
      return;
    }

    uint16_t offset = data->init_data_offset;
    uint16_t total = config->init_data_len;

    if (offset < IQS915X_INIT_DATA_MAIN_SIZE)
    {
      uint16_t remaining = IQS915X_INIT_DATA_MAIN_SIZE - offset;
      uint16_t chunk = MIN(remaining, IQS915X_INIT_WRITE_CHUNK_SIZE);
      uint16_t addr = IQS915X_INIT_DATA_BASE_ADDR + offset;

      uint8_t buffer[IQS915X_INIT_WRITE_CHUNK_SIZE];
      memcpy(buffer, &config->init_data[offset], chunk);

      for (int i = 0; i < chunk; i++)
      {
        uint16_t current_addr = addr + i;
        uint8_t original = buffer[i];
        if (current_addr == IQS915X_SYSTEM_CONTROL)
        {
          // ACK_RESET(bit7), REATI_ALP(bit6), REATI_TP(bit5) は
          // 初期化シーケンス中に誤って実行されないよう強制クリアする
          buffer[i] &= ~0xE0;
        }
        else if (current_addr == IQS915X_CONFIG_SETTINGS)
        {
          // TERMINATE_COMMS(bit6), FORCE_COMMS_METHOD(bit4) は
          // クロックストレッチ＋I2C STOPによる標準動作のため強制クリアし、
          // MANUAL_CONTROL(bit7) は常に有効化する
          buffer[i] = (buffer[i] & ~0x50) | 0x80;
        }
        else if (current_addr == IQS915X_CONFIG_SETTINGS + 1)
        {
          // EVENT_MODE(bit8, 上位バイトのbit0) は必須機能のため強制セットする
          buffer[i] |= 0x01;
        }
        // init-dataのバイト値がドライバにより上書きされた場合はWRNを出力する
        if (buffer[i] != original)
        {
          LOG_WRN("Init: init-data overridden at reg 0x%04x: "
                  "init-data=0x%02x -> forced=0x%02x",
                  current_addr, original, buffer[i]);
        }
      }

      ret = iqs915x_write_block(dev, addr, buffer, chunk);
      if (ret < 0)
      {
        LOG_ERR("Failed to write init-data at 0x%04x: %d", addr, ret);
        return; // 次のRDYでリトライ
      }
      LOG_DBG("Init: Wrote %d bytes at 0x%04x (%d/%d)", chunk, addr,
              offset + chunk, total);
      data->init_data_offset = offset + chunk;
    }
    else if (offset < total)
    {
      uint16_t eng_offset = offset - IQS915X_INIT_DATA_MAIN_SIZE;
      uint16_t remaining = IQS915X_INIT_DATA_ENG_SIZE - eng_offset;
      uint16_t chunk = MIN(remaining, IQS915X_INIT_WRITE_CHUNK_SIZE);
      uint16_t addr = IQS915X_INIT_DATA_ENG_ADDR + eng_offset;

      ret = iqs915x_write_block(dev, addr, &config->init_data[offset], chunk);
      if (ret < 0)
      {
        LOG_ERR("Failed to write init-data at 0x%04x: %d", addr, ret);
        return;
      }
      LOG_DBG("Init: Wrote %d eng bytes at 0x%04x (%d/%d)", chunk, addr,
              offset + chunk, total);
      data->init_data_offset = offset + chunk;
    }

    if (data->init_data_offset >= total)
    {
      LOG_INF("Init: All init-data written (%d bytes)", total);
      // init-data書き込み完了後は、ACK ResetとATI実行へ進む
      data->init_step = INIT_ACK_RESET;
    }
    break;
  }

  case INIT_SINGLE_FINGER_GESTURES:
  {
    // ソフトウェアによるTap-and-Drag実装のため、ハードウェアのPRESS_AND_HOLDは常にクリアする
    uint16_t clear_mask = IQS915X_SINGLE_TAP | IQS915X_PRESS_AND_HOLD;
    uint16_t set_mask = 0;
    if (config->one_finger_tap)
      set_mask |= IQS915X_SINGLE_TAP;
    // config->press_and_hold が true の場合でも、ハードウェア機能は有効化しない
    // （ドライバ側ソフトウェアでTap-and-Dragとしてエミュレートするため）

    ret = iqs915x_modify_reg16(dev, IQS915X_SINGLE_FINGER_GESTURES_ENABLE,
                               clear_mask, set_mask);
    if (ret < 0)
    {
      LOG_ERR("Failed to configure single finger gestures: %d", ret);
      return;
    }
    // ハードウェアのPRESS_AND_HOLDフラグの状態に関わらず、DTS設定の値をログ出力する
    if (ret > 0 && config->init_data_len > 0)
    {
      LOG_WRN("Init: SF_GESTURES_ENABLE (0x%04x) overridden from init-data "
              "value by DTS (one_finger_tap=%d, press_and_hold=%d [SW Emulated])",
              IQS915X_SINGLE_FINGER_GESTURES_ENABLE, config->one_finger_tap,
              config->press_and_hold);
    }
    LOG_DBG("Init: Single finger gestures configured (HW Hold disabled for SW Tap-and-Drag)");
    data->init_step = INIT_HOLD_TIME;
    break;
  }

  case INIT_HOLD_TIME:
    ret = iqs915x_update_reg16(dev, IQS915X_HOLD_TIME,
                               config->press_and_hold_time);
    if (ret < 0)
    {
      LOG_ERR("Failed to configure hold time: %d", ret);
      return;
    }
    if (ret > 0 && config->init_data_len > 0)
    {
      LOG_WRN("Init: HOLD_TIME (0x%04x) overridden from init-data value "
              "by DTS (press_and_hold_time=%d ms)",
              IQS915X_HOLD_TIME, config->press_and_hold_time);
    }
    LOG_DBG("Init: Hold time configured");
    data->init_step = INIT_TWO_FINGER_GESTURES;
    break;

  case INIT_TWO_FINGER_GESTURES:
  {
    uint16_t clear_mask = IQS915X_TWO_FINGER_TAP | IQS915X_SCROLL;
    uint16_t set_mask = 0;
    if (config->two_finger_tap)
      set_mask |= IQS915X_TWO_FINGER_TAP;
    if (config->scroll)
      set_mask |= IQS915X_SCROLL;

    ret = iqs915x_modify_reg16(dev, IQS915X_TWO_FINGER_GESTURES_ENABLE,
                               clear_mask, set_mask);
    if (ret < 0)
    {
      LOG_ERR("Failed to configure two finger gestures: %d", ret);
      return;
    }
    if (ret > 0 && config->init_data_len > 0)
    {
      LOG_WRN("Init: TF_GESTURES_ENABLE (0x%04x) overridden from init-data "
              "value by DTS (two_finger_tap=%d, scroll=%d)",
              IQS915X_TWO_FINGER_GESTURES_ENABLE, config->two_finger_tap,
              config->scroll);
    }
    LOG_DBG("Init: Two finger gestures configured");
    data->init_step = INIT_TRACKPAD_SETTINGS;
    break;
  }

  case INIT_TRACKPAD_SETTINGS:
  {
    uint8_t clear_mask =
        IQS915X_FLIP_X | IQS915X_FLIP_Y | IQS915X_SWITCH_XY_AXIS;
    uint8_t set_mask = 0;
    if (config->flip_x)
      set_mask |= IQS915X_FLIP_X;
    if (config->flip_y)
      set_mask |= IQS915X_FLIP_Y;
    if (config->switch_xy)
      set_mask |= IQS915X_SWITCH_XY_AXIS;

    ret = iqs915x_modify_reg8(dev, IQS915X_TRACKPAD_SETTINGS, clear_mask,
                              set_mask);
    if (ret < 0)
    {
      LOG_ERR("Failed to configure trackpad settings: %d", ret);
      return;
    }
    if (ret > 0 && config->init_data_len > 0)
    {
      LOG_WRN("Init: TRACKPAD_SETTINGS (0x%04x) overridden from init-data "
              "value by DTS (flip_x=%d, flip_y=%d, switch_xy=%d)",
              IQS915X_TRACKPAD_SETTINGS, config->flip_x, config->flip_y,
              config->switch_xy);
    }
    LOG_DBG("Init: Trackpad settings configured");
    data->init_step = INIT_TAP_TIME;
    break;
  }

  case INIT_TAP_TIME:
    if (config->tap_time > 0)
    {
      ret = iqs915x_update_reg16(dev, IQS915X_TAP_TIME, config->tap_time);
      if (ret < 0)
      {
        LOG_ERR("Failed to configure tap time: %d", ret);
        return;
      }
      if (ret > 0 && config->init_data_len > 0)
      {
        LOG_WRN("Init: TAP_TIME (0x%04x) overridden from init-data value "
                "by DTS (tap_time=%d ms)",
                IQS915X_TAP_TIME, config->tap_time);
      }
      LOG_DBG("Init: Tap time set to %d ms", config->tap_time);
    }
    data->init_step = INIT_ACTIVE_REPORT_RATE;
    break;

  case INIT_ACTIVE_REPORT_RATE:
    // report-rate-msはActive modeのレポートレートのみに適用する。
    // Idle-Touch/Idle/LP1/LP2モードのレポートレートはinit-dataの設定に委ねる。
    if (config->report_rate_ms > 0)
    {
      ret = iqs915x_update_reg16(dev, IQS915X_ACTIVE_MODE_REPORT_RATE,
                                 config->report_rate_ms);
      if (ret < 0)
      {
        LOG_ERR("Failed to configure active report rate: %d", ret);
        return;
      }
      if (ret > 0)
      {
        LOG_WRN("Init: ACTIVE_REPORT_RATE (0x%04x) overridden from init-data "
                "value by DTS (report_rate_ms=%d ms)",
                IQS915X_ACTIVE_MODE_REPORT_RATE, config->report_rate_ms);
      }
      LOG_DBG("Init: Active report rate set to %d ms", config->report_rate_ms);
    }
    data->init_step = INIT_VERIFY_EVENT_MODE;
    break;

  case INIT_VERIFY_EVENT_MODE:
  {
    // DTS設定完了後、Event ModeとManual Controlが有効になっているかを確認する。
    // init-dataバッファパッチやDTS設定で設定済のはずだが、
    // 実際にレジスタを読み返して確認する。
    uint16_t cfg = 0;
    ret = iqs915x_read_reg16(dev, IQS915X_CONFIG_SETTINGS, &cfg);
    if (ret < 0)
    {
      LOG_ERR("Failed to read CONFIG_SETTINGS: %d", ret);
      return;
    }
    if ((cfg & (IQS915X_EVENT_MODE | IQS915X_MANUAL_CONTROL)) ==
        (IQS915X_EVENT_MODE | IQS915X_MANUAL_CONTROL))
    {
      LOG_DBG("Init: Event Mode + Manual Control confirmed (CONFIG_SETTINGS=0x%04x)",
              cfg);
    }
    else
    {
      // 必須ビットが欠けている場合は警告を出して強制設定する
      LOG_WRN("Init: CONFIG_SETTINGS missing required bits (0x%04x). "
              "Forcing Event Mode + Manual Control.",
              cfg);
      ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS,
                                cfg | IQS915X_EVENT_MODE |
                                    IQS915X_MANUAL_CONTROL);
      if (ret < 0)
      {
        LOG_ERR("Failed to force Event Mode + Manual Control: %d", ret);
        return;
      }
    }
    data->init_step = INIT_VERIFY_RESET;
    break;
  }

  case INIT_VERIFY_RESET:
  {
    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0)
    {
      LOG_ERR("Failed to verify reset: %d", ret);
      return;
    }
    LOG_DBG("Init: Verify reset flags: 0x%04x", stream.info_flags);
    // 初期化が完了していればこれ以上のACK_RESETは不要
    data->init_step = INIT_COMPLETE;
    data->initialized = true;
    data->work_state = WORK_READ_DATA;
    data->last_info_flags = stream.info_flags;
    LOG_INF("IQS915x initialization complete");
    break;
  }

  case INIT_FINAL_ACK_RESET:
  {
    // 古いコードですが安全のため残します
    data->init_step = INIT_WAIT_REATI;
    break;
  }

  case INIT_WAIT_REATI:
  {
    data->wait_count++;

    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0)
    {
      LOG_ERR("Failed to read during Re-ATI wait: %d", ret);
      return; // 次のRDYでリトライ
    }

    if (stream.info_flags == 0xEEEE)
    {
      // ICがまだビジー状態、次のRDYで再試行
      LOG_DBG("Init: IC busy (0xEEEE) during Re-ATI wait, cycle=%d",
              data->wait_count);
      break;
    }

    // REATI_OCCURRED (bit4) フラグでRe-ATI完了を検出する
    // このフラグはRe-ATIが実行されたRDYサイクルで1回だけセットされる
    if (stream.info_flags & IQS915X_REATI_OCCURRED)
    {
      LOG_INF("Init: Re-ATI occurred (flags=0x%04x) after %d cycles",
              stream.info_flags, data->wait_count);
      // Re-ATI完了後はDTS設定の書き込みへ進む
      data->init_step = INIT_SINGLE_FINGER_GESTURES;
      break;
    }

    if (data->wait_count > 60)
    {
      // タイムアウト: Re-ATIが検出できなかったが次のステップへ進む
      LOG_WRN("Init: Re-ATI timeout after %d cycles (flags=0x%04x), proceeding "
              "anyway",
              data->wait_count, stream.info_flags);
      data->init_step = INIT_SINGLE_FINGER_GESTURES;
      break;
    }

    // Re-ATIはまだ発生していない、次のRDYで再確認
    LOG_DBG("Init: Waiting for Re-ATI (flags=0x%04x, cycle=%d/60)",
            stream.info_flags, data->wait_count);
    break;
  }

  case INIT_COMPLETE:
    break;
  }
}

/* ============================================================
 * メインスレッド
 *
 * RDY割り込みでセマフォが解放され、ストリーミングデータをraw読み取りする。
 * i2c_write_readを用いてレジスタアドレスから16バイトを一括読み取りする。
 * ============================================================ */
static void iqs915x_thread_main(void *p1, void *p2, void *p3)
{
  struct iqs915x_data *data = p1;
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  int ret;

  while (true)
  {
    if (!data->initialized)
    {
      // SHOW_RESETフラグが立っている期間、IQSは自律的にRDYをトグルし続ける仕様のため
      // マスター側からForce Comms（RDY High時にI2C
      // STARTを発行）を行う必要はない。
      // RDY割り込みをひたすら待ち、割り込み駆動で初期化ステップを進める。
      //
      // ただし割り込みのエッジ取りこぼし対策として：
      // - すでにRDYがLowになっている場合はセマフォをgiveしてすぐ進む
      // - 長めのタイムアウトで完全停止を防ぐ（ICが応答しない異常時の安全策）
      if (gpio_pin_get_dt(&config->rdy_gpio) > 0)
      {
        k_sem_give(&data->rdy_sem);
      }
      k_sem_take(&data->rdy_sem, K_MSEC(2000));
      iqs915x_init_step_handler(dev);
      continue;
    }

    // ===== Active/LP2 pending処理 =====
    // iqs915x_set_enabled()から設定されたpendingフラグを処理する

    if (data->active_pending)
    {
      ret = iqs915x_write_power_mode(dev, IQS915X_MODE_ACTIVE);
      if (ret == 0)
      {
        data->active_pending = false;
        LOG_INF("Trackpad entered Active mode");
      }
      else
      {
        LOG_ERR("Failed to enter Active mode, will retry");
        // リトライのためにセマフォをgiveして即座に再試行
        k_sem_give(&data->rdy_sem);
      }
      continue;
    }

    if (data->lp2_pending)
    {
      ret = iqs915x_write_power_mode(dev, IQS915X_MODE_LP2);
      if (ret == 0)
      {
        data->lp2_pending = false;
        LOG_INF("Trackpad entered LP2 mode");
      }
      else
      {
        LOG_ERR("Failed to enter LP2 mode: %d, will retry", ret);
        k_sem_give(&data->rdy_sem);
      }
      continue;
    }

    // ===== トラックパッド無効時はRDYを待つだけで何もしない =====
    if (!data->enabled)
    {
      // Suspend中でもRDYが来る可能性（SHOW_RESETなど）があるため、
      // セマフォをタイムアウト付きで待ち、起き上がったら再度ループ先頭で
      // active_pending/lp2_pendingをチェックする
      k_sem_take(&data->rdy_sem, K_MSEC(500));
      continue;
    }

    // 初期化完了後の通常モードはポーリングなしでRDY割り込みを待機
    k_sem_take(&data->rdy_sem, K_FOREVER);

    // ストリーミングデータをraw読み取り
    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0)
    {
      LOG_ERR("Failed to read stream: %d", ret);
      continue;
    }

    // info_flagsに立っているビットを個別にDBGログ出力する（連続RDY原因調査用）
    if (stream.info_flags != 0 && stream.info_flags != 0xEEEE)
    {
      // Charging Mode (bit2-0) のデコード
      static const char *const mode_names[] = {"ACTIVE", "IDLE_TOUCH", "IDLE",
                                               "LP1", "LP2"};
      uint8_t mode = stream.info_flags & IQS915X_CHARGING_MODE_MASK;
      const char *mode_str = (mode < 5) ? mode_names[mode] : "UNKNOWN";

      LOG_DBG(
          "info_flags=0x%04x trackpad_flags=0x%04x gesture_sf=0x%04x "
          "gesture_tf=0x%04x rel=(%d,%d) abs=(%u,%u) mode=%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
          stream.info_flags, stream.trackpad_flags, stream.gesture_sf,
          stream.gesture_tf, stream.rel_x, stream.rel_y, stream.abs_x,
          stream.abs_y, mode_str,
          (stream.info_flags & IQS915X_ATI_ERROR) ? " ATI_ERROR" : "",
          (stream.info_flags & IQS915X_REATI_OCCURRED) ? " REATI_OCCURRED" : "",
          (stream.info_flags & IQS915X_ALP_ATI_ERROR) ? " ALP_ATI_ERROR" : "",
          (stream.info_flags & IQS915X_ALP_REATI_OCCURRED) ? " ALP_REATI_OCC"
                                                           : "",
          (stream.info_flags & IQS915X_SHOW_RESET) ? " SHOW_RESET" : "",
          (stream.info_flags & IQS915X_ALP_PROX_STATUS) ? " ALP_PROX_STS" : "",
          (stream.info_flags & IQS915X_GLOBAL_TP_TOUCH) ? " GLOBAL_TP_TOUCH"
                                                        : "",
          (stream.info_flags & IQS915X_SWITCH_PRESSED) ? " SWITCH_PRESSED" : "",
          (stream.info_flags & IQS915X_GLOBAL_SNAP) ? " GLOBAL_SNAP" : "",
          (stream.info_flags & IQS915X_ALP_PROX_TOGGLED) ? " ALP_PROX_TOG" : "",
          (stream.info_flags & IQS915X_TP_TOUCH_TOGGLED) ? " TP_TOUCH_TOG"
                                                         : "",
          (stream.info_flags & IQS915X_SWITCH_TOGGLED) ? " SWITCH_TOG" : "",
          (stream.info_flags & IQS915X_SNAP_TOGGLED) ? " SNAP_TOG" : "");
    }

    // IQS915xのランタイムリセット検出
    // INIT_WAIT_REATIステップでSHOW_RESETクリアを確認済みなので、
    // 通常モードでSHOW_RESETが立っている場合は真のランタイムリセット。
    if (stream.info_flags & IQS915X_SHOW_RESET)
    {
      LOG_WRN("IQS915x runtime reset detected (flags=0x%04x), "
              "re-initializing...",
              stream.info_flags);
      data->initialized = false;
      data->init_step = INIT_CHECK_SHOW_RESET;
      data->init_data_offset = 0;
      // ドラッグ中だった場合はZMKへボタンリリースを確実に通知する
      if (data->active_hold)
      {
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      }
      data->active_hold = false;
      data->is_touching = false;
      data->last_touch_up_time = 0;
      data->last_touch_down_time = 0;
      iqs915x_reset_absolute_tracking(data);
      iqs915x_reset_runtime_gesture_state(data);
      data->buttons_pressed = 0;
      // 慣性スクロールもキャンセル
      iqs915x_reset_scroll_inertia(data);
      data->gesture_pointer_suppress_ticks = 0;
      continue;
    }

    // =========================================================
    // ドラッグ解除チェック: has_tp_event に依存せず毎フレーム実行
    // Global TP Touchの立下がりを使って確実にドラッグを解除する
    // =========================================================
    bool is_touching_now =
        (stream.info_flags & IQS915X_GLOBAL_TP_TOUCH) != 0;
    bool drag_just_released = false;

    if (config->press_and_hold && data->active_hold && !is_touching_now)
    {
      drag_just_released = true;
      data->active_hold = false;
      data->last_touch_up_time = 0;
      input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      data->buttons_pressed &= ~BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
    }

    // Global TP Touchの現在値と前回値からtrackpad-wideのtouch遷移を合成する
    bool was_touching = data->is_touching;
    bool touch_down = is_touching_now && !was_touching;
    bool touch_up = !is_touching_now && was_touching;
    bool touch_state_changed = touch_down || touch_up;
    bool tp_movement = (stream.trackpad_flags & IQS915X_TP_MOVEMENT) != 0;
    bool gesture_active = stream.gesture_sf != 0 || stream.gesture_tf != 0;
    uint8_t num_fingers = stream.trackpad_flags & IQS915X_NUM_FINGERS_MASK;
    data->is_touching = is_touching_now;
    iqs915x_update_finger_state(config, data, &stream);

    bool has_tp_event = is_touching_now || touch_state_changed;
    if (gesture_active || tp_movement)
    {
      has_tp_event = true;
    }

    if (has_tp_event)
    {
      bool suppress_pointer_tail =
          !gesture_active && data->gesture_pointer_suppress_ticks > 0;
      bool suppress_pointer = gesture_active || suppress_pointer_tail;
      bool single_finger_pointer = num_fingers == 1;
      bool allow_pointer_report = !suppress_pointer && single_finger_pointer;
      bool scroll = (stream.gesture_tf & IQS915X_SCROLL) != 0;

      if (gesture_active)
      {
        data->gesture_pointer_suppress_ticks =
            GESTURE_POINTER_SUPPRESS_TAIL_TICKS;
      }
      else if (suppress_pointer_tail)
      {
        data->gesture_pointer_suppress_ticks--;
      }

      if (touch_up)
      {
        data->gesture_pointer_suppress_ticks = 0;
      }

      // 診断: ジェスチャーフラグが非ゼロのときにログ出力
      if (gesture_active)
      {
        LOG_DBG(
            "gesture: sf=0x%04x tf=0x%04x gx=%d gy=%d rx=%d ry=%d tp=0x%04x",
            stream.gesture_sf, stream.gesture_tf, (int16_t)stream.gesture_x,
            (int16_t)stream.gesture_y, stream.rel_x, stream.rel_y,
            stream.trackpad_flags);
      }

      int64_t now_ms = k_uptime_get();

      if (!scroll)
      {
        // 慣性スクロール: スクロール中だったのに今回スクロールでなくなった場合
        if (data->scroll_inertia_state.elapsed_ms > 0)
        {
          int32_t seed_x_fp = 0;
          int32_t seed_y_fp = 0;

          if (iqs915x_calc_scroll_inertia_seed(config, data, now_ms, &seed_x_fp,
                                               &seed_y_fp))
          {
            data->scroll_inertia_state.active = true;
            data->scroll_inertia_state.velocity_x_fp = seed_x_fp;
            data->scroll_inertia_state.velocity_y_fp = seed_y_fp;
            data->scroll_inertia_state.accum_x_fp = 0;
            data->scroll_inertia_state.accum_y_fp = 0;
            data->scroll_inertia_state.last_ms = now_ms;
            k_work_schedule(&data->scroll_inertia_work,
                            K_MSEC(config->scroll_inertia.interval_ms));
            LOG_DBG("Scroll inertia started: vx_fp=%d vy_fp=%d", seed_x_fp,
                    seed_y_fp);
          }
        }
        // スクロール終了: アキュムレータとサンプルをリセット
        if (!data->scroll_inertia_state.active)
        {
          data->scroll_x_acc = 0;
          data->scroll_y_acc = 0;
        }
        iqs915x_reset_motion_history(&data->scroll_motion_history);
        data->scroll_inertia_state.elapsed_ms = 0;
      }

      // タップジェスチャー判定
      uint16_t button_code = 0;
      bool single_tap_pressed = false;
      bool two_finger_tap_pressed = false;
      if (stream.gesture_sf & IQS915X_SINGLE_TAP)
      {
        single_tap_pressed = true;
        button_code = INPUT_BTN_0;
      }
      else if (stream.gesture_tf & IQS915X_TWO_FINGER_TAP)
      {
        two_finger_tap_pressed = true;
        button_code = INPUT_BTN_1;
      }

      // ソフトウェアによる Tap-and-Drag (Tap-and-Hold) の状態遷移 (高速応答用 生タッチ追跡版)
      // 注: ドラッグ解除は上で has_tp_event の外で処理済み
      bool hold_became_active = false;

      if (config->press_and_hold && !data->active_hold)
      {
        // ----- 生のTouch Down / Up イベントによる高速判定 -----
        if (touch_down)
        {
          // Touch Down (指が触れた瞬間)
          data->last_touch_down_time = now_ms;

          // 前回のタッチリリース(Touch Up)から300ms以内に再び触れられたらドラッグ開始！
          if (data->last_touch_up_time > 0 && (now_ms - data->last_touch_up_time) < 300)
          {
            hold_became_active = true;
            data->active_hold = true;
            data->last_touch_up_time = 0; // 消費したため窓を閉じる
          }
        }
        else if (touch_up)
        {
          // Touch Up (指が離れた瞬間)
          // 今回のタッチ期間(Down -> Up)が極端に短いか（タップとみなせるか）確認 (今回は200ms以内と判定)
          if ((now_ms - data->last_touch_down_time) < 200)
          {
            // 短いタップとして認定し、ここから300msの窓を開始
            data->last_touch_up_time = now_ms;
          }
          else
          {
            // 長く触っていた場合はタップではないので窓を開かない
            data->last_touch_up_time = 0;
          }
        }

        // ハードウェアのシングルタップフラグからのフォールバック対応
        if (single_tap_pressed)
        {
          data->last_touch_up_time = now_ms;
        }

        // 時間窓の有効期限切れを定期チェック
        if (data->last_touch_up_time > 0 && now_ms > (data->last_touch_up_time + 300))
        {
          data->last_touch_up_time = 0;
        }
      }
      else if (!config->press_and_hold)
      {
        data->last_touch_up_time = 0;
      }

      // 移動とジェスチャーの処理
      // NOTE: 状態クリアとボタンリリースが確実に競合しないようにする
      if (hold_became_active)
      {
        k_work_cancel_delayable(&data->button_release_work);
        input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
        data->buttons_pressed |= BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
      }
      else if (drag_just_released)
      {
        // ドラッグ解除はhas_tp_eventの外で処理済み（ここはスキップ）
      }

      // hold_released のフレームでもシングルタップ/ダブルタップは処理可能にする（干渉を防ぐため else if にしない）
      if (!hold_became_active && (single_tap_pressed || two_finger_tap_pressed))
      {
        k_work_cancel_delayable(&data->button_release_work);
        input_report_key(dev, button_code, 1, true, K_FOREVER);
        data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);
        k_work_schedule(&data->button_release_work, K_MSEC(100));
      }

      if (scroll)
      {
        // 慣性スクロール中に通常スクロールが再開された場合は慣性を打ち切る
        iqs915x_cancel_scroll_inertia(data);

        // スクロール: GESTURE_X/Yをスクロールデルタとして使用
        // (REL_X/REL_Yはスクロール中はゼロ)
        int16_t gx = (int16_t)stream.gesture_x;
        int16_t gy = (int16_t)stream.gesture_y;
        int16_t scroll_div = config->scroll_divisor;
        if (gx != 0)
        {
          data->scroll_x_acc += gx;
          if (abs(data->scroll_x_acc) >= scroll_div)
          {
            input_report_rel(dev, INPUT_REL_HWHEEL,
                             data->scroll_x_acc / scroll_div, true, K_FOREVER);
            data->scroll_x_acc %= scroll_div;
          }
        }
        if (gy != 0)
        {
          data->scroll_y_acc += gy;
          if (abs(data->scroll_y_acc) >= scroll_div)
          {
            input_report_rel(dev, INPUT_REL_WHEEL,
                             data->scroll_y_acc / scroll_div, true, K_FOREVER);
            data->scroll_y_acc %= scroll_div;
          }
        }

        // 慣性スクロール用: 方向補正済みのデルタを時刻付きでサンプリング
        iqs915x_record_scroll_sample(data, now_ms, gx, gy);
        data->scroll_inertia_state.elapsed_ms++;
      }
      else if (config->report_absolute)
      {
        if (touch_up)
        {
          iqs915x_reset_absolute_tracking(data);
        }
        else if (!allow_pointer_report)
        {
          if (!suppress_pointer && (touch_down || tp_movement))
          {
            iqs915x_cancel_scroll_inertia(data);
          }

          if (!single_finger_pointer)
          {
            iqs915x_reset_absolute_tracking(data);
          }

          if (touch_down || tp_movement)
          {
            LOG_DBG(
                "tp_absolute suppressed: sf=0x%04x tf=0x%04x fingers=%u x=%u y=%u",
                stream.gesture_sf, stream.gesture_tf, num_fingers,
                stream.abs_x, stream.abs_y);
          }
        }
        else
        {
          // 通常のポインタ操作が再開したら慣性スクロールを止める
          iqs915x_cancel_scroll_inertia(data);

          if (touch_down || tp_movement)
          {
            if (!data->last_abs_valid || stream.abs_x != data->last_abs_x ||
                stream.abs_y != data->last_abs_y)
            {
              LOG_DBG("tp_absolute: x=%u, y=%u", stream.abs_x, stream.abs_y);
              input_report_abs(dev, INPUT_ABS_X, stream.abs_x, false,
                               K_FOREVER);
              input_report_abs(dev, INPUT_ABS_Y, stream.abs_y, true,
                               K_FOREVER);
              data->last_abs_x = stream.abs_x;
              data->last_abs_y = stream.abs_y;
              data->last_abs_valid = true;
            }
          }
        }
      }
      else if (tp_movement)
      {
        if (!allow_pointer_report)
        {
          if (!suppress_pointer)
          {
            iqs915x_cancel_scroll_inertia(data);
          }

          LOG_DBG(
              "tp_movement suppressed: sf=0x%04x tf=0x%04x fingers=%u rel_x=%d rel_y=%d",
              stream.gesture_sf, stream.gesture_tf, num_fingers,
              stream.rel_x, stream.rel_y);
        }
        else
        {
          // カーソル移動中は慣性スクロールを打ち切る
          iqs915x_cancel_scroll_inertia(data);

          if (stream.rel_x != 0 || stream.rel_y != 0)
          {
            LOG_DBG("tp_movement: rel_x=%d, rel_y=%d", stream.rel_x,
                    stream.rel_y);
            input_report_rel(dev, INPUT_REL_X, stream.rel_x, false,
                             K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, stream.rel_y, true,
                             K_FOREVER);
          }
        }
      }
    }
  }
}

/* ============================================================
 * RDY GPIO割り込みハンドラ
 * ============================================================ */
static void iqs915x_rdy_handler(const struct device *port,
                                struct gpio_callback *cb,
                                gpio_port_pins_t pins)
{
  struct iqs915x_data *data = CONTAINER_OF(cb, struct iqs915x_data, rdy_cb);

  LOG_DBG("rdy_handler called");
  k_sem_give(&data->rdy_sem);
}

/* ============================================================
 * デバイス初期化関数
 * ============================================================ */
static int iqs915x_init(const struct device *dev)
{
  const struct iqs915x_config *config = dev->config;
  struct iqs915x_data *data = dev->data;
  int ret;

  if (!i2c_is_ready_dt(&config->i2c))
  {
    LOG_ERR("I2C device not ready");
    return -ENODEV;
  }

  data->dev = dev;
  data->init_step = INIT_CHECK_SHOW_RESET;
  data->work_state = WORK_READ_DATA;
  data->initialized = false;
  // disabled-by-defaultの場合は初期化完了後にLP2へ移行し、そうでなければ有効
  data->enabled = !config->disabled_by_default;
  data->lp2_pending = config->disabled_by_default;
  data->active_pending = false;
  iqs915x_reset_absolute_tracking(data);
  iqs915x_reset_runtime_gesture_state(data);

  k_sem_init(&data->rdy_sem, 0, 1);
  k_work_init_delayable(&data->button_release_work,
                        iqs915x_button_release_work_handler);
  k_work_init_delayable(&data->scroll_inertia_work,
                        iqs915x_scroll_inertia_work_handler);

  // 専用スレッドの起動 (優先度: 高め K_PRIO_COOP(2))
  k_thread_create(&data->thread, data->thread_stack,
                  K_KERNEL_STACK_SIZEOF(data->thread_stack),
                  iqs915x_thread_main, data, NULL, NULL, K_PRIO_COOP(2), 0,
                  K_NO_WAIT);
  k_thread_name_set(&data->thread, "iqs915x");

  // リセットGPIOの設定（オプショナル）
  if (config->reset_gpio.port)
  {
    if (!gpio_is_ready_dt(&config->reset_gpio))
    {
      LOG_ERR("Reset GPIO not ready");
      return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
      LOG_ERR("Failed to configure reset GPIO: %d", ret);
      return ret;
    }

    gpio_pin_set_dt(&config->reset_gpio, 1);
    k_msleep(1);
    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_msleep(10);
  }

  // RDY GPIOの設定
  if (!gpio_is_ready_dt(&config->rdy_gpio))
  {
    LOG_ERR("RDY GPIO not ready");
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
  if (ret < 0)
  {
    LOG_ERR("Failed to configure RDY GPIO: %d", ret);
    return ret;
  }

  gpio_init_callback(&data->rdy_cb, iqs915x_rdy_handler,
                     BIT(config->rdy_gpio.pin));
  ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
  if (ret < 0)
  {
    LOG_ERR("Failed to add RDY callback: %d", ret);
    return ret;
  }

  ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
  if (ret < 0)
  {
    LOG_ERR("Failed to configure RDY interrupt: %d", ret);
    return ret;
  }

  LOG_INF("IQS915x driver loaded, waiting for first RDY...");

  return 0;
}

/* ============================================================
 * デバイスインスタンスマクロ
 * ============================================================ */
#define IQS915X_INIT(n)                                                               \
  static const uint8_t iqs915x_init_data_##n[] =                                      \
      DT_INST_PROP_OR(n, azoteq_init_data, {0});                                      \
  static struct iqs915x_data iqs915x_data_##n;                                        \
  static const struct iqs915x_config iqs915x_config_##n = {                           \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                                 \
      .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                \
      .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                    \
      .init_data = iqs915x_init_data_##n,                                             \
      .init_data_len =                                                                \
          COND_CODE_1(DT_INST_NODE_HAS_PROP(n, azoteq_init_data),                     \
                      (DT_INST_PROP_LEN(n, azoteq_init_data)), (0)),                  \
      .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                              \
      .press_and_hold = DT_INST_PROP(n, press_and_hold),                              \
      .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                              \
      .scroll = DT_INST_PROP(n, scroll),                                              \
      .scroll_divisor = DT_INST_PROP(n, scroll_divisor),                              \
      .scroll_inertia = {                                                             \
          .enabled = DT_INST_PROP(n, scroll_inertia),                                 \
          .interval_ms = DT_INST_PROP_OR(n, scroll_inertia_interval_ms, 15),          \
          .decay_x1000 = DT_INST_PROP_OR(n, scroll_inertia_decay, 850),               \
          .recent_window_ms =                                                         \
              DT_INST_PROP_OR(n, scroll_inertia_recent_window_ms, 60),                \
          .stale_gap_ms =                                                             \
              DT_INST_PROP_OR(n, scroll_inertia_stale_gap_ms, 35),                    \
          .min_samples =                                                              \
              DT_INST_PROP_OR(n, scroll_inertia_min_samples, 1),                      \
          .min_avg_speed =                                                            \
              DT_INST_PROP_OR(n, scroll_inertia_min_avg_speed, 4),                    \
      },                                                                              \
      .pinch_inertia = {                                                              \
          .enabled = DT_INST_PROP(n, pinch_inertia),                                  \
          .interval_ms = DT_INST_PROP_OR(n, pinch_inertia_interval_ms, 10),           \
          .decay_x1000 = DT_INST_PROP_OR(n, pinch_inertia_decay, 980),                \
          .recent_window_ms = DT_INST_PROP_OR(n, pinch_inertia_recent_window_ms, 60), \
          .stale_gap_ms = DT_INST_PROP_OR(n, pinch_inertia_stale_gap_ms, 35),         \
          .min_samples = DT_INST_PROP_OR(n, pinch_inertia_min_samples, 1),            \
          .min_avg_speed = DT_INST_PROP_OR(n, pinch_inertia_min_avg_speed, 4),        \
      },                                                                              \
      .finger_debounce_frames = DT_INST_PROP_OR(n, finger_debounce_frames, 2),        \
      .tap_time = DT_INST_PROP_OR(n, tap_time, 0),                                    \
      .report_rate_ms = DT_INST_PROP_OR(n, report_rate_ms, 0),                        \
      .report_absolute = DT_INST_PROP(n, report_absolute),                            \
      .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),            \
      .switch_xy = DT_INST_PROP(n, switch_xy),                                        \
      .flip_x = DT_INST_PROP(n, flip_x),                                              \
      .flip_y = DT_INST_PROP(n, flip_y),                                              \
      .disabled_by_default = DT_INST_PROP(n, disabled_by_default),                    \
  };                                                                                  \
  DEVICE_DT_INST_DEFINE(n, iqs915x_init, NULL, &iqs915x_data_##n,                     \
                        &iqs915x_config_##n, POST_KERNEL,                             \
                        CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS915X_INIT)

/* ============================================================
 * 公開API: power mode制御
 * ============================================================ */

int iqs915x_set_enabled(const struct device *dev, bool enabled)
{
  struct iqs915x_data *data = dev->data;

  if (data->enabled == enabled)
  {
    return 0; // 既に同じ状態
  }

  if (!enabled)
  {
    // ===== 無効化: 即座にイベント破棄を開始し、LP2へ遷移 =====
    data->enabled = false;

    // 進行中の操作を安全に解除
    // ドラッグ中の場合はボタンリリースを送信
    if (data->active_hold)
    {
      input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      data->active_hold = false;
    }

    // 押下中のボタンをすべてリリース
    for (int i = 0; i < 3; i++)
    {
      if (data->buttons_pressed & BIT(i))
      {
        input_report_key(dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
      }
    }
    data->buttons_pressed = 0;
    k_work_cancel_delayable(&data->button_release_work);

    // 慣性スクロールをキャンセル
    iqs915x_reset_scroll_inertia(data);
    data->gesture_pointer_suppress_ticks = 0;

    // タッチ状態をリセット
    data->is_touching = false;
    data->last_touch_down_time = 0;
    data->last_touch_up_time = 0;
    iqs915x_reset_absolute_tracking(data);
    iqs915x_reset_runtime_gesture_state(data);
    data->scroll_x_acc = 0;
    data->scroll_y_acc = 0;

    // IQS915xへのLP2遷移を予約
    data->lp2_pending = true;
    data->active_pending = false;

    // メインスレッドを起こしてlp2_pendingを処理させる
    k_sem_give(&data->rdy_sem);

    LOG_INF("Trackpad disabled (LP2 pending)");
  }
  else
  {
    // ===== 有効化: Active modeへの復帰開始 =====
    data->enabled = true;
    data->gesture_pointer_suppress_ticks = 0;

    // IQS915xのActive mode復帰を予約
    data->active_pending = true;
    data->lp2_pending = false;

    // メインスレッドを起こしてactive_pendingを処理させる
    k_sem_give(&data->rdy_sem);

    LOG_INF("Trackpad enabled (Active pending)");
  }

  return 0;
}

bool iqs915x_get_enabled(const struct device *dev)
{
  struct iqs915x_data *data = dev->data;
  return data->enabled;
}
