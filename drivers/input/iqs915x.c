/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * Azoteq IQS9150/IQS9151 トラックパッドドライバ
 *
 * IQS915xのI2C特性:
 *   - リトルエンディアンのバイトオーダー
 *   - デフォルトではI2C STOPで通信ウィンドウが閉じる
 *   - ストリーミング読み取りはREL_Xのレジスタアドレス指定後に
 *     i2c_write_readで連続読み取りする
 *   - ストリーミング出力はREL_X (0x1014) から開始、
 *     本ドライバではFINGER4_X/Yまで含めて44バイトを読む
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
#include <zephyr/sys/util.h>

#include <dt-bindings/input/iqs915x_gestures.h>
#include <iqs915x.h>
#include "iqs915x_init_data_bretagne_array.h"
#include "iqs915x_regs.h"

LOG_MODULE_REGISTER(iqs915x, CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL);

BUILD_ASSERT(ARRAY_SIZE(iqs915x_init_data_bretagne) == IQS915X_INIT_DATA_TOTAL_SIZE,
             "Built-in init-data size must match IQS915X_INIT_DATA_TOTAL_SIZE");

#define GESTURE_POINTER_SUPPRESS_TAIL_TICKS 1

#define IQS915X_DEFAULT_SWIPE_THRESHOLD_FALLBACK 32
#define IQS915X_INIT_CHUNK_WRITE_MAX_RETRIES 3
#define IQS915X_INIT_MAX_RESTARTS 3
#define IQS915X_INIT_SHOW_RESET_CLEAR_MAX_WAIT 10
#define IQS915X_INIT_EVENT_MODE_MAX_RETRIES 3
#define IQS915X_INIT_REATI_MAX_WAIT 60
#define IQS915X_EVENT_MODE_RELATCH_MAX_RETRIES 3
#define IQS915X_BUTTON_TAP_RELEASE_MS 100
#define IQS915X_TAP_TOUCH_TIME_FALLBACK_MS 200
#define IQS915X_TAP_AIR_TIME_FALLBACK_MS 150
#define IQS915X_TAP_DISTANCE_FALLBACK 100U

static void iqs915x_cancel_scroll_inertia(struct iqs915x_data *data);
static void iqs915x_restart_initialization(const struct device *dev,
                                           const char *reason);

static void iqs915x_reset_event_mode_relatch_state(struct iqs915x_data *data)
{
  data->event_mode_relatch_retry_count = 0;
}

static uint16_t iqs915x_apply_config_settings_policy(uint16_t cfg)
{
  cfg |= IQS915X_EVENT_MODE | IQS915X_MANUAL_CONTROL | IQS915X_TP_EVENT;
  cfg &= ~(IQS915X_GESTURE_EVENT | IQS915X_TP_TOUCH_EVENT);
  return cfg;
}

static uint16_t iqs915x_config_settings_without_event_mode(uint16_t cfg)
{
  cfg = iqs915x_apply_config_settings_policy(cfg);
  return cfg & ~IQS915X_EVENT_MODE;
}

static void iqs915x_schedule_event_mode_relatch(struct iqs915x_data *data,
                                                const char *reason)
{
  data->event_mode_relatch_retry_count = 0;
  k_sem_reset(&data->rdy_sem);
  data->work_state = WORK_RELATCH_EVENT_MODE_DISABLE;
  LOG_INF("Event Mode relatch: scheduled after %s", reason);
}

/* ============================================================
 * I2C通信関数
 *
 * 書き込み: 通常のi2c_write（アドレス+データ）
 * 読み取り: i2c_write_read（スレーブアドレス+Write -> レジスタアドレス ->
 * Repeated START -> スレーブアドレス+Read -> 読み出し -> STOP/NACK）
 *   Zephyrのi2c_write_read_dtでデータシート要件の読み取りシーケンスが完結する。
 * ストリーミング出力: 0x1014〜0x103F の44バイト
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

static int iqs915x_read_block(const struct device *dev, uint16_t reg,
                              uint8_t *data, uint16_t len)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_addr[2] = {reg & 0xFF, reg >> 8};

  if (len > IQS915X_INIT_WRITE_CHUNK_SIZE)
  {
    return -EINVAL;
  }

  return i2c_write_read_dt(&config->i2c, reg_addr, sizeof(reg_addr), data, len);
}

/* ============================================================
 * ストリーミングデータの読み取り
 *
 * IQS9150はRDY信号後にレジスタアドレスなしでI2C読み取りを行うと、
 * REL_X(0x1014)から44バイトのストリーミングデータを返す。
 *
 * メモリレイアウト (44 bytes, リトルエンディアン):
 *   [0-1]:   REL_X (signed int16)
 *   [2-3]:   REL_Y (signed int16)
 *   [4-5]:   GESTURE_X (uint16, read for diagnostics only)
 *   [6-7]:   GESTURE_Y (uint16, read for diagnostics only)
 *   [8-9]:   SINGLE_FINGER_GESTURES (uint16, not used for recognition)
 *   [10-11]: TWO_FINGER_GESTURES (uint16, not used for recognition)
 *   [12-13]: INFO_FLAGS (uint16)
 *   [14-15]: TRACKPAD_FLAGS (uint16)
 *   [16-17]: FINGER1_X (uint16)
 *   [18-19]: FINGER1_Y (uint16)
 *   [20-21]: FINGER1_STRENGTH (uint16)
 *   [22-23]: FINGER1_AREA (uint16)
 *   [24-25]: FINGER2_X (uint16)
 *   [26-27]: FINGER2_Y (uint16)
 *   [32-33]: FINGER3_X (uint16)
 *   [34-35]: FINGER3_Y (uint16)
 *   [40-41]: FINGER4_X (uint16)
 *   [42-43]: FINGER4_Y (uint16)
 * ============================================================ */

// ストリーミングデータ構造体
struct iqs915x_stream_data
{
  uint16_t gesture_x;  // Diagnostics only; driver-side gestures use coordinates.
  uint16_t gesture_y;  // Diagnostics only; driver-side gestures use coordinates.
  uint16_t gesture_sf; // Diagnostics only; not used as a recognition source.
  uint16_t gesture_tf; // Diagnostics only; not used as a recognition source.
  uint16_t info_flags;
  uint16_t trackpad_flags;
  uint16_t abs_x;
  uint16_t abs_y;
  uint16_t finger2_x;
  uint16_t finger2_y;
  uint16_t finger3_x;
  uint16_t finger3_y;
  uint16_t finger4_x;
  uint16_t finger4_y;
};

#define IQS915X_COORD_CAL_X_BLOCKS 6U
#define IQS915X_COORD_CAL_Y_BLOCKS 4U
#define IQS915X_COORD_LUT_STEPS 16U
#define IQS915X_COORD_LUT_Q15_SCALE BIT(15)

/*
 * Half-block correction curves generated from docs/logs/*.txt. Block boundaries
 * and centers are fixed points; the same per-axis curve is mirrored across each
 * block half.
 */
static const uint16_t iqs915x_coord_lut_x_q15[] = {
    0,     3596,  4314,  6980,  9152,  11723, 14282, 15850, 17002,
    19860, 21795, 23755, 25945, 27475, 29719, 30583, 32768};

static const uint16_t iqs915x_coord_lut_y_q15[] = {
    0,     3901,  3933,  6509,  9466,  10858, 13425, 14987, 17489,
    19561, 21659, 23583, 25359, 27318, 29356, 31054, 32768};

BUILD_ASSERT(ARRAY_SIZE(iqs915x_coord_lut_x_q15) == IQS915X_COORD_LUT_STEPS + 1U);
BUILD_ASSERT(ARRAY_SIZE(iqs915x_coord_lut_y_q15) == IQS915X_COORD_LUT_STEPS + 1U);

static uint16_t iqs915x_correct_half_block_distance(uint16_t distance,
                                                    uint16_t half_block,
                                                    const uint16_t *lut,
                                                    size_t lut_len)
{
  uint32_t pos;
  uint32_t idx;
  uint32_t rem;
  uint32_t low;
  uint32_t high;
  uint32_t corrected_q15;

  if (distance == 0 || half_block == 0)
  {
    return 0;
  }

  if (distance >= half_block)
  {
    return half_block;
  }

  if (lut_len < 2)
  {
    return distance;
  }

  pos = (uint32_t)distance * (lut_len - 1U);
  idx = pos / half_block;
  rem = pos % half_block;

  if (idx >= lut_len - 1U)
  {
    return half_block;
  }

  low = lut[idx];
  high = lut[idx + 1U];
  corrected_q15 = low + (((high - low) * rem) / half_block);

  return (uint16_t)(((uint32_t)half_block * corrected_q15 +
                     (IQS915X_COORD_LUT_Q15_SCALE / 2U)) /
                    IQS915X_COORD_LUT_Q15_SCALE);
}

static uint16_t iqs915x_correct_axis_coordinate(uint16_t raw, uint16_t resolution,
                                                uint8_t blocks,
                                                const uint16_t *lut,
                                                size_t lut_len)
{
  uint32_t block;
  uint32_t block_start;
  uint32_t block_end;
  uint32_t block_center;
  uint16_t half_block;
  uint16_t distance;
  uint16_t corrected_distance;

  if (resolution == 0 || blocks == 0)
  {
    return raw;
  }

  if (raw >= resolution)
  {
    raw = resolution;
  }

  block = ((uint32_t)raw * blocks) / resolution;
  if (block >= blocks)
  {
    block = blocks - 1U;
  }

  block_start = ((uint32_t)resolution * block) / blocks;
  block_end = ((uint32_t)resolution * (block + 1U)) / blocks;
  block_center = (block_start + block_end) / 2U;

  if ((uint32_t)raw == block_start || (uint32_t)raw == block_center ||
      (uint32_t)raw == block_end)
  {
    return raw;
  }

  if ((uint32_t)raw < block_center)
  {
    half_block = (uint16_t)(block_center - block_start);
    distance = (uint16_t)(block_center - raw);
    corrected_distance =
        iqs915x_correct_half_block_distance(distance, half_block, lut, lut_len);
    return (uint16_t)(block_center - corrected_distance);
  }

  half_block = (uint16_t)(block_end - block_center);
  distance = (uint16_t)(raw - block_center);
  corrected_distance =
      iqs915x_correct_half_block_distance(distance, half_block, lut, lut_len);
  return (uint16_t)(block_center + corrected_distance);
}

static void iqs915x_correct_stream_coordinates(const struct iqs915x_data *driver_data,
                                               struct iqs915x_stream_data *data)
{
  uint16_t res_x = driver_data->swipe_resolution_x;
  uint16_t res_y = driver_data->swipe_resolution_y;

  data->abs_x = iqs915x_correct_axis_coordinate(
      data->abs_x, res_x, IQS915X_COORD_CAL_X_BLOCKS, iqs915x_coord_lut_x_q15,
      ARRAY_SIZE(iqs915x_coord_lut_x_q15));
  data->finger2_x = iqs915x_correct_axis_coordinate(
      data->finger2_x, res_x, IQS915X_COORD_CAL_X_BLOCKS,
      iqs915x_coord_lut_x_q15, ARRAY_SIZE(iqs915x_coord_lut_x_q15));
  data->finger3_x = iqs915x_correct_axis_coordinate(
      data->finger3_x, res_x, IQS915X_COORD_CAL_X_BLOCKS,
      iqs915x_coord_lut_x_q15, ARRAY_SIZE(iqs915x_coord_lut_x_q15));
  data->finger4_x = iqs915x_correct_axis_coordinate(
      data->finger4_x, res_x, IQS915X_COORD_CAL_X_BLOCKS,
      iqs915x_coord_lut_x_q15, ARRAY_SIZE(iqs915x_coord_lut_x_q15));

  data->abs_y = iqs915x_correct_axis_coordinate(
      data->abs_y, res_y, IQS915X_COORD_CAL_Y_BLOCKS, iqs915x_coord_lut_y_q15,
      ARRAY_SIZE(iqs915x_coord_lut_y_q15));
  data->finger2_y = iqs915x_correct_axis_coordinate(
      data->finger2_y, res_y, IQS915X_COORD_CAL_Y_BLOCKS,
      iqs915x_coord_lut_y_q15, ARRAY_SIZE(iqs915x_coord_lut_y_q15));
  data->finger3_y = iqs915x_correct_axis_coordinate(
      data->finger3_y, res_y, IQS915X_COORD_CAL_Y_BLOCKS,
      iqs915x_coord_lut_y_q15, ARRAY_SIZE(iqs915x_coord_lut_y_q15));
  data->finger4_y = iqs915x_correct_axis_coordinate(
      data->finger4_y, res_y, IQS915X_COORD_CAL_Y_BLOCKS,
      iqs915x_coord_lut_y_q15, ARRAY_SIZE(iqs915x_coord_lut_y_q15));
}

static void iqs915x_log_stream_coordinates(const struct iqs915x_stream_data *data)
{
  uint8_t fingers;

  if (!IS_ENABLED(CONFIG_INPUT_AZOTEQ_IQS915X_COORD_LOG))
  {
    return;
  }

  fingers = data->trackpad_flags & IQS915X_NUM_FINGERS_MASK;
  if (fingers == 0 && (data->info_flags & IQS915X_GLOBAL_TP_TOUCH) == 0)
  {
    return;
  }

  LOG_INF("coord,t=%lld,f=%u,info=0x%04x,flags=0x%04x,"
          "x1=%u,y1=%u,x2=%u,y2=%u,x3=%u,y3=%u,x4=%u,y4=%u",
          (long long)k_uptime_get(), fingers, data->info_flags,
          data->trackpad_flags, data->abs_x, data->abs_y, data->finger2_x,
          data->finger2_y, data->finger3_x, data->finger3_y, data->finger4_x,
          data->finger4_y);
}

// ストリーミングデータを読み取る
static int iqs915x_read_stream(const struct device *dev,
                               struct iqs915x_stream_data *data)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_addr[2] = {IQS915X_REL_X & 0xFF, IQS915X_REL_X >> 8};
  uint8_t buf[44];
  int ret;

  // アドレスを指定してRESTARTで読み取る
  ret = i2c_write_read_dt(&config->i2c, reg_addr, 2, buf, sizeof(buf));
  if (ret < 0)
  {
    return ret;
  }

  // リトルエンディアンでデコード
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
  data->finger3_x = (buf[33] << 8) | buf[32];
  data->finger3_y = (buf[35] << 8) | buf[34];
  data->finger4_x = (buf[41] << 8) | buf[40];
  data->finger4_y = (buf[43] << 8) | buf[42];
  iqs915x_log_stream_coordinates(data);
  if (config->coordinate_correction)
  {
    iqs915x_correct_stream_coordinates(dev->data, data);
  }

  return 0;
}

static void iqs915x_reset_absolute_tracking(struct iqs915x_data *data)
{
  data->last_abs_x = 0;
  data->last_abs_y = 0;
  data->last_abs_valid = false;
}

static uint16_t iqs915x_absolute_discontinuity_threshold(const struct iqs915x_data *data)
{
  uint16_t res_x = data->swipe_resolution_x;
  uint16_t res_y = data->swipe_resolution_y;
  uint16_t min_res;

  if (res_x == 0 && res_y == 0)
  {
    return 1024;
  }

  if (res_x == 0)
  {
    min_res = res_y;
  }
  else if (res_y == 0)
  {
    min_res = res_x;
  }
  else
  {
    min_res = MIN(res_x, res_y);
  }

  return MAX((uint16_t)(min_res / 3U), (uint16_t)512U);
}

static bool iqs915x_absolute_delta_is_discontinuity(
    const struct iqs915x_data *data, int32_t rel_x, int32_t rel_y)
{
  uint16_t threshold = iqs915x_absolute_discontinuity_threshold(data);

  return abs(rel_x) > threshold || abs(rel_y) > threshold;
}

static uint32_t iqs915x_axis_movement(int32_t dx, int32_t dy)
{
  return (uint32_t)MAX(abs(dx), abs(dy));
}

static int16_t iqs915x_clamp_i16(int32_t value)
{
  if (value < -32768)
  {
    return -32768;
  }

  if (value > 32767)
  {
    return 32767;
  }

  return (int16_t)value;
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
  data->swipe_last_centroid_x = 0;
  data->swipe_last_centroid_y = 0;
  data->swipe_centroid_valid = false;
  data->swipe_active_fingers = 0;
  data->swipe_valid_frames = 0;
  data->swipe_triggered = false;
  data->multifinger_swipe_latched = false;
  data->scroll_sequence_active = false;
  data->scroll_blocked_until_low_contact = false;
  data->tap_drag_raw_max_fingers = 0;
  data->tap_drag_raw_gesture_seen = false;
  data->raw_single_tap_reported = false;
  data->raw_two_finger_tap_reported = false;
  data->tap_start_valid = false;
  data->tap_start_x = 0;
  data->tap_start_y = 0;
  data->tap_max_movement = 0;
  data->completed_two_finger_movement = 0;
  data->single_tap_pending = false;
  data->tap_sequence_second_touch = false;
  data->tap_and_hold_start_pending = false;
  data->pending_tap_up_time = 0;
  iqs915x_reset_two_finger_session(data);
}

static void iqs915x_reset_multifinger_swipe_state(struct iqs915x_data *data)
{
  data->swipe_last_centroid_x = 0;
  data->swipe_last_centroid_y = 0;
  data->swipe_centroid_valid = false;
  data->swipe_active_fingers = 0;
  data->swipe_valid_frames = 0;
  data->swipe_triggered = false;
}

static void iqs915x_update_sequence_gates(struct iqs915x_data *data)
{
  struct iqs915x_finger_tracker *tracker = &data->finger_tracker;

  if (tracker->stable_count <= 1)
  {
    data->multifinger_swipe_latched = false;
    data->scroll_blocked_until_low_contact = false;
    iqs915x_reset_multifinger_swipe_state(data);

    if (tracker->stable_count == 0)
    {
      data->scroll_sequence_active = false;
    }
    return;
  }

  if (tracker->sequence_max_count >= 3)
  {
    data->scroll_blocked_until_low_contact = true;
  }
}

static bool iqs915x_get_init_data_reg16(const struct iqs915x_config *config,
                                        uint16_t reg, uint16_t *val)
{
  uint16_t offset;

  if (!config->init_data || config->init_data_len == 0)
  {
    return false;
  }

  if (reg < IQS915X_INIT_DATA_BASE_ADDR ||
      reg + 1 >= (IQS915X_INIT_DATA_BASE_ADDR + IQS915X_INIT_DATA_MAIN_SIZE))
  {
    return false;
  }

  offset = reg - IQS915X_INIT_DATA_BASE_ADDR;
  if (offset + 1 >= config->init_data_len)
  {
    return false;
  }

  *val = ((uint16_t)config->init_data[offset + 1] << 8) |
         (uint16_t)config->init_data[offset];
  return true;
}

static void iqs915x_configure_tap_profile(const struct iqs915x_config *config,
                                          struct iqs915x_data *data)
{
  uint16_t tap_touch_time = 0;
  uint16_t tap_air_time = 0;
  uint16_t tap_distance = 0;

  if (!iqs915x_get_init_data_reg16(config, IQS915X_TAP_TOUCH_TIME,
                                   &tap_touch_time) ||
      tap_touch_time == 0)
  {
    tap_touch_time = IQS915X_TAP_TOUCH_TIME_FALLBACK_MS;
  }

  if (!iqs915x_get_init_data_reg16(config, IQS915X_TAP_WAIT_TIME,
                                   &tap_air_time) ||
      tap_air_time == 0)
  {
    tap_air_time = IQS915X_TAP_AIR_TIME_FALLBACK_MS;
  }

  if (!iqs915x_get_init_data_reg16(config, IQS915X_TAP_DISTANCE,
                                   &tap_distance) ||
      tap_distance == 0)
  {
    tap_distance = IQS915X_TAP_DISTANCE_FALLBACK;
  }

  data->tap_touch_time_ms = tap_touch_time;
  data->tap_air_time_ms = tap_air_time;
  data->tap_distance = tap_distance;

  LOG_INF("Tap profile: touch=%u ms air=%u ms distance=%u",
          data->tap_touch_time_ms, data->tap_air_time_ms, data->tap_distance);
}

static void iqs915x_configure_swipe_thresholds(const struct iqs915x_config *config,
                                               struct iqs915x_data *data)
{
  uint16_t res_x = 0;
  uint16_t res_y = 0;
  uint16_t num = config->swipe_threshold_numerator;
  uint16_t den = config->swipe_threshold_denominator;
  bool has_x = iqs915x_get_init_data_reg16(config, IQS915X_X_RESOLUTION, &res_x);
  bool has_y = iqs915x_get_init_data_reg16(config, IQS915X_Y_RESOLUTION, &res_y);

  if (num == 0)
  {
    num = 1;
  }
  if (den == 0)
  {
    den = 5;
  }

  data->swipe_resolution_x = has_x ? res_x : 0;
  data->swipe_resolution_y = has_y ? res_y : 0;

  if (config->swipe_step > 0)
  {
    data->swipe_threshold_x = config->swipe_step;
    data->swipe_threshold_y = config->swipe_step;
    LOG_INF("Gesture threshold override: %u px", config->swipe_step);
    return;
  }

  if (has_x && has_y)
  {
    uint16_t base_res = MIN(res_x, res_y);
    uint16_t threshold = MAX(1U, ((uint32_t)base_res * num) / den);

    data->swipe_threshold_x = threshold;
    data->swipe_threshold_y = threshold;
    LOG_INF("Gesture threshold from min resolution: %u (res=%ux%u, ratio=%u/%u)",
            threshold, res_x, res_y, num, den);
    return;
  }

  data->swipe_threshold_x = IQS915X_DEFAULT_SWIPE_THRESHOLD_FALLBACK;
  data->swipe_threshold_y = IQS915X_DEFAULT_SWIPE_THRESHOLD_FALLBACK;
  LOG_WRN("Failed to read X/Y resolution from init-data (0x11E6/0x11E8). "
          "Using fallback threshold=%u px",
          IQS915X_DEFAULT_SWIPE_THRESHOLD_FALLBACK);
}

static uint16_t iqs915x_get_multifinger_swipe_gesture(uint8_t fingers, int32_t dx,
                                                      int32_t dy)
{
  bool horizontal = abs(dx) >= abs(dy);

  if (fingers == 3)
  {
    if (horizontal)
    {
      return dx > 0 ? IQS915X_GESTURE_3F_RIGHT : IQS915X_GESTURE_3F_LEFT;
    }
    return dy > 0 ? IQS915X_GESTURE_3F_DOWN : IQS915X_GESTURE_3F_UP;
  }

  if (fingers == 4)
  {
    if (horizontal)
    {
      return dx > 0 ? IQS915X_GESTURE_4F_RIGHT : IQS915X_GESTURE_4F_LEFT;
    }
    return dy > 0 ? IQS915X_GESTURE_4F_DOWN : IQS915X_GESTURE_4F_UP;
  }

  return 0;
}

static void iqs915x_emit_gesture_tap(const struct device *dev, uint16_t gesture_code)
{
  input_report(dev, IQS915X_INPUT_EV_GESTURE, gesture_code, 1, false, K_FOREVER);
  input_report(dev, IQS915X_INPUT_EV_GESTURE, gesture_code, 0, true, K_FOREVER);
}

static void iqs915x_update_finger_state(struct iqs915x_data *data,
                                        const struct iqs915x_stream_data *stream,
                                        bool is_touching_now,
                                        bool touch_down_event,
                                        bool touch_up_event)
{
  struct iqs915x_finger_tracker *tracker = &data->finger_tracker;
  struct iqs915x_two_finger_session *two_finger = &data->two_finger;
  uint8_t reported_count = stream->trackpad_flags & IQS915X_NUM_FINGERS_MASK;
  bool global_tp_touch = (stream->info_flags & IQS915X_GLOBAL_TP_TOUCH) != 0;
  uint8_t raw_count = reported_count;
  uint8_t stable_before = tracker->stable_count;

  tracker->current_count = raw_count;
  tracker->previous_count = stable_before;

  if (touch_down_event)
  {
    // GLOBAL_TP_TOUCHは取りこぼしがあるため、接触境界はNUM_FINGERSだけで判定する。
    tracker->completed_one_tap_path = false;
    tracker->completed_two_tap_path = false;
    tracker->sequence_active = false;
    tracker->sequence_max_count = 0;
    tracker->sequence_seen_one = false;
    tracker->sequence_seen_two = false;
  }

  tracker->stable_count = raw_count;

  if (tracker->stable_count != stable_before)
  {
    LOG_DBG("finger count changed: %u -> %u raw=%u reported=%u "
            "touch_down=%u touch_up=%u global_tp_touch=%u "
            "flags=0x%04x info=0x%04x",
            stable_before, tracker->stable_count, raw_count, reported_count,
            touch_down_event, touch_up_event, global_tp_touch,
            stream->trackpad_flags, stream->info_flags);
  }

  if (stable_before == 0 && tracker->stable_count > 0)
  {
    tracker->sequence_active = true;
    tracker->sequence_max_count = tracker->stable_count;
    tracker->sequence_seen_one = tracker->stable_count == 1;
    tracker->sequence_seen_two = tracker->stable_count == 2;
    tracker->completed_one_tap_path = false;
    tracker->completed_two_tap_path = false;
  }
  else if (tracker->sequence_active && tracker->stable_count > 0)
  {
    if (tracker->stable_count > tracker->sequence_max_count)
    {
      tracker->sequence_max_count = tracker->stable_count;
    }

    if (tracker->stable_count == 1)
    {
      tracker->sequence_seen_one = true;
    }
    else if (tracker->stable_count == 2)
    {
      tracker->sequence_seen_two = true;
    }
  }

  if (stable_before > 0 && tracker->stable_count == 0)
  {
    tracker->completed_one_tap_path =
        tracker->sequence_active && tracker->sequence_seen_one &&
        tracker->sequence_max_count == 1;
    tracker->completed_two_tap_path =
        tracker->sequence_active && tracker->sequence_seen_two &&
        tracker->sequence_max_count == 2;
    if (tracker->completed_two_tap_path && two_finger->active)
    {
      data->completed_two_finger_movement = two_finger->max_centroid_movement;
    }
    tracker->sequence_active = false;
    tracker->sequence_max_count = 0;
    tracker->sequence_seen_one = false;
    tracker->sequence_seen_two = false;
  }

  tracker->tail_suppressed =
      stable_before == 2 && tracker->stable_count == 1;

  if (stable_before >= 2 && tracker->stable_count == 1)
  {
    // 2本以上から1本へ遷移したら、0本接触を確認するまで
    // 単指ポインタを再開しない。
    tracker->awaiting_zero_contact = true;
  }

  if (!is_touching_now)
  {
    // 0本遷移はNUM_FINGERS==0だけで判定する。
    tracker->awaiting_zero_contact = false;
  }

  if (tracker->stable_count != 2 || raw_count < 2)
  {
    if (tracker->stable_count == 1 && data->scroll_sequence_active &&
        two_finger->active && two_finger->mode == IQS915X_2F_MODE_SCROLL)
    {
      two_finger->rebaseline_pending = true;
      two_finger->centroid_dx = 0;
      two_finger->centroid_dy = 0;
      two_finger->distance_delta = 0;
    }
    else
    {
      iqs915x_reset_two_finger_session(data);
    }
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
    two_finger->rebaseline_pending = false;
    two_finger->mode = IQS915X_2F_MODE_NONE;
    two_finger->centroid_last_x = centroid_x;
    two_finger->centroid_last_y = centroid_y;
    two_finger->centroid_start_x = centroid_x;
    two_finger->centroid_start_y = centroid_y;
    two_finger->max_centroid_movement = 0;
    two_finger->distance_last = distance;
    return;
  }

  if (two_finger->rebaseline_pending)
  {
    two_finger->rebaseline_pending = false;
    two_finger->centroid_dx = 0;
    two_finger->centroid_dy = 0;
    two_finger->distance_delta = 0;
    two_finger->centroid_last_x = centroid_x;
    two_finger->centroid_last_y = centroid_y;
    two_finger->distance_last = distance;
    return;
  }

  two_finger->centroid_dx = centroid_x - two_finger->centroid_last_x;
  two_finger->centroid_dy = centroid_y - two_finger->centroid_last_y;
  two_finger->distance_delta = distance - two_finger->distance_last;
  two_finger->max_centroid_movement =
      MAX(two_finger->max_centroid_movement,
          iqs915x_axis_movement(centroid_x - two_finger->centroid_start_x,
                                centroid_y - two_finger->centroid_start_y));
  two_finger->centroid_last_x = centroid_x;
  two_finger->centroid_last_y = centroid_y;
  two_finger->distance_last = distance;
}

static bool iqs915x_handle_multifinger_swipe(const struct iqs915x_config *config,
                                             struct iqs915x_data *data,
                                             const struct iqs915x_stream_data *stream)
{
  uint8_t stable_fingers = data->finger_tracker.stable_count;
  bool enabled = (stable_fingers == 3 && config->three_finger_swipe) ||
                 (stable_fingers == 4 && config->four_finger_swipe);
  int32_t centroid_x;
  int32_t centroid_y;
  int32_t dx;
  int32_t dy;
  bool horizontal;
  int32_t axis_delta;
  int32_t abs_dx;
  int32_t abs_dy;
  int32_t major_delta;
  int32_t minor_delta;
  uint16_t lock_num;
  uint16_t lock_den;
  uint16_t axis_threshold;
  uint16_t gesture_code;

  if (!enabled)
  {
    iqs915x_reset_multifinger_swipe_state(data);
    return false;
  }

  if (data->multifinger_swipe_latched || data->scroll_sequence_active ||
      (stable_fingers == 3 && data->finger_tracker.sequence_max_count >= 4))
  {
    iqs915x_reset_multifinger_swipe_state(data);
    return true;
  }

  centroid_x = (int32_t)stream->abs_x + (int32_t)stream->finger2_x +
               (int32_t)stream->finger3_x;
  centroid_y = (int32_t)stream->abs_y + (int32_t)stream->finger2_y +
               (int32_t)stream->finger3_y;
  if (stable_fingers == 4)
  {
    centroid_x += (int32_t)stream->finger4_x;
    centroid_y += (int32_t)stream->finger4_y;
  }
  centroid_x /= stable_fingers;
  centroid_y /= stable_fingers;

  if (!data->swipe_centroid_valid || data->swipe_active_fingers != stable_fingers)
  {
    data->swipe_last_centroid_x = centroid_x;
    data->swipe_last_centroid_y = centroid_y;
    data->swipe_centroid_valid = true;
    data->swipe_active_fingers = stable_fingers;
    data->swipe_valid_frames = 0;
    data->swipe_triggered = false;
    return true;
  }

  if (data->swipe_triggered)
  {
    return true;
  }

  // スワイプ開始時の重心位置を基準に方向を判定し、
  // 1本以下を経由するまで一度だけgesture eventを発火する。
  dx = centroid_x - data->swipe_last_centroid_x;
  dy = centroid_y - data->swipe_last_centroid_y;

  if (data->swipe_valid_frames < UINT16_MAX)
  {
    data->swipe_valid_frames++;
  }

  if (data->swipe_valid_frames <= config->swipe_direction_settle_frames)
  {
    return true;
  }

  abs_dx = abs(dx);
  abs_dy = abs(dy);
  horizontal = abs_dx >= abs_dy;
  axis_delta = horizontal ? dx : dy;
  major_delta = horizontal ? abs_dx : abs_dy;
  minor_delta = horizontal ? abs_dy : abs_dx;
  axis_threshold = horizontal ? data->swipe_threshold_x : data->swipe_threshold_y;

  if (abs(axis_delta) < axis_threshold)
  {
    return true;
  }

  lock_num = config->swipe_direction_lock_numerator > 0
                 ? config->swipe_direction_lock_numerator
                 : 3;
  lock_den = config->swipe_direction_lock_denominator > 0
                 ? config->swipe_direction_lock_denominator
                 : 2;

  if ((int64_t)major_delta * lock_den < (int64_t)minor_delta * lock_num)
  {
    LOG_DBG("Gesture direction undecided: %uF dx=%d dy=%d lock=%u/%u frames=%u",
            stable_fingers, (int)dx, (int)dy, lock_num, lock_den,
            data->swipe_valid_frames);
    return true;
  }

  gesture_code = iqs915x_get_multifinger_swipe_gesture(stable_fingers, dx, dy);
  if (gesture_code == 0)
  {
    return true;
  }

  iqs915x_cancel_scroll_inertia(data);
  LOG_INF(
      "Gesture emit before: %uF %s gesture=%u dx=%d dy=%d thr_x=%u thr_y=%u "
      "lock=%u/%u frames=%u flags=0x%04x sf=0x%04x tf=0x%04x "
      "latched=%d triggered=%d",
      stable_fingers,
      horizontal ? (dx > 0 ? "RIGHT" : "LEFT") : (dy > 0 ? "DOWN" : "UP"),
      gesture_code, (int)dx, (int)dy, data->swipe_threshold_x,
      data->swipe_threshold_y, lock_num, lock_den, data->swipe_valid_frames,
      stream->trackpad_flags, stream->gesture_sf, stream->gesture_tf,
      data->multifinger_swipe_latched, data->swipe_triggered);
  iqs915x_emit_gesture_tap(data->dev, gesture_code);
  LOG_INF(
      "Gesture emit after: %uF %s gesture=%u dx=%d dy=%d flags=0x%04x sf=0x%04x "
      "tf=0x%04x",
      stable_fingers,
      horizontal ? (dx > 0 ? "RIGHT" : "LEFT") : (dy > 0 ? "DOWN" : "UP"),
      gesture_code, (int)dx, (int)dy, stream->trackpad_flags, stream->gesture_sf,
      stream->gesture_tf);
  data->swipe_triggered = true;
  data->multifinger_swipe_latched = true;
  data->tap_drag_raw_gesture_seen = true;
  LOG_INF("Gesture triggered: %uF %s gesture=%u dx=%d dy=%d thr_x=%u thr_y=%u",
          stable_fingers,
          horizontal ? (dx > 0 ? "RIGHT" : "LEFT") : (dy > 0 ? "DOWN" : "UP"),
          gesture_code, (int)dx, (int)dy, data->swipe_threshold_x,
          data->swipe_threshold_y);

  return true;
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

static void iqs915x_handle_event_mode_relatch_step(const struct device *dev)
{
  struct iqs915x_data *data = dev->data;
  int ret;

  switch (data->work_state)
  {
  case WORK_RELATCH_EVENT_MODE_DISABLE:
  {
    uint16_t cfg =
        iqs915x_config_settings_without_event_mode(data->confirmed_config_settings);

    ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS, cfg);
    if (ret < 0)
    {
      data->event_mode_relatch_retry_count++;
      LOG_ERR("Event Mode relatch: failed to disable Event Mode: %d", ret);
      if (data->event_mode_relatch_retry_count >
          IQS915X_EVENT_MODE_RELATCH_MAX_RETRIES)
      {
        data->initialized = false;
        iqs915x_restart_initialization(dev, "Event Mode relatch disable failed");
      }
      return;
    }

    data->confirmed_config_settings = cfg;
    data->event_mode_relatch_retry_count = 0;
    data->work_state = WORK_RELATCH_EVENT_MODE_ENABLE;
    LOG_INF("Event Mode relatch: disabling (CONFIG_SETTINGS=0x%04x)", cfg);
    break;
  }

  case WORK_RELATCH_EVENT_MODE_ENABLE:
  {
    uint16_t cfg =
        iqs915x_apply_config_settings_policy(data->confirmed_config_settings);

    ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS, cfg);
    if (ret < 0)
    {
      data->event_mode_relatch_retry_count++;
      LOG_ERR("Event Mode relatch: failed to enable Event Mode: %d", ret);
      if (data->event_mode_relatch_retry_count >
          IQS915X_EVENT_MODE_RELATCH_MAX_RETRIES)
      {
        data->initialized = false;
        iqs915x_restart_initialization(dev, "Event Mode relatch enable failed");
      }
      return;
    }

    data->confirmed_config_settings = cfg;
    data->event_mode_relatch_retry_count = 0;
    data->work_state = WORK_READ_DATA;
    LOG_INF("Event Mode relatch: enabling (CONFIG_SETTINGS=0x%04x)", cfg);
    LOG_INF("Event Mode relatch: complete");
    break;
  }

  default:
    break;
  }
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
      // ドラッグ中（active_tap_hold）の場合は左クリック(i=0)の離上をスキップ
      if (i == 0 && data->active_tap_hold)
      {
        continue;
      }
      input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
      data->buttons_pressed &= ~BIT(i);
    }
  }
}

static void iqs915x_report_button_tap(struct iqs915x_data *data,
                                      uint16_t button_code)
{
  k_work_cancel_delayable(&data->button_release_work);
  input_report_key(data->dev, button_code, 1, true, K_FOREVER);
  data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);
  k_work_schedule(&data->button_release_work,
                  K_MSEC(IQS915X_BUTTON_TAP_RELEASE_MS));
}

static void iqs915x_report_button_double_tap(struct iqs915x_data *data,
                                             uint16_t button_code)
{
  uint8_t button_bit = BIT(button_code - INPUT_BTN_0);

  k_work_cancel_delayable(&data->button_release_work);
  if (data->buttons_pressed & button_bit)
  {
    input_report_key(data->dev, button_code, 0, true, K_FOREVER);
    data->buttons_pressed &= ~button_bit;
  }

  input_report_key(data->dev, button_code, 1, true, K_FOREVER);
  input_report_key(data->dev, button_code, 0, true, K_FOREVER);
  input_report_key(data->dev, button_code, 1, true, K_FOREVER);
  data->buttons_pressed |= button_bit;
  k_work_schedule(&data->button_release_work,
                  K_MSEC(IQS915X_BUTTON_TAP_RELEASE_MS));
}

static void iqs915x_start_tap_and_hold_drag(struct iqs915x_data *data,
                                            const char *reason)
{
  if (data->active_tap_hold)
  {
    return;
  }

  if (data->tap_and_hold_release_pending)
  {
    k_work_cancel_delayable(&data->tap_and_hold_release_work);
    data->tap_and_hold_release_pending = false;
  }

  data->active_tap_hold = true;
  data->tap_and_hold_start_pending = false;
  data->single_tap_pending = false;
  data->tap_sequence_second_touch = false;
  input_report_key(data->dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
  data->buttons_pressed |= BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
  LOG_DBG("tap-and-drag started: %s", reason);
}

static void iqs915x_update_single_tap_movement(
    struct iqs915x_data *data, const struct iqs915x_stream_data *stream,
    uint8_t num_fingers)
{
  if (num_fingers != 1 ||
      (stream->trackpad_flags & IQS915X_FINGER1_CONFIDENCE) == 0)
  {
    return;
  }

  if (!data->tap_start_valid)
  {
    data->tap_start_x = stream->abs_x;
    data->tap_start_y = stream->abs_y;
    data->tap_start_valid = true;
    data->tap_max_movement = 0;
    return;
  }

  data->tap_max_movement =
      MAX(data->tap_max_movement,
          iqs915x_axis_movement((int32_t)stream->abs_x - data->tap_start_x,
                                (int32_t)stream->abs_y - data->tap_start_y));

  if (data->tap_max_movement >= data->tap_distance)
  {
    data->tap_drag_raw_gesture_seen = true;
  }
}

#define IQS915X_SCROLL_UNITS_PER_AXIS 512
#define IQS915X_SCROLL_FALLBACK_RESOLUTION 4096
#define IQS915X_SCROLL_CROSS_AXIS_DEADBAND_RATIO 4

static int32_t iqs915x_abs32(int32_t value)
{
  return value < 0 ? -value : value;
}

static void iqs915x_filter_scroll_cross_axis(int16_t *x, int16_t *y)
{
  int32_t abs_x = iqs915x_abs32(*x);
  int32_t abs_y = iqs915x_abs32(*y);

  if (abs_x == 0 || abs_y == 0)
  {
    return;
  }

  if ((abs_x * IQS915X_SCROLL_CROSS_AXIS_DEADBAND_RATIO) < abs_y)
  {
    *x = 0;
  }
  else if ((abs_y * IQS915X_SCROLL_CROSS_AXIS_DEADBAND_RATIO) < abs_x)
  {
    *y = 0;
  }
}

static bool iqs915x_emit_normalized_scroll_axis(struct iqs915x_data *data,
                                                int32_t *accumulator,
                                                uint16_t code, int16_t delta,
                                                uint16_t resolution,
                                                uint16_t divisor)
{
  int32_t output;
  int32_t denom =
      (int32_t)(resolution > 0 ? resolution : IQS915X_SCROLL_FALLBACK_RESOLUTION) *
      (int32_t)(divisor > 0 ? divisor : 1);

  *accumulator += (int32_t)delta * IQS915X_SCROLL_UNITS_PER_AXIS;
  if (iqs915x_abs32(*accumulator) < denom)
  {
    return false;
  }

  output = *accumulator / denom;
  input_report_rel(data->dev, code, output, true, K_FOREVER);
  *accumulator %= denom;
  return output != 0;
}

static bool iqs915x_handle_two_finger_scroll(
    const struct iqs915x_config *config, struct iqs915x_data *data,
    const struct iqs915x_stream_data *stream)
{
  struct iqs915x_two_finger_session *two_finger = &data->two_finger;
  int16_t gx;
  int16_t gy;
  bool emitted = false;

  if (!config->scroll || data->finger_tracker.stable_count != 2 ||
      !two_finger->active || data->scroll_blocked_until_low_contact)
  {
    return false;
  }

  if (two_finger->mode != IQS915X_2F_MODE_SCROLL)
  {
    if (two_finger->max_centroid_movement < data->tap_distance)
    {
      return false;
    }

    two_finger->mode = IQS915X_2F_MODE_SCROLL;
    data->scroll_sequence_active = true;
    data->tap_drag_raw_gesture_seen = true;
    LOG_DBG("scroll started from centroid movement: max=%u threshold=%u",
            two_finger->max_centroid_movement, data->tap_distance);
  }

  gx = iqs915x_clamp_i16(two_finger->centroid_dx);
  gy = iqs915x_clamp_i16(two_finger->centroid_dy);
  iqs915x_filter_scroll_cross_axis(&gx, &gy);

  if (gx == 0 && gy == 0)
  {
    return true;
  }

  if (config->scroll_inertia.enabled)
  {
    if (data->scroll_inertia_state.active &&
        data->scroll_inertia_state.is_inertial)
    {
      iqs915x_cancel_scroll_inertia(data);
    }

    if (gx != 0)
    {
      data->scroll_inertia_state.vx = gx;
      data->scroll_inertia_state.ema_vx =
          (int16_t)((gx + data->scroll_inertia_state.ema_vx) >> 1);
    }

    if (gy != 0)
    {
      data->scroll_inertia_state.vy = gy;
      data->scroll_inertia_state.ema_vy =
          (int16_t)((gy + data->scroll_inertia_state.ema_vy) >> 1);
    }

    if (abs(data->scroll_inertia_state.vx) >=
            config->scroll_inertia.threshold_start ||
        abs(data->scroll_inertia_state.vy) >=
            config->scroll_inertia.threshold_start)
    {
      data->scroll_inertia_state.active = true;
      data->scroll_inertia_state.is_inertial = false;
      k_work_reschedule(&data->scroll_inertia_work,
                        K_MSEC(config->scroll_inertia.trigger_ms));
    }
  }

  if (gx != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->scroll_x_acc, INPUT_REL_HWHEEL, gx,
        data->swipe_resolution_x, config->scroll_divisor);
  }

  if (gy != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->scroll_y_acc, INPUT_REL_WHEEL, gy,
        data->swipe_resolution_y, config->scroll_divisor);
  }

  if (emitted)
  {
    data->scroll_inertia_state.zero_output_ticks = 0;
  }

  LOG_DBG("scroll centroid: dx=%d dy=%d flags=0x%04x", gx, gy,
          stream->trackpad_flags);
  return true;
}

static void iqs915x_single_tap_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, single_tap_work);

  if (!data->single_tap_pending)
  {
    return;
  }

  data->single_tap_pending = false;

  if (data->active_tap_hold)
  {
    return;
  }

  if (!data->is_touching)
  {
    iqs915x_report_button_tap(data, LEFT_BUTTON_CODE);
    LOG_DBG("single tap air time elapsed: click reported");
  }
}

static void iqs915x_tap_and_hold_start_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, tap_and_hold_start_work);

  if (!data->tap_and_hold_start_pending || !data->tap_sequence_second_touch ||
      !data->is_touching)
  {
    data->tap_and_hold_start_pending = false;
    return;
  }

  iqs915x_start_tap_and_hold_drag(data, "second touch held past tap time");
}

static void iqs915x_tap_and_hold_release_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, tap_and_hold_release_work);

  data->tap_and_hold_release_pending = false;

  if (!data->active_tap_hold)
  {
    return;
  }

  if (data->is_touching)
  {
    LOG_DBG("tap-and-hold release timeout ignored: touch resumed");
    return;
  }

  data->active_tap_hold = false;
  input_report_key(data->dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
  data->buttons_pressed &= ~BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
  LOG_DBG("tap-and-hold release timeout fired: drag released");
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
#define IQS915X_SCROLL_INERTIA_Q8_HALF (BIT(IQS915X_SCROLL_INERTIA_FP_BITS - 1))
#define IQS915X_SCROLL_INERTIA_ZERO_OUTPUT_LIMIT 3

static void iqs915x_calculate_decayed_movement_fixed(
    int16_t in_dx, int16_t in_dy, int16_t decay_factor_q8,
    int16_t *out_dx, int16_t *out_dy, int16_t *rem_x, int16_t *rem_y)
{
  int32_t ideal_dx_q8 = ((int32_t)in_dx << IQS915X_SCROLL_INERTIA_FP_BITS) + *rem_x;
  int32_t ideal_dy_q8 = ((int32_t)in_dy << IQS915X_SCROLL_INERTIA_FP_BITS) + *rem_y;

  int32_t decayed_dx_q8 = (ideal_dx_q8 * decay_factor_q8) >> IQS915X_SCROLL_INERTIA_FP_BITS;
  int32_t decayed_dy_q8 = (ideal_dy_q8 * decay_factor_q8) >> IQS915X_SCROLL_INERTIA_FP_BITS;

  int16_t output_dx =
      (int16_t)((decayed_dx_q8 + IQS915X_SCROLL_INERTIA_Q8_HALF) >> IQS915X_SCROLL_INERTIA_FP_BITS);
  int16_t output_dy =
      (int16_t)((decayed_dy_q8 + IQS915X_SCROLL_INERTIA_Q8_HALF) >> IQS915X_SCROLL_INERTIA_FP_BITS);

  *rem_x = (int16_t)(decayed_dx_q8 - ((int32_t)output_dx << IQS915X_SCROLL_INERTIA_FP_BITS));
  *rem_y = (int16_t)(decayed_dy_q8 - ((int32_t)output_dy << IQS915X_SCROLL_INERTIA_FP_BITS));

  *out_dx = output_dx;
  *out_dy = output_dy;
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
}

static void iqs915x_scroll_inertia_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, scroll_inertia_work);
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  const struct iqs915x_scroll_inertia_profile *profile = &config->scroll_inertia;
  struct iqs915x_scroll_inertia_state *state = &data->scroll_inertia_state;
  int16_t step_x;
  int16_t step_y;
  int16_t decay_factor_q8;
  bool emitted = false;

  if (!state->active || !profile->enabled)
  {
    return;
  }

  step_x = state->ema_vx;
  step_y = state->ema_vy;
  decay_factor_q8 = (int16_t)((profile->decay_factor_int * IQS915X_SCROLL_INERTIA_FP_SCALE) / 100);

  iqs915x_calculate_decayed_movement_fixed(
      state->ema_vx, state->ema_vy, decay_factor_q8, &step_x, &step_y,
      &state->remainder_x_q8, &state->remainder_y_q8);

  if (abs(step_x) <= profile->threshold_stop && abs(step_y) <= profile->threshold_stop)
  {
    iqs915x_stop_scroll_inertia(data);
    LOG_DBG("Scroll inertia stopped (velocity below threshold)");
    return;
  }

  state->vx = step_x;
  state->vy = step_y;
  state->ema_vx = step_x;
  state->ema_vy = step_y;
  state->is_inertial = true;
  iqs915x_filter_scroll_cross_axis(&step_x, &step_y);

  if (step_x != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->scroll_x_acc, INPUT_REL_HWHEEL, step_x,
        data->swipe_resolution_x, config->scroll_divisor);
  }

  if (step_y != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->scroll_y_acc, INPUT_REL_WHEEL, step_y,
        data->swipe_resolution_y, config->scroll_divisor);
  }

  if (emitted)
  {
    state->zero_output_ticks = 0;
  }
  else if (++state->zero_output_ticks >= IQS915X_SCROLL_INERTIA_ZERO_OUTPUT_LIMIT)
  {
    iqs915x_stop_scroll_inertia(data);
    LOG_DBG("Scroll inertia stopped (no HID output)");
    return;
  }

  // 次のティックをスケジュール
  k_work_reschedule(&data->scroll_inertia_work, K_MSEC(profile->interval_ms));
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

static int iqs915x_prepare_init_chunk(const struct device *dev,
                                      uint16_t offset, uint16_t *addr,
                                      uint16_t *chunk, uint8_t *buffer,
                                      bool log_changes)
{
  const struct iqs915x_config *config = dev->config;

  if (!config->init_data || config->init_data_len == 0 ||
      offset >= config->init_data_len)
  {
    return -EINVAL;
  }

  if (offset < IQS915X_INIT_DATA_MAIN_SIZE)
  {
    uint16_t remaining = IQS915X_INIT_DATA_MAIN_SIZE - offset;
    *chunk = MIN(remaining, IQS915X_INIT_WRITE_CHUNK_SIZE);
    *addr = IQS915X_INIT_DATA_BASE_ADDR + offset;
    memcpy(buffer, &config->init_data[offset], *chunk);

    for (int i = 0; i < *chunk; i++)
    {
      uint16_t current_addr = *addr + i;
      uint8_t original = buffer[i];
      bool dts_patch = false;

      // === 強制パッチ: ドライバ正常動作に必須のビット修正 ===
      if (current_addr == IQS915X_SYSTEM_CONTROL)
      {
        // ACK_RESET(bit7), REATI_ALP(bit6), REATI_TP(bit5) は
        // 初期化シーケンス中に誤って実行されないよう強制クリアする
        buffer[i] &= ~0xE0;
      }
      else if (current_addr == IQS915X_CONFIG_SETTINGS)
      {
        uint16_t cfg = (uint16_t)buffer[i] | ((uint16_t)buffer[i + 1] << 8);

        // TERMINATE_COMMS(bit6), FORCE_COMMS_METHOD(bit4) は
        // クロックストレッチ＋I2C STOPによる標準動作のため強制クリアし、
        // Event ModeではTP_EVENTのみをイベント源にする。
        cfg &= ~(IQS915X_FORCE_COMMS_METHOD | IQS915X_TERMINATE_COMMS);
        cfg = iqs915x_config_settings_without_event_mode(cfg);
        buffer[i] = cfg & 0xFF;
      }
      else if (current_addr == IQS915X_CONFIG_SETTINGS + 1)
      {
        uint16_t cfg = (uint16_t)buffer[i - 1] | ((uint16_t)buffer[i] << 8);

        // 16-bit little-endian: high byte bit0 == EVENT_MODE。
        // SHOW_RESET clear後に明示writeするためEVENT_MODEだけclearし、
        // GESTURE_EVENT/TP_TOUCH_EVENTは無効、TP_EVENTのみ有効にする。
        cfg = iqs915x_config_settings_without_event_mode(cfg);
        buffer[i] = (cfg >> 8) & 0xFF;
      }
      // === DTSプリパッチ: DTS設定値を事前適用（Re-ATI完了時点で最終値が有効になるよう） ===
      else if (current_addr == IQS915X_ACTIVE_MODE_REPORT_RATE &&
               config->report_rate_ms > 0)
      {
        buffer[i] = config->report_rate_ms & 0xFF;
        dts_patch = true;
      }
      else if (current_addr == IQS915X_ACTIVE_MODE_REPORT_RATE + 1 &&
               config->report_rate_ms > 0)
      {
        buffer[i] = (config->report_rate_ms >> 8) & 0xFF;
        dts_patch = true;
      }
      else if (current_addr == IQS915X_TRACKPAD_SETTINGS)
      {
        uint8_t clr = IQS915X_FLIP_X | IQS915X_FLIP_Y | IQS915X_SWITCH_XY_AXIS;
        uint8_t set = 0;
        if (config->flip_x)
          set |= IQS915X_FLIP_X;
        if (config->flip_y)
          set |= IQS915X_FLIP_Y;
        if (config->switch_xy)
          set |= IQS915X_SWITCH_XY_AXIS;
        buffer[i] = (buffer[i] & ~clr) | set;
        dts_patch = true;
      }
      else if (current_addr == IQS915X_SINGLE_FINGER_GESTURES_ENABLE)
      {
        // tap/holdはドライバ側で指本数・位置・時間から判定する。
        uint8_t clr = (uint8_t)(IQS915X_SINGLE_TAP | IQS915X_PRESS_AND_HOLD);
        buffer[i] &= ~clr;
        dts_patch = true;
      }
      else if (current_addr == IQS915X_TWO_FINGER_GESTURES_ENABLE)
      {
        // two-finger tap/scrollはabsolute座標からドライバ側で判定する。
        uint8_t clr = (uint8_t)(IQS915X_TWO_FINGER_TAP | IQS915X_SCROLL);
        buffer[i] &= ~clr;
        dts_patch = true;
      }
      // init-dataのバイト値がドライバにより変更された場合はWRNを出力する
      if (log_changes && buffer[i] != original)
      {
        if (dts_patch)
        {
          LOG_WRN("Init: init-data pre-patched by DTS at reg 0x%04x: "
                  "0x%02x -> 0x%02x",
                  current_addr, original, buffer[i]);
        }
        else
        {
          LOG_WRN("Init: init-data overridden at reg 0x%04x: "
                  "init-data=0x%02x -> forced=0x%02x",
                  current_addr, original, buffer[i]);
        }
      }
    }

    return 0;
  }

  uint16_t eng_offset = offset - IQS915X_INIT_DATA_MAIN_SIZE;
  uint16_t remaining = IQS915X_INIT_DATA_ENG_SIZE - eng_offset;

  *chunk = MIN(remaining, IQS915X_INIT_WRITE_CHUNK_SIZE);
  *addr = IQS915X_INIT_DATA_ENG_ADDR + eng_offset;
  memcpy(buffer, &config->init_data[offset], *chunk);

  return 0;
}

static void iqs915x_restart_initialization(const struct device *dev,
                                           const char *reason)
{
  struct iqs915x_data *data = dev->data;

  if (data->init_restart_count >= IQS915X_INIT_MAX_RESTARTS)
  {
    LOG_ERR("Init: restart limit reached after %u attempts (%s). Halting init.",
            data->init_restart_count, reason);
    data->init_step = INIT_FAILED;
    return;
  }

  data->init_restart_count++;
  data->init_step = INIT_SOFTWARE_RESET;
  data->init_data_offset = 0;
  data->wait_count = 0;
  data->init_chunk_retry_count = 0;
  data->init_pending_cfg = 0;
  data->confirmed_config_settings = 0;
  iqs915x_reset_event_mode_relatch_state(data);
  LOG_WRN("Init: restarting via software reset (%u/%u): %s",
          data->init_restart_count, IQS915X_INIT_MAX_RESTARTS, reason);
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
      data->init_chunk_retry_count = 0;
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
      data->init_chunk_retry_count = 0;
    }
    else
    {
      LOG_DBG("Init: Waiting for SHOW_RESET (flags=0x%04x)", stream.info_flags);
    }
    break;
  }

  case INIT_ACK_RESET:
  {
    uint16_t sys_ctrl = IQS915X_ACK_RESET;
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, sys_ctrl);
    if (ret < 0)
    {
      LOG_ERR("Failed to send initial ACK reset: %d", ret);
      return;
    }
    LOG_INF("Init: ACK_RESET sent");
    data->init_step = INIT_VERIFY_SHOW_RESET_CLEAR;
    data->wait_count = 0;
    break;
  }

  case INIT_VERIFY_SHOW_RESET_CLEAR:
  {
    uint16_t info_flags = 0;

    data->wait_count++;
    ret = iqs915x_read_reg16(dev, IQS915X_INFO_FLAGS, &info_flags);
    if (ret < 0)
    {
      LOG_ERR("Failed to read Info Flags after ACK reset: %d", ret);
      return;
    }

    if (info_flags == 0xEEEE)
    {
      LOG_DBG("Init: IC busy (0xEEEE) while waiting for SHOW_RESET clear");
      break;
    }

    if ((info_flags & IQS915X_SHOW_RESET) == 0)
    {
      LOG_INF("Init: SHOW_RESET cleared (flags=0x%04x)", info_flags);
      data->init_step = INIT_REQUEST_REATI;
      data->wait_count = 0;
      break;
    }

    if (data->wait_count > IQS915X_INIT_SHOW_RESET_CLEAR_MAX_WAIT)
    {
      LOG_ERR("Init: SHOW_RESET did not clear after ACK reset (flags=0x%04x)",
              info_flags);
      iqs915x_restart_initialization(dev, "SHOW_RESET stuck after ACK_RESET");
      break;
    }

    LOG_DBG("Init: Waiting for SHOW_RESET clear (flags=0x%04x, cycle=%d/%d)",
            info_flags, data->wait_count,
            IQS915X_INIT_SHOW_RESET_CLEAR_MAX_WAIT);
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
    uint16_t addr = 0;
    uint16_t chunk = 0;
    uint8_t buffer[IQS915X_INIT_WRITE_CHUNK_SIZE];

    ret = iqs915x_prepare_init_chunk(dev, offset, &addr, &chunk, buffer,
                                     data->init_chunk_retry_count == 0);
    if (ret < 0)
    {
      LOG_ERR("Failed to prepare init-data at offset %u: %d", offset, ret);
      iqs915x_restart_initialization(dev, "invalid init-data offset");
      break;
    }

    ret = iqs915x_write_block(dev, addr, buffer, chunk);
    if (ret < 0)
    {
      data->init_chunk_retry_count++;
      LOG_ERR("Failed to write init-data at 0x%04x: %d (retry %u/%u)",
              addr, ret, data->init_chunk_retry_count,
              IQS915X_INIT_CHUNK_WRITE_MAX_RETRIES);
      if (data->init_chunk_retry_count >
          IQS915X_INIT_CHUNK_WRITE_MAX_RETRIES)
      {
        data->init_step = INIT_VERIFY_INIT_CHUNK;
      }
      return; // 次のRDYで同じチャンクをリトライまたはread-backする
    }

    LOG_DBG("Init: Wrote %d bytes at 0x%04x (%d/%d)", chunk, addr,
            offset + chunk, total);
    data->init_data_offset = offset + chunk;
    data->init_chunk_retry_count = 0;

    if (data->init_data_offset >= total)
    {
      LOG_INF("Init: All init-data written (%d bytes)", total);
      data->init_step = INIT_ACK_RESET;
      data->wait_count = 0;
    }
    break;
  }

  case INIT_VERIFY_INIT_CHUNK:
  {
    uint16_t offset = data->init_data_offset;
    uint16_t addr = 0;
    uint16_t chunk = 0;
    uint8_t expected[IQS915X_INIT_WRITE_CHUNK_SIZE];
    uint8_t actual[IQS915X_INIT_WRITE_CHUNK_SIZE];

    ret = iqs915x_prepare_init_chunk(dev, offset, &addr, &chunk, expected,
                                     false);
    if (ret < 0)
    {
      LOG_ERR("Failed to prepare init-data verify chunk at offset %u: %d",
              offset, ret);
      iqs915x_restart_initialization(dev, "invalid init-data verify offset");
      break;
    }

    ret = iqs915x_read_block(dev, addr, actual, chunk);
    if (ret < 0)
    {
      LOG_ERR("Failed to read back init-data at 0x%04x: %d", addr, ret);
      iqs915x_restart_initialization(dev, "init-data read-back failed");
      break;
    }

    if (memcmp(expected, actual, chunk) == 0)
    {
      LOG_WRN("Init: write error at 0x%04x but read-back matches; continuing",
              addr);
      data->init_data_offset = offset + chunk;
      data->init_chunk_retry_count = 0;
      if (data->init_data_offset >= config->init_data_len)
      {
        LOG_INF("Init: All init-data written (%d bytes)", config->init_data_len);
        data->init_step = INIT_ACK_RESET;
        data->wait_count = 0;
      }
      else
      {
        data->init_step = INIT_WRITE_INIT_DATA;
      }
      break;
    }

    for (int i = 0; i < chunk; i++)
    {
      if (expected[i] != actual[i])
      {
        LOG_ERR("Init: init-data mismatch at reg 0x%04x: expected=0x%02x actual=0x%02x",
                addr + i, expected[i], actual[i]);
        break;
      }
    }
    iqs915x_restart_initialization(dev, "init-data read-back mismatch");
    break;
  }

  case INIT_REQUEST_REATI:
  {
    uint16_t sys_ctrl = IQS915X_REATI_TP | IQS915X_REATI_ALP;

    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, sys_ctrl);
    if (ret < 0)
    {
      LOG_ERR("Failed to request Re-ATI: %d", ret);
      return;
    }
    LOG_INF("Init: Re-ATI requested");
    data->init_step = INIT_WAIT_REATI;
    data->wait_count = 0;
    break;
  }

  case INIT_PREPARE_EVENT_MODE:
  {
    // SHOW_RESET clearとTP Re-ATI完了後にEvent Modeを明示writeする。
    // Event sourceはTP_EVENTのみ有効化し、IQS915x gesture eventと
    // diamond pattern channel変化用のTP_TOUCH_EVENTは無効化する。
    uint16_t cfg = 0;
    ret = iqs915x_read_reg16(dev, IQS915X_CONFIG_SETTINGS, &cfg);
    if (ret < 0)
    {
      LOG_ERR("Failed to read CONFIG_SETTINGS: %d", ret);
      return;
    }
    data->init_pending_cfg = iqs915x_apply_config_settings_policy(cfg);
    data->wait_count = 0;
    data->init_step = INIT_SET_EVENT_MODE;
    break;
  }

  case INIT_SET_EVENT_MODE:
    ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS,
                              data->init_pending_cfg);
    if (ret < 0)
    {
      LOG_ERR("Failed to force Event Mode + Manual Control: %d", ret);
      return;
    }
    LOG_INF("Init: Event Mode explicitly enabled (CONFIG_SETTINGS=0x%04x)",
            data->init_pending_cfg);
    data->init_step = INIT_CONFIRM_EVENT_MODE;
    break;

  case INIT_CONFIRM_EVENT_MODE:
  {
    uint16_t cfg = 0;

    data->wait_count++;
    ret = iqs915x_read_reg16(dev, IQS915X_CONFIG_SETTINGS, &cfg);
    if (ret < 0)
    {
      LOG_ERR("Failed to confirm CONFIG_SETTINGS: %d", ret);
      return;
    }

    uint16_t expected = IQS915X_EVENT_MODE | IQS915X_MANUAL_CONTROL |
                        IQS915X_TP_EVENT;
    uint16_t forbidden = IQS915X_GESTURE_EVENT | IQS915X_TP_TOUCH_EVENT;

    if ((cfg & expected) == expected && (cfg & forbidden) == 0)
    {
      LOG_INF("Init: Event Mode confirmed (CONFIG_SETTINGS=0x%04x)",
              cfg);
      data->init_step = INIT_COMPLETE;
      data->initialized = true;
      data->work_state = WORK_READ_DATA;
      data->last_info_flags = 0;
      data->init_restart_count = 0;
      data->init_chunk_retry_count = 0;
      data->init_pending_cfg = 0;
      data->confirmed_config_settings = cfg;
      iqs915x_reset_event_mode_relatch_state(data);
      LOG_INF("IQS915x initialization complete");
      break;
    }

    if (data->wait_count > IQS915X_INIT_EVENT_MODE_MAX_RETRIES)
    {
      LOG_ERR("Init: Event Mode + Manual Control did not stick "
              "(CONFIG_SETTINGS=0x%04x)",
              cfg);
      iqs915x_restart_initialization(dev, "Event Mode verification failed");
      break;
    }

    LOG_WRN("Init: CONFIG_SETTINGS still missing TP_EVENT-only policy (0x%04x), "
            "retrying force (%d/%d)",
            cfg, data->wait_count, IQS915X_INIT_EVENT_MODE_MAX_RETRIES);
    data->init_pending_cfg = iqs915x_apply_config_settings_policy(cfg);
    data->init_step = INIT_SET_EVENT_MODE;
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

    // REATI_OCCURRED (bit4) フラグでTP Re-ATI完了を検出する。
    // ALP Re-ATIはLP1/LP2で実行されるため、初期化完了の必須条件にしない。
    // このフラグはRe-ATIが実行されたRDYサイクルで1回だけセットされる
    if (stream.info_flags & IQS915X_REATI_OCCURRED)
    {
      LOG_INF("Init: TP Re-ATI occurred (flags=0x%04x) after %d cycles",
              stream.info_flags, data->wait_count);
      // Re-ATI完了後にEvent Modeを明示writeする
      data->init_step = INIT_PREPARE_EVENT_MODE;
      break;
    }

    if (stream.info_flags & IQS915X_ALP_REATI_OCCURRED)
    {
      LOG_DBG("Init: ALP Re-ATI occurred while waiting for TP Re-ATI "
              "(flags=0x%04x)",
              stream.info_flags);
    }

    if (data->wait_count > IQS915X_INIT_REATI_MAX_WAIT)
    {
      LOG_ERR("Init: TP Re-ATI timeout after %d cycles (flags=0x%04x)",
              data->wait_count, stream.info_flags);
      iqs915x_restart_initialization(dev, "TP Re-ATI timeout");
      break;
    }

    // Re-ATIはまだ発生していない、次のRDYで再確認
    LOG_DBG("Init: Waiting for TP Re-ATI (flags=0x%04x, cycle=%d/%d)",
            stream.info_flags, data->wait_count, IQS915X_INIT_REATI_MAX_WAIT);
    break;
  }

  case INIT_COMPLETE:
    break;

  case INIT_FAILED:
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
      ret = k_sem_take(&data->rdy_sem, K_MSEC(2000));
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
        iqs915x_schedule_event_mode_relatch(data, "Active mode transition");
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
        iqs915x_schedule_event_mode_relatch(data, "LP2 mode transition");
        LOG_INF("Trackpad entered LP2 mode");
      }
      else
      {
        LOG_ERR("Failed to enter LP2 mode: %d, will retry", ret);
        k_sem_give(&data->rdy_sem);
      }
      continue;
    }

    // 初期化完了後の通常モードはポーリングなしでRDY割り込みを待機
    k_sem_take(&data->rdy_sem, K_FOREVER);

    if (data->work_state == WORK_RELATCH_EVENT_MODE_DISABLE ||
        data->work_state == WORK_RELATCH_EVENT_MODE_ENABLE)
    {
      iqs915x_handle_event_mode_relatch_step(dev);
      continue;
    }

    // ===== トラックパッド無効時はRDYを待つだけで何もしない =====
    if (!data->enabled)
    {
      continue;
    }

    // ストリーミングデータをraw読み取り
    struct iqs915x_stream_data stream;
    ret = iqs915x_read_stream(dev, &stream);
    if (ret < 0)
    {
      LOG_ERR("Failed to read stream: %d", ret);
      continue;
    }

    // IQS915xのランタイムリセット検出
    // 初期化中のINIT_VERIFY_SHOW_RESET_CLEARでSHOW_RESETクリアを確認済みなので、
    // 通常モードでSHOW_RESETが立っている場合は真のランタイムリセット。
    if (stream.info_flags & IQS915X_SHOW_RESET)
    {
      LOG_WRN("IQS915x runtime reset detected (flags=0x%04x), "
              "re-initializing...",
              stream.info_flags);
      data->initialized = false;
      data->init_step = INIT_CHECK_SHOW_RESET;
      data->init_data_offset = 0;
      data->wait_count = 0;
      data->init_chunk_retry_count = 0;
      data->init_restart_count = 0;
      data->init_pending_cfg = 0;
      data->confirmed_config_settings = 0;
      data->work_state = WORK_READ_DATA;
      iqs915x_reset_event_mode_relatch_state(data);
      // ドラッグ中だった場合はZMKへボタンリリースを確実に通知する
      if (data->active_tap_hold)
      {
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      }
      k_work_cancel_delayable(&data->tap_and_hold_release_work);
      k_work_cancel_delayable(&data->single_tap_work);
      k_work_cancel_delayable(&data->tap_and_hold_start_work);
      data->tap_and_hold_release_pending = false;
      data->single_tap_pending = false;
      data->tap_sequence_second_touch = false;
      data->tap_and_hold_start_pending = false;
      data->active_tap_hold = false;
      data->is_touching = false;
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
    // GLOBAL_TP_TOUCHは取りこぼしがあるため、接触境界には使わない。
    // =========================================================
    uint8_t reported_fingers = stream.trackpad_flags & IQS915X_NUM_FINGERS_MASK;
    uint8_t num_fingers = reported_fingers;
    bool global_tp_touch =
        (stream.info_flags & IQS915X_GLOBAL_TP_TOUCH) != 0;
    bool is_touching_now = num_fingers > 0;
    bool was_touching = data->is_touching;
    bool touch_down = is_touching_now && !was_touching;
    bool touch_up = !is_touching_now && was_touching;
    bool touch_state_changed = touch_down || touch_up;
    bool tp_movement = (stream.trackpad_flags & IQS915X_TP_MOVEMENT) != 0;

    if (touch_state_changed)
    {
      LOG_DBG("touch state changed: %u -> %u reported_fingers=%u "
              "effective_fingers=%u global_tp_touch=%u "
              "flags=0x%04x info=0x%04x",
              was_touching, is_touching_now, reported_fingers, num_fingers,
              global_tp_touch, stream.trackpad_flags, stream.info_flags);
    }

    if (touch_down)
    {
      iqs915x_reset_absolute_tracking(data);
      data->tap_drag_raw_max_fingers = num_fingers > 0 ? num_fingers : 1;
      data->tap_drag_raw_gesture_seen = false;
      data->raw_single_tap_reported = false;
      data->raw_two_finger_tap_reported = false;
      data->completed_two_finger_movement = 0;
      data->tap_start_valid = false;
      data->tap_max_movement = 0;
    }
    else if (is_touching_now && num_fingers > data->tap_drag_raw_max_fingers)
    {
      data->tap_drag_raw_max_fingers = num_fingers;
    }

    if (touch_up)
    {
      iqs915x_reset_absolute_tracking(data);
    }

    data->is_touching = is_touching_now;
    iqs915x_update_finger_state(data, &stream, is_touching_now,
                                touch_down, touch_up);
    iqs915x_update_sequence_gates(data);

    iqs915x_update_single_tap_movement(data, &stream, num_fingers);

    bool has_tp_event = is_touching_now || touch_state_changed;
    if (tp_movement)
    {
      has_tp_event = true;
    }

    if (has_tp_event)
    {
      bool scroll = iqs915x_handle_two_finger_scroll(config, data, &stream);
      bool gesture_active = scroll || data->scroll_sequence_active ||
                            data->multifinger_swipe_latched;
      bool suppress_pointer_tail =
          !gesture_active && data->gesture_pointer_suppress_ticks > 0;
      bool suppress_pointer = gesture_active || suppress_pointer_tail;
      bool single_finger_pointer = data->finger_tracker.stable_count == 1;
      bool allow_pointer_report = !suppress_pointer && single_finger_pointer &&
                                  !data->finger_tracker.awaiting_zero_contact;

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

      int64_t now_ms = k_uptime_get();

      if (touch_down)
      {
        data->last_touch_down_time = now_ms;
      }

      if (!scroll)
      {
        // スクロール入力が停止しても慣性がアクティブなら継続させる。
        // 非アクティブ時のみアキュムレータをクリアする。
        if (!data->scroll_inertia_state.active)
        {
          data->scroll_x_acc = 0;
          data->scroll_y_acc = 0;
        }
      }

      // タップジェスチャー判定
      uint16_t button_code = 0;
      bool two_finger_tap_pressed = false;
      bool single_tap_enabled = config->one_finger_tap || config->tap_and_hold;
      bool allow_single_tap = data->finger_tracker.completed_one_tap_path;
      bool allow_two_finger_tap = data->finger_tracker.completed_two_tap_path;
      bool raw_single_tap_path =
          !is_touching_now && data->tap_drag_raw_max_fingers == 1 &&
          !data->tap_drag_raw_gesture_seen &&
          data->tap_max_movement < data->tap_distance;
      int64_t touch_duration_ms =
          touch_up && data->last_touch_down_time > 0
              ? now_ms - data->last_touch_down_time
              : 0;
      bool tap_duration_ok =
          touch_up && touch_duration_ms <= data->tap_touch_time_ms;
      bool raw_single_tap_completed =
          tap_duration_ok &&
          raw_single_tap_path;
      bool raw_two_finger_tap_completed =
          tap_duration_ok && allow_two_finger_tap &&
          data->tap_drag_raw_max_fingers == 2 &&
          data->completed_two_finger_movement < data->tap_distance &&
          !data->tap_drag_raw_gesture_seen &&
          !data->raw_two_finger_tap_reported;

      if (touch_down && data->single_tap_pending)
      {
        k_work_cancel_delayable(&data->single_tap_work);
        data->single_tap_pending = false;
        data->tap_sequence_second_touch = true;
        LOG_DBG("single tap pending canceled by second touch: air=%lld ms",
                (long long)(now_ms - data->pending_tap_up_time));

        if (config->tap_and_hold)
        {
          data->tap_and_hold_start_pending = true;
          k_work_schedule(&data->tap_and_hold_start_work,
                          K_MSEC(data->tap_touch_time_ms));
        }
      }
      else if (touch_down)
      {
        data->tap_sequence_second_touch = false;
      }

      if (data->tap_sequence_second_touch && is_touching_now &&
          config->tap_and_hold && !data->active_tap_hold &&
          data->tap_max_movement >= data->tap_distance)
      {
        k_work_cancel_delayable(&data->tap_and_hold_start_work);
        iqs915x_start_tap_and_hold_drag(data, "second touch moved past tap distance");
      }

      if (single_tap_enabled && touch_up && raw_single_tap_completed &&
          !data->raw_single_tap_reported)
      {
        data->raw_single_tap_reported = true;

        if (data->tap_sequence_second_touch)
        {
          k_work_cancel_delayable(&data->tap_and_hold_start_work);
          data->tap_and_hold_start_pending = false;
          data->tap_sequence_second_touch = false;
          iqs915x_report_button_double_tap(data, INPUT_BTN_0);
          LOG_DBG("double tap reported: duration=%lld ms movement=%u "
                  "distance=%u",
                  (long long)touch_duration_ms, data->tap_max_movement,
                  data->tap_distance);
        }
        else
        {
          data->single_tap_pending = true;
          data->pending_tap_up_time = now_ms;
          k_work_schedule(&data->single_tap_work,
                          K_MSEC(data->tap_air_time_ms));
          LOG_DBG("single tap pending: duration=%lld ms movement=%u "
                  "distance=%u air=%u ms stable_path=%u",
                  (long long)touch_duration_ms, data->tap_max_movement,
                  data->tap_distance, data->tap_air_time_ms, allow_single_tap);
        }
      }
      else if (touch_up && single_tap_enabled &&
               data->tap_drag_raw_max_fingers == 1 &&
               !data->raw_single_tap_reported &&
               !data->active_tap_hold)
      {
        LOG_DBG("single tap suppressed: duration=%lld movement=%u threshold=%u "
                "raw_gesture=%u path=%u",
                (long long)touch_duration_ms, data->tap_max_movement,
                data->tap_distance, data->tap_drag_raw_gesture_seen,
                allow_single_tap);
        data->tap_sequence_second_touch = false;
        data->tap_and_hold_start_pending = false;
        k_work_cancel_delayable(&data->tap_and_hold_start_work);
      }

      if (config->two_finger_tap && raw_two_finger_tap_completed)
      {
        two_finger_tap_pressed = true;
        button_code = INPUT_BTN_1;
        data->raw_two_finger_tap_reported = true;
        LOG_DBG("two-finger tap detected from touch sequence: duration=%lld ms "
                "movement=%u threshold=%u",
                (long long)touch_duration_ms,
                data->completed_two_finger_movement, data->tap_distance);
      }
      else if (touch_up && config->two_finger_tap &&
               data->tap_drag_raw_max_fingers == 2 &&
               !data->raw_two_finger_tap_reported)
      {
        LOG_DBG("two-finger tap suppressed: duration=%lld movement=%u "
                "threshold=%u path=%u scroll=%u",
                (long long)touch_duration_ms,
                data->completed_two_finger_movement, data->tap_distance,
                allow_two_finger_tap, data->tap_drag_raw_gesture_seen);
      }

      if (config->tap_and_hold && data->active_tap_hold)
      {
        if (touch_down && data->tap_and_hold_release_pending)
        {
          k_work_cancel_delayable(&data->tap_and_hold_release_work);
          data->tap_and_hold_release_pending = false;
          LOG_DBG("tap-and-hold release timeout canceled: touch resumed");
        }
        else if (touch_up && !data->tap_and_hold_release_pending)
        {
          k_work_schedule(&data->tap_and_hold_release_work,
                          K_MSEC(config->tap_and_hold_release_timeout_ms));
          data->tap_and_hold_release_pending = true;
          LOG_DBG("tap-and-hold release timeout scheduled: %u ms",
                  config->tap_and_hold_release_timeout_ms);
        }
      }

      if (two_finger_tap_pressed)
      {
        iqs915x_report_button_tap(data, button_code);
      }

      if (scroll)
      {
        // 2本指scrollはabsolute centroid deltaから処理済み。
      }
      else if (iqs915x_handle_multifinger_swipe(config, data, &stream))
      {
        // 3/4本指の連続スワイプ出力を優先する。
      }
      else
      {
        bool abs_finger_valid =
            is_touching_now && num_fingers == 1 &&
            (stream.trackpad_flags & IQS915X_FINGER1_CONFIDENCE) != 0;

        if (touch_up)
        {
          iqs915x_reset_absolute_tracking(data);
        }
        else if (!allow_pointer_report || !abs_finger_valid)
        {
          if (!suppress_pointer && (touch_down || tp_movement))
          {
            iqs915x_cancel_scroll_inertia(data);
          }

          iqs915x_reset_absolute_tracking(data);

          if (touch_down || tp_movement)
          {
            LOG_DBG(
                "tp_absolute suppressed: fingers=%u stable=%u await0=%u "
                "conf=%u scroll=%u gesture_seen=%u x=%u y=%u",
                num_fingers,
                data->finger_tracker.stable_count,
                data->finger_tracker.awaiting_zero_contact,
                (stream.trackpad_flags & IQS915X_FINGER1_CONFIDENCE) != 0,
                data->scroll_sequence_active,
                data->tap_drag_raw_gesture_seen,
                stream.abs_x, stream.abs_y);
          }
        }
        else
        {
          // 通常のポインタ操作が再開したら慣性スクロールを止める
          iqs915x_cancel_scroll_inertia(data);

          if (touch_down || tp_movement)
          {
            if (!data->last_abs_valid)
            {
              // 初回は基準点のみ保存し、次フレーム以降をデルタ報告する
              data->last_abs_x = stream.abs_x;
              data->last_abs_y = stream.abs_y;
              data->last_abs_valid = true;
            }
            else
            {
              int32_t rel_x = (int32_t)stream.abs_x - (int32_t)data->last_abs_x;
              int32_t rel_y = (int32_t)stream.abs_y - (int32_t)data->last_abs_y;

              data->last_abs_x = stream.abs_x;
              data->last_abs_y = stream.abs_y;

              if (iqs915x_absolute_delta_is_discontinuity(data, rel_x, rel_y))
              {
                LOG_DBG("tp_absrel discontinuity: rel_x=%d rel_y=%d "
                        "threshold=%u x=%u y=%u",
                        (int)rel_x, (int)rel_y,
                        iqs915x_absolute_discontinuity_threshold(data),
                        stream.abs_x, stream.abs_y);
              }
              else if (rel_x != 0 || rel_y != 0)
              {
                LOG_DBG("tp_absrel: rel_x=%d, rel_y=%d (x=%u y=%u)",
                        (int)rel_x, (int)rel_y, stream.abs_x, stream.abs_y);
                input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
              }
            }
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
  data->init_data_offset = 0;
  data->wait_count = 0;
  data->init_chunk_retry_count = 0;
  data->init_restart_count = 0;
  data->init_pending_cfg = 0;
  data->confirmed_config_settings = 0;
  iqs915x_reset_event_mode_relatch_state(data);
  // disabled-by-defaultの場合は初期化完了後にLP2へ移行し、mode遷移後に
  // Event Modeを再ラッチする。
  data->enabled = !config->disabled_by_default;
  data->lp2_pending = config->disabled_by_default;
  data->active_pending = false;
  iqs915x_reset_absolute_tracking(data);
  iqs915x_reset_runtime_gesture_state(data);
  iqs915x_configure_swipe_thresholds(config, data);
  iqs915x_configure_tap_profile(config, data);
  data->tap_and_hold_release_pending = false;
  data->single_tap_pending = false;
  data->tap_sequence_second_touch = false;
  data->tap_and_hold_start_pending = false;

  k_sem_init(&data->rdy_sem, 0, 1);
  k_work_init_delayable(&data->button_release_work,
                        iqs915x_button_release_work_handler);
  k_work_init_delayable(&data->tap_and_hold_release_work,
                        iqs915x_tap_and_hold_release_work_handler);
  k_work_init_delayable(&data->single_tap_work,
                        iqs915x_single_tap_work_handler);
  k_work_init_delayable(&data->tap_and_hold_start_work,
                        iqs915x_tap_and_hold_start_work_handler);
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
#define IQS915X_INIT(n)                                                                                                                                                                            \
  static struct iqs915x_data iqs915x_data_##n;                                                                                                                                                     \
  static const struct iqs915x_config iqs915x_config_##n = {                                                                                                                                        \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                                                                                                                                              \
      .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                                                                                                                             \
      .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                                                                                                                                 \
      .init_data = iqs915x_init_data_bretagne,                                                                                                                                                     \
      .init_data_len = IQS915X_INIT_DATA_TOTAL_SIZE,                                                                                                                                               \
      .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                                                                                                                           \
      .tap_and_hold = DT_INST_PROP(n, tap_and_hold),                                                                                                                                               \
      .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                                                                                                                           \
      .scroll = DT_INST_PROP(n, scroll),                                                                                                                                                           \
      .scroll_divisor = DT_INST_PROP(n, scroll_divisor),                                                                                                                                           \
      .scroll_inertia = {                                                                                                                                                                          \
          .enabled = DT_INST_PROP(n, scroll_inertia) ||                                                                                                                                            \
                     DT_INST_NODE_HAS_PROP(n, trigger_ms) ||                                                                                                                                       \
                     DT_INST_NODE_HAS_PROP(n, scroll_decay_factor_int) ||                                                                                                                          \
                     DT_INST_NODE_HAS_PROP(n, scroll_report_interval_ms) ||                                                                                                                        \
                     DT_INST_NODE_HAS_PROP(n, scroll_threshold_start) ||                                                                                                                           \
                     DT_INST_NODE_HAS_PROP(n, scroll_threshold_stop),                                                                                                                              \
          .trigger_ms = DT_INST_NODE_HAS_PROP(n, trigger_ms) ? DT_INST_PROP(n, trigger_ms) : DT_INST_PROP_OR(n, scroll_inertia_stale_gap_ms, 35),                                                  \
          .decay_factor_int = DT_INST_NODE_HAS_PROP(n, scroll_decay_factor_int) ? DT_INST_PROP(n, scroll_decay_factor_int) : DIV_ROUND_CLOSEST(DT_INST_PROP_OR(n, scroll_inertia_decay, 850), 10), \
          .interval_ms = DT_INST_NODE_HAS_PROP(n, scroll_report_interval_ms) ? DT_INST_PROP(n, scroll_report_interval_ms) : DT_INST_PROP_OR(n, scroll_inertia_interval_ms, 65),                    \
          .threshold_start = DT_INST_NODE_HAS_PROP(n, scroll_threshold_start) ? DT_INST_PROP(n, scroll_threshold_start) : DT_INST_PROP_OR(n, scroll_inertia_min_avg_speed, 2),                     \
          .threshold_stop = DT_INST_NODE_HAS_PROP(n, scroll_threshold_stop) ? DT_INST_PROP(n, scroll_threshold_stop) : DT_INST_PROP_OR(n, scroll_inertia_min_avg_speed, 0),                        \
      },                                                                                                                                                                                           \
      .pinch_inertia = {                                                                                                                                                                           \
          .enabled = DT_INST_PROP(n, pinch_inertia),                                                                                                                                               \
          .interval_ms = DT_INST_PROP_OR(n, pinch_inertia_interval_ms, 10),                                                                                                                        \
          .decay_x1000 = DT_INST_PROP_OR(n, pinch_inertia_decay, 980),                                                                                                                             \
          .recent_window_ms = DT_INST_PROP_OR(n, pinch_inertia_recent_window_ms, 60),                                                                                                              \
          .stale_gap_ms = DT_INST_PROP_OR(n, pinch_inertia_stale_gap_ms, 35),                                                                                                                      \
          .min_samples = DT_INST_PROP_OR(n, pinch_inertia_min_samples, 1),                                                                                                                         \
          .min_avg_speed = DT_INST_PROP_OR(n, pinch_inertia_min_avg_speed, 4),                                                                                                                     \
      },                                                                                                                                                                                           \
      .three_finger_swipe = DT_INST_PROP(n, three_finger_swipe),                                                                                                                                   \
      .four_finger_swipe = DT_INST_PROP(n, four_finger_swipe),                                                                                                                                     \
      .swipe_step = DT_INST_PROP_OR(n, swipe_step, 0),                                                                                                                                             \
      .swipe_threshold_numerator = DT_INST_PROP_OR(n, swipe_threshold_numerator, 1),                                                                                                               \
      .swipe_threshold_denominator = DT_INST_PROP_OR(n, swipe_threshold_denominator, 5),                                                                                                           \
      .swipe_direction_settle_frames = DT_INST_PROP_OR(n, swipe_direction_settle_frames, 2),                                                                                                      \
      .swipe_direction_lock_numerator = DT_INST_PROP_OR(n, swipe_direction_lock_numerator, 3),                                                                                                     \
      .swipe_direction_lock_denominator = DT_INST_PROP_OR(n, swipe_direction_lock_denominator, 2),                                                                                                 \
      .report_rate_ms = DT_INST_PROP_OR(n, report_rate_ms, 0),                                                                                                                                     \
      .coordinate_correction = DT_INST_PROP(n, coordinate_correction),                                                                                                                             \
      .tap_and_hold_release_timeout_ms = DT_INST_PROP_OR(n, tap_and_hold_release_timeout_ms, 500),                                                                                                  \
      .switch_xy = DT_INST_PROP(n, switch_xy),                                                                                                                                                     \
      .flip_x = DT_INST_PROP(n, flip_x),                                                                                                                                                           \
      .flip_y = DT_INST_PROP(n, flip_y),                                                                                                                                                           \
      .disabled_by_default = DT_INST_PROP(n, disabled_by_default),                                                                                                                                 \
  };                                                                                                                                                                                               \
  DEVICE_DT_INST_DEFINE(n, iqs915x_init, NULL, &iqs915x_data_##n,                                                                                                                                  \
                        &iqs915x_config_##n, POST_KERNEL,                                                                                                                                          \
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
    if (data->active_tap_hold)
    {
      input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
      data->active_tap_hold = false;
    }
    k_work_cancel_delayable(&data->tap_and_hold_release_work);
    k_work_cancel_delayable(&data->single_tap_work);
    k_work_cancel_delayable(&data->tap_and_hold_start_work);
    data->tap_and_hold_release_pending = false;
    data->single_tap_pending = false;
    data->tap_sequence_second_touch = false;
    data->tap_and_hold_start_pending = false;

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
    if (data->tap_and_hold_release_pending)
    {
      k_work_cancel_delayable(&data->tap_and_hold_release_work);
      data->tap_and_hold_release_pending = false;
    }
    k_work_cancel_delayable(&data->single_tap_work);
    k_work_cancel_delayable(&data->tap_and_hold_start_work);
    data->single_tap_pending = false;
    data->tap_sequence_second_touch = false;
    data->tap_and_hold_start_pending = false;

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
