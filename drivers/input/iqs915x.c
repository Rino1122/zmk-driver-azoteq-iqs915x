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

#include <errno.h>
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
#include "iqs915x_internal.h"
#include "iqs915x_power.h"

LOG_MODULE_REGISTER(iqs915x, CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL);

#define GESTURE_POINTER_SUPPRESS_TAIL_TICKS 1

static const uint8_t iqs915x_device_api = 0;

#define IQS915X_DEFAULT_SWIPE_THRESHOLD_FALLBACK 32
#define IQS915X_INIT_CHUNK_WRITE_MAX_RETRIES 3
#define IQS915X_INIT_MAX_RESTARTS 3
#define IQS915X_INIT_SHOW_RESET_CLEAR_MAX_WAIT 10
#define IQS915X_INIT_EVENT_MODE_MAX_RETRIES 3
#define IQS915X_INIT_REATI_MAX_WAIT 60
#define IQS915X_POWER_TRANSITION_MAX_RETRIES 3
#define IQS915X_LP2_RESEED_INTERVAL_MS 60000
#define IQS915X_LP2_SAMPLING_PERIOD_MS 500U
#define IQS915X_BUTTON_TAP_RELEASE_MS 100
#define IQS915X_TAP_TOUCH_TIME_FALLBACK_MS 200
#define IQS915X_TAP_AIR_TIME_FALLBACK_MS 150
#define IQS915X_TAP_DISTANCE_FALLBACK 100U
#define IQS915X_POINTER_RESUME_GUARD_FRAMES 2U

static void iqs915x_restart_initialization(const struct device *dev,
                                           const char *reason);
static void iqs915x_stop_scroll_inertia_locked(struct iqs915x_data *data);
static void iqs915x_reset_input_session(struct iqs915x_data *data);

static void iqs915x_mark_initialized(struct iqs915x_data *data, bool initialized)
{
  k_mutex_lock(&data->settings_lock, K_FOREVER);
  data->initialized = initialized;
  if (initialized)
  {
    atomic_set(&data->settings_ready, 1);
  }
  else
  {
    atomic_clear(&data->settings_ready);
  }
  k_mutex_unlock(&data->settings_lock);
}

static uint32_t iqs915x_request_generation(const struct iqs915x_data *data)
{
  return (uint32_t)atomic_get(&data->request_generation);
}

static bool iqs915x_output_is_enabled(const struct iqs915x_data *data)
{
  return atomic_get(&data->output_enabled) != 0;
}

static bool iqs915x_work_session_is_current(const struct iqs915x_data *data,
                                            uint32_t generation)
{
  return iqs915x_output_is_enabled(data) &&
         generation == iqs915x_request_generation(data);
}

static uint32_t iqs915x_power_retry_backoff_ms(uint8_t retry)
{
  static const uint16_t backoff_ms[] = {20, 50, 100};

  return backoff_ms[MIN(retry, ARRAY_SIZE(backoff_ms) - 1)];
}

static void iqs915x_complete_transition(struct iqs915x_data *data, int result)
{
  /* A rapid request reversal supersedes the old transition. Never wake a
   * synchronous PM caller with a result belonging to that stale generation. */
  if (data->transition_generation != iqs915x_request_generation(data)) {
    return;
  }

  atomic_set(&data->transition_result, result);
  k_sem_give(&data->transition_sem);
}

bool iqs915x_report_event(struct iqs915x_data *data, uint16_t type,
                          uint16_t code, int32_t value, bool sync)
{
  if (!iqs915x_output_is_enabled(data))
  {
    return false;
  }

  return input_report(data->dev, type, code, value, sync, K_FOREVER) == 0;
}

static bool iqs915x_report_key(struct iqs915x_data *data, uint16_t code,
                               int32_t value, bool sync)
{
  if (!iqs915x_output_is_enabled(data))
  {
    return false;
  }

  return input_report_key(data->dev, code, value, sync, K_FOREVER) == 0;
}

static bool iqs915x_report_rel(struct iqs915x_data *data, uint16_t code,
                               int32_t value, bool sync)
{
  if (!iqs915x_output_is_enabled(data))
  {
    return false;
  }

  return input_report_rel(data->dev, code, value, sync, K_FOREVER) == 0;
}

static void iqs915x_report_pointer_pair(struct iqs915x_data *data,
                                        int32_t x, int32_t y)
{
  if (!iqs915x_output_is_enabled(data))
  {
    return;
  }

  (void)iqs915x_report_rel(data, INPUT_REL_X, x, false);
  (void)iqs915x_report_rel(data, INPUT_REL_Y, y, true);
}

static uint16_t iqs915x_apply_config_settings_policy(uint16_t cfg)
{
  cfg |= IQS915X_EVENT_MODE | IQS915X_MANUAL_CONTROL | IQS915X_TP_EVENT;
  cfg |= IQS915X_TP_REATI_ENABLE | IQS915X_REATI_EVENT;
  cfg &= ~(IQS915X_GESTURE_EVENT | IQS915X_TP_TOUCH_EVENT |
           IQS915X_ALP_REATI_ENABLE | IQS915X_ALP_EVENT);
  return cfg;
}

static uint16_t iqs915x_config_settings_without_event_mode(uint16_t cfg)
{
  cfg = iqs915x_apply_config_settings_policy(cfg);
  return cfg & ~IQS915X_EVENT_MODE;
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

  struct iqs915x_data *data = dev->data;
  k_sem_reset(&data->rdy_sem);
  int ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
  data->comm_completed_ms = k_uptime_get();
  return ret;
}

// 16bitレジスタを読み込む
static int iqs915x_read_reg16(const struct device *dev, uint16_t reg,
                              uint16_t *val)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t reg_addr[2] = {reg & 0xFF, reg >> 8};
  uint8_t buf[2];
  k_sem_reset(&((struct iqs915x_data *)dev->data)->rdy_sem);
  int ret = i2c_write_read_dt(&config->i2c, reg_addr, 2, buf, sizeof(buf));
  ((struct iqs915x_data *)dev->data)->comm_completed_ms = k_uptime_get();
  if (ret < 0)
  {
    return ret;
  }
  *val = (buf[1] << 8) | buf[0];
  return 0;
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

#define IQS915X_COORD_LUT_Q15_SCALE BIT(15)


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

  if (raw == UINT16_MAX || (resolution > 0 && raw > resolution))
  {
    return UINT16_MAX;
  }

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

static void iqs915x_correct_stream_coordinates(const struct iqs915x_config *config,
                                               const struct iqs915x_data *driver_data,
                                               struct iqs915x_stream_data *data)
{
  uint16_t res_x = driver_data->swipe_resolution_x;
  uint16_t res_y = driver_data->swipe_resolution_y;

  data->abs_x = iqs915x_correct_axis_coordinate(
      data->abs_x, res_x, config->coord_x_blocks, config->coord_lut_x_q15,
      config->coord_lut_x_len);
  data->finger2_x = iqs915x_correct_axis_coordinate(
      data->finger2_x, res_x, config->coord_x_blocks,
      config->coord_lut_x_q15, config->coord_lut_x_len);
  data->finger3_x = iqs915x_correct_axis_coordinate(
      data->finger3_x, res_x, config->coord_x_blocks,
      config->coord_lut_x_q15, config->coord_lut_x_len);
  data->finger4_x = iqs915x_correct_axis_coordinate(
      data->finger4_x, res_x, config->coord_x_blocks,
      config->coord_lut_x_q15, config->coord_lut_x_len);

  data->abs_y = iqs915x_correct_axis_coordinate(
      data->abs_y, res_y, config->coord_y_blocks, config->coord_lut_y_q15,
      config->coord_lut_y_len);
  data->finger2_y = iqs915x_correct_axis_coordinate(
      data->finger2_y, res_y, config->coord_y_blocks,
      config->coord_lut_y_q15, config->coord_lut_y_len);
  data->finger3_y = iqs915x_correct_axis_coordinate(
      data->finger3_y, res_y, config->coord_y_blocks,
      config->coord_lut_y_q15, config->coord_lut_y_len);
  data->finger4_y = iqs915x_correct_axis_coordinate(
      data->finger4_y, res_y, config->coord_y_blocks,
      config->coord_lut_y_q15, config->coord_lut_y_len);
}

/* Validate before LUT correction so an unused 0xffff slot cannot become an
 * apparently valid coordinate at the edge. Apply this even without correction. */
static void iqs915x_validate_stream_coordinates(const struct iqs915x_data *data,
                                                struct iqs915x_stream_data *stream)
{
  uint16_t *x[] = {&stream->abs_x, &stream->finger2_x,
                   &stream->finger3_x, &stream->finger4_x};
  uint16_t *y[] = {&stream->abs_y, &stream->finger2_y,
                   &stream->finger3_y, &stream->finger4_y};

  for (uint8_t slot = 0; slot < ARRAY_SIZE(x); slot++)
  {
    if (*x[slot] == UINT16_MAX || *y[slot] == UINT16_MAX ||
        (data->swipe_resolution_x > 0 && *x[slot] > data->swipe_resolution_x) ||
        (data->swipe_resolution_y > 0 && *y[slot] > data->swipe_resolution_y))
    {
      *x[slot] = UINT16_MAX;
      *y[slot] = UINT16_MAX;
    }
  }
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
  k_sem_reset(&((struct iqs915x_data *)dev->data)->rdy_sem);
  ret = i2c_write_read_dt(&config->i2c, reg_addr, 2, buf, sizeof(buf));
  ((struct iqs915x_data *)dev->data)->comm_completed_ms = k_uptime_get();
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
  iqs915x_validate_stream_coordinates(dev->data, data);
  for (uint8_t slot = 0; slot < IQS915X_OBSERVED_FINGERS; slot++) {
    uint16_t x = UINT16_MAX, y = UINT16_MAX;
    bool valid = iqs915x_get_finger_coordinates(data, slot, &x, &y);
    uint8_t count = data->trackpad_flags & IQS915X_NUM_FINGERS_MASK;
    data->raw_point[slot] = (struct iqs915x_stuck_point){
        .valid = valid && count > 0 && count <= IQS915X_MAX_FINGERS,
        .x = x, .y = y,
    };
  }
  if (config->coordinate_correction)
  {
    iqs915x_correct_stream_coordinates(config, dev->data, data);
  }

  return 0;
}

static void iqs915x_reset_pointer_accumulators(struct iqs915x_data *data)
{
  k_mutex_lock(&data->settings_lock, K_FOREVER);
  data->pointer_x_acc = 0;
  data->pointer_y_acc = 0;
  k_mutex_unlock(&data->settings_lock);
}

static void iqs915x_reset_absolute_tracking(struct iqs915x_data *data)
{
  data->last_abs_x = 0;
  data->last_abs_y = 0;
  data->last_abs_valid = false;
  data->pointer_slot = UINT8_MAX;
  iqs915x_reset_pointer_accumulators(data);
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

bool iqs915x_absolute_delta_is_discontinuity(
    const struct iqs915x_data *data, int32_t rel_x, int32_t rel_y)
{
  uint16_t threshold = iqs915x_absolute_discontinuity_threshold(data);

  return abs(rel_x) > threshold || abs(rel_y) > threshold;
}

uint32_t iqs915x_axis_movement(int32_t dx, int32_t dy)
{
  return (uint32_t)MAX(abs(dx), abs(dy));
}

static uint32_t iqs915x_pointer_speed_10ms(
    const struct iqs915x_config *config, int32_t rel_x, int32_t rel_y)
{
  uint32_t speed = iqs915x_axis_movement(rel_x, rel_y);
  uint32_t report_rate_ms =
      config->report_rate_ms > 0 ? config->report_rate_ms : 10U;

  return DIV_ROUND_CLOSEST(speed * 10U, report_rate_ms);
}

static uint16_t iqs915x_pointer_scale_percent(
    const struct iqs915x_config *config,
    const struct iqs915x_pointer_settings *settings,
    int32_t rel_x, int32_t rel_y)
{
  uint32_t base_percent = settings->sensitivity_percent;
  uint32_t max_percent = settings->max_percent;
  uint32_t threshold = settings->threshold;
  uint32_t saturation = settings->saturation;
  uint32_t speed;
  uint32_t scale;

  if (!settings->enabled || max_percent <= base_percent)
  {
    return (uint16_t)base_percent;
  }

  speed = iqs915x_pointer_speed_10ms(config, rel_x, rel_y);
  if (speed <= threshold)
  {
    return (uint16_t)base_percent;
  }

  if (saturation <= threshold || speed >= saturation)
  {
    return (uint16_t)max_percent;
  }

  scale = base_percent +
          ((max_percent - base_percent) * (speed - threshold)) /
              (saturation - threshold);

  return (uint16_t)scale;
}

static int32_t iqs915x_apply_pointer_scale_axis(
    int32_t delta, uint16_t scale_percent, int32_t *remainder)
{
  int32_t scaled = delta * (int32_t)scale_percent + *remainder;
  int32_t output = scaled / 100;

  *remainder = scaled - (output * 100);

  return output;
}

static uint16_t iqs915x_apply_pointer_scale(
    const struct iqs915x_config *config, struct iqs915x_data *data,
    int32_t raw_x, int32_t raw_y, int32_t *out_x, int32_t *out_y)
{
  /* Caller holds settings_lock across scaling and the corresponding report. */
  uint16_t scale_percent;
  const struct iqs915x_pointer_settings *settings =
      &data->runtime_settings.pointer;

  if (!settings->enabled && settings->sensitivity_percent == 100U)
  {
    *out_x = raw_x;
    *out_y = raw_y;
    return 100U;
  }

  scale_percent = iqs915x_pointer_scale_percent(config, settings, raw_x, raw_y);
  *out_x = iqs915x_apply_pointer_scale_axis(
      raw_x, scale_percent, &data->pointer_x_acc);
  *out_y = iqs915x_apply_pointer_scale_axis(
      raw_y, scale_percent, &data->pointer_y_acc);

  return scale_percent;
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


/* ============================================================
 * Power mode control
 *
 * Manual Control有効時は、ホストがSystem ControlのMode Selectで
 * Active/LP2を切り替える。イベントモード中はRDY待ちのない期間も
 * あるため、Event Modeを離れる最初の操作はForce Commsで進める。
 * Streaming中の操作はRDYを待ち、3 sampling periodsでフォールバックする。
 * ============================================================ */

/* All runtime control steps consume one communication window. Streaming reads
 * are RDY driven; Force Comms is only used after 3T, or to leave Event Mode. */
static void iqs915x_clear_stuck(struct iqs915x_data *data, const char *reason)
{
  for (unsigned int i = 0; i < IQS915X_OBSERVED_FINGERS; i++)
  {
    const struct iqs915x_stuck_candidate *c = &data->stuck.candidate[i];
    if (c->active)
    {
      LOG_INF("stuck end id=%u slot=%u reason=%s", c->id, c->slot, reason);
    }
  }
  iqs915x_stuck_clear(&data->stuck);
  data->stuck_remap = false;
  data->stuck_probe_after_ms = 0;
}

static void iqs915x_reseed_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data = CONTAINER_OF(dwork, struct iqs915x_data, reseed_work);
  atomic_clear(&data->reseed_timer_armed);
  if (!atomic_get(&data->pm_suspended) && !atomic_get(&data->requested_enabled))
  {
    atomic_set(&data->reseed_due, 1);
    k_sem_give(&data->rdy_sem);
  }
}

void iqs915x_schedule_lp2_reseed(struct iqs915x_data *data)
{
  /* Returning from a coordinate probe must preserve both a due request and
   * the original timer. Contact postponement is retried on the next sample. */
  if (data->initialized && !atomic_get(&data->requested_enabled) &&
      !atomic_get(&data->pm_suspended) && !atomic_get(&data->reseed_due) &&
      atomic_cas(&data->reseed_timer_armed, 0, 1))
  {
    k_work_reschedule(&data->reseed_work, K_MSEC(IQS915X_LP2_RESEED_INTERVAL_MS));
  }
}

static void iqs915x_runtime_reset(struct iqs915x_data *data, const char *reason)
{
  atomic_clear(&data->output_enabled);
  data->enabled = false;
  iqs915x_reset_input_session(data);
  iqs915x_clear_stuck(data, reason);
  k_work_cancel_delayable(&data->reseed_work);
  atomic_clear(&data->reseed_timer_armed);
  atomic_clear(&data->reseed_due);
  data->reseed_state = RESEED_IDLE;
  data->no_touch_scans = 0;
  data->power_retry_count = 0;
  data->power_force_comms = false;
  data->work_state = WORK_READ_DATA;
  data->active_pending = false;
  data->lp2_pending = false;
  data->ati_error_seen = false;
  data->runtime_busy_count = 0;
  data->comm_fallback_active = true;
  iqs915x_mark_initialized(data, false);
  data->init_step = INIT_CHECK_SHOW_RESET;
  data->init_data_offset = 0;
  data->wait_count = 0;
  data->init_chunk_retry_count = 0;
  data->init_restart_count = 0;
  data->init_pending_cfg = 0;
  data->confirmed_config_settings = 0;
  iqs915x_complete_transition(data, -EIO);
  LOG_WRN("communication reset reason=%s", reason);
}

static bool iqs915x_runtime_info(struct iqs915x_data *data, uint16_t info)
{
  if (info == 0xEEEE)
  {
    data->no_touch_scans = 0;
    if (++data->runtime_busy_count >= IQS915X_INIT_REATI_MAX_WAIT)
    {
      iqs915x_runtime_reset(data, "busy-timeout");
    }
    return false;
  }
  data->runtime_busy_count = 0;
  if (info & IQS915X_SHOW_RESET)
  {
    iqs915x_runtime_reset(data, "show-reset");
    return false;
  }
  bool reati = (info & IQS915X_REATI_OCCURRED) != 0;
  bool error = (info & IQS915X_ATI_ERROR) != 0;
  if (error != data->ati_error_seen || (error && reati))
  {
    if (error)
    {
      LOG_WRN("reati error=1 flags=0x%04x retry_s=1 t=%lld", info,
              (long long)k_uptime_get());
    }
    else
    {
      LOG_INF("reati error=0 flags=0x%04x t=%lld", info, (long long)k_uptime_get());
    }
  }
  data->ati_error_seen = error;
  if (reati)
  {
    bool reopen = iqs915x_output_is_enabled(data);
    uint32_t generation = iqs915x_request_generation(data);
    atomic_clear(&data->output_enabled);
    iqs915x_reset_input_session(data);
    iqs915x_clear_stuck(data, "reati");
    data->no_touch_scans = 0;
    data->pointer_resume_guard_frames = IQS915X_POINTER_RESUME_GUARD_FRAMES;
    if (reopen && generation == iqs915x_request_generation(data) &&
        atomic_get(&data->requested_enabled) && !atomic_get(&data->pm_suspended))
    {
      atomic_set(&data->output_enabled, 1);
    }
    LOG_INF("reati occurred flags=0x%04x mode=%u enabled=%u t=%lld", info,
            info & IQS915X_CHARGING_MODE_MASK,
            (unsigned int)atomic_get(&data->requested_enabled),
            (long long)k_uptime_get());
  }
  data->last_info_flags = info;
  return true;
}

static void iqs915x_comm_failure(struct iqs915x_data *data, int ret)
{
  data->power_retry_count++;
  data->no_touch_scans = 0;
  LOG_WRN("communication failure state=%u reseed=%u retry=%u rc=%d",
          data->work_state, data->reseed_state, data->power_retry_count, ret);
  if (data->power_retry_count >= IQS915X_POWER_TRANSITION_MAX_RETRIES)
  {
    iqs915x_runtime_reset(data, "communication-retries-exhausted");
  }
  else
  {
    k_sleep(K_MSEC(iqs915x_power_retry_backoff_ms(data->power_retry_count - 1)));
  }
}

static void iqs915x_begin_mode(struct iqs915x_data *data, uint16_t mode, bool output)
{
  atomic_clear(&data->output_enabled);
  data->enabled = false;
  data->power_target_mode = mode;
  data->relatch_target_enabled = output;
  data->transition_generation = iqs915x_request_generation(data);
  data->power_retry_count = 0;
  data->power_force_comms = output;
  /* LP2 already has a verified Streaming configuration. Enabling output must
   * not spend two additional LP2 periods rewriting and checking that value.
   * Maintenance transitions retain their RDY-driven verification sequence. */
  bool streaming_confirmed = data->streaming_expected &&
      data->confirmed_config_settings == iqs915x_config_settings_without_event_mode(
          data->confirmed_config_settings);
  data->work_state = output && streaming_confirmed ? WORK_SET_POWER : WORK_SET_STREAMING;
}

static void iqs915x_restore_mode(struct iqs915x_data *data)
{
  bool output = atomic_get(&data->requested_enabled) != 0;
  iqs915x_begin_mode(data, output ? IQS915X_MODE_ACTIVE : IQS915X_MODE_LP2, output);
}

static uint32_t iqs915x_sampling_period(const struct iqs915x_data *data)
{
  uint32_t period = data->confirmed_mode == IQS915X_MODE_LP2 ?
                    IQS915X_LP2_SAMPLING_PERIOD_MS : data->active_sampling_period_ms;
  if (data->work_state != WORK_READ_DATA && data->power_target_mode == IQS915X_MODE_LP2)
  {
    period = MAX(period, IQS915X_LP2_SAMPLING_PERIOD_MS);
  }
  return period;
}

/* Return false on a timer/API wake without an asserted RDY. Such wakes must
 * neither count as samples nor extend the watchdog's last-STOP deadline. */
static bool iqs915x_wait_window(struct iqs915x_data *data, bool force, int64_t timer)
{
  const struct iqs915x_config *config = data->dev->config;
  int64_t watchdog = data->comm_completed_ms + 3LL * iqs915x_sampling_period(data);
  if (data->applied_generation != iqs915x_request_generation(data)) { return false; }
  if (force) { return true; }
  for (;;)
  {
    if (data->applied_generation != iqs915x_request_generation(data)) { return false; }
    if (gpio_pin_get_dt(&config->rdy_gpio) > 0)
    {
      k_sem_reset(&data->rdy_sem);
      if (data->comm_fallback_active)
      {
        LOG_INF("communication recovered mode=%u t=%lld", data->confirmed_mode,
                (long long)k_uptime_get());
        data->comm_fallback_active = false;
      }
      return true;
    }
    int64_t now = k_uptime_get();
    if (data->streaming_expected && now >= watchdog)
    {
      LOG_WRN("communication fallback mode=%u period_ms=%u elapsed_ms=%lld",
              data->confirmed_mode, iqs915x_sampling_period(data),
              (long long)(now - data->comm_completed_ms));
      data->comm_fallback_active = true;
      return true;
    }
    if (now >= timer) { return false; }
    int64_t deadline = data->streaming_expected ? MIN(timer, watchdog) : timer;
    k_timeout_t wait = deadline == INT64_MAX ? K_FOREVER : K_MSEC(MAX(0LL, deadline - now));
    k_sem_take(&data->rdy_sem, wait);
    /* The GPIO, not an API/timer semaphore token, identifies a fresh window. */
  }
}

static void iqs915x_mode_step(struct iqs915x_data *data)
{
  const struct device *dev = data->dev;
  uint16_t cfg = iqs915x_apply_config_settings_policy(data->confirmed_config_settings);
  uint16_t value = 0;
  int ret = 0;
  bool event = data->work_state == WORK_SET_EVENT_MODE ||
               data->work_state == WORK_CONFIRM_EVENT_MODE;
  if (!event) { cfg &= ~IQS915X_EVENT_MODE; }
  switch (data->work_state)
  {
  case WORK_SET_STREAMING:
  case WORK_SET_EVENT_MODE:
    ret = iqs915x_write_reg16(dev, IQS915X_CONFIG_SETTINGS, cfg);
    if (!ret)
    {
      data->streaming_expected = !event;
      data->work_state = event ? WORK_CONFIRM_EVENT_MODE : WORK_CONFIRM_STREAMING;
    }
    break;
  case WORK_CONFIRM_STREAMING:
  case WORK_CONFIRM_EVENT_MODE:
    ret = iqs915x_read_reg16(dev, IQS915X_CONFIG_SETTINGS, &value);
    if (!ret && value != cfg)
    {
      LOG_WRN("communication config mismatch expected=0x%04x actual=0x%04x", cfg, value);
      data->work_state = event ? WORK_SET_EVENT_MODE : WORK_SET_STREAMING;
      ret = -EIO;
    }
    else if (!ret)
    {
      data->confirmed_config_settings = value;
      if (!event)
      {
        data->work_state = WORK_SET_POWER;
      }
      else
      {
        data->work_state = WORK_READ_DATA;
        if (data->transition_generation == iqs915x_request_generation(data) &&
            atomic_get(&data->requested_enabled) && !atomic_get(&data->pm_suspended))
        {
          data->enabled = true;
          data->pointer_resume_guard_frames = IQS915X_POINTER_RESUME_GUARD_FRAMES;
          atomic_set(&data->output_enabled, 1);
          iqs915x_complete_transition(data, 0);
        }
      }
      LOG_INF("communication config confirmed mode=%u event=%u enabled=%u",
              data->confirmed_mode, event, (unsigned int)atomic_get(&data->requested_enabled));
    }
    break;
  case WORK_SET_POWER:
    ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL, data->power_target_mode);
    if (!ret) { data->work_state = WORK_CONFIRM_POWER; }
    break;
  case WORK_CONFIRM_POWER:
    ret = iqs915x_read_reg16(dev, IQS915X_INFO_FLAGS, &value);
    if (!ret && !iqs915x_runtime_info(data, value)) { return; }
    if (!ret && (value & IQS915X_CHARGING_MODE_MASK) != data->power_target_mode)
    {
      data->work_state = WORK_SET_POWER;
      ret = -EIO;
    }
    else if (!ret)
    {
      if (data->confirmed_mode == IQS915X_MODE_LP2 && data->power_target_mode == IQS915X_MODE_ACTIVE)
      {
        data->stuck_remap = true;
      }
      data->confirmed_mode = data->power_target_mode;
      data->active_pending = false;
      data->lp2_pending = false;
      data->work_state = data->relatch_target_enabled ? WORK_SET_EVENT_MODE : WORK_READ_DATA;
      if (data->confirmed_mode == IQS915X_MODE_LP2)
      {
        iqs915x_schedule_lp2_reseed(data);
        iqs915x_complete_transition(data, 0);
      }
      LOG_INF("communication mode confirmed mode=%u output=%u t=%lld",
              data->confirmed_mode, data->relatch_target_enabled, (long long)k_uptime_get());
    }
    break;
  default:
    return;
  }
  if (ret < 0) { iqs915x_comm_failure(data, ret); }
  else if (data->work_state == WORK_READ_DATA || data->work_state == WORK_SET_POWER)
  {
    data->power_retry_count = 0;
  }
}

static uint8_t iqs915x_observe_stuck(struct iqs915x_data *data,
                                    const struct iqs915x_stream_data *stream)
{
  struct iqs915x_stuck_candidate before[IQS915X_OBSERVED_FINGERS];
  memcpy(before, data->stuck.candidate, sizeof(before));
  int64_t now = k_uptime_get();
  uint8_t mature = iqs915x_stuck_observe(&data->stuck, stream->raw_point, now, data->stuck_remap);
  data->stuck_remap = false;
  for (unsigned int i = 0; i < IQS915X_OBSERVED_FINGERS; i++)
  {
    const struct iqs915x_stuck_candidate *c = &data->stuck.candidate[i];
    if (before[i].active && (!c->active || c->id != before[i].id))
    {
      LOG_DBG("stuck end id=%u reason=movement-or-missing", before[i].id);
    }
    if (c->active && !before[i].active)
    {
      LOG_INF("stuck start id=%u slot=%u x=%u y=%u threshold=%u t=%lld",
              c->id, c->slot, c->last_x, c->last_y, data->stuck.threshold, (long long)now);
    }
    if (c->active)
    {
      LOG_DBG("stuck sample id=%u slot=%u range_x=%u range_y=%u elapsed_ms=%lld mode=%u",
              c->id, c->slot, c->max_x - c->min_x, c->max_y - c->min_y,
              (long long)(now - c->since_ms), data->confirmed_mode);
    }
    if (mature & BIT(i))
    {
      LOG_INF("stuck mature id=%u slot=%u range_x=%u range_y=%u elapsed_ms=%lld threshold=%u",
              c->id, c->slot, c->max_x - c->min_x, c->max_y - c->min_y,
              (long long)(now - c->since_ms), data->stuck.threshold);
    }
  }
  return mature;
}

static void iqs915x_begin_reseed(struct iqs915x_data *data, bool forced)
{
  data->reseed_forced = forced;
  data->reseed_id++;
  atomic_clear(&data->output_enabled);
  iqs915x_reset_input_session(data);
  data->reseed_state = RESEED_ISSUE_TP_RESEED;
  if (!data->streaming_expected)
  {
    iqs915x_begin_mode(data, IQS915X_MODE_ACTIVE, false);
  }
  LOG_INF("reseed prepare id=%u forced=%u enabled=%u t=%lld", data->reseed_id,
          forced, (unsigned int)atomic_get(&data->requested_enabled), (long long)k_uptime_get());
}

/* Called only with a fresh status/sample, never with a debounce snapshot. */
static bool iqs915x_maintenance_sample(struct iqs915x_data *data,
                                      const struct iqs915x_stream_data *stream)
{
  uint16_t info = stream->info_flags;
  bool touch = (info & IQS915X_GLOBAL_TP_TOUCH) != 0;
  bool suspended = atomic_get(&data->pm_suspended) != 0;
  if (!iqs915x_runtime_info(data, info)) { return false; }
  if (data->reseed_state == RESEED_WAIT_TP_SCAN)
  {
    if ((info & IQS915X_CHARGING_MODE_MASK) != IQS915X_MODE_ACTIVE)
    {
      iqs915x_comm_failure(data, -EIO);
      return false;
    }
    LOG_INF("reseed scan id=%u flags=0x%04x reati=%u ati_error=%u t=%lld",
            data->reseed_id, info, (info & IQS915X_REATI_OCCURRED) != 0,
            (info & IQS915X_ATI_ERROR) != 0, (long long)k_uptime_get());
    /* A missing Re-ATI flag does not prove that reference drift was small. */
    iqs915x_clear_stuck(data, "reseed-complete");
    data->reseed_state = RESEED_IDLE;
    data->power_retry_count = 0;
    atomic_clear(&data->reseed_due);
    iqs915x_restore_mode(data);
    return false;
  }
  if (suspended) { return false; }
  if ((info & IQS915X_CHARGING_MODE_MASK) != data->confirmed_mode)
  {
    iqs915x_comm_failure(data, -EIO);
    return false;
  }
  data->power_retry_count = 0;
  if (data->confirmed_mode == IQS915X_MODE_LP2)
  {
    bool watch = iqs915x_stuck_deadline(&data->stuck) != INT64_MAX;
    if (!touch) { iqs915x_clear_stuck(data, "lp2-no-touch"); }
    int64_t now = k_uptime_get();
    bool due = atomic_get(&data->reseed_due) != 0;
    if ((!touch && (due || watch)) ||
        (touch && (due || watch) && now >= data->stuck_probe_after_ms &&
         (!watch || now >= iqs915x_stuck_deadline(&data->stuck))))
    {
      LOG_INF("reseed probe touch=%u due=%u t=%lld", touch, due, (long long)now);
      data->no_touch_scans = 0;
      data->reseed_state = RESEED_OBSERVE_ACTIVE;
      iqs915x_begin_mode(data, IQS915X_MODE_ACTIVE, false);
    }
    return false;
  }
  if (info & IQS915X_REATI_OCCURRED) { return false; }
  uint8_t mature = iqs915x_observe_stuck(data, stream);
  if (mature)
  {
    iqs915x_begin_reseed(data, true);
    return false;
  }
  if (data->reseed_state == RESEED_OBSERVE_ACTIVE)
  {
    uint8_t fingers = stream->trackpad_flags & IQS915X_NUM_FINGERS_MASK;
    if (!touch && fingers == 0)
    {
      if (++data->no_touch_scans >= 4) { iqs915x_begin_reseed(data, false); }
      LOG_DBG("reseed no-touch scans=%u", data->no_touch_scans);
    }
    else
    {
      LOG_INF("reseed postponed touch=%u fingers=%u flags=0x%04x", touch, fingers, info);
      data->reseed_state = RESEED_IDLE;
      data->stuck_probe_after_ms = k_uptime_get() + IQS915X_STUCK_TIME_MS;
      iqs915x_restore_mode(data);
    }
    return false;
  }
  return !(info & IQS915X_REATI_OCCURRED);
}

/* ============================================================
 * ボタンリリース遅延処理
 * ============================================================ */
static void iqs915x_button_release_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, button_release_work);

  if (!iqs915x_work_session_is_current(data,
                                       data->button_work_generation))
  {
    return;
  }

  for (int i = 0; i < 3; i++)
  {
    if (data->buttons_pressed & BIT(i))
    {
      // ドラッグ中（active_tap_hold）の場合は左クリック(i=0)の離上をスキップ
      if (i == 0 && data->active_tap_hold)
      {
        continue;
      }
      if (iqs915x_report_key(data, INPUT_BTN_0 + i, 0, true))
      {
        data->buttons_pressed &= ~BIT(i);
      }
    }
  }
}

static void iqs915x_report_button_tap(struct iqs915x_data *data,
                                      uint16_t button_code)
{
  k_work_cancel_delayable(&data->button_release_work);
  if (!iqs915x_report_key(data, button_code, 1, true))
  {
    return;
  }
  data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);
  data->button_work_generation = iqs915x_request_generation(data);
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
    if (iqs915x_report_key(data, button_code, 0, true))
    {
      data->buttons_pressed &= ~button_bit;
    }
  }

  if (!iqs915x_report_key(data, button_code, 1, true))
  {
    return;
  }
  iqs915x_report_key(data, button_code, 0, true);
  if (!iqs915x_report_key(data, button_code, 1, true))
  {
    return;
  }
  data->buttons_pressed |= button_bit;
  data->button_work_generation = iqs915x_request_generation(data);
  k_work_schedule(&data->button_release_work,
                  K_MSEC(IQS915X_BUTTON_TAP_RELEASE_MS));
}

static void iqs915x_start_tap_and_hold_drag(struct iqs915x_data *data,
                                            const char *reason)
{
  if (data->active_tap_hold || data->tap_drag_raw_max_fingers != 1 ||
      data->finger_tracker.stable_count != 1)
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
  if (!iqs915x_report_key(data, LEFT_BUTTON_CODE, 1, true))
  {
    data->active_tap_hold = false;
    return;
  }
  data->buttons_pressed |= BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
  LOG_DBG("tap-and-drag started: %s", reason);
}


#define IQS915X_SCROLL_UNITS_PER_AXIS 512
#define IQS915X_SCROLL_FALLBACK_RESOLUTION 4096
#define IQS915X_SCROLL_CROSS_AXIS_DEADBAND_RATIO 4

static void iqs915x_filter_scroll_cross_axis(int32_t *x, int32_t *y)
{
  int64_t abs_x = llabs(*x);
  int64_t abs_y = llabs(*y);

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
                                                int64_t *accumulator,
                                                uint16_t code, int32_t delta,
                                                uint16_t resolution,
                                                uint16_t divisor,
                                                const char *source)
{
  int32_t output = 0;
  int result = 0;
  const char *status = "buffered";
  int64_t acc_before = *accumulator;
  int64_t denom =
      (int64_t)(resolution > 0 ? resolution : IQS915X_SCROLL_FALLBACK_RESOLUTION) *
      (int64_t)(divisor > 0 ? divisor : 1);

  *accumulator += (int64_t)delta * IQS915X_SCROLL_UNITS_PER_AXIS;
  int64_t acc_added = *accumulator;
  if (*accumulator >= denom || *accumulator <= -denom)
  {
    output = (int32_t)CLAMP(*accumulator / denom, INT32_MIN, INT32_MAX);
    if (!iqs915x_output_is_enabled(data))
    {
      status = "disabled";
      result = -EACCES;
    }
    else
    {
      result = input_report_rel(data->dev, code, output, true, K_FOREVER);
      status = result == 0 ? "sent" : "failed";
      if (result == 0)
      {
        *accumulator -= (int64_t)output * denom;
      }
    }
  }

  /* This records driver submission, not delivery to the central or host. */
  LOG_DBG("scroll_output,t=%lld,source=%s,axis=%s,delta=%d,wheel=%d,"
          "status=%s,rc=%d,acc_before=%lld,acc_added=%lld,acc_after=%lld,denom=%lld",
          (long long)k_uptime_get(), source,
          code == INPUT_REL_WHEEL ? "wheel" : "hwheel", delta, output,
          status, result, (long long)acc_before, (long long)acc_added,
          (long long)*accumulator, (long long)denom);
  return output != 0 && result == 0;
}

static bool iqs915x_handle_two_finger_scroll(
    const struct iqs915x_config *config, struct iqs915x_data *data,
    const struct iqs915x_stream_data *stream)
{
  struct iqs915x_two_finger_session *two_finger = &data->two_finger;
  int32_t gx;
  int32_t gy;
  int32_t motion_x;
  int32_t motion_y;
  bool emitted = false;
  bool started_scroll = false;

  if (!config->scroll || data->finger_tracker.stable_count != 2 ||
      !two_finger->active || !two_finger->frame_valid ||
      data->scroll_blocked_until_low_contact ||
      (stream->trackpad_flags & IQS915X_NUM_FINGERS_MASK) != 2)
  {
    return false;
  }

  k_mutex_lock(&data->settings_lock, K_FOREVER);

  motion_x = iqs915x_clamp_i16(two_finger->centroid_dx);
  motion_y = iqs915x_clamp_i16(two_finger->centroid_dy);
  iqs915x_filter_scroll_cross_axis(&motion_x, &motion_y);

  if (two_finger->reset_velocity)
  {
    /* Recovered movement spans multiple reports; never use it as a velocity. */
    iqs915x_stop_scroll_inertia_locked(data);
  }
  if (data->runtime_settings.scroll_inertia.enabled)
  {
    /* Track the valid baseline and motion before scroll recognition too.
     * Recovered/rebaselined coordinates establish a new zero-motion baseline. */
    iqs915x_motion_history_add(&data->scroll_motion_history, k_uptime_get(),
                              two_finger->reset_velocity ? 0 : motion_x,
                              two_finger->reset_velocity ? 0 : motion_y);
  }

  if (two_finger->mode != IQS915X_2F_MODE_SCROLL)
  {
    if (two_finger->max_centroid_movement < data->tap_distance)
    {
      k_mutex_unlock(&data->settings_lock);
      return false;
    }

    two_finger->mode = IQS915X_2F_MODE_SCROLL;
    data->scroll_sequence_active = true;
    data->tap_drag_raw_gesture_seen = true;
    started_scroll = true;
    LOG_DBG("scroll started from centroid movement: max=%u threshold=%u",
            two_finger->max_centroid_movement, data->tap_distance);
  }

  if (started_scroll)
  {
    gx = iqs915x_clamp_i16(two_finger->pending_dx);
    gy = iqs915x_clamp_i16(two_finger->pending_dy);
    two_finger->pending_dx = 0;
    two_finger->pending_dy = 0;
  }
  else
  {
    gx = motion_x;
    gy = motion_y;
  }

  iqs915x_filter_scroll_cross_axis(&gx, &gy);

  if (gx == 0 && gy == 0)
  {
    k_mutex_unlock(&data->settings_lock);
    return true;
  }

  if (gx != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->scroll_x_acc, INPUT_REL_HWHEEL, gx,
        data->swipe_resolution_x, config->scroll_divisor, "manual");
  }

  if (gy != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->scroll_y_acc, INPUT_REL_WHEEL, gy,
        data->swipe_resolution_y, config->scroll_divisor, "manual");
  }

  if (emitted)
  {
    data->scroll_inertia_state.last_output_ms = k_uptime_get();
  }

  LOG_DBG("scroll centroid: dx=%d dy=%d flags=0x%04x", gx, gy,
          stream->trackpad_flags);
  k_mutex_unlock(&data->settings_lock);
  return true;
}

static void iqs915x_single_tap_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, single_tap_work);

  if (!iqs915x_work_session_is_current(
          data, data->single_tap_work_generation))
  {
    return;
  }

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

  if (!iqs915x_work_session_is_current(
          data, data->tap_and_hold_start_work_generation))
  {
    return;
  }

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

  if (!iqs915x_work_session_is_current(
          data, data->tap_and_hold_release_work_generation))
  {
    return;
  }

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
  if (iqs915x_report_key(data, LEFT_BUTTON_CODE, 0, true))
  {
    data->buttons_pressed &= ~BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
  }
  LOG_DBG("tap-and-hold release timeout fired: drag released");
}

/* ============================================================
 * スクロール慣性処理
 *
 * 指が離れた後、直近100 msの平均スクロール速度に基づいて減衰しながら
 * スクロール信号を送り続ける。タイマーの各ティックで速度に
 * 減衰率（friction）を掛けて減速し、閾値未満になったら停止する。
 * ============================================================ */

static void iqs915x_stop_scroll_inertia_locked(struct iqs915x_data *data)
{
  k_work_cancel_delayable(&data->scroll_inertia_work);
  memset(&data->scroll_inertia_state, 0, sizeof(data->scroll_inertia_state));
  iqs915x_motion_history_reset(&data->scroll_motion_history);
  data->inertia_scroll_x_acc = 0;
  data->inertia_scroll_y_acc = 0;
}

static void iqs915x_reset_scroll_inertia(struct iqs915x_data *data)
{
  k_mutex_lock(&data->settings_lock, K_FOREVER);
  iqs915x_stop_scroll_inertia_locked(data);
  data->scroll_x_acc = 0;
  data->scroll_y_acc = 0;
  data->scroll_contact_fingers = 0;
  k_mutex_unlock(&data->settings_lock);
}

static void iqs915x_update_scroll_contact(
    struct iqs915x_data *data,
    uint8_t num_fingers, uint8_t raw_fingers, int64_t release_ms,
    bool released_scroll_sequence)
{
  struct iqs915x_scroll_inertia_state *state =
      &data->scroll_inertia_state;
  const struct iqs915x_scroll_inertia_settings *profile;
  uint8_t previous_fingers;
  int64_t now_ms = k_uptime_get();

  k_mutex_lock(&data->settings_lock, K_FOREVER);
  previous_fingers = data->finger_tracker.previous_count;
  /* Raw contact cancels inertia immediately, even before count confirmation. */
  data->scroll_contact_fingers = raw_fingers;
  profile = &data->runtime_settings.scroll_inertia;

  if (num_fingers > 0)
  {
    if (state->active)
    {
      iqs915x_stop_scroll_inertia_locked(data);
    }

    if (num_fingers >= 3)
    {
      iqs915x_stop_scroll_inertia_locked(data);
      data->scroll_x_acc = 0;
      data->scroll_y_acc = 0;
    }

    k_mutex_unlock(&data->settings_lock);
    return;
  }

  if (previous_fingers == 0)
  {
    k_mutex_unlock(&data->settings_lock);
    return;
  }

  int16_t velocity_x;
  int16_t velocity_y;
  uint16_t elapsed_ms;
  bool moving = iqs915x_motion_history_velocity(
      &data->scroll_motion_history, release_ms,
      &velocity_x, &velocity_y, &elapsed_ms);
  bool fast_enough = MAX(abs(velocity_x), abs(velocity_y)) >=
                     profile->threshold_start;
  bool start = released_scroll_sequence && profile->enabled && moving && fast_enough;

  LOG_DBG("scroll_inertia_release,t=%lld,window_ms=%u,vx=%d,vy=%d,"
          "threshold=%u,initial_velocity_percent=%u,start=%u",
          (long long)release_ms, elapsed_ms, velocity_x, velocity_y,
          profile->threshold_start, profile->initial_velocity_percent, start);

  if (start)
  {
    const struct iqs915x_config *config = data->dev->config;
    iqs915x_inertia_motion_init(&state->motion, velocity_x, velocity_y,
                                profile->initial_velocity_percent,
                                profile->decay_factor_percent,
                                MAX(1, config->scroll_inertia.interval_ms));
    state->active = true;
    state->is_inertial = false;
    state->last_ms = 0;
    state->last_output_ms = 0;
    data->inertia_scroll_x_acc = data->scroll_x_acc;
    data->inertia_scroll_y_acc = data->scroll_y_acc;
    data->scroll_x_acc = 0;
    data->scroll_y_acc = 0;
    data->scroll_inertia_work_generation =
        iqs915x_request_generation(data);
    k_work_reschedule(&data->scroll_inertia_work,
                      K_MSEC(MAX(0LL, release_ms + profile->trigger_ms - now_ms)));
  }
  else
  {
    iqs915x_stop_scroll_inertia_locked(data);
    data->scroll_x_acc = 0;
    data->scroll_y_acc = 0;
  }

  iqs915x_motion_history_reset(&data->scroll_motion_history);
  k_mutex_unlock(&data->settings_lock);
}

static void iqs915x_scroll_inertia_work_handler(struct k_work *work)
{
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct iqs915x_data *data =
      CONTAINER_OF(dwork, struct iqs915x_data, scroll_inertia_work);
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  struct iqs915x_scroll_inertia_state *state = &data->scroll_inertia_state;
  int32_t step_x;
  int32_t step_y;
  bool emitted = false;
  int64_t now_ms;
  uint16_t next_delay_ms;
  const struct iqs915x_scroll_inertia_settings *profile;

  k_mutex_lock(&data->settings_lock, K_FOREVER);
  profile = &data->runtime_settings.scroll_inertia;

  if (!iqs915x_work_session_is_current(
          data, data->scroll_inertia_work_generation) ||
      !state->active || !profile->enabled)
  {
    k_mutex_unlock(&data->settings_lock);
    return;
  }

  if (data->scroll_contact_fingers != 0)
  {
    iqs915x_stop_scroll_inertia_locked(data);
    k_mutex_unlock(&data->settings_lock);
    return;
  }

  now_ms = k_uptime_get();
  if (!state->is_inertial)
  {
    state->started_ms = now_ms;
    state->last_ms = now_ms;
    state->last_output_ms = now_ms;
    state->is_inertial = true;
    /* Start the time integrator; a 1 ms wake avoids a full report-period pause. */
    k_work_reschedule(&data->scroll_inertia_work, K_MSEC(1));
    k_mutex_unlock(&data->settings_lock);
    return;
  }
  else if (profile->max_duration_ms > 0 &&
           state->last_ms - state->started_ms >= profile->max_duration_ms)
  {
    iqs915x_stop_scroll_inertia_locked(data);
    LOG_DBG("Scroll inertia stopped (maximum duration reached)");
    k_mutex_unlock(&data->settings_lock);
    return;
  }

  int64_t elapsed_ms = now_ms - state->last_ms;
  if (elapsed_ms <= 0)
  {
    k_work_reschedule(&data->scroll_inertia_work, K_MSEC(1));
    k_mutex_unlock(&data->settings_lock);
    return;
  }
  /* A delayed worker must not integrate past an explicit duration limit. */
  if (profile->max_duration_ms > 0)
  {
    elapsed_ms = MIN(elapsed_ms, state->started_ms + profile->max_duration_ms - state->last_ms);
  }
  iqs915x_inertia_motion_step(&state->motion, (uint32_t)MIN(elapsed_ms, UINT32_MAX),
                              &step_x, &step_y);
  state->last_ms = now_ms;
  iqs915x_filter_scroll_cross_axis(&step_x, &step_y);

  if (step_x != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->inertia_scroll_x_acc, INPUT_REL_HWHEEL, step_x,
        data->swipe_resolution_x, config->scroll_divisor, "inertia");
  }

  if (step_y != 0)
  {
    emitted |= iqs915x_emit_normalized_scroll_axis(
        data, &data->inertia_scroll_y_acc, INPUT_REL_WHEEL, step_y,
        data->swipe_resolution_y, config->scroll_divisor, "inertia");
  }

  if (emitted)
  {
    state->last_output_ms = now_ms;
  }
  else if (now_ms - state->last_output_ms >=
           (int64_t)MAX(1, config->scroll_inertia.interval_ms) * 3)
  {
    iqs915x_stop_scroll_inertia_locked(data);
    LOG_DBG("Scroll inertia stopped (no HID output)");
    k_mutex_unlock(&data->settings_lock);
    return;
  }

  if (llabs(state->motion.vx_q16) * 10 <=
          (int64_t)profile->threshold_stop * IQS915X_INERTIA_POSITION_SCALE &&
      llabs(state->motion.vy_q16) * 10 <=
          (int64_t)profile->threshold_stop * IQS915X_INERTIA_POSITION_SCALE)
  {
    iqs915x_stop_scroll_inertia_locked(data);
    k_mutex_unlock(&data->settings_lock);
    return;
  }

  // 次のティックをスケジュール
  next_delay_ms = profile->interval_ms;
  if (profile->max_duration_ms > 0)
  {
    int64_t remaining = profile->max_duration_ms - (k_uptime_get() - state->started_ms);
    if (remaining <= 0)
    {
      iqs915x_stop_scroll_inertia_locked(data);
      k_mutex_unlock(&data->settings_lock);
      return;
    }
    next_delay_ms = MIN(next_delay_ms, (uint16_t)remaining);
  }
  k_work_reschedule(&data->scroll_inertia_work, K_MSEC(next_delay_ms));
  k_mutex_unlock(&data->settings_lock);
}

// スクロール慣性を打ち切る（新しい操作が入った場合に呼ばれる）
void iqs915x_cancel_scroll_inertia(struct iqs915x_data *data)
{
  k_mutex_lock(&data->settings_lock, K_FOREVER);
  if (data->scroll_inertia_state.active)
  {
    iqs915x_stop_scroll_inertia_locked(data);
    LOG_DBG("Scroll inertia cancelled by new input");
  }
  k_mutex_unlock(&data->settings_lock);
}

static const struct iqs915x_settings_limits iqs915x_supported_settings_limits = {
    .version = IQS915X_SETTINGS_VERSION_2,
    .pointer_sensitivity_percent = {.min = 25, .max = 400},
    .pointer_threshold = {.min = 0, .max = 1024},
    .pointer_saturation = {.min = 1, .max = 2048},
    .pointer_max_percent = {.min = 25, .max = 400},
    .inertia_trigger_ms = {.min = 0, .max = 500},
    .inertia_decay_factor_percent = {.min = 0, .max = 99},
    .inertia_interval_ms = {.min = 5, .max = 100},
    .inertia_threshold_start = {.min = 0, .max = 32767},
    .inertia_threshold_stop = {.min = 0, .max = 32767},
    .inertia_max_duration_ms = {.min = 50, .max = 5000},
    .inertia_initial_velocity_percent = {.min = 100, .max = 1000},
};

int iqs915x_get_settings_limits(struct iqs915x_settings_limits *limits)
{
  if (limits == NULL)
  {
    return -EINVAL;
  }

  *limits = iqs915x_supported_settings_limits;
  return 0;
}

static bool iqs915x_setting_is_in_range(struct iqs915x_setting_range range,
                                       uint16_t value)
{
  return value >= range.min && value <= range.max;
}

int iqs915x_validate_settings(const struct iqs915x_settings *settings)
{
  if (settings == NULL || (settings->version != IQS915X_SETTINGS_VERSION_1 &&
                           settings->version != IQS915X_SETTINGS_VERSION_2))
  {
    return -EINVAL;
  }

  const struct iqs915x_pointer_settings *pointer = &settings->pointer;
  const struct iqs915x_scroll_inertia_settings *inertia =
      &settings->scroll_inertia;

  if (!iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.pointer_sensitivity_percent,
          pointer->sensitivity_percent) ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.pointer_threshold,
          pointer->threshold) ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.pointer_saturation,
          pointer->saturation) ||
      pointer->saturation <= pointer->threshold ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.pointer_max_percent,
          pointer->max_percent) ||
      pointer->max_percent < pointer->sensitivity_percent ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.inertia_trigger_ms,
          inertia->trigger_ms) ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.inertia_decay_factor_percent,
          inertia->decay_factor_percent) ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.inertia_interval_ms,
          inertia->interval_ms) ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.inertia_threshold_start,
          inertia->threshold_start) ||
      !iqs915x_setting_is_in_range(
          iqs915x_supported_settings_limits.inertia_threshold_stop,
          inertia->threshold_stop) ||
      inertia->threshold_stop > inertia->threshold_start ||
      (settings->version == IQS915X_SETTINGS_VERSION_2 &&
       !iqs915x_setting_is_in_range(
           iqs915x_supported_settings_limits.inertia_initial_velocity_percent,
           inertia->initial_velocity_percent)) ||
      (inertia->max_duration_ms != 0 &&
       !iqs915x_setting_is_in_range(
           iqs915x_supported_settings_limits.inertia_max_duration_ms,
           inertia->max_duration_ms)))
  {
    return -EINVAL;
  }

  return 0;
}

static int iqs915x_settings_device_check(const struct device *dev,
                                        struct iqs915x_data **data_out)
{
  if (dev == NULL || !device_is_ready(dev) || dev->data == NULL ||
      dev->api != &iqs915x_device_api)
  {
    return -ENODEV;
  }

  *data_out = dev->data;
  return 0;
}

int iqs915x_get_settings(const struct device *dev,
                         struct iqs915x_settings *settings)
{
  struct iqs915x_data *data;
  int ret;

  if (settings == NULL)
  {
    return -EINVAL;
  }

  ret = iqs915x_settings_device_check(dev, &data);
  if (ret < 0)
  {
    return ret;
  }

  k_mutex_lock(&data->settings_lock, K_FOREVER);
  if (atomic_get(&data->pm_suspended) != 0)
  {
    ret = -EBUSY;
  }
  else if (atomic_get(&data->settings_ready) == 0)
  {
    ret = -EAGAIN;
  }
  else
  {
    *settings = data->runtime_settings;
    ret = 0;
  }
  k_mutex_unlock(&data->settings_lock);

  return ret;
}

int iqs915x_apply_settings(const struct device *dev,
                           const struct iqs915x_settings *settings)
{
  struct iqs915x_data *data;
  int ret = iqs915x_validate_settings(settings);

  if (ret < 0)
  {
    return ret;
  }

  ret = iqs915x_settings_device_check(dev, &data);
  if (ret < 0)
  {
    return ret;
  }

  k_mutex_lock(&data->settings_lock, K_FOREVER);
  if (atomic_get(&data->pm_suspended) != 0)
  {
    ret = -EBUSY;
  }
  else if (atomic_get(&data->settings_ready) == 0)
  {
    ret = -EAGAIN;
  }
  else
  {
    /* The inertia worker takes this same lock, so a queued or running tick
     * cannot observe a partially applied profile. */
    iqs915x_stop_scroll_inertia_locked(data);
    data->pointer_x_acc = 0;
    data->pointer_y_acc = 0;
    data->runtime_settings = *settings;
    data->runtime_settings.version = IQS915X_SETTINGS_VERSION_2;
    if (settings->version == IQS915X_SETTINGS_VERSION_1)
    {
      data->runtime_settings.scroll_inertia.initial_velocity_percent = 100;
    }
    ret = 0;
  }
  k_mutex_unlock(&data->settings_lock);

  return ret;
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

      // === 強制パッチ: ドライバの固定設定と必須ビット修正 ===
      if (current_addr == IQS915X_SYSTEM_CONTROL)
      {
        // ACK/ATI/reseedをinit-data転送中に実行しない。
        buffer[i] &= ~0xF8;
      }
      else if (current_addr == IQS915X_CONFIG_SETTINGS)
      {
        uint16_t cfg = (uint16_t)buffer[i] | ((uint16_t)buffer[i + 1] << 8);

        // TERMINATE_COMMS(bit6), FORCE_COMMS_METHOD(bit4) は
        // クロックストレッチ＋I2C STOPによる標準動作のため強制クリアし、
        // TP/Re-ATIをイベント源にし、初期化中はStreamingにする。
        cfg &= ~(IQS915X_FORCE_COMMS_METHOD | IQS915X_TERMINATE_COMMS);
        cfg = iqs915x_config_settings_without_event_mode(cfg);
        buffer[i] = cfg & 0xFF;
      }
      else if (current_addr == IQS915X_CONFIG_SETTINGS + 1)
      {
        uint16_t cfg = (uint16_t)buffer[i - 1] | ((uint16_t)buffer[i] << 8);

        // 16-bit little-endian: high byte bit0 == EVENT_MODE。
        // SHOW_RESET clear後に明示writeするためEVENT_MODEだけclearし、
        // GESTURE_EVENT/TP_TOUCH_EVENT/ALP_EVENTは無効にする。
        cfg = iqs915x_config_settings_without_event_mode(cfg);
        buffer[i] = (cfg >> 8) & 0xFF;
      }
      else if (current_addr == IQS915X_LP2_MODE_REPORT_RATE)
      {
        // TP channelsを500 msごとにセンシングする。
        buffer[i] = IQS915X_LP2_SAMPLING_PERIOD_MS & 0xFF;
      }
      else if (current_addr == IQS915X_LP2_MODE_REPORT_RATE + 1)
      {
        buffer[i] = (IQS915X_LP2_SAMPLING_PERIOD_MS >> 8) & 0xFF;
      }
      else if (current_addr == IQS915X_ALP_SETUP + 3)
      {
        // ALP Enable (bit31) = 0: TP channels sense in LP1/LP2.
        buffer[i] &= ~BIT(7);
      }
      else if (current_addr == IQS915X_OTHER_SETTINGS)
      {
        // Auto-Prox can skip communication cycles; every streaming scan is needed.
        buffer[i] &= ~(BIT(5) | BIT(4));
      }
      else if (current_addr == IQS915X_REATI_RETRY_TIME)
      {
        buffer[i] = 1; // ATI Error suppression time, in seconds.
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
  data->power_retry_count = 0;
  atomic_set(&data->transition_result, 0);
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
    uint16_t sys_ctrl = IQS915X_MODE_ACTIVE | IQS915X_REATI_TP;

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
    // 出力有効ActiveはTP/Re-ATI Event、無効起動はStreamingを使う。
    // Gesture/TP Touch/ALP Eventは無効にする。
    uint16_t cfg = 0;
    ret = iqs915x_read_reg16(dev, IQS915X_CONFIG_SETTINGS, &cfg);
    if (ret < 0)
    {
      LOG_ERR("Failed to read CONFIG_SETTINGS: %d", ret);
      return;
    }
    data->init_pending_cfg = atomic_get(&data->requested_enabled) ?
        iqs915x_apply_config_settings_policy(cfg) :
        iqs915x_config_settings_without_event_mode(cfg);
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
    LOG_INF("Init: communication policy written (CONFIG_SETTINGS=0x%04x)",
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

    uint16_t expected = IQS915X_MANUAL_CONTROL | IQS915X_TP_EVENT |
                        IQS915X_TP_REATI_ENABLE | IQS915X_REATI_EVENT;
    uint16_t forbidden = IQS915X_GESTURE_EVENT | IQS915X_TP_TOUCH_EVENT |
                         IQS915X_ALP_REATI_ENABLE | IQS915X_ALP_EVENT;
    if (atomic_get(&data->requested_enabled)) { expected |= IQS915X_EVENT_MODE; }
    else { forbidden |= IQS915X_EVENT_MODE; }

    if ((cfg & expected) == expected && (cfg & forbidden) == 0)
    {
      LOG_INF("Init: communication policy confirmed (CONFIG_SETTINGS=0x%04x)",
              cfg);
      data->init_step = INIT_COMPLETE;
      iqs915x_mark_initialized(data, true);
      data->work_state = WORK_READ_DATA;
      data->last_info_flags = 0;
      data->init_restart_count = 0;
      data->init_chunk_retry_count = 0;
      data->init_pending_cfg = 0;
      data->confirmed_config_settings = cfg;
      data->confirmed_mode = IQS915X_MODE_ACTIVE;
      data->streaming_expected = !(cfg & IQS915X_EVENT_MODE);
      data->ati_error_seen = false;
      data->comm_fallback_active = false;
      iqs915x_clear_stuck(data, "initialization");
      data->applied_generation = iqs915x_request_generation(data);
      data->transition_generation = data->applied_generation;
      data->active_pending = false;
      if (atomic_get(&data->requested_enabled) != 0)
      {
        iqs915x_reset_absolute_tracking(data);
        data->is_touching = false;
        data->pointer_resume_guard_frames =
            IQS915X_POINTER_RESUME_GUARD_FRAMES;
        data->lp2_pending = false;
        data->enabled = true;
        atomic_set(&data->output_enabled, 1);
      }
      else
      {
        data->pointer_resume_guard_frames = 0;
        data->lp2_pending = true;
        data->enabled = false;
        atomic_clear(&data->output_enabled);
      }
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

    LOG_WRN("Init: CONFIG_SETTINGS policy mismatch (0x%04x), "
            "retrying force (%d/%d)",
            cfg, data->wait_count, IQS915X_INIT_EVENT_MODE_MAX_RETRIES);
    data->init_pending_cfg = atomic_get(&data->requested_enabled) ?
        iqs915x_apply_config_settings_policy(cfg) :
        iqs915x_config_settings_without_event_mode(cfg);
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
    // ALPは無効なので、TP Re-ATIだけを初期化完了の必須条件にする。
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

static void iqs915x_reset_input_session(struct iqs915x_data *data)
{
  const struct device *dev = data->dev;
  struct k_work_sync button_sync;
  struct k_work_sync tap_release_sync;
  struct k_work_sync single_tap_sync;
  struct k_work_sync tap_start_sync;
  struct k_work_sync inertia_sync;
  uint8_t pressed;

  k_work_cancel_delayable_sync(&data->tap_and_hold_release_work,
                               &tap_release_sync);
  k_work_cancel_delayable_sync(&data->single_tap_work, &single_tap_sync);
  k_work_cancel_delayable_sync(&data->tap_and_hold_start_work,
                               &tap_start_sync);
  k_work_cancel_delayable_sync(&data->button_release_work, &button_sync);
  k_work_cancel_delayable_sync(&data->scroll_inertia_work, &inertia_sync);
  pressed = data->buttons_pressed;

  if (data->active_tap_hold &&
      (pressed & BIT(LEFT_BUTTON_CODE - INPUT_BTN_0)) == 0)
  {
    input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
  }

  for (int i = 0; i < 3; i++)
  {
    if (pressed & BIT(i))
    {
      input_report_key(dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
    }
  }

  data->buttons_pressed = 0;
  data->active_tap_hold = false;
  data->tap_and_hold_release_pending = false;
  data->single_tap_pending = false;
  data->tap_sequence_second_touch = false;
  data->tap_and_hold_start_pending = false;
  data->is_touching = false;
  data->last_touch_down_time = 0;
  data->gesture_pointer_suppress_ticks = 0;
  data->pointer_resume_guard_frames = 0;
  iqs915x_reset_absolute_tracking(data);
  iqs915x_reset_runtime_gesture_state(data);
  iqs915x_reset_scroll_inertia(data);
}

static void iqs915x_apply_pending_power_request(struct iqs915x_data *data)
{
  uint32_t generation = iqs915x_request_generation(data);
  if (generation == data->applied_generation && !data->active_pending && !data->lp2_pending)
  {
    return;
  }
  bool output = atomic_get(&data->requested_enabled) != 0;
  bool suspended = atomic_get(&data->pm_suspended) != 0;
  bool stable_lp2 = data->confirmed_mode == IQS915X_MODE_LP2 &&
                    data->work_state == WORK_READ_DATA && data->reseed_state == RESEED_IDLE;
  atomic_clear(&data->output_enabled);
  data->enabled = false;
  iqs915x_reset_input_session(data);
  data->applied_generation = generation;
  data->transition_generation = generation;
  data->active_pending = false;
  data->lp2_pending = false;
  if (output || suspended)
  {
    k_work_cancel_delayable(&data->reseed_work);
    atomic_clear(&data->reseed_timer_armed);
    atomic_clear(&data->reseed_due);
  }
  if (suspended) { iqs915x_clear_stuck(data, "pm-suspend"); }
  if (!output && stable_lp2)
  {
    data->power_force_comms = false;
    iqs915x_schedule_lp2_reseed(data);
    iqs915x_complete_transition(data, 0);
    return;
  }
  /* A reseed write can have reached the device even when I2C reported an error.
   * Drain its next Active scan before honoring a new mode request. */
  if (data->reseed_state == RESEED_WAIT_TP_SCAN) { return; }
  if (data->reseed_state == RESEED_ISSUE_TP_RESEED && !suspended)
  {
    iqs915x_begin_mode(data, IQS915X_MODE_ACTIVE, false);
    data->power_force_comms = output;
    return;
  }
  data->reseed_state = RESEED_IDLE;
  iqs915x_restore_mode(data);
  LOG_INF("communication request generation=%u enabled=%u suspended=%u force=%u skip_streaming=%u",
          generation, output, suspended, data->power_force_comms,
          data->work_state == WORK_SET_POWER);
}

/* ============================================================
 * メインスレッド
 *
 * RDY割り込みでセマフォが解放され、ストリーミングデータをraw読み取りする。
 * ActiveはREL_Xから44 bytes、LP2はINFO_FLAGSのみを読む。
 * ============================================================ */
static void iqs915x_thread_main(void *p1, void *p2, void *p3)
{
  struct iqs915x_data *data = p1;
  const struct device *dev = data->dev;
  const struct iqs915x_config *config = dev->config;
  int ret;
  struct iqs915x_stream_data last_stream = {0};
  uint32_t last_stream_generation = 0;

  while (true)
  {
    if (!data->initialized)
    {
      // Event Mode can stop RDY immediately when no finger event is present.
      // Force both the write and read-back so verification and policy retries
      // do not depend on touch or a split central. Each step ends with STOP.
      if (data->init_step == INIT_SET_EVENT_MODE ||
          data->init_step == INIT_CONFIRM_EVENT_MODE ||
          data->init_step == INIT_SOFTWARE_RESET ||
          (data->init_step == INIT_CHECK_SHOW_RESET && data->comm_fallback_active))
      {
        enum iqs915x_init_step previous_step = data->init_step;

        iqs915x_init_step_handler(dev);
        if (data->init_step == previous_step)
        {
          // An I2C error leaves the step unchanged. Keep the original two-second
          // retry interval instead of spinning on a non-responsive device.
          k_sleep(K_MSEC(2000));
        }
        continue;
      }

      // Configuration transfer and Re-ATI wait remain RDY-driven. The first
      // status check also supports a warm IC left in Event Mode.
      //
      // ただし割り込みのエッジ取りこぼし対策として：
      // - すでにRDYがLowになっている場合はセマフォをgiveしてすぐ進む
      // - 長めのタイムアウトで完全停止を防ぐ（ICが応答しない異常時の安全策）
      if (gpio_pin_get_dt(&config->rdy_gpio) > 0)
      {
        k_sem_give(&data->rdy_sem);
      }
      ret = k_sem_take(&data->rdy_sem, K_MSEC(2000));
      if (ret == 0 || data->init_step == INIT_CHECK_SHOW_RESET) {
        iqs915x_init_step_handler(dev);
      } else {
        LOG_WRN("Timed out waiting for IQS915x RDY during initialization");
      }
      continue;
    }

    iqs915x_apply_pending_power_request(data);

    if (atomic_get(&data->pm_suspended) && data->work_state == WORK_READ_DATA &&
        data->reseed_state == RESEED_IDLE && data->confirmed_mode == IQS915X_MODE_LP2)
    {
      iqs915x_clear_stuck(data, "pm-suspend");
      /* Zephyr may suspend the I2C controller after the PM transition. */
      k_sem_take(&data->rdy_sem, K_FOREVER);
      continue;
    }
    if (data->work_state != WORK_READ_DATA)
    {
      /* Output enable (including PM resume) is latency-sensitive. Initiate
       * each transaction immediately; the IC clock-stretches to its next safe
       * communication window. Each STOP still ends one separate window. */
      bool force = data->power_force_comms || !data->streaming_expected ||
                   data->work_state == WORK_CONFIRM_EVENT_MODE;
      if (iqs915x_wait_window(data, force, INT64_MAX)) { iqs915x_mode_step(data); }
      continue;
    }
    if (data->reseed_state == RESEED_ISSUE_TP_RESEED)
    {
      if (!iqs915x_wait_window(data, false, INT64_MAX)) { continue; }
      ret = iqs915x_write_reg16(dev, IQS915X_SYSTEM_CONTROL,
                                IQS915X_MODE_ACTIVE | IQS915X_TP_RESEED);
      LOG_INF("reseed request id=%u forced=%u rc=%d t=%lld", data->reseed_id,
              data->reseed_forced, ret, (long long)k_uptime_get());
      data->reseed_state = RESEED_WAIT_TP_SCAN;
      if (ret < 0) { iqs915x_comm_failure(data, ret); }
      continue;
    }

    uint32_t frame_generation = iqs915x_request_generation(data);
    int64_t deadline = INT64_MAX;
    if (!data->streaming_expected && iqs915x_output_is_enabled(data))
    {
      deadline = iqs915x_stuck_deadline(&data->stuck);
      if (data->finger_tracker.count_change_pending)
      {
        deadline = MIN(deadline, data->finger_tracker.candidate_since_ms +
                                 IQS915X_FINGER_COUNT_DEBOUNCE_MS);
      }
    }
    bool window = iqs915x_wait_window(data, false, deadline);
    if (data->applied_generation != iqs915x_request_generation(data)) { continue; }
    bool probe_due = !data->streaming_expected &&
                     k_uptime_get() >= iqs915x_stuck_deadline(&data->stuck);
    struct iqs915x_stream_data stream = {0};
    if (probe_due) { window = true; }
    if (!window)
    {
      if (!data->finger_tracker.count_change_pending ||
          last_stream_generation != frame_generation) { continue; }
      stream = last_stream;
      stream.trackpad_flags &= ~IQS915X_TP_MOVEMENT;
      stream.gesture_sf = 0;
      stream.gesture_tf = 0;
    }
    else
    {
      if (data->confirmed_mode == IQS915X_MODE_LP2)
      {
        ret = iqs915x_read_reg16(dev, IQS915X_INFO_FLAGS, &stream.info_flags);
      }
      else
      {
        ret = iqs915x_read_stream(dev, &stream);
      }
      if (ret < 0) { iqs915x_comm_failure(data, ret); continue; }
      bool allow_input = iqs915x_maintenance_sample(data, &stream);
      if (!allow_input) { continue; }
      last_stream = stream;
      last_stream_generation = frame_generation;
    }
    if (!data->enabled || !iqs915x_output_is_enabled(data) ||
        frame_generation != iqs915x_request_generation(data)) { continue; }

    // =========================================================
    // ドラッグ解除チェック: has_tp_event に依存せず毎フレーム実行
    // 接触境界はNUM_FINGERSを20 ms安定化して判定する。
    // =========================================================
    uint8_t reported_fingers = stream.trackpad_flags & IQS915X_NUM_FINGERS_MASK;
    uint8_t num_fingers = iqs915x_filter_finger_count(
        data, reported_fingers, k_uptime_get());
    bool finger_count_changed = num_fingers != data->finger_tracker.stable_count;
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
    iqs915x_update_finger_state(data, &stream, num_fingers,
                                touch_down, touch_up);
    iqs915x_update_scroll_contact(data, num_fingers, reported_fingers,
                                  data->finger_tracker.transition_ms,
                                  data->scroll_sequence_active);
    iqs915x_update_sequence_gates(data);

    /* Once multiple fingers are confirmed, single-finger input stays blocked
     * for this contact sequence. Only a confirmed zero count ends it. */
    bool single_finger_blocked = data->finger_tracker.sequence_max_count >= 2;
    if (single_finger_blocked)
    {
      k_work_cancel_delayable(&data->single_tap_work);
      k_work_cancel_delayable(&data->tap_and_hold_start_work);
      data->single_tap_pending = false;
      data->tap_sequence_second_touch = false;
      data->tap_and_hold_start_pending = false;

      if (data->active_tap_hold)
      {
        k_work_cancel_delayable(&data->tap_and_hold_release_work);
        data->tap_and_hold_release_pending = false;
        data->active_tap_hold = false;
        if (iqs915x_report_key(data, LEFT_BUTTON_CODE, 0, true))
        {
          data->buttons_pressed &= ~BIT(LEFT_BUTTON_CODE - INPUT_BTN_0);
        }
        else
        {
          data->button_work_generation = iqs915x_request_generation(data);
          k_work_reschedule(&data->button_release_work,
                            K_MSEC(IQS915X_BUTTON_TAP_RELEASE_MS));
        }
        LOG_DBG("tap-and-drag canceled by multiple fingers");
      }
    }

    if (reported_fingers == num_fingers)
    {
      iqs915x_update_single_tap_movement(data, &stream, num_fingers);
    }
    uint8_t pointer_slot = UINT8_MAX;
    uint16_t pointer_x = 0, pointer_y = 0;
    bool pointer_coordinates_valid =
        iqs915x_select_single_finger(&stream, &pointer_slot, &pointer_x, &pointer_y);
    if (finger_count_changed && num_fingers == 1 && !single_finger_blocked)
    {
      data->gesture_pointer_suppress_ticks = 0;
      iqs915x_reset_absolute_tracking(data);
      if (pointer_coordinates_valid)
      {
        data->pointer_slot = pointer_slot;
        data->last_abs_x = pointer_x;
        data->last_abs_y = pointer_y;
        data->last_abs_valid = true;
      }
    }

    bool has_tp_event = is_touching_now || touch_state_changed;
    if (tp_movement)
    {
      has_tp_event = true;
    }

    if (has_tp_event)
    {
      bool scroll = iqs915x_handle_two_finger_scroll(config, data, &stream);
      bool gesture_active = scroll ||
                            (num_fingers >= 2 && data->scroll_sequence_active) ||
                            data->multifinger_swipe_latched;
      bool suppress_pointer_tail =
          !gesture_active && data->gesture_pointer_suppress_ticks > 0;
      bool suppress_pointer = gesture_active || suppress_pointer_tail ||
                              single_finger_blocked;
      bool single_finger_pointer = data->finger_tracker.stable_count == 1;
      bool allow_pointer_report = !suppress_pointer && single_finger_pointer &&
                                  reported_fingers == 1;

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

      if (!scroll && !data->scroll_sequence_active)
      {
        // スクロールセッション外では、慣性とは独立した手動端数を破棄する。
        k_mutex_lock(&data->settings_lock, K_FOREVER);
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
        k_mutex_unlock(&data->settings_lock);
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
              ? data->finger_tracker.transition_ms - data->last_touch_down_time
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
          data->tap_and_hold_start_work_generation =
              iqs915x_request_generation(data);
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
          data->pending_tap_up_time = data->finger_tracker.transition_ms;
          data->single_tap_work_generation =
              iqs915x_request_generation(data);
          k_work_schedule(&data->single_tap_work,
                          K_MSEC(MAX(0LL, data->pending_tap_up_time +
                                         data->tap_air_time_ms - now_ms)));
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
          data->tap_and_hold_release_work_generation =
              iqs915x_request_generation(data);
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
            is_touching_now && num_fingers == 1 && pointer_coordinates_valid;

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
                "tp_absolute suppressed: fingers=%u stable=%u pending=%u "
                "mask=0x%x scroll=%u gesture_seen=%u x=%u y=%u",
                num_fingers,
                data->finger_tracker.stable_count,
                data->finger_tracker.count_change_pending,
                iqs915x_valid_finger_mask(&stream),
                data->scroll_sequence_active,
                data->tap_drag_raw_gesture_seen,
                pointer_x, pointer_y);
          }
        }
        else
        {
          // 通常のポインタ操作が再開したら慣性スクロールを止める
          iqs915x_cancel_scroll_inertia(data);

          if (data->pointer_slot != pointer_slot)
          {
            LOG_DBG("pointer slot changed: %u -> %u", data->pointer_slot, pointer_slot);
            iqs915x_reset_absolute_tracking(data);
          }
          data->pointer_slot = pointer_slot;
          if (touch_down || tp_movement || !data->last_abs_valid)
          {
            if (data->pointer_resume_guard_frames > 0)
            {
              data->last_abs_x = pointer_x;
              data->last_abs_y = pointer_y;
              data->last_abs_valid = true;
              iqs915x_reset_pointer_accumulators(data);
              data->pointer_resume_guard_frames--;
              LOG_DBG("tp_resume_baseline: generation=%u remaining=%u x=%u y=%u",
                      iqs915x_request_generation(data),
                      data->pointer_resume_guard_frames,
                      pointer_x, pointer_y);
            }
            else if (!data->last_abs_valid)
            {
              // 初回は基準点のみ保存し、次フレーム以降をデルタ報告する
              data->last_abs_x = pointer_x;
              data->last_abs_y = pointer_y;
              data->last_abs_valid = true;
            }
            else
            {
              int32_t rel_x = (int32_t)pointer_x - (int32_t)data->last_abs_x;
              int32_t rel_y = (int32_t)pointer_y - (int32_t)data->last_abs_y;

              data->last_abs_x = pointer_x;
              data->last_abs_y = pointer_y;

              if (iqs915x_absolute_delta_is_discontinuity(data, rel_x, rel_y))
              {
                iqs915x_reset_pointer_accumulators(data);
                LOG_DBG("tp_absrel discontinuity: rel_x=%d rel_y=%d "
                        "threshold=%u x=%u y=%u",
                        (int)rel_x, (int)rel_y,
                        iqs915x_absolute_discontinuity_threshold(data),
                        pointer_x, pointer_y);
              }
              else if (rel_x != 0 || rel_y != 0)
              {
                int32_t raw_rel_x = rel_x;
                int32_t raw_rel_y = rel_y;
                uint16_t pointer_scale;

                k_mutex_lock(&data->settings_lock, K_FOREVER);
                pointer_scale = iqs915x_apply_pointer_scale(
                    config, data, raw_rel_x, raw_rel_y, &rel_x, &rel_y);

                LOG_DBG("tp_absrel: rel_x=%d, rel_y=%d scale=%u out_x=%d "
                        "out_y=%d (x=%u y=%u)",
                        (int)raw_rel_x, (int)raw_rel_y, pointer_scale,
                        (int)rel_x, (int)rel_y, pointer_x, pointer_y);

                if (rel_x != 0 || rel_y != 0)
                {
                  iqs915x_report_pointer_pair(data, rel_x, rel_y);
                }
                k_mutex_unlock(&data->settings_lock);
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

  k_mutex_init(&data->settings_lock);
  atomic_clear(&data->settings_ready);
  data->runtime_settings = (struct iqs915x_settings){
      .version = IQS915X_SETTINGS_VERSION_2,
      .pointer = {
          .enabled = config->pointer_accel,
          .sensitivity_percent = config->pointer_sensitivity_percent,
          .threshold = config->pointer_accel_threshold,
          .saturation = config->pointer_accel_saturation,
          .max_percent = config->pointer_accel_max_percent,
      },
      .scroll_inertia = {
          .enabled = config->scroll_inertia.enabled,
          .trigger_ms = config->scroll_inertia.trigger_ms,
          .decay_factor_percent = config->scroll_inertia.decay_factor_int,
          .interval_ms = config->scroll_inertia.interval_ms,
          .threshold_start = config->scroll_inertia.threshold_start,
          .threshold_stop = config->scroll_inertia.threshold_stop,
          .initial_velocity_percent = config->scroll_inertia.initial_velocity_percent,
          .max_duration_ms = 0,
      },
  };
  data->dev = dev;
  data->init_step = INIT_CHECK_SHOW_RESET;
  data->work_state = WORK_READ_DATA;
  iqs915x_mark_initialized(data, false);
  data->init_data_offset = 0;
  data->wait_count = 0;
  data->init_chunk_retry_count = 0;
  data->init_restart_count = 0;
  data->init_pending_cfg = 0;
  data->confirmed_config_settings = 0;
  // disabled-by-defaultの場合は初期化完了後にStreamingのLP2へ移行する。
  atomic_set(&data->requested_enabled, !config->disabled_by_default);
  atomic_clear(&data->output_enabled);
  atomic_set(&data->request_generation, 0);
  data->applied_generation = 0;
  data->transition_generation = 0;
  data->relatch_target_enabled = false;
  data->pointer_resume_guard_frames = 0;
  data->button_work_generation = 0;
  data->tap_and_hold_release_work_generation = 0;
  data->single_tap_work_generation = 0;
  data->tap_and_hold_start_work_generation = 0;
  data->scroll_inertia_work_generation = 0;
  data->enabled = false;
  data->lp2_pending = config->disabled_by_default;
  data->active_pending = false;
  data->pm_saved_enabled = false;
  iqs915x_reset_absolute_tracking(data);
  iqs915x_reset_runtime_gesture_state(data);
  iqs915x_configure_swipe_thresholds(config, data);
  iqs915x_configure_tap_profile(config, data);
  data->stuck.enabled = data->swipe_resolution_x > 0 && data->swipe_resolution_y > 0;
  data->stuck.threshold = MIN(data->swipe_resolution_x, data->swipe_resolution_y) / 10;
  uint16_t profile_period = 0;
  iqs915x_get_init_data_reg16(config, IQS915X_ACTIVE_MODE_REPORT_RATE, &profile_period);
  data->active_sampling_period_ms = config->report_rate_ms ? config->report_rate_ms : profile_period;
  if (!data->active_sampling_period_ms) { return -EINVAL; }
  data->confirmed_mode = IQS915X_MODE_ACTIVE;
  data->power_target_mode = IQS915X_MODE_ACTIVE;
  data->streaming_expected = true;
  data->comm_completed_ms = k_uptime_get();
  LOG_INF("reseed policy interval_ms=60000 lp2_ms=500 active_ms=%u stuck_ms=10000 threshold=%u retry_s=1",
          data->active_sampling_period_ms, data->stuck.threshold);
  data->tap_and_hold_release_pending = false;
  data->single_tap_pending = false;
  data->tap_sequence_second_touch = false;
  data->tap_and_hold_start_pending = false;

  k_sem_init(&data->rdy_sem, 0, 1);
  k_sem_init(&data->transition_sem, 0, 1);
  atomic_clear(&data->pm_suspended);
  atomic_clear(&data->reseed_due);
  atomic_clear(&data->reseed_timer_armed);
  data->reseed_state = RESEED_IDLE;
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
  k_work_init_delayable(&data->reseed_work,
                        iqs915x_reseed_work_handler);

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

  /* Start only after I2C, GPIO, and the RDY interrupt are ready. */
  k_thread_create(&data->thread, data->thread_stack,
                  K_KERNEL_STACK_SIZEOF(data->thread_stack),
                  iqs915x_thread_main, data, NULL, NULL,
                  K_PRIO_PREEMPT(CONFIG_INPUT_AZOTEQ_IQS915X_THREAD_PRIORITY),
                  0, K_NO_WAIT);
  k_thread_name_set(&data->thread, "iqs915x");

  LOG_INF("IQS915x driver loaded, waiting for first RDY...");

  return 0;
}

/* ============================================================
 * デバイスインスタンスマクロ
 * ============================================================ */
#define IQS915X_INIT(n)                                                                                                                                                                            \
  BUILD_ASSERT(DT_INST_PROP(n, scroll_inertia_initial_velocity_percent) >= 100 && DT_INST_PROP(n, scroll_inertia_initial_velocity_percent) <= 1000, "IQS915x inertia initial velocity must be 100-1000 percent"); \
  static struct iqs915x_data iqs915x_data_##n;                                                                                                                                                     \
  static const uint8_t iqs915x_init_data_##n[] = DT_PROP(DT_INST_PHANDLE(n, profile), init_data);                                                                                                \
  static const uint16_t iqs915x_coord_lut_x_##n[] = DT_PROP(DT_INST_PHANDLE(n, profile), x_coordinate_lut_q15);                                                                                \
  static const uint16_t iqs915x_coord_lut_y_##n[] = DT_PROP(DT_INST_PHANDLE(n, profile), y_coordinate_lut_q15);                                                                                \
  BUILD_ASSERT(ARRAY_SIZE(iqs915x_init_data_##n) == IQS915X_INIT_DATA_TOTAL_SIZE, "IQS915x profile init-data must be 1174 bytes");                                                           \
  BUILD_ASSERT(ARRAY_SIZE(iqs915x_coord_lut_x_##n) == 17, "IQS915x X coordinate LUT must have 17 entries");                                                                                     \
  BUILD_ASSERT(ARRAY_SIZE(iqs915x_coord_lut_y_##n) == 17, "IQS915x Y coordinate LUT must have 17 entries");                                                                                     \
  BUILD_ASSERT(DT_PROP(DT_INST_PHANDLE(n, profile), x_coordinate_blocks) > 0, "IQS915x X coordinate block count must be positive");                                                            \
  BUILD_ASSERT(DT_PROP(DT_INST_PHANDLE(n, profile), y_coordinate_blocks) > 0, "IQS915x Y coordinate block count must be positive");                                                            \
  static const struct iqs915x_config iqs915x_config_##n = {                                                                                                                                        \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                                                                                                                                              \
      .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                                                                                                                             \
      .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                                                                                                                                 \
      .init_data = iqs915x_init_data_##n,                                                                                                                                                           \
      .init_data_len = ARRAY_SIZE(iqs915x_init_data_##n),                                                                                                                                          \
      .coord_lut_x_q15 = iqs915x_coord_lut_x_##n,                                                                                                                                                  \
      .coord_lut_x_len = ARRAY_SIZE(iqs915x_coord_lut_x_##n),                                                                                                                                      \
      .coord_lut_y_q15 = iqs915x_coord_lut_y_##n,                                                                                                                                                  \
      .coord_lut_y_len = ARRAY_SIZE(iqs915x_coord_lut_y_##n),                                                                                                                                      \
      .coord_x_blocks = DT_PROP(DT_INST_PHANDLE(n, profile), x_coordinate_blocks),                                                                                                                 \
      .coord_y_blocks = DT_PROP(DT_INST_PHANDLE(n, profile), y_coordinate_blocks),                                                                                                                 \
      .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                                                                                                                           \
      .tap_and_hold = DT_INST_PROP(n, tap_and_hold),                                                                                                                                               \
      .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                                                                                                                           \
      .scroll = DT_INST_PROP(n, scroll),                                                                                                                                                           \
      .scroll_divisor = DT_INST_PROP(n, scroll_divisor),                                                                                                                                           \
      .pointer_accel = DT_INST_PROP(n, pointer_accel),                                                                                                                                             \
      .pointer_sensitivity_percent = DT_INST_PROP(n, pointer_sensitivity_percent),                                                                                                                  \
      .pointer_accel_threshold = DT_INST_PROP(n, pointer_accel_threshold),                                                                                                                          \
      .pointer_accel_saturation = DT_INST_PROP(n, pointer_accel_saturation),                                                                                                                        \
      .pointer_accel_max_percent = DT_INST_PROP(n, pointer_accel_max_percent),                                                                                                                      \
      .scroll_inertia = {                                                                                                                                                                          \
          .enabled = DT_INST_PROP(n, scroll_inertia) ||                                                                                                                                            \
                     DT_INST_NODE_HAS_PROP(n, trigger_ms) ||                                                                                                                                       \
                     DT_INST_NODE_HAS_PROP(n, scroll_decay_factor_int) ||                                                                                                                          \
                     DT_INST_NODE_HAS_PROP(n, scroll_report_interval_ms) ||                                                                                                                        \
                     DT_INST_NODE_HAS_PROP(n, scroll_threshold_start) ||                                                                                                                           \
                     DT_INST_NODE_HAS_PROP(n, scroll_threshold_stop) ||                                                                                                                              \
                     DT_INST_NODE_HAS_PROP(n, scroll_inertia_initial_velocity_percent), \
          .trigger_ms = DT_INST_PROP(n, trigger_ms),                                                                                                                                              \
          .decay_factor_int = DT_INST_PROP(n, scroll_decay_factor_int),                                                                                                                           \
          .interval_ms = DT_INST_PROP(n, scroll_report_interval_ms),                                                                                                                              \
          .threshold_start = DT_INST_PROP(n, scroll_threshold_start),                                                                                                                             \
          .threshold_stop = DT_INST_PROP(n, scroll_threshold_stop),                                                                                                                              \
          .initial_velocity_percent = DT_INST_PROP(n, scroll_inertia_initial_velocity_percent), \
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
  IQS915X_PM_DEVICE_DEFINE(n);                                                                                                                                                                    \
  DEVICE_DT_INST_DEFINE(n, iqs915x_init, IQS915X_PM_DEVICE_GET(n), &iqs915x_data_##n,                                                                                                             \
                        &iqs915x_config_##n, POST_KERNEL,                                                                                                                                          \
                        CONFIG_INPUT_AZOTEQ_IQS915X_INIT_PRIORITY, &iqs915x_device_api);

DT_INST_FOREACH_STATUS_OKAY(IQS915X_INIT)
