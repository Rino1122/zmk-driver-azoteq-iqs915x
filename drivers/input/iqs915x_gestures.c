/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 *
 * IQS915x tap, scroll and multi-finger gesture recognition.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <dt-bindings/input/iqs915x_gestures.h>
#include "iqs915x_internal.h"

LOG_MODULE_REGISTER(iqs915x_gestures, CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL);

static void iqs915x_reset_two_finger_session(struct iqs915x_data *data)
{
  memset(&data->two_finger, 0, sizeof(data->two_finger));
}

void iqs915x_reset_runtime_gesture_state(struct iqs915x_data *data)
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

void iqs915x_update_sequence_gates(struct iqs915x_data *data)
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
  struct iqs915x_data *data = dev->data;

  if (!iqs915x_report_event(data, IQS915X_INPUT_EV_GESTURE,
                            gesture_code, 1, false))
  {
    return;
  }
  iqs915x_report_event(data, IQS915X_INPUT_EV_GESTURE,
                       gesture_code, 0, true);
}

void iqs915x_update_finger_state(struct iqs915x_data *data,
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
    two_finger->centroid_last_x = centroid_x;
    two_finger->centroid_last_y = centroid_y;
    two_finger->distance_last = distance;
    return;
  }

  two_finger->centroid_dx = centroid_x - two_finger->centroid_last_x;
  two_finger->centroid_dy = centroid_y - two_finger->centroid_last_y;
  two_finger->max_centroid_movement =
      MAX(two_finger->max_centroid_movement,
          iqs915x_axis_movement(centroid_x - two_finger->centroid_start_x,
                                centroid_y - two_finger->centroid_start_y));
  two_finger->centroid_last_x = centroid_x;
  two_finger->centroid_last_y = centroid_y;
  two_finger->distance_last = distance;
}

bool iqs915x_handle_multifinger_swipe(const struct iqs915x_config *config,
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

void iqs915x_update_single_tap_movement(
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
