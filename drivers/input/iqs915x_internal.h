/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 *
 * Internal boundary shared by the IQS915x core and gesture implementation.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>

#include "iqs915x_regs.h"

#define IQS915X_FINGER_COUNT_DEBOUNCE_MS 20

struct iqs915x_stream_data {
  uint16_t gesture_x;
  uint16_t gesture_y;
  uint16_t gesture_sf;
  uint16_t gesture_tf;
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
  struct iqs915x_stuck_point raw_point[IQS915X_OBSERVED_FINGERS];
};

/* A confidence bit is not an occupancy flag. Select slots using valid XY pairs. */
bool iqs915x_get_finger_coordinates(const struct iqs915x_stream_data *stream,
                                    uint8_t slot, uint16_t *x, uint16_t *y);
uint8_t iqs915x_valid_finger_mask(const struct iqs915x_stream_data *stream);
bool iqs915x_select_single_finger(const struct iqs915x_stream_data *stream,
                                  uint8_t *slot, uint16_t *x, uint16_t *y);
bool iqs915x_absolute_delta_is_discontinuity(
    const struct iqs915x_data *data, int32_t rel_x, int32_t rel_y);
uint32_t iqs915x_axis_movement(int32_t dx, int32_t dy);
bool iqs915x_report_event(struct iqs915x_data *data, uint16_t type,
                          uint16_t code, int32_t value, bool sync);

void iqs915x_reset_runtime_gesture_state(struct iqs915x_data *data);
void iqs915x_update_sequence_gates(struct iqs915x_data *data);
uint8_t iqs915x_filter_finger_count(struct iqs915x_data *data,
                                    uint8_t raw_count, int64_t now_ms);
void iqs915x_update_finger_state(struct iqs915x_data *data,
                                 const struct iqs915x_stream_data *stream,
                                 uint8_t stable_count, bool touch_down_event,
                                 bool touch_up_event);
void iqs915x_update_single_tap_movement(
    struct iqs915x_data *data, const struct iqs915x_stream_data *stream,
    uint8_t num_fingers);
bool iqs915x_handle_multifinger_swipe(
    const struct iqs915x_config *config, struct iqs915x_data *data,
    const struct iqs915x_stream_data *stream);
void iqs915x_cancel_scroll_inertia(struct iqs915x_data *data);
