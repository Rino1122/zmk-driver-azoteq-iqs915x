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
};

uint32_t iqs915x_axis_movement(int32_t dx, int32_t dy);
bool iqs915x_report_event(struct iqs915x_data *data, uint16_t type,
                          uint16_t code, int32_t value, bool sync);

void iqs915x_reset_runtime_gesture_state(struct iqs915x_data *data);
void iqs915x_update_sequence_gates(struct iqs915x_data *data);
void iqs915x_update_finger_state(struct iqs915x_data *data,
                                 const struct iqs915x_stream_data *stream,
                                 bool is_touching_now, bool touch_down_event,
                                 bool touch_up_event);
void iqs915x_update_single_tap_movement(
    struct iqs915x_data *data, const struct iqs915x_stream_data *stream,
    uint8_t num_fingers);
bool iqs915x_handle_multifinger_swipe(
    const struct iqs915x_config *config, struct iqs915x_data *data,
    const struct iqs915x_stream_data *stream);
void iqs915x_cancel_scroll_inertia(struct iqs915x_data *data);
