/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define IQS915X_INERTIA_MOTION_WINDOW_MS 100
#define IQS915X_INERTIA_VELOCITY_UNIT_MS 10
/* Millisecond timestamps are coalesced. Keep every endpoint in the window
 * plus the endpoint preceding its left boundary, even at a 1 ms report rate. */
#define IQS915X_INERTIA_MOTION_HISTORY_SIZE (IQS915X_INERTIA_MOTION_WINDOW_MS + 2)

struct iqs915x_motion_sample
{
  int64_t ms;
  int32_t x;
  int32_t y;
};

struct iqs915x_motion_history
{
  struct iqs915x_motion_sample samples[IQS915X_INERTIA_MOTION_HISTORY_SIZE];
  uint8_t head;
  uint8_t count;
};

static inline void iqs915x_motion_history_reset(struct iqs915x_motion_history *history)
{
  history->head = 0;
  history->count = 0;
}

static inline const struct iqs915x_motion_sample *iqs915x_motion_history_sample(
    const struct iqs915x_motion_history *history, uint8_t index)
{
  return &history->samples[(history->head + IQS915X_INERTIA_MOTION_HISTORY_SIZE -
                           history->count + index) % IQS915X_INERTIA_MOTION_HISTORY_SIZE];
}

/* The first sample establishes a baseline. Each later delta belongs to the
 * interval since the preceding sample, including zero-motion reports. */
static inline void iqs915x_motion_history_add(struct iqs915x_motion_history *history,
                                             int64_t ms, int16_t x, int16_t y)
{
  if (history->count > 0)
  {
    struct iqs915x_motion_sample *last =
        &history->samples[(history->head + IQS915X_INERTIA_MOTION_HISTORY_SIZE - 1) %
                          IQS915X_INERTIA_MOTION_HISTORY_SIZE];
    if (ms <= last->ms)
    {
      if (ms == last->ms && history->count > 1)
      {
        last->x += x;
        last->y += y;
      }
      return;
    }
  }

  while (history->count > 1 &&
         iqs915x_motion_history_sample(history, 1)->ms <=
             ms - IQS915X_INERTIA_MOTION_WINDOW_MS)
  {
    history->count--;
  }

  history->samples[history->head] = (struct iqs915x_motion_sample){
      .ms = ms, .x = history->count > 0 ? x : 0, .y = history->count > 0 ? y : 0};
  history->head = (history->head + 1) % IQS915X_INERTIA_MOTION_HISTORY_SIZE;
  if (history->count < IQS915X_INERTIA_MOTION_HISTORY_SIZE)
  {
    history->count++;
  }
}

static inline int16_t iqs915x_motion_velocity_round(int64_t numerator, int64_t denominator)
{
  int64_t magnitude = numerator < 0 ? -numerator : numerator;
  int64_t rounded = (magnitude + denominator / 2) / denominator;

  if (numerator < 0)
  {
    return rounded > 32768 ? INT16_MIN : (int16_t)-rounded;
  }
  return rounded > INT16_MAX ? INT16_MAX : (int16_t)rounded;
}

/* Estimate signed displacement / elapsed time at the first zero-finger report.
 * A boundary-crossing interval is prorated; the stationary tail up to release
 * contributes time but no motion. Q16 preserves fractional boundary motion. */
static inline bool iqs915x_motion_history_velocity(
    const struct iqs915x_motion_history *history, int64_t release_ms,
    int16_t *vx, int16_t *vy, uint16_t *elapsed_ms)
{
  *vx = 0;
  *vy = 0;
  *elapsed_ms = 0;
  if (history->count < 2)
  {
    return false;
  }

  int64_t start_ms = release_ms - IQS915X_INERTIA_MOTION_WINDOW_MS;
  const struct iqs915x_motion_sample *previous =
      iqs915x_motion_history_sample(history, 0);
  if (previous->ms > start_ms)
  {
    start_ms = previous->ms;
  }
  if (release_ms <= start_ms)
  {
    return false;
  }

  int64_t sum_x_q16 = 0;
  int64_t sum_y_q16 = 0;
  for (uint8_t i = 1; i < history->count; i++)
  {
    const struct iqs915x_motion_sample *sample =
        iqs915x_motion_history_sample(history, i);
    if (sample->ms > release_ms)
    {
      break;
    }
    int64_t interval_start_ms = previous->ms > start_ms ? previous->ms : start_ms;
    if (sample->ms > interval_start_ms)
    {
      int64_t overlap_ms = sample->ms - interval_start_ms;
      int64_t interval_ms = sample->ms - previous->ms;
      sum_x_q16 += (int64_t)sample->x * overlap_ms * 65536 / interval_ms;
      sum_y_q16 += (int64_t)sample->y * overlap_ms * 65536 / interval_ms;
    }
    previous = sample;
  }

  *elapsed_ms = (uint16_t)(release_ms - start_ms);
  int64_t denominator = (int64_t)*elapsed_ms * 65536;
  *vx = iqs915x_motion_velocity_round(
      sum_x_q16 * IQS915X_INERTIA_VELOCITY_UNIT_MS, denominator);
  *vy = iqs915x_motion_velocity_round(
      sum_y_q16 * IQS915X_INERTIA_VELOCITY_UNIT_MS, denominator);
  return *vx != 0 || *vy != 0;
}
