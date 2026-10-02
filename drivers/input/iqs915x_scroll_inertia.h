/* Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>

#define IQS915X_INERTIA_POSITION_SCALE 65536LL
#define IQS915X_INERTIA_RETENTION_SCALE (1U << 24)

struct iqs915x_inertia_motion {
  int64_t vx_q16; /* coordinate units / ms, multiplied by 65536 */
  int64_t vy_q16;
  int64_t remainder_x_q16;
  int64_t remainder_y_q16;
  uint32_t retention_q24; /* retention per millisecond */
};

static inline uint32_t iqs915x_inertia_power(uint32_t factor, uint32_t ms)
{
  uint64_t result = IQS915X_INERTIA_RETENTION_SCALE;
  uint64_t base = factor;
  while (ms > 0) {
    if (ms & 1U) {
      result = result * base / IQS915X_INERTIA_RETENTION_SCALE;
    }
    base = base * base / IQS915X_INERTIA_RETENTION_SCALE;
    ms >>= 1;
  }
  return (uint32_t)result;
}

/* Integer root: retention over reference_ms is the configured percentage.
 * The reference is the boot-time DTS interval, independent of runtime cadence. */
static inline uint32_t iqs915x_inertia_retention(uint16_t percent, uint16_t reference_ms)
{
  uint32_t target = (uint64_t)percent * IQS915X_INERTIA_RETENTION_SCALE / 100;
  uint32_t low = 0, high = IQS915X_INERTIA_RETENTION_SCALE;
  if (percent >= 100) {
    return high;
  }
  if (percent == 0 || reference_ms == 0) {
    return 0;
  }
  while (low + 1 < high) {
    uint32_t middle = low + (high - low) / 2;
    if (iqs915x_inertia_power(middle, reference_ms) <= target) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return low;
}

static inline void iqs915x_inertia_motion_init(struct iqs915x_inertia_motion *motion,
    int16_t vx_per_10ms, int16_t vy_per_10ms, uint16_t initial_velocity_percent,
    uint16_t decay_percent, uint16_t reference_ms)
{
  *motion = (struct iqs915x_inertia_motion){
      .vx_q16 = (int64_t)vx_per_10ms * initial_velocity_percent *
                IQS915X_INERTIA_POSITION_SCALE / 1000,
      .vy_q16 = (int64_t)vy_per_10ms * initial_velocity_percent *
                IQS915X_INERTIA_POSITION_SCALE / 1000,
      .retention_q24 = iqs915x_inertia_retention(decay_percent, reference_ms),
  };
}

static inline int32_t iqs915x_inertia_integrate_axis(int64_t *velocity_q16,
    int64_t *remainder_q16, uint32_t retention_q24, uint32_t factor_q24, uint32_t ms)
{
  int64_t previous = *velocity_q16;
  *velocity_q16 = previous * factor_q24 / IQS915X_INERTIA_RETENTION_SCALE;
  int64_t displacement_q16;
  if (retention_q24 == IQS915X_INERTIA_RETENTION_SCALE) {
    displacement_q16 = previous * ms;
  } else {
    /* Sum the geometric decay at 1 ms resolution without iterating over ticks. */
    displacement_q16 = (previous - *velocity_q16) * retention_q24 /
        (IQS915X_INERTIA_RETENTION_SCALE - retention_q24);
  }
  displacement_q16 += *remainder_q16;
  int64_t whole = displacement_q16 / IQS915X_INERTIA_POSITION_SCALE;
  int32_t output = whole > INT32_MAX ? INT32_MAX :
                   whole < INT32_MIN ? INT32_MIN : (int32_t)whole;
  *remainder_q16 = displacement_q16 - (int64_t)output * IQS915X_INERTIA_POSITION_SCALE;
  return output;
}

static inline void iqs915x_inertia_motion_step(struct iqs915x_inertia_motion *motion,
    uint32_t elapsed_ms, int32_t *x, int32_t *y)
{
  uint32_t factor = iqs915x_inertia_power(motion->retention_q24, elapsed_ms);
  *x = iqs915x_inertia_integrate_axis(&motion->vx_q16, &motion->remainder_x_q16,
                                    motion->retention_q24, factor, elapsed_ms);
  *y = iqs915x_inertia_integrate_axis(&motion->vy_q16, &motion->remainder_y_q16,
                                    motion->retention_q24, factor, elapsed_ms);
}
