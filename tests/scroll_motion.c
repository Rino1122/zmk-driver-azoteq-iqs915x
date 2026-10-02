/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 *
 * Host-side regression tests for release-time scroll velocity estimation.
 */

#include <assert.h>
#include <stdio.h>

#include "iqs915x_scroll_motion.h"

static void expect_velocity(const struct iqs915x_motion_history *history,
                            int64_t release_ms, int16_t expected_x,
                            int16_t expected_y, uint16_t expected_elapsed)
{
  int16_t x, y;
  uint16_t elapsed;
  bool moving = iqs915x_motion_history_velocity(history, release_ms, &x, &y, &elapsed);

  assert(x == expected_x);
  assert(y == expected_y);
  assert(elapsed == expected_elapsed);
  assert(moving == (expected_x != 0 || expected_y != 0));
}

static void test_report_rates_and_ring_wrap(void)
{
  const int rates[] = {1, 2, 5, 10, 20, 50};
  for (unsigned rate = 0; rate < sizeof(rates) / sizeof(rates[0]); rate++)
  {
    struct iqs915x_motion_history history = {0};
    int dt = rates[rate];
    iqs915x_motion_history_add(&history, 0, 0, 0);
    for (int ms = dt; ms <= 1000; ms += dt)
    {
      iqs915x_motion_history_add(&history, ms, 2 * dt, -dt);
      assert(history.count <= IQS915X_INERTIA_MOTION_HISTORY_SIZE);
    }
    expect_velocity(&history, 1000, 20, -10, 100);
  }
}

static void test_irregular_timing_and_window_boundary(void)
{
  struct iqs915x_motion_history history = {0};
  const int times[] = {0, 7, 23, 51, 72, 105, 142, 190, 203};
  for (unsigned i = 0; i < sizeof(times) / sizeof(times[0]); i++)
  {
    int dt = i == 0 ? 0 : times[i] - times[i - 1];
    iqs915x_motion_history_add(&history, times[i], 3 * dt, -2 * dt);
  }
  expect_velocity(&history, 203, 30, -20, 100);

  iqs915x_motion_history_reset(&history);
  iqs915x_motion_history_add(&history, 0, 0, 0);
  iqs915x_motion_history_add(&history, 60, 600, -600);
  iqs915x_motion_history_add(&history, 120, 60, -60);
  /* Window [50, 150]: 1/6 of the first delta, then 60, then a 30 ms pause. */
  expect_velocity(&history, 150, 16, -16, 100);
}

static void test_last_frame_does_not_decide_velocity(void)
{
  struct iqs915x_motion_history steady = {0}, small_last = {0};
  iqs915x_motion_history_add(&steady, 0, 0, 0);
  iqs915x_motion_history_add(&small_last, 0, 0, 0);
  for (int ms = 10; ms <= 100; ms += 10)
  {
    iqs915x_motion_history_add(&steady, ms, 0, 100);
    int delta = ms == 10 ? 199 : ms == 100 ? 1 : 100;
    iqs915x_motion_history_add(&small_last, ms, 0, delta);
  }
  /* Both traces travel 1000 units / 100 ms, even though one ends below the
   * default start threshold of 2. Both must seed the same release velocity. */
  expect_velocity(&steady, 100, 0, 100, 100);
  expect_velocity(&small_last, 100, 0, 100, 100);
}

static void test_short_gesture_and_stationary_tail(void)
{
  struct iqs915x_motion_history history = {0};
  iqs915x_motion_history_add(&history, 0, 0, 0);
  iqs915x_motion_history_add(&history, 10, 0, 20);
  iqs915x_motion_history_add(&history, 20, 0, 20);
  expect_velocity(&history, 20, 0, 20, 20);
  expect_velocity(&history, 40, 0, 10, 40);
  /* Event Mode need not supply zero-motion reports to age out motion. */
  expect_velocity(&history, 120, 0, 0, 100);
  expect_velocity(&history, 500, 0, 0, 100);

  for (int ms = 30; ms <= 120; ms += 10)
  {
    iqs915x_motion_history_add(&history, ms, 0, 0);
  }
  expect_velocity(&history, 120, 0, 0, 100);
}

static void test_reversal_and_rounding_symmetry(void)
{
  struct iqs915x_motion_history history = {0};
  iqs915x_motion_history_add(&history, 0, 0, 0);
  iqs915x_motion_history_add(&history, 50, 100, -100);
  iqs915x_motion_history_add(&history, 100, -100, 100);
  expect_velocity(&history, 100, 0, 0, 100);

  iqs915x_motion_history_reset(&history);
  iqs915x_motion_history_add(&history, 0, 0, 0);
  iqs915x_motion_history_add(&history, 100, 15, -15);
  expect_velocity(&history, 100, 2, -2, 100);
}

static void test_reset_and_no_elapsed_time(void)
{
  struct iqs915x_motion_history history = {0};
  expect_velocity(&history, 100, 0, 0, 0);
  iqs915x_motion_history_add(&history, 100, 0, 0);
  iqs915x_motion_history_add(&history, 100, 200, -200);
  expect_velocity(&history, 100, 0, 0, 0);
  iqs915x_motion_history_add(&history, 110, 100, -100);
  expect_velocity(&history, 110, 100, -100, 10);

  /* Recovery/rebaselining must discard the old candidate and recovered delta. */
  iqs915x_motion_history_reset(&history);
  iqs915x_motion_history_add(&history, 120, 30000, -30000);
  expect_velocity(&history, 130, 0, 0, 0);
  iqs915x_motion_history_add(&history, 130, 0, 0);
  expect_velocity(&history, 130, 0, 0, 10);
  iqs915x_motion_history_add(&history, 140, 5, -5);
  expect_velocity(&history, 140, 3, -3, 20);
}

static void test_coalescing_and_saturation(void)
{
  struct iqs915x_motion_history history = {0};
  iqs915x_motion_history_add(&history, 0, 0, 0);
  iqs915x_motion_history_add(&history, 10, 5, -5);
  iqs915x_motion_history_add(&history, 10, 5, -5);
  expect_velocity(&history, 10, 10, -10, 10);

  iqs915x_motion_history_reset(&history);
  iqs915x_motion_history_add(&history, 0, 0, 0);
  iqs915x_motion_history_add(&history, 1, INT16_MAX, INT16_MIN);
  expect_velocity(&history, 1, INT16_MAX, INT16_MIN, 1);
}

int main(void)
{
  test_report_rates_and_ring_wrap();
  test_irregular_timing_and_window_boundary();
  test_last_frame_does_not_decide_velocity();
  test_short_gesture_and_stationary_tail();
  test_reversal_and_rounding_symmetry();
  test_reset_and_no_elapsed_time();
  test_coalescing_and_saturation();
  puts("scroll motion tests passed");
  return 0;
}
