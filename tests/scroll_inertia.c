/* Copyright (c) 2026; SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "iqs915x_scroll_inertia.h"

static int64_t travel(unsigned interval, unsigned gain, bool irregular)
{
  struct iqs915x_inertia_motion motion;
  iqs915x_inertia_motion_init(&motion, 1000, -1000, gain, 90, 15);
  assert(motion.vx_q16 == (int64_t)1000 * gain * 65536 / 1000);
  int64_t sum_x = 0, sum_y = 0;
  unsigned elapsed = 0;
  while (elapsed < 1000) {
    unsigned dt = irregular ? (elapsed % 47) + 1 : interval;
    if (elapsed + dt > 1000) dt = 1000 - elapsed;
    int32_t x, y;
    iqs915x_inertia_motion_step(&motion, dt, &x, &y);
    sum_x += x; sum_y += y; elapsed += dt;
  }
  assert(sum_x == -sum_y);
  return sum_x;
}
int main(void)
{
  int64_t reference = travel(1, 100, false);
  for (unsigned interval = 5; interval <= 100; interval += 5) {
    int64_t sum = travel(interval, 100, false);
    assert(llabs(sum - reference) <= 2);
  }
  assert(llabs(travel(1, 100, true) - reference) <= 2);
  assert(llabs(travel(15, 200, false) - 2 * reference) <= 3);
  assert(llabs(travel(15, 1000, false) - 10 * reference) <= 10);
  struct iqs915x_inertia_motion motion;
  iqs915x_inertia_motion_init(&motion, 100, -100, 100, 0, 15);
  int32_t x, y;
  iqs915x_inertia_motion_step(&motion, 100, &x, &y);
  assert(x == 0 && y == 0);
  iqs915x_inertia_motion_init(&motion, 100, -100, 100, 100, 15);
  iqs915x_inertia_motion_step(&motion, 25, &x, &y);
  assert(x == 250 && y == -250);
  iqs915x_inertia_motion_init(&motion, INT16_MAX, INT16_MIN, 1000, 99, 100);
  iqs915x_inertia_motion_step(&motion, 5000, &x, &y);
  assert(x > 0 && y < 0);
  puts("scroll inertia tests passed");
}
