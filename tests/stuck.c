/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "../drivers/input/iqs915x_stuck.h"

static struct iqs915x_stuck_point point(uint16_t x, uint16_t y)
{
  return (struct iqs915x_stuck_point){.valid = true, .x = x, .y = y};
}

int main(void)
{
  struct iqs915x_stuck_tracker t = {.enabled = true, .threshold = 100};
  struct iqs915x_stuck_point p[4] = {point(100, 100)};
  assert(!iqs915x_stuck_observe(&t, p, 0, false));
  assert(!iqs915x_stuck_observe(&t, p, 9999, false));
  assert(iqs915x_stuck_observe(&t, p, 10000, false) == 1);
  /* Total range, not adjacent differences: +60 then +60 restarts the age. */
  iqs915x_stuck_clear(&t);
  p[0] = point(100, 100);
  iqs915x_stuck_observe(&t, p, 0, false);
  uint32_t id = t.candidate[0].id;
  p[0] = point(160, 160);
  iqs915x_stuck_observe(&t, p, 5000, false);
  p[0] = point(220, 220);
  assert(!iqs915x_stuck_observe(&t, p, 10000, false));
  assert(t.candidate[0].id != id && t.candidate[0].since_ms == 10000);
  /* Inclusive boundary on both axes, and one stationary of four is enough. */
  iqs915x_stuck_clear(&t);
  for (int i = 0; i < 4; i++) { p[i] = point(100 + i * 1000, 100); }
  iqs915x_stuck_observe(&t, p, 0, false);
  p[0] = point(200, 200);
  for (int i = 1; i < 4; i++) { p[i].x += 101; }
  assert(iqs915x_stuck_observe(&t, p, 10000, false) == 1);
  assert(t.candidate[1].since_ms == 10000);
  /* Missing coordinates cancel only their own candidate. */
  p[0].valid = false;
  iqs915x_stuck_observe(&t, p, 10001, false);
  assert(!t.candidate[0].active && t.candidate[1].active);
  /* LP2 recapture swaps slots while preserving candidate ages and IDs. */
  iqs915x_stuck_clear(&t);
  p[0] = point(100, 100); p[1] = point(1000, 1000);
  p[2].valid = p[3].valid = false;
  iqs915x_stuck_observe(&t, p, 0, false);
  id = t.candidate[0].id;
  struct iqs915x_stuck_point q[4] = {p[1], p[0]};
  assert(iqs915x_stuck_observe(&t, q, 10000, true) == 3);
  assert(t.candidate[0].id == id && t.candidate[0].slot == 1);
  /* Find maximum matching, rather than greedily stealing a shared slot. */
  iqs915x_stuck_clear(&t);
  p[0] = point(100, 100); p[1] = point(190, 100);
  iqs915x_stuck_observe(&t, p, 0, false);
  q[0] = point(150, 100); q[1] = point(10, 100);
  uint8_t match[4];
  iqs915x_stuck_match(&t, q, match);
  assert(match[0] == 1 && match[1] == 0);
  /* Equal distances deterministically favor ascending ID, then slot. */
  iqs915x_stuck_clear(&t);
  p[0] = p[1] = point(100, 100);
  iqs915x_stuck_observe(&t, p, 0, false);
  q[0] = q[1] = point(100, 100);
  iqs915x_stuck_match(&t, q, match);
  assert(match[0] == 0 && match[1] == 1);
  /* One observation cannot keep two candidates alive. */
  q[1].valid = false;
  assert(iqs915x_stuck_observe(&t, q, 10000, true) == 1);
  assert(!t.candidate[1].active);
  /* A release clears all; a new finger gets a new ten-second interval. */
  memset(q, 0, sizeof(q));
  iqs915x_stuck_observe(&t, q, 10001, false);
  assert(iqs915x_stuck_deadline(&t) == INT64_MAX);
  q[3] = point(100, 100);
  assert(!iqs915x_stuck_observe(&t, q, 10002, false));
  assert(iqs915x_stuck_deadline(&t) == 20002);
  puts("IQS915x stuck monitor: range, multi-finger, remapping and release passed");
}
