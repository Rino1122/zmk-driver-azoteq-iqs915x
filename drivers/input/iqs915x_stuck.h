/* SPDX-License-Identifier: MIT */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define IQS915X_OBSERVED_FINGERS 4
#define IQS915X_STUCK_TIME_MS 10000

struct iqs915x_stuck_point {
  bool valid;
  uint16_t x, y;
};

struct iqs915x_stuck_candidate {
  bool active;
  uint8_t slot;
  uint32_t id;
  int64_t since_ms;
  uint16_t min_x, max_x, min_y, max_y, last_x, last_y;
};

struct iqs915x_stuck_tracker {
  struct iqs915x_stuck_candidate candidate[IQS915X_OBSERVED_FINGERS];
  uint32_t next_id;
  uint16_t threshold;
  bool enabled;
};

static inline void iqs915x_stuck_clear(struct iqs915x_stuck_tracker *tracker)
{
  memset(tracker->candidate, 0, sizeof(tracker->candidate));
}

static inline int64_t iqs915x_stuck_deadline(const struct iqs915x_stuck_tracker *tracker)
{
  int64_t deadline = INT64_MAX;
  for (unsigned int i = 0; i < IQS915X_OBSERVED_FINGERS; i++) {
    if (tracker->candidate[i].active &&
        tracker->candidate[i].since_ms + IQS915X_STUCK_TIME_MS < deadline) {
      deadline = tracker->candidate[i].since_ms + IQS915X_STUCK_TIME_MS;
    }
  }
  return deadline;
}

static inline bool iqs915x_stuck_fits(const struct iqs915x_stuck_candidate *c,
                                      struct iqs915x_stuck_point p, uint16_t threshold)
{
  uint16_t min_x = p.x < c->min_x ? p.x : c->min_x;
  uint16_t max_x = p.x > c->max_x ? p.x : c->max_x;
  uint16_t min_y = p.y < c->min_y ? p.y : c->min_y;
  uint16_t max_y = p.y > c->max_y ? p.y : c->max_y;
  return p.valid && max_x - min_x <= threshold && max_y - min_y <= threshold;
}

/* Across LP2, slot identity is not reliable. Exhaustively select the maximum
 * one-to-one matching, then minimum squared distance. There are only 5^4
 * assignments (four slots or unmatched). Ascending IDs/slots break ties. */
static inline void iqs915x_stuck_match(const struct iqs915x_stuck_tracker *tracker,
                                       const struct iqs915x_stuck_point point[4],
                                       uint8_t match[4])
{
  uint8_t order[4] = {0, 1, 2, 3};
  unsigned int best_count = 0;
  uint64_t best_distance = UINT64_MAX;
  memset(match, 4, 4);
  for (unsigned int i = 1; i < 4; i++) {
    uint8_t index = order[i];
    unsigned int j = i;
    while (j > 0 && tracker->candidate[order[j - 1]].id > tracker->candidate[index].id) {
      order[j] = order[j - 1];
      j--;
    }
    order[j] = index;
  }
  for (unsigned int assignment = 0; assignment < 625; assignment++) {
    unsigned int code = assignment, used = 0, count = 0;
    uint64_t distance = 0;
    uint8_t trial[4];
    bool valid = true;
    for (int i = 3; i >= 0; i--) {
      trial[order[i]] = code % 5;
      code /= 5;
    }
    for (unsigned int i = 0; i < 4; i++) {
      unsigned int slot = trial[i];
      const struct iqs915x_stuck_candidate *c = &tracker->candidate[i];
      if (slot == 4) {
        continue;
      }
      if (!c->active || (used & (1U << slot)) ||
          !iqs915x_stuck_fits(c, point[slot], tracker->threshold)) {
        valid = false;
        break;
      }
      int64_t dx = (int32_t)point[slot].x - c->last_x;
      int64_t dy = (int32_t)point[slot].y - c->last_y;
      distance += (uint64_t)(dx * dx + dy * dy);
      used |= 1U << slot;
      count++;
    }
    if (valid && (count > best_count || (count == best_count && distance < best_distance))) {
      memcpy(match, trial, 4);
      best_count = count;
      best_distance = distance;
    }
  }
}

/* Returns a mask of candidate array indices that matured in this fresh sample.
 * Moving/removed fingers affect only their own candidates. */
static inline uint8_t iqs915x_stuck_observe(struct iqs915x_stuck_tracker *tracker,
                                            const struct iqs915x_stuck_point point[4],
                                            int64_t now_ms, bool remap)
{
  uint8_t match[4], used = 0, expired = 0;
  if (!tracker->enabled) {
    return 0;
  }
  if (remap) {
    iqs915x_stuck_match(tracker, point, match);
  } else {
    for (unsigned int i = 0; i < 4; i++) {
      match[i] = tracker->candidate[i].active ? tracker->candidate[i].slot : 4;
    }
  }
  for (unsigned int i = 0; i < 4; i++) {
    struct iqs915x_stuck_candidate *c = &tracker->candidate[i];
    unsigned int slot = match[i];
    if (slot == 4 || !iqs915x_stuck_fits(c, point[slot], tracker->threshold)) {
      c->active = false;
      continue;
    }
    c->slot = slot;
    c->last_x = point[slot].x;
    c->last_y = point[slot].y;
    if (c->last_x < c->min_x) c->min_x = c->last_x;
    if (c->last_x > c->max_x) c->max_x = c->last_x;
    if (c->last_y < c->min_y) c->min_y = c->last_y;
    if (c->last_y > c->max_y) c->max_y = c->last_y;
    used |= 1U << slot;
    if (now_ms - c->since_ms >= IQS915X_STUCK_TIME_MS) expired |= 1U << i;
  }
  for (unsigned int slot = 0; slot < 4; slot++) {
    if (!point[slot].valid || (used & (1U << slot))) continue;
    for (unsigned int i = 0; i < 4; i++) {
      struct iqs915x_stuck_candidate *c = &tracker->candidate[i];
      if (c->active) continue;
      tracker->next_id++;
      if (tracker->next_id == 0) tracker->next_id++;
      *c = (struct iqs915x_stuck_candidate){
          .active = true, .slot = slot, .id = tracker->next_id, .since_ms = now_ms,
          .min_x = point[slot].x, .max_x = point[slot].x,
          .min_y = point[slot].y, .max_y = point[slot].y,
          .last_x = point[slot].x, .last_y = point[slot].y,
      };
      break;
    }
  }
  return expired;
}
