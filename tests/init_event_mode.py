#!/usr/bin/env python3
"""Run actual initialization control flow with an always-inactive RDY GPIO.

Extract the affected C blocks rather than maintaining a second implementation.
Only Zephyr services, I2C, and unrelated initialization stages are mocked.
"""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def block(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path)
    args = parser.parse_args()
    source = (args.source or root / "drivers/input/iqs915x.c").read_text()
    registers = (root / "drivers/input/iqs915x_regs.h").read_text()
    enum = block(registers, "enum iqs915x_init_step") + ";"
    names = (
        "IQS915X_CONFIG_SETTINGS", "IQS915X_EVENT_MODE", "IQS915X_MANUAL_CONTROL",
        "IQS915X_TP_EVENT", "IQS915X_GESTURE_EVENT", "IQS915X_TP_TOUCH_EVENT",
        "IQS915X_INIT_EVENT_MODE_MAX_RETRIES", "IQS915X_POINTER_RESUME_GUARD_FRAMES",
    )
    definitions = "\n".join(
        re.search(r"^#define " + name + r"\b.*$", source + registers, re.M)[0]
        for name in names
    )
    policy = block(source, "static uint16_t iqs915x_apply_config_settings_policy")
    cases = source[source.index("  case INIT_PREPARE_EVENT_MODE:"):
                   source.index("  case INIT_WAIT_REATI:", source.index("  case INIT_PREPARE_EVENT_MODE:"))]
    thread = source[source.index("static void iqs915x_thread_main("):]
    initialization = block(thread, "    if (!data->initialized)")
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define BIT(n) (1U << (n))
#define K_MSEC(n) (n)
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define WORK_READ_DATA 0
/* Actual register constants and initialization states. */
DEFINITIONS
ENUM
struct iqs915x_data {
  enum iqs915x_init_step init_step;
  bool initialized, active_pending, lp2_pending, enabled;
  int wait_count, work_state, last_info_flags;
  int init_restart_count, init_chunk_retry_count, pointer_resume_guard_frames;
  uint16_t init_pending_cfg, confirmed_config_settings;
  uint32_t applied_generation, transition_generation;
  int requested_enabled, output_enabled, rdy_sem;
  bool is_touching;
};
struct iqs915x_config { int rdy_gpio; };
struct device { struct iqs915x_data *data; const struct iqs915x_config *config; };
static int writes, reads, sleeps, waits, rdy_checks, rejects, write_errors, read_errors;
static uint16_t ic_cfg;
static int atomic_get(const int *value) { return *value; }
static void atomic_set(int *value, int next) { *value = next; }
static void atomic_clear(int *value) { *value = 0; }
static uint32_t iqs915x_request_generation(const struct iqs915x_data *data) {
  (void)data; return 0;
}
static void iqs915x_mark_initialized(struct iqs915x_data *data, bool value) {
  data->initialized = value;
}
static void iqs915x_reset_event_mode_relatch_state(struct iqs915x_data *data) { (void)data; }
static void iqs915x_reset_absolute_tracking(struct iqs915x_data *data) { (void)data; }
static void iqs915x_restart_initialization(const struct device *dev, const char *reason) {
  (void)reason; dev->data->init_step = INIT_FAILED;
}
static int gpio_pin_get_dt(const int *gpio) { (void)gpio; rdy_checks++; return 0; }
static void k_sem_give(int *sem) { *sem = 1; }
static int k_sem_take(int *sem, int timeout) {
  assert(timeout == 2000); waits++;
  if (*sem) { *sem = 0; return 0; }
  return -1;
}
static void k_sleep(int delay) { assert(delay == 2000); sleeps++; }
static int iqs915x_write_reg16(const struct device *dev, uint16_t reg, uint16_t cfg) {
  (void)dev; assert(reg == IQS915X_CONFIG_SETTINGS); writes++;
  if (write_errors) { write_errors--; return -5; }
  if (rejects) { rejects--; return 0; }
  ic_cfg = cfg; return 0;
}
static int iqs915x_read_reg16(const struct device *dev, uint16_t reg, uint16_t *cfg) {
  (void)dev; assert(reg == IQS915X_CONFIG_SETTINGS); reads++;
  if (read_errors) { read_errors--; return -5; }
  *cfg = ic_cfg; return 0;
}
POLICY
static void iqs915x_init_step_handler(const struct device *dev) {
  struct iqs915x_data *data = dev->data;
  int ret;
  switch (data->init_step) {
CASES
  default: assert(data->init_step == INIT_WAIT_REATI); break;
  }
}
static void run(const struct device *dev, int iterations) {
  struct iqs915x_data *data = dev->data;
  const struct iqs915x_config *config = dev->config;
  int ret;
  for (int i = 0; i < iterations && !data->initialized; i++) {
INITIALIZATION
  }
}
static struct iqs915x_data setup(bool enabled) {
  writes = reads = sleeps = waits = rdy_checks = rejects = write_errors = read_errors = 0;
  ic_cfg = 0;
  return (struct iqs915x_data){
    .init_step = INIT_SET_EVENT_MODE,
    .init_pending_cfg = iqs915x_apply_config_settings_policy(0),
    .requested_enabled = enabled,
  };
}
int main(void) {
  (void)k_sleep; /* The old driver does not call the backoff stub. */
  const struct iqs915x_config config = {0};
  struct iqs915x_data data = setup(false);
  struct device dev = {&data, &config};
  /* No split central, no finger event, no RDY: initialization still completes. */
  run(&dev, 12);
  assert(data.initialized && data.init_step == INIT_COMPLETE);
  assert(data.lp2_pending && !data.enabled && !data.output_enabled);
  assert(writes == 1 && reads == 1 && waits == 0 && rdy_checks == 0 && sleeps == 0);
  /* An enabled startup opens the normal output gate instead. */
  data = setup(true);
  run(&dev, 12);
  assert(data.initialized && data.enabled && data.output_enabled && !data.lp2_pending);
  assert(data.pointer_resume_guard_frames == IQS915X_POINTER_RESUME_GUARD_FRAMES);
  /* Read-back mismatch must force the rewrite too, without an RDY. */
  data = setup(false);
  rejects = 1;
  run(&dev, 12);
  assert(data.initialized && writes == 2 && reads == 2 && waits == 0);
  /* Persistent policy mismatch retains the existing retry limit. */
  data = setup(false);
  rejects = 20;
  run(&dev, 8);
  assert(!data.initialized && data.init_step == INIT_FAILED);
  assert(writes == 4 && reads == 4 && waits == 0);
  /* Transient I2C failures back off before retrying, without requiring RDY. */
  data = setup(false);
  write_errors = 1;
  read_errors = 1;
  run(&dev, 12);
  assert(data.initialized && sleeps == 2 && writes == 2 && reads == 2 && waits == 0);
  /* Re-ATI still waits for RDY and performs no forced config transaction. */
  data = setup(false);
  data.init_step = INIT_WAIT_REATI;
  run(&dev, 1);
  assert(!data.initialized && data.init_step == INIT_WAIT_REATI);
  assert(waits == 1 && rdy_checks == 1 && writes == 0 && reads == 0);
  puts("IQS915x Event Mode init: 6 regression scenarios passed");
}
'''
    for marker, code in (("DEFINITIONS", definitions), ("ENUM", enum),
                         ("POLICY", policy), ("CASES", cases),
                         ("INITIALIZATION", initialization)):
        harness = harness.replace(marker, code)
    with tempfile.TemporaryDirectory(prefix="iqs915x-init-test-") as temporary:
        source_path = Path(temporary) / "test.c"
        executable = Path(temporary) / "test"
        source_path.write_text(harness)
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        str(source_path), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
