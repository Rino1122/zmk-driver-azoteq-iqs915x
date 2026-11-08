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
        "IQS915X_TRACKPAD_SETTINGS", "IQS915X_X_RESOLUTION", "IQS915X_CONFIG_SETTINGS", "IQS915X_EVENT_MODE", "IQS915X_MANUAL_CONTROL",
        "IQS915X_INFO_FLAGS", "IQS915X_SHOW_RESET", "IQS915X_SYSTEM_CONTROL",
        "IQS915X_ACK_RESET", "IQS915X_INIT_SHOW_RESET_CLEAR_MAX_WAIT",
        "IQS915X_REATI_TP", "IQS915X_REATI_OCCURRED", "IQS915X_ALP_REATI_OCCURRED",
        "IQS915X_MODE_LP2", "IQS915X_INIT_REATI_MAX_WAIT",
        "IQS915X_TP_EVENT", "IQS915X_GESTURE_EVENT", "IQS915X_TP_TOUCH_EVENT",
        "IQS915X_TP_REATI_ENABLE", "IQS915X_REATI_EVENT",
        "IQS915X_SWITCH_EVENT", "IQS915X_SNAP_EVENT",
        "IQS915X_ALP_REATI_ENABLE", "IQS915X_ALP_EVENT", "IQS915X_MODE_ACTIVE",
        "IQS915X_INIT_EVENT_MODE_MAX_RETRIES", "IQS915X_POINTER_RESUME_GUARD_FRAMES",
        "IQS915X_FORCE_COMMS_METHOD", "IQS915X_TERMINATE_COMMS",
        "IQS915X_INIT_BUSY_RETRY_MS", "IQS915X_INIT_BUSY", "IQS915X_ATI_ERROR", "IQS915X_INIT_ATI_TIMEOUT_MS",
    )
    definitions = "\n".join(
        re.search(r"^#define " + name + r"\b.*$", source + registers, re.M)[0]
        for name in names
    )
    policy = block(source, "static uint16_t iqs915x_apply_config_settings_policy")
    policy += "\n" + block(source, "static uint16_t iqs915x_config_settings_without_event_mode")
    handler = block(source, "static int iqs915x_init_step_handler(")
    cases = handler[handler.index("  case INIT_PREPARE_EVENT_MODE:"):
                   handler.index("  case INIT_WAIT_REATI:", handler.index("  case INIT_PREPARE_EVENT_MODE:"))]
    cases += handler[handler.index("  case INIT_CHECK_SHOW_RESET:"):
                    handler.index("  case INIT_SOFTWARE_RESET:")]
    cases += handler[handler.index("  case INIT_PREPARE_STREAMING:"):
                    handler.index("  case INIT_PREPARE_EVENT_MODE:")]
    cases += block(handler, "  case INIT_ACK_RESET:")
    cases += block(handler, "  case INIT_VERIFY_SHOW_RESET_CLEAR:")
    cases += block(handler, "  case INIT_WAIT_REATI:")
    thread = source[source.index("static void iqs915x_thread_main("):]
    initialization = block(thread, "    if (!data->initialized)")
    reset_gpio = block(source, "  if (config->reset_gpio.port)")
    harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#define TRACKPAD_SETTINGS_0_0 0x28
#define GPIO_OUTPUT_INACTIVE 0
#define BIT(n) (1U << (n))
#define K_MSEC(n) (n)
#define LOG_INF(...) ((void)0)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define WORK_READ_DATA 0
#define IQS915X_POWER_ACTIVE 0
#define IQS915X_INITIALIZING 1
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
  int actual_mode, probe_failures, lifecycle, ati_error, ati_sequence;
  uint32_t init_rdy_accesses, init_force_accesses, init_window_timeouts;
  int64_t ati_deadline;
  bool is_touching, streaming_expected, ati_error_seen, comm_fallback_active;
  uint16_t confirmed_mode, init_data_offset;
};
struct reset_gpio { int port; };
struct iqs915x_config { int rdy_gpio; struct reset_gpio reset_gpio; };
struct device { struct iqs915x_data *data; const struct iqs915x_config *config; };
static int iqs915x_check_fixed_sensor_config(const struct device *dev){(void)dev;return 0;}
static int64_t now;
static int64_t k_uptime_get(void) { return now; }
static int busy_waits, busy_edges, read_error_code;
static int writes, reads, sleeps, waits, rdy_checks, rejects, write_errors, read_errors;
static uint16_t ic_cfg, ic_info;
static bool reati_ready, reset_during_reati;
static int reati_requests, stream_reads;
static uint16_t last_write_reg, last_write_value;
struct iqs915x_stream_data { uint16_t info_flags; };
static int atomic_get(const int *value) { return *value; }
static void atomic_set(int *value, int next) { *value = next; }
static void atomic_inc(int *value) { (*value)++; }
static void atomic_clear(int *value) { *value = 0; }
static uint32_t iqs915x_request_generation(const struct iqs915x_data *data) {
  (void)data; return 0;
}
static void iqs915x_mark_initialized(struct iqs915x_data *data, bool value) {
  data->initialized = value;
}
static unsigned timeout_captures;
static void iqs915x_ati_timeout_capture(struct iqs915x_data *d){assert(d->init_step==INIT_WAIT_REATI);timeout_captures++;}
static void iqs915x_mismatch_capture(struct iqs915x_data *d,uint16_t e,uint16_t o) {(void)d;(void)e;(void)o;}
static void iqs915x_reset_absolute_tracking(struct iqs915x_data *data) { (void)data; }
static void iqs915x_restart_initialization(const struct device *dev, const char *reason) {
  (void)reason; if(dev->data->init_step==INIT_WAIT_REATI)assert(timeout_captures);
  dev->data->init_step = INIT_FAILED;
}
static int gpio_pin_get_dt(const int *gpio) { (void)gpio; rdy_checks++; return reati_ready; }
static void k_sem_give(int *sem) { *sem = 1; }
static int k_sem_take(int *sem, int timeout) {
  if (timeout == IQS915X_INIT_BUSY_RETRY_MS) {
    busy_waits++;
    if (busy_edges) { busy_edges--; ic_info=IQS915X_SHOW_RESET; return 0; }
    return -1;
  }
  assert(timeout == 100); waits++;
  if (*sem) { *sem = 0; return 0; }
  return -1;
}
static void k_sleep(int delay) { assert(delay == 100 || delay == 2000); sleeps++; }
static int iqs915x_write_reg16(const struct device *dev, uint16_t reg, uint16_t cfg) {
  (void)dev; writes++;
  last_write_reg=reg; last_write_value=cfg;
  if (write_errors) { write_errors--; return -5; }
  if (reg==IQS915X_SYSTEM_CONTROL) {
    /* Normal warm startup must only request Active TP Re-ATI, never SW reset. */
    if (cfg==IQS915X_ACK_RESET){ic_info &= ~IQS915X_SHOW_RESET;reati_ready=true;return 0;}
    if (cfg==IQS915X_MODE_ACTIVE) return 0;
    assert(cfg==(IQS915X_MODE_ACTIVE|IQS915X_REATI_TP));
    reati_requests++; reati_ready=true;
    ic_info=reset_during_reati ? IQS915X_SHOW_RESET : IQS915X_REATI_OCCURRED;
    return 0;
  }
  assert(reg==IQS915X_CONFIG_SETTINGS);
  if (rejects) { rejects--; return 0; }
  ic_cfg = cfg; return 0;
}
static int iqs915x_read_reg16(const struct device *dev, uint16_t reg, uint16_t *cfg) {
  (void)dev; reads++;
  if (read_errors) { read_errors--; return read_error_code; }
  if (reg==IQS915X_X_RESOLUTION) { *cfg=6000; return 0; }
  assert(reg==IQS915X_CONFIG_SETTINGS || reg==IQS915X_INFO_FLAGS);
  *cfg = reg==IQS915X_CONFIG_SETTINGS ? ic_cfg : ic_info; return 0;
}
static int iqs915x_read_stream(const struct device *dev, struct iqs915x_stream_data *stream) {
  (void)dev; assert(reati_ready || now>=5000); assert(!(ic_cfg & IQS915X_EVENT_MODE));
  stream_reads++; stream->info_flags=ic_info; reati_ready=false; return 0;
}
static int gpio_is_ready_dt(const struct reset_gpio *gpio) { return gpio->port != 0; }
static int reset_configurations;
static int gpio_pin_configure_dt(const struct reset_gpio *gpio, int flags) {
  assert(gpio->port && flags==GPIO_OUTPUT_INACTIVE); reset_configurations++; return 0;
}
static int configure_reset(const struct device *dev) {
  const struct iqs915x_config *config=dev->config;
  int ret;
RESET_GPIO
  return 0;
}
POLICY
static uint8_t retained_trackpad_settings;
static int iqs915x_read_block(const struct device *dev,uint16_t reg,uint8_t *bytes,uint16_t len) {
  (void)dev;assert(reg==IQS915X_TRACKPAD_SETTINGS&&len==1);bytes[0]=retained_trackpad_settings;return 0;
}
static int mask_check_result,mask_checks;
static int iqs915x_check_channel_profile(const struct device *dev) {(void)dev;mask_checks++;return mask_check_result;}
static void iqs915x_get_init_data_reg16(const void *config, int reg, uint16_t *out) { (void)config; (void)reg; *out=6000; }
static void iqs915x_terminal(struct iqs915x_data *data, bool absent, int error) { (void)absent; (void)error; data->init_step=INIT_FAILED; }
static int iqs915x_init_step_handler(const struct device *dev) {
  struct iqs915x_data *data = dev->data;
  const struct iqs915x_config *config = dev->config;
  int ret;
  switch (data->init_step) {
CASES
  default: assert(data->init_step == INIT_WRITE_INIT_DATA); break;
  }
  return 0;
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
  busy_waits = busy_edges = 0; read_error_code=-5;
  writes = reads = sleeps = waits = rdy_checks = rejects = write_errors = read_errors = 0;
  ic_cfg = ic_info = 0; retained_trackpad_settings=8;
  reati_ready=reset_during_reati=false; reati_requests=stream_reads=0;
  return (struct iqs915x_data){.ati_deadline=5000,
    .init_step = INIT_SET_EVENT_MODE,
    .init_pending_cfg = iqs915x_apply_config_settings_policy(IQS915X_ALP_EVENT),
    .requested_enabled = enabled,
  };
}
int main(void) {
  assert(!(iqs915x_apply_config_settings_policy(0x0de6) &
           (IQS915X_FORCE_COMMS_METHOD | IQS915X_TERMINATE_COMMS)));
  (void)k_sleep; /* The old driver does not call the backoff stub. */
  const struct iqs915x_config config = {0};
  struct iqs915x_data data = setup(false);
  struct device dev = {&data, &config};
  /* Real cold path: ACK -> reset clear -> Streaming -> readback -> ATI. */
  for(int enabled=0;enabled<2;enabled++) {
    data=setup(enabled);data.init_step=INIT_ACK_RESET;
    ic_info=IQS915X_SHOW_RESET|IQS915X_ATI_ERROR;
    ic_cfg=iqs915x_apply_config_settings_policy(0);
    reati_ready=true;run(&dev,2);
    assert(data.init_step==INIT_PREPARE_STREAMING && !reati_requests);
    run(&dev,1);assert(data.init_step==INIT_SET_STREAMING);
    run(&dev,1);assert(data.init_step==INIT_TRACE_STREAMING && !reati_requests);
    assert(!(ic_cfg & IQS915X_EVENT_MODE));
    run(&dev,1);assert(data.init_step==INIT_REQUEST_REATI && !reati_requests);
    run(&dev,1);assert(data.init_step==INIT_WAIT_REATI && reati_requests==1);
    run(&dev,12);assert(data.initialized);
  }
  /* Busy/error/mismatch cannot queue ATI; matching readback opens the gate. */
  data=setup(true);data.init_step=INIT_SET_STREAMING;
  data.init_pending_cfg=0x0c86;run(&dev,1);
  assert(data.init_step==INIT_TRACE_STREAMING && writes==1 && !reati_requests);
  read_errors=1;run(&dev,1);
  assert(data.init_step==INIT_TRACE_STREAMING && reads==1 && !reati_requests);
  run(&dev,1);assert(data.init_step==INIT_REQUEST_REATI && !reati_requests);
  run(&dev,1);assert(data.init_step==INIT_WAIT_REATI && reati_requests==1);
  data=setup(true);data.init_step=INIT_TRACE_STREAMING;ic_cfg=0xEEEE;
  run(&dev,1);assert(data.init_step==INIT_TRACE_STREAMING && reads==1 && !writes && !reati_requests);
  assert(!data.wait_count && busy_waits==1);
  ic_cfg=data.init_pending_cfg;run(&dev,1);assert(data.init_step==INIT_REQUEST_REATI && !reati_requests);
  run(&dev,1);assert(reati_requests==1);
  data=setup(true);data.init_step=INIT_TRACE_STREAMING;ic_cfg=0x061e;
  run(&dev,1);assert(data.init_step==INIT_SET_STREAMING && data.wait_count==1 && !reati_requests);
  run(&dev,3);assert(data.init_step==INIT_WAIT_REATI && reati_requests==1);
  data=setup(true);data.init_step=INIT_TRACE_STREAMING;ic_cfg=0x061e;rejects=10;
  run(&dev,10);assert(data.init_step==INIT_FAILED && !reati_requests);
  data=setup(false);
  /* No split central, no finger event, no RDY: initialization still completes. */
  run(&dev, 12);
  assert(data.initialized && data.init_step == INIT_COMPLETE);
  assert(data.lp2_pending && !data.enabled && !data.output_enabled);
  assert(!data.streaming_expected && (ic_cfg & IQS915X_EVENT_MODE));
  assert(!(ic_cfg & IQS915X_ALP_EVENT));
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
  /* Actual preparation and retry must select Event Mode for either output
   * request, and clear a stale ALP Event bit before writing the policy. */
  for (int enabled=0; enabled<2; enabled++) {
    data = setup(enabled);
    data.init_step = INIT_PREPARE_EVENT_MODE;
    ic_cfg = IQS915X_ALP_EVENT;
    iqs915x_init_step_handler(&dev);
    assert(data.init_pending_cfg & IQS915X_EVENT_MODE);
    assert(!(data.init_pending_cfg & IQS915X_ALP_EVENT));
    rejects = 1;
    run(&dev, 12);
    assert(data.initialized && !data.streaming_expected);
    assert((ic_cfg & IQS915X_EVENT_MODE) && !(ic_cfg & IQS915X_ALP_EVENT));
  }
  /* A busy configuration is never converted into a policy write or mismatch.
   * Event waiting remains bounded by the outer startup deadline. */
  data=setup(false);data.init_step=INIT_PREPARE_EVENT_MODE;ic_cfg=0xEEEE;
  run(&dev,5);assert(data.init_step==INIT_PREPARE_EVENT_MODE && !writes && busy_waits==5);
  ic_cfg=0;run(&dev,1);assert(data.init_step==INIT_SET_EVENT_MODE);
  run(&dev,1);assert(data.init_step==INIT_CONFIRM_EVENT_MODE);
  ic_cfg=0xEEEE;run(&dev,5);
  assert(data.init_step==INIT_CONFIRM_EVENT_MODE && data.wait_count==0 && writes==1);
  ic_cfg=iqs915x_apply_config_settings_policy(0);run(&dev,1);
  assert(data.initialized);
  /* Warm MCU boot retains calibration, including a recoverable ATI error. */
  for (int enabled=0;enabled<2;enabled++) {
    data=setup(enabled);data.init_step=INIT_CHECK_SHOW_RESET;
    ic_info=IQS915X_MODE_LP2 | IQS915X_ATI_ERROR;
    ic_cfg=iqs915x_apply_config_settings_policy(0) & ~IQS915X_TP_EVENT;
    run(&dev,1);
    assert(data.init_step==INIT_PREPARE_EVENT_MODE && data.ati_error);
    run(&dev,12);
    assert(data.initialized && reati_requests==0 && stream_reads==0);
    assert(!data.init_data_offset && !data.streaming_expected);
    assert(data.output_enabled==enabled && data.lp2_pending==!enabled);
    assert(ic_cfg & IQS915X_EVENT_MODE);
  }
  /* An Occurred notification with ATI Error completes configuration, leaving
   * calibration recovery to the IC. No notification uses elapsed time. */
  data=setup(true);data.init_step=INIT_WAIT_REATI;data.ati_deadline=5000;
  ic_cfg=0;reati_ready=true;ic_info=IQS915X_REATI_OCCURRED|IQS915X_ATI_ERROR;
  run(&dev,1);assert(data.init_step==INIT_PREPARE_EVENT_MODE && data.ati_error);
  data=setup(false);data.init_step=INIT_WAIT_REATI;data.ati_deadline=5000;
  ic_cfg=0;ic_info=0;now=4999;
  for(int i=0;i<100;i++){reati_ready=true;run(&dev,1);}
  assert(data.init_step==INIT_WAIT_REATI); /* even after >61 reads */
  now=5000;reati_ready=false;run(&dev,1);
  assert(data.init_step==INIT_FAILED && timeout_captures==1); /* captured before restart */
  now=0;

  /* Real SHOW_RESET retains the full init-data transfer path. */
  data=setup(false); data.init_step=INIT_CHECK_SHOW_RESET;
  data.init_data_offset=123; data.init_chunk_retry_count=2;
  ic_info=IQS915X_SHOW_RESET;
  run(&dev,1);
  assert(data.init_step==INIT_WRITE_INIT_DATA && !data.init_data_offset);
  assert(!data.init_chunk_retry_count && writes==1 && !reati_requests && !waits);
  /* Busy uses a bounded RDY wait, even without edges; I2C errors back off. */
  data=setup(false); data.init_step=INIT_CHECK_SHOW_RESET; ic_info=0xEEEE;
  run(&dev,1); assert(data.init_step==INIT_CHECK_SHOW_RESET && !sleeps && busy_waits==1 && !writes);
  ic_info=IQS915X_MODE_LP2; run(&dev,1);
  read_errors=1; run(&dev,1);
  assert(data.init_step==INIT_PREPARE_EVENT_MODE && sleeps==1);
  run(&dev,12); assert(data.initialized && reati_requests==0);
  /* A new RDY wakes the busy wait and the very next check starts transfer. */
  data=setup(false); data.init_step=INIT_CHECK_SHOW_RESET; ic_info=0xEEEE;
  busy_edges=1;
  run(&dev,1);
  assert(data.init_step==INIT_CHECK_SHOW_RESET && busy_waits==1 && !sleeps);
  run(&dev,1);
  assert(data.init_step==INIT_WRITE_INIT_DATA && reads==2 && writes==1 && !waits);
  /* Actual startup read errors retain the long backoff, without a busy wait. */
  data=setup(false); data.init_step=INIT_CHECK_SHOW_RESET; read_errors=1; read_error_code=-EBUSY;
  run(&dev,1);
  assert(data.init_step==INIT_CHECK_SHOW_RESET && sleeps==1 && !busy_waits);
  ic_info=IQS915X_SHOW_RESET; run(&dev,1);
  assert(data.init_step==INIT_WRITE_INIT_DATA && reads==2);
  /* A new reset while waiting for a cold ATI returns to full transfer. */
  data=setup(false);data.init_step=INIT_WAIT_REATI;
  ic_cfg=0;ic_info=IQS915X_SHOW_RESET;reati_ready=true;
  run(&dev,1);assert(data.init_step==INIT_CHECK_SHOW_RESET);
  run(&dev,1);assert(data.init_step==INIT_WRITE_INIT_DATA);
  /* Optional reset wiring must not reset the IC before SHOW_RESET is read. */
  struct iqs915x_config wired=config;
  wired.reset_gpio.port=1;
  dev.config=&wired;
  assert(!configure_reset(&dev) && reset_configurations==1);
  dev.config=&config;
  assert(!configure_reset(&dev) && reset_configurations==1);
  data=setup(false);data.init_step=INIT_CHECK_SHOW_RESET;retained_trackpad_settings=0;
  run(&dev,1);assert(data.init_step==INIT_SOFTWARE_RESET && !data.initialized);
  puts("IQS915x init: Event verification, warm startup without RDY, forced Re-ATI and reset routing passed");
}
'''
    for marker, code in (("DEFINITIONS", definitions), ("ENUM", enum),
                         ("POLICY", policy), ("CASES", cases),
                         ("INITIALIZATION", initialization), ("RESET_GPIO", reset_gpio)):
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
