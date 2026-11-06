#!/usr/bin/env python3
"""Exercise the actual runtime power and automatic Re-ATI handling with a deterministic IC/clock.

The register/data structures, RDY wait, I2C wrappers, state machine and runtime
loop prefix are compiled from the driver. Only Zephyr services and the IC are
mocked. No firmware workspace is required.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import sys
sys.dont_write_bytecode = True
from init_event_mode import block

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'drivers/input/iqs915x.c').read_text()
STUB = r'''
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#define BIT(n) (1U << (n))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define CONTAINER_OF(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define CONFIG_INPUT_AZOTEQ_IQS915X_THREAD_STACK_SIZE 1
#define K_KERNEL_STACK_MEMBER(name,size) char name[size]
#define K_MSEC(n) (n)
#define GPIO_INT_DISABLE 0
static int gpio_pin_interrupt_configure_dt(const void *p, int mode) { (void)p; (void)mode; return 0; }
#define K_FOREVER INT64_MAX
#define K_NO_WAIT 0
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_ERR(...) ((void)0)
typedef int atomic_t;
typedef int atomic_val_t;
typedef int64_t k_timeout_t;
struct device { void *data; const void *config; };
struct gpio_dt_spec { int pin; void *port; };
struct i2c_dt_spec { int address; };
struct gpio_callback { int unused; };
struct k_sem { int count; };
struct k_thread { int unused; };
struct k_mutex { int unused; };
struct k_work { int unused; };
struct k_work_delayable { struct k_work work; int scheduled; };
static int atomic_get(const atomic_t *p) { return *p; }
static void atomic_set(atomic_t *p,int n) { *p=n; }
static void atomic_clear(atomic_t *p) { *p=0; }
static int k_mutex_lock(struct k_mutex *m, int64_t timeout) { (void)m; (void)timeout; return 0; }
static void k_mutex_unlock(struct k_mutex *m) { (void)m; }
static bool k_is_in_isr(void) { return false; }
static bool device_is_ready(const struct device *dev) { return dev != NULL; }
static void atomic_inc(atomic_t *v) { (*v)++; }
static bool atomic_cas(atomic_t *v,int old,int next) { if (*v!=old) return false; *v=next; return true; }
static void k_sem_reset(struct k_sem *s) { s->count=0; }
static void k_sem_give(struct k_sem *s) { s->count=1; }
'''
HARNESS = r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include "iqs915x_internal.h"
#include "iqs915x_sensor_config.h"
DEFINITIONS
#define IQS915X_POWER_TRANSITION_WAIT_MS 2000
enum pm_device_action { PM_DEVICE_ACTION_SUSPEND, PM_DEVICE_ACTION_RESUME };
static void (*wait_hook)(void);
static int64_t now, next_rdy;
static uint16_t ic_mode, ic_cfg, info_extra, fingers;
static bool no_rdy, touch, event_rdy, stale_tp;
static int input_resets, input_frames, reads, writes;
static int reject_cfg, reject_mode, io_errors;
static int sem_waits, force_stretch_ms, forced_transactions;
static int boot_calls, boot_result, parked;
static uint16_t txn_reg[1024];
static bool txn_write[1024];
static int64_t txn_time[1024];
static int txns;
static uint16_t coords[8];
static uint16_t period(void) { return ic_mode==IQS915X_MODE_LP2 ? 500 : 10; }
static int64_t k_uptime_get(void) { return now; }
static bool periodic_rdy(void) {
  /* Model a final Active TP event persisting while ALP scans in LP2. */
  return !(ic_cfg & IQS915X_EVENT_MODE) ||
         (stale_tp && ic_mode==IQS915X_MODE_LP2 && (ic_cfg & IQS915X_TP_EVENT));
}
static int gpio_pin_get_dt(const struct gpio_dt_spec *g) {
  (void)g; return !no_rdy && (event_rdy ||
      (periodic_rdy() && now >= next_rdy));
}
static int k_sem_take(struct k_sem *s,k_timeout_t timeout) {
  sem_waits++;
  if (wait_hook && (timeout==2000 || timeout==K_FOREVER)) wait_hook();
  if (s->count) { s->count=0; return 0; }
  if (timeout==K_FOREVER) { parked++; return -1; }
  int64_t end=now+timeout;
  if (!no_rdy && periodic_rdy() && next_rdy <= end) {
    now=MAX(now,next_rdy); return 0;
  }
  assert(end!=INT64_MAX); now=end; return -1;
}
static void k_sleep(int64_t delay) { now+=delay; }
static void record(uint16_t reg,bool write) {
  if (no_rdy || (!event_rdy && ((ic_cfg & IQS915X_EVENT_MODE) || now < next_rdy))) {
    forced_transactions++;
    now+=force_stretch_ms; /* IC-imposed latency; no driver RDY timeout. */
  }
  event_rdy=false;
  assert(txns<1024); txn_reg[txns]=reg; txn_write[txns]=write; txn_time[txns++]=now;
  next_rdy=now+period();
}
static void put16(uint8_t *p,uint16_t v) { p[0]=v; p[1]=v>>8; }
static bool simulate_ati;
static int i2c_write_dt(const struct i2c_dt_spec *i,const uint8_t *b,size_t len) {
  (void)i; assert(len==4); uint16_t reg=b[0]|b[1]<<8, value=b[2]|b[3]<<8;
  record(reg,true); writes++;
  if (reg==IQS915X_CONFIG_SETTINGS) {
    if (reject_cfg) reject_cfg--; else ic_cfg=value;
  } else {
    assert(reg==IQS915X_SYSTEM_CONTROL);
    if (reject_mode) reject_mode--; else ic_mode=value & IQS915X_CHARGING_MODE_MASK;
    assert(!(value & (IQS915X_TP_RESEED | BIT(4))));
  }
  if(simulate_ati && (value & IQS915X_REATI_TP))info_extra=IQS915X_REATI_OCCURRED;
  next_rdy=now+period();
  if (io_errors) { io_errors--; return -EIO; }
  return 0;
}
static int i2c_write_read_dt(const struct i2c_dt_spec *i,const uint8_t *address,
                            size_t alen,uint8_t *b,size_t len) {
  (void)i; assert(alen==2); uint16_t reg=address[0]|address[1]<<8;
  record(reg,false); reads++;
  if (io_errors) { io_errors--; return -EIO; }
  uint16_t info=ic_mode|info_extra|(touch?IQS915X_GLOBAL_TP_TOUCH:0);
  if (reg==IQS915X_CONFIG_SETTINGS) { assert(len==2); put16(b,ic_cfg); }
  else if (reg==IQS915X_SYSTEM_CONTROL) {
    assert(len==2 || len==16);memset(b,0x5A,len);put16(b,ic_mode);
    if(len==16)put16(b+2,ic_cfg);
  }
  else if (reg==IQS915X_INFO_FLAGS) { assert(len==2); put16(b,info); info_extra = info == 0xEEEE ? 0 : info_extra & IQS915X_ATI_ERROR; }
  else {
    assert(reg==IQS915X_REL_X && len==44); memset(b,0,len);
    put16(b+12,info); put16(b+14,fingers);
    for (int j=0;j<4;j++) { put16(b+16+j*8,coords[j*2]); put16(b+18+j*8,coords[j*2+1]); }
    info_extra = info == 0xEEEE ? 0 : info_extra & IQS915X_ATI_ERROR;
  }
  return 0;
}
static void iqs915x_reset_input_session(struct iqs915x_data *d) {
  d->contacts.blocked=true; d->contacts.probe_pending=true;
  input_resets++; d->buttons_pressed=0; d->active_tap_hold=false;
  d->finger_tracker.count_change_pending=false;
}
static void iqs915x_mark_initialized(struct iqs915x_data *d,bool init) { d->initialized=init; }
static void trace_sample(struct iqs915x_data *d,const struct iqs915x_stream_data *s) {(void)d;(void)s;}
static void iqs915x_processing_sample(const struct device *dev,struct iqs915x_stream_data *s) {
  (void)dev; s->abs_x+=50;
}
static void iqs915x_log_stream_coordinates(const struct iqs915x_stream_data *s) { (void)s; }
FUNCTIONS
static int iqs915x_init_step_handler(const struct device *dev) { (void)dev; boot_calls++; return boot_result; }
BOOT
static void run(struct iqs915x_data *data,int steps) {
  const struct device *dev=data->dev;
  int ret;
  struct iqs915x_stream_data last_stream={0};
  uint32_t last_stream_generation=0;
  for (int iteration=0;iteration<steps && data->initialized;iteration++) {
PREFIX
    input_frames++;
  }
}
static struct iqs915x_config config;
static struct device dev;
static struct iqs915x_data d;
static void setup(bool output) {
  now=0; next_rdy=output?INT64_MAX:500; txns=0;
  no_rdy=touch=event_rdy=stale_tp=false; input_resets=input_frames=reads=writes=0;
  sem_waits=force_stretch_ms=forced_transactions=0;
  reject_cfg=reject_mode=io_errors=info_extra=fingers=0;
  for (int j=0;j<8;j++) coords[j]=UINT16_MAX;
  memset(&d,0,sizeof(d)); memset(&config,0,sizeof(config));
  dev=(struct device){&d,&config}; d.dev=&dev; d.initialized=true;
  d.swipe_resolution_x=d.swipe_resolution_y=1000;
  d.active_sampling_period_ms=10;
  d.enabled=d.requested_enabled=d.output_enabled=output;
  d.requested_mode=output?IQS915X_POWER_ACTIVE:IQS915X_POWER_LP2;
  ic_mode=d.confirmed_mode=output?IQS915X_MODE_ACTIVE:IQS915X_MODE_LP2;
  d.streaming_expected=false;
  ic_cfg=d.confirmed_config_settings=iqs915x_config_settings_for_mode(0,ic_mode);
}
static void complete_pm(void) {
  iqs915x_apply_pending_power_request(&d);
  for (int i=0;i<30 && d.work_state!=WORK_READ_DATA;i++) iqs915x_mode_step(&d);
}
static void until_idle(void) {
  for (int i=0;i<40 && d.work_state!=WORK_READ_DATA;i++) run(&d,1);
  assert(d.initialized && d.work_state==WORK_READ_DATA);
}
static void diagnostic_wake(void){atomic_set(&d.channel_pending,1);k_sem_give(&d.rdy_sem);}
int main(void) {
  /* Diagnostic ATI works from LP2 without modifying user intent and restores it. */
  setup(false);d.lifecycle=IQS915X_READY;simulate_ati=true;
  uint32_t job;
  assert(!iqs915x_diagnostic_start(&dev,true,&job));
  assert(iqs915x_diagnostic_start(&dev,true,&job)==-EBUSY);
  atomic_clear(&d.channel_pending);iqs915x_diagnostic_execute(&d);
  assert(!d.channel_snapshot.status && !d.channel_snapshot.restore_status);
  assert(!d.channel_snapshot.ati_status && !(d.channel_snapshot.info_after & IQS915X_ATI_ERROR));
  assert(d.requested_mode==IQS915X_POWER_LP2 && d.actual_mode==IQS915X_POWER_LP2 && !d.output_enabled);
  assert(!d.diagnostic_job && !d.channel_running);simulate_ati=false;

  setup(false);d.lifecycle=IQS915X_READY;d.requested_mode=IQS915X_POWER_SUSPEND;
  d.actual_mode=IQS915X_POWER_SUSPEND;d.ic_suspended=true;
  assert(!iqs915x_diagnostic_start(&dev,true,&job));
  atomic_clear(&d.channel_pending);iqs915x_diagnostic_execute(&d);
  assert(d.channel_snapshot.status==-ETIMEDOUT && d.channel_snapshot.ati_status==-ETIMEDOUT);
  assert(!d.channel_snapshot.restore_status && d.ic_suspended && d.actual_mode==IQS915X_POWER_SUSPEND);
  setup(false);d.lifecycle=IQS915X_READY;
  assert(!iqs915x_diagnostic_start(&dev,false,&job));d.request_generation++;
  atomic_clear(&d.channel_pending);iqs915x_diagnostic_execute(&d);
  assert(d.channel_snapshot.status==-ECANCELED && !writes && !d.diagnostic_job);
  d.pm_suspended=1;assert(iqs915x_diagnostic_start(&dev,false,&job)==-EACCES);

  setup(true);d.ati_error=1;d.request_generation=7;
  assert(iqs915x_ati_assist(&d)==2000 && !writes);
  now=1999;assert(iqs915x_ati_assist(&d)==2000 && !writes);
  d.requested_enabled=0;assert(iqs915x_ati_assist(&d)==INT64_MAX);
  now=3000;d.requested_enabled=1;assert(iqs915x_ati_assist(&d)==5000);
  now=5000;assert(iqs915x_ati_assist(&d)==10000 && writes==1);
  now=9999;iqs915x_ati_assist(&d);assert(writes==1);
  now=10000;assert(iqs915x_ati_assist(&d)==INT64_MAX && writes==1);
  now=12000;iqs915x_ati_assist(&d);assert(writes==1);

  /* Startup uses real per-transaction gate, not only the outer dispatch.
   * Stale semaphore wakes cannot authorize a read while RDY is inactive. */
  setup(true);d.initialized=false;d.init_deadline=15000;
  d.lifecycle=IQS915X_INITIALIZING;d.init_step=INIT_TRACE_STREAMING;
  no_rdy=true;k_sem_give(&d.rdy_sem);uint16_t observed=0x1234;
  assert(iqs915x_read_reg16(&dev,IQS915X_CONFIG_SETTINGS,&observed)==-EAGAIN);
  assert(!txns && observed==0x1234 && d.init_window_timeouts==1);
  /* An actual subsequent RDY opens the window; consecutive reads wait for
   * the next measurement, and block reads share the same STOP timestamp. */
  no_rdy=false;ic_cfg &= ~IQS915X_EVENT_MODE;next_rdy=now+10;
  assert(!iqs915x_read_reg16(&dev,IQS915X_CONFIG_SETTINGS,&observed));
  int64_t first=now;uint8_t bytes[2];
  assert(!iqs915x_read_block(&dev,IQS915X_CONFIG_SETTINGS,bytes,2));
  assert(now>=first+10 && d.comm_completed_ms==now);
  assert(d.init_rdy_accesses==2 && !forced_transactions);
  /* Quiet retained Event Mode may use bounded Force Comms; strict phases
   * and the total deadline may never fall back to speculative traffic. */
  no_rdy=true;d.init_step=INIT_CONFIRM_EVENT_MODE;first=now;
  assert(!iqs915x_read_reg16(&dev,IQS915X_CONFIG_SETTINGS,&observed));
  assert(now>=first+150 && d.init_force_accesses==1 && forced_transactions==1);
  int before=txns;d.init_deadline=now;
  assert(iqs915x_read_reg16(&dev,IQS915X_CONFIG_SETTINGS,&observed)==-EAGAIN);
  assert(txns==before);
  /* First mismatch survives subsequent writes, IC retry and terminal evidence. */
  setup(true);d.lifecycle=IQS915X_INITIALIZING;d.init_step=INIT_CONFIRM_EVENT_MODE;d.wait_count=1;
  ic_cfg=0x061e;iqs915x_mismatch_capture(&d,0x0d86,0x061e);
  struct iqs915x_mismatch_snapshot mismatch;int first_txns=txns;
  assert(!iqs915x_mismatch_get(&dev,&mismatch) && txns==first_txns && reads==2 && !writes);
  assert(mismatch.expected==0x0d86 && mismatch.observed==0x061e && !mismatch.raw_status);
  assert(mismatch.raw[2]==0x1e && mismatch.raw[3]==0x06 && mismatch.raw[4]==0x5A);
  assert(mismatch.evidence.config==0x061e && mismatch.evidence.valid_mask==7);
  ic_cfg=0x0d86;iqs915x_mismatch_capture(&d,0x0d86,0x0000);
  assert(txns==first_txns); /* no recapture on retries */
  info_extra=0xEEEE;iqs915x_terminal(&d,false,-ETIMEDOUT);
  assert(d.fault_snapshot.busy && d.fault_snapshot.error==-ETIMEDOUT);
  assert(!iqs915x_mismatch_get(&dev,&mismatch) && mismatch.observed==0x061e && mismatch.evidence.restart_count==0);
  setup(true);io_errors=1;iqs915x_mismatch_capture(&d,0x0d86,0x061e);
  assert(!iqs915x_mismatch_get(&dev,&mismatch) && mismatch.raw_status==-EIO && mismatch.evidence.valid_mask==1);
  setup(true);ic_cfg=0xEEEE;iqs915x_mismatch_capture(&d,0x0d86,0x061e);
  assert(!iqs915x_mismatch_get(&dev,&mismatch) && mismatch.evidence.busy && !(mismatch.evidence.valid_mask&2));
  setup(true);assert(iqs915x_mismatch_get(&dev,&mismatch)==-ENODATA);

  /* Pre-reset ATI timeout record is bounded and independent of terminal evidence. */
  setup(true);struct iqs915x_fault_snapshot ati;
  assert(iqs915x_ati_timeout_get(&dev,&ati)==-ENODATA);
  d.lifecycle=IQS915X_INITIALIZING;d.init_step=INIT_WAIT_REATI;d.wait_count=69;
  ic_cfg=0x0c86;info_extra=IQS915X_ATI_ERROR;
  iqs915x_ati_timeout_capture(&d);first_txns=txns;
  assert(reads==3 && !writes && !iqs915x_ati_timeout_get(&dev,&ati) && txns==first_txns);
  assert(ati.error==-ETIMEDOUT && ati.stop_status==-ENODATA && ati.valid_mask==7);
  assert(ati.config==0x0c86 && ati.wait_count==69 && ati.init_step==INIT_WAIT_REATI);
  iqs915x_ati_timeout_capture(&d);assert(txns==first_txns);
  info_extra=0xEEEE;iqs915x_terminal(&d,false,-ETIMEDOUT);
  assert(!iqs915x_ati_timeout_get(&dev,&ati) && ati.config==0x0c86 && !ati.busy);
  assert(d.fault_snapshot.busy && !d.mismatch_snapshot_valid);
  setup(true);info_extra=0xEEEE;iqs915x_ati_timeout_capture(&d);
  assert(reads==1 && !iqs915x_ati_timeout_get(&dev,&ati) && ati.busy && !ati.valid_mask);
  setup(true);io_errors=1;iqs915x_ati_timeout_capture(&d);
  assert(!iqs915x_ati_timeout_get(&dev,&ati) && ati.read_status[0]==-EIO && !ati.valid_mask);

  /* Explicit diagnostics must break a quiet Event-mode wait without a finger
   * or power-generation change, including an API wake inside the wait loop. */
  setup(true);atomic_set(&d.channel_pending,1);
  assert(!iqs915x_wait_window(&d,false,INT64_MAX) && !txns && !sem_waits);
  setup(true);atomic_set(&d.channel_pending,1);
  assert(!iqs915x_wait_window(&d,true,INT64_MAX) && !txns);
  setup(true);wait_hook=diagnostic_wake;
  assert(!iqs915x_wait_window(&d,false,INT64_MAX) && !txns && sem_waits==1);
  wait_hook=NULL;

  /* Auto capture only once per error episode, only selected stable Active. */
  setup(false);d.lifecycle=IQS915X_READY;d.ati_error=1;
  assert(!iqs915x_channels_auto_request(&d) && !txns && !d.channel_auto_attempted);
  setup(true);d.lifecycle=IQS915X_READY;d.actual_mode=IQS915X_POWER_ACTIVE;d.ati_error=1;
  assert(iqs915x_channels_auto_request(&d) && d.channel_pending && !txns);
  assert(!iqs915x_channels_auto_request(&d));
  atomic_clear(&d.channel_pending);d.channel_snapshot.status=-ECANCELED;
  assert(!iqs915x_channels_auto_request(&d)); /* no automatic retry loop */
  iqs915x_runtime_info(&d,0);d.ati_error=1;
  assert(iqs915x_channels_auto_request(&d)); /* actual error clearance rearms */
  setup(true);d.lifecycle=IQS915X_READY;d.actual_mode=IQS915X_POWER_ACTIVE;d.ati_error=1;d.pm_suspended=1;
  assert(!iqs915x_channels_auto_request(&d) && !txns);

  setup(true);d.lifecycle=IQS915X_READY;
  uint32_t capture_id=0,other_id=0;struct iqs915x_channel_page page;
  assert(!iqs915x_channels_start(&dev,&capture_id) && capture_id);
  assert(iqs915x_channels_start(&dev,&other_id)==-EBUSY);
  assert(!iqs915x_channels_page(&dev,capture_id,0,&page) && page.status==-EINPROGRESS && !page.count);
  assert(iqs915x_channels_page(&dev,capture_id+1,0,&page)==-ESTALE);
  assert(iqs915x_channels_page(&dev,capture_id,216,&page)==-EINVAL);
  d.request_generation++; /* Power/layer changes before service must cancel. */
  atomic_clear(&d.channel_pending);iqs915x_channels_capture_pending(&d);
  assert(!txns && !d.channel_running);
  assert(!iqs915x_channels_page(&dev,capture_id,0,&page) && page.status==-ECANCELED);
  assert(!iqs915x_channels_start(&dev,&capture_id));
  iqs915x_terminal(&d,true,-ENODEV);
  assert(!d.channel_pending && !iqs915x_channels_page(&dev,capture_id,0,&page) && page.status==-ECANCELED);

  setup(false);iqs915x_restart_initialization(&dev,"test missing notification");
  assert(d.init_step==INIT_SOFTWARE_RESET && d.init_restart_count==1);
  iqs915x_restart_initialization(&dev,"second missing notification");
  assert(d.init_step==INIT_FAILED && d.init_restart_count==1);
  setup(true);iqs915x_terminal(&d,false,-ETIMEDOUT);
  assert(d.lifecycle==IQS915X_FAULT && d.actual_mode==IQS915X_POWER_SUSPEND);
  assert(writes==1 && reads==3); /* bounded evidence, then one Suspend */
  assert(d.last_error==-ETIMEDOUT);
  struct iqs915x_fault_snapshot fault;int evidence_txns=txns;
  assert(!iqs915x_fault_get(&dev,&fault) && txns==evidence_txns);
  assert(fault.error==-ETIMEDOUT && fault.stop_status==0 && fault.valid_mask==7);
  assert(!fault.busy && fault.lifecycle==IQS915X_FAULT);
  assert(!iqs915x_write_reg16(&dev,IQS915X_SYSTEM_CONTROL,IQS915X_SUSPEND));
  assert(d.last_error==-ETIMEDOUT); /* later successful I/O cannot erase cause */
  io_errors=1;assert(iqs915x_write_reg16(&dev,IQS915X_SYSTEM_CONTROL,IQS915X_SUSPEND)==-EIO);
  assert(d.last_error==-ETIMEDOUT);
  setup(true);info_extra=0xEEEE;iqs915x_terminal(&d,false,-ETIMEDOUT);
  assert(d.last_error==-ETIMEDOUT && reads==1 && writes==1);
  assert(!iqs915x_fault_get(&dev,&fault) && fault.busy && !fault.valid_mask && fault.info==0xEEEE);
  assert(fault.read_status[1]==-ENODATA);
  setup(true);io_errors=2;iqs915x_terminal(&d,false,-ETIMEDOUT);
  assert(!iqs915x_fault_get(&dev,&fault) && fault.stop_status==-EIO && d.last_error==-ETIMEDOUT);
  setup(true);io_errors=1;iqs915x_terminal(&d,false,-ETIMEDOUT);
  assert(!iqs915x_fault_get(&dev,&fault) && fault.read_status[0]==-EIO && !fault.valid_mask);


  /* A normal power cycle must not require all-up; an unresolved mode/fault
   * barrier must survive exactly the same sequence of power requests. */
  for (int strict=0;strict<2;strict++) {
    setup(true);d.output_mode=IQS915X_OUTPUT_CONTACTS;
    d.contacts.blocked=false;d.contact_restart_requested=strict;
    d.requested_enabled=0;d.requested_mode=IQS915X_POWER_LP2;d.request_generation++;
    iqs915x_apply_pending_power_request(&d);until_idle();
    assert(d.contacts.blocked==(bool)strict);
    d.requested_enabled=1;d.requested_mode=IQS915X_POWER_ACTIVE;d.request_generation++;
    iqs915x_apply_pending_power_request(&d);until_idle();
    assert(d.contacts.blocked==(bool)strict && iqs915x_contact_probe_required(&d));
    fingers=1;coords[0]=100;coords[1]=200;
    run(&d,1); /* Forced fresh snapshot with a finger already on the surface. */
    assert(d.contacts.active==(strict?0:1));
    assert(!iqs915x_contact_probe_required(&d));
  }
  setup(false); d.initialized=false; d.init_step=INIT_CHECK_SHOW_RESET;
  d.init_deadline=10000; boot_calls=parked=0; boot_result=-ENODEV;
  boot_run(&d,10);
  assert(boot_calls==3 && parked==7 && d.lifecycle==IQS915X_ABSENT && !d.output_enabled);
  /* A valid busy reply is not an absent IC, but initialization is bounded. */
  setup(false); d.initialized=false; d.init_step=INIT_CHECK_SHOW_RESET;
  d.init_deadline=10000; boot_calls=parked=0; boot_result=IQS915X_INIT_BUSY;
  boot_run(&d,120);
  assert(d.lifecycle==IQS915X_FAULT && now>=10000 && parked>0);
  setup(true); d.power_retry_count=IQS915X_POWER_TRANSITION_MAX_RETRIES-1;
  iqs915x_comm_failure(&d,-EIO);
  assert(d.lifecycle==IQS915X_FAULT); /* Responsive but invalid mode is not absence. */
  /* IC Suspend performs exactly one final write and never a confirming read. */
  setup(false); d.requested_mode=IQS915X_POWER_SUSPEND; d.request_generation++;
  iqs915x_apply_pending_power_request(&d);
  assert(d.ic_suspended && d.actual_mode==IQS915X_POWER_SUSPEND && !d.output_enabled);
  assert(txns==1 && txn_write[0] && txn_reg[0]==IQS915X_SYSTEM_CONTROL && !reads);
  d.requested_mode=IQS915X_POWER_ACTIVE; d.requested_enabled=1; d.request_generation++;
  iqs915x_apply_pending_power_request(&d); until_idle();
  assert(!d.ic_suspended && d.output_enabled && d.actual_mode==IQS915X_POWER_ACTIVE);
  setup(true); iqs915x_terminal(&d, true, -ENODEV);
  assert(d.lifecycle==IQS915X_ABSENT && !d.output_enabled && !d.initialized && d.last_error==-ENODEV);
  setup(true); d.lifecycle=IQS915X_READY; wait_hook=complete_pm;
  assert(!iqs915x_pm_action(&dev, PM_DEVICE_ACTION_SUSPEND));
  assert(d.pm_suspended && d.actual_mode==IQS915X_POWER_SUSPEND && d.ic_suspended);
  assert(iqs915x_set_mode(&dev, IQS915X_POWER_ACTIVE)==-EBUSY);
  assert(!iqs915x_pm_action(&dev, PM_DEVICE_ACTION_RESUME));
  assert(!d.pm_suspended && d.output_enabled && d.actual_mode==IQS915X_POWER_ACTIVE);
  wait_hook=NULL;
  setup(false); iqs915x_terminal(&d,true,-ENODEV);
  int stopped=txns;
  assert(iqs915x_set_mode(&dev,IQS915X_POWER_ACTIVE)==0);
  assert(d.requested_mode==IQS915X_POWER_ACTIVE);
  assert(!iqs915x_pm_action(&dev,PM_DEVICE_ACTION_SUSPEND) && txns==stopped);
  struct iqs915x_status cached;
  assert(!iqs915x_get_status(&dev,&cached) && cached.lifecycle==IQS915X_ABSENT && txns==stopped);
  /* LP2 is Event Mode: no event means no forced polling, even after minutes.
   * A bounded timer lets the harness observe the normally indefinite wait. */
  setup(false); now=150000;
  assert(!iqs915x_wait_window(&d,false,now+100));
  assert(!reads && !writes && !forced_transactions && !d.comm_fallback_active);
  /* Fresh LP2 status is read only on RDY, without opening input output. */
  for (int contact=0; contact<2; contact++) {
    setup(false); touch=contact;
    for (int i=0;i<300;i++) {
      now=(i+1)*500; event_rdy=true; run(&d,1);
    }
    assert(now==150000 && reads==300 && !writes && !input_frames && !input_resets);
    assert(d.confirmed_mode==IQS915X_MODE_LP2 && !d.output_enabled);
    assert((ic_cfg & IQS915X_EVENT_MODE) && !forced_transactions);
    for (int i=0;i<txns;i++) assert(txn_reg[i]==IQS915X_INFO_FLAGS && !txn_write[i]);
  }
  /* A stationary single or multiple contact in Active never forces reseed.
   * Only actual events produce reads; elapsed time causes no control writes. */
  for (int count=1; count<=4; count++) {
    setup(true); touch=true; fingers=count;
    for (int j=0;j<count*2;j++) coords[j]=100+(j/2)*200;
    for (int i=0;i<121;i++) {
      now=i*1000; event_rdy=true; run(&d,1);
    }
    assert(reads==121 && input_frames==121 && !writes && !input_resets);
    assert(d.output_enabled && (ic_cfg & IQS915X_EVENT_MODE) && !forced_transactions);
  }
  /* A stopped stream waits precisely 3T from the previous STOP, repeatedly. */
  setup(false); d.streaming_expected=true; ic_cfg&=~IQS915X_EVENT_MODE;
  no_rdy=true; run(&d,1); assert(now==1500 && reads==1 && d.comm_fallback_active);
  run(&d,1); assert(now==3000 && reads==2);
  no_rdy=false; run(&d,1); assert(!d.comm_fallback_active);
  /* Actual Active period, spurious wakes, and a transition use the right 3T. */
  setup(false); d.streaming_expected=true; no_rdy=true; d.confirmed_mode=IQS915X_MODE_ACTIVE;
  d.active_sampling_period_ms=37;
  k_sem_give(&d.rdy_sem);
  assert(iqs915x_wait_window(&d,false,INT64_MAX) && now==111);
  d.work_state=WORK_SET_POWER; d.power_target_mode=IQS915X_MODE_LP2;
  now=120; d.comm_completed_ms=111;
  k_sem_give(&d.rdy_sem);
  assert(iqs915x_wait_window(&d,false,INT64_MAX) && now==1611);
  /* Enable with no RDY at all, both untouched and already touched. The first
   * transaction switches to Active; the only delay is mocked clock stretching.
   * No output is permitted before the final Event Mode read-back. */
  for (int contact=0; contact<2; contact++) {
    setup(false); no_rdy=true; force_stretch_ms=7;
    touch=contact; fingers=contact; coords[0]=coords[1]=100;
    d.requested_enabled=1; d.request_generation++;
    k_sem_give(&d.rdy_sem);
    for (int step=0; step<4; step++) {
      run(&d,1);
      assert(!input_frames && d.output_enabled==(step==3));
    }
    assert(d.enabled && ic_mode==IQS915X_MODE_ACTIVE);
    assert(d.work_state==WORK_READ_DATA && (ic_cfg & IQS915X_EVENT_MODE));
    assert(sem_waits==0 && forced_transactions==4 && now==28);
    assert(txns==4 && txn_reg[0]==IQS915X_SYSTEM_CONTROL && txn_write[0]);
    assert(txn_reg[1]==IQS915X_INFO_FLAGS && !txn_write[1]);
    assert(txn_reg[2]==IQS915X_CONFIG_SETTINGS && txn_write[2]);
    assert(txn_reg[3]==IQS915X_CONFIG_SETTINGS && !txn_write[3]);
    assert(d.pointer_resume_guard_frames==IQS915X_POINTER_RESUME_GUARD_FRAMES);
    assert(ic_cfg & IQS915X_TP_EVENT);
    /* Steady-state input returns to event RDY, rather than forced polling. */
    no_rdy=false; event_rdy=true; now+=10; run(&d,1);
    assert(input_frames==1 && forced_transactions==4 && sem_waits==0);
  }
  /* Disable also forces every stage, completes only after Event read-back,
   * and keeps ALP events off even when stale settings contain that bit. */
  setup(true); no_rdy=true; force_stretch_ms=7;
  ic_cfg=d.confirmed_config_settings|=IQS915X_ALP_EVENT|IQS915X_SWITCH_EVENT|IQS915X_SNAP_EVENT;
  d.requested_enabled=0; d.request_generation++;
  for (int step=0;step<4;step++) {
    run(&d,1);
    assert(!d.output_enabled && !input_frames);
    assert(d.transition_sem.count==(step==3));
  }
  assert(d.work_state==WORK_READ_DATA && ic_mode==IQS915X_MODE_LP2);
  assert(!(ic_cfg & IQS915X_TP_EVENT));
  assert((ic_cfg & IQS915X_EVENT_MODE) && !(ic_cfg & IQS915X_ALP_EVENT));
  assert(!d.streaming_expected && !sem_waits && forced_transactions==4 && now==28);
  assert(txn_reg[0]==IQS915X_SYSTEM_CONTROL && txn_write[0]);
  assert(txn_reg[1]==IQS915X_INFO_FLAGS && !txn_write[1]);
  assert(txn_reg[2]==IQS915X_CONFIG_SETTINGS && txn_write[2]);
  assert(txn_reg[3]==IQS915X_CONFIG_SETTINGS && !txn_write[3]);
  /* Disable with a held finger and a stale TP event: LP2 must remain quiet
   * after the forced control sequence rather than generating 500 ms RDY. */
  setup(true); touch=stale_tp=true; fingers=1; coords[0]=coords[1]=100;
  d.requested_enabled=0; d.request_generation++; run(&d,4);
  assert(ic_mode==IQS915X_MODE_LP2 && !d.output_enabled && !input_frames);
  assert((ic_cfg & IQS915X_EVENT_MODE) && !(ic_cfg & IQS915X_TP_EVENT));
  assert(!(ic_cfg & (IQS915X_ALP_EVENT|IQS915X_TP_TOUCH_EVENT|IQS915X_GESTURE_EVENT|
                    IQS915X_SWITCH_EVENT|IQS915X_SNAP_EVENT)));
  int txns_after_disable=txns;
  assert(!iqs915x_wait_window(&d,false,now+2000));
  assert(txns==txns_after_disable && !d.comm_fallback_active);
  /* Recovery events are still serviced; re-enable restores TP events. */
  info_extra=IQS915X_REATI_OCCURRED; event_rdy=true; run(&d,1);
  assert(txns==txns_after_disable+1 && !d.output_enabled);
  d.requested_enabled=1; d.request_generation++; run(&d,4);
  assert(d.output_enabled && ic_mode==IQS915X_MODE_ACTIVE && (ic_cfg & IQS915X_TP_EVENT));
  /* Disabled startup and PM suspend follow the same forced LP2 sequence. */
  for (int pm=0;pm<2;pm++) {
    setup(true); no_rdy=true; d.requested_enabled=0;
    d.pm_suspended=pm; d.lp2_pending=true;
    run(&d,2); assert(!d.transition_sem.count);
    run(&d,2);
    assert(d.transition_sem.count && !d.output_enabled && !sem_waits);
    assert(ic_mode==IQS915X_MODE_LP2 && (ic_cfg & IQS915X_EVENT_MODE));
  }
  /* An unconfirmed configuration is repaired and verified, still without RDY. */
  setup(false); no_rdy=true; force_stretch_ms=7;
  ic_cfg=d.confirmed_config_settings=0;
  d.requested_enabled=1; d.request_generation++;
  run(&d,4);
  assert(d.output_enabled && forced_transactions==4 && sem_waits==0 && now==28);
  assert(txn_reg[0]==IQS915X_SYSTEM_CONTROL && txn_write[0]);
  assert(txn_reg[2]==IQS915X_CONFIG_SETTINGS && txn_write[2]);
  /* Failed/mismatched Event Mode confirmation never opens the output gate. */
  setup(false); no_rdy=true; d.requested_enabled=1; d.request_generation++;
  run(&d,2); ic_cfg&=~IQS915X_TP_EVENT; reject_cfg=10;
  run(&d,6); assert(!d.initialized && !d.output_enabled && !input_frames && sem_waits==0);
  /* LP2 mode read-back mismatch is retried without RDY, before completion. */
  setup(true); no_rdy=true; d.requested_enabled=0; d.request_generation++;
  reject_mode=1; run(&d,2);
  assert(ic_mode==IQS915X_MODE_ACTIVE && !d.transition_sem.count && !d.output_enabled);
  until_idle();
  assert(ic_mode==IQS915X_MODE_LP2 && d.transition_sem.count && !sem_waits);
  /* A failed disable Event read-back cannot report PM suspend complete. */
  setup(true); no_rdy=true; d.requested_enabled=0; d.pm_suspended=1;
  d.request_generation++; run(&d,3); io_errors=1; run(&d,1);
  assert(!d.transition_sem.count && !d.output_enabled);
  until_idle(); assert(d.transition_sem.count && !sem_waits);
  /* An I2C error on an urgent control step keeps the normal bounded backoff. */
  setup(false); no_rdy=true; d.requested_enabled=1; d.request_generation++;
  io_errors=1; run(&d,1); assert(!d.output_enabled && now==20 && d.power_retry_count==1);
  until_idle(); assert(d.output_enabled && sem_waits==0);
  /* A newer request must stop even forced control before another transaction. */
  setup(false); d.requested_enabled=1; d.request_generation++;
  run(&d,1); assert(txns==1 && d.power_force_comms && !d.output_enabled);
  d.requested_enabled=0; d.request_generation++;
  int txns_before=txns;
  assert(!iqs915x_wait_window(&d,true,INT64_MAX) && txns==txns_before);
  until_idle(); assert(!d.output_enabled && !d.power_force_comms && ic_mode==IQS915X_MODE_LP2);
  /* Reverse either request after each intermediate stage, without an RDY. */
  for (int output=0;output<2;output++) {
    for (int step=1;step<4;step++) {
      setup(output); no_rdy=true;
      d.requested_enabled=!output; d.request_generation++; run(&d,step);
      d.requested_enabled=output; d.request_generation++; run(&d,1); until_idle();
      assert(d.output_enabled==output && !sem_waits);
      assert(ic_mode==(output?IQS915X_MODE_ACTIVE:IQS915X_MODE_LP2));
      assert((ic_cfg & IQS915X_EVENT_MODE) && !d.streaming_expected);
    }
  }
  /* A busy frame is ignored; repeated busy status or I2C errors recover by reset. */
  setup(false); info_extra=0xEEEE; event_rdy=true; run(&d,1);
  assert(d.initialized && d.runtime_busy_count==1 && !writes && !input_frames);
  event_rdy=true; run(&d,1); assert(d.runtime_busy_count==0);
  for (int i=0;i<IQS915X_INIT_REATI_MAX_WAIT;i++) {
    info_extra=0xEEEE; event_rdy=true; run(&d,1);
  }
  assert(!d.initialized && !d.output_enabled && input_resets==1);
  setup(false); io_errors=3;
  for (int i=0;i<3;i++) { event_rdy=true; run(&d,1); }
  assert(!d.initialized && !d.output_enabled && input_resets==1);
  /* Config writes that don't latch are verified and bounded. */
  setup(true); ic_cfg|=IQS915X_ALP_EVENT;
  d.requested_enabled=0; d.request_generation++; reject_cfg=10;
  run(&d,8); assert(!d.initialized && !d.output_enabled);
  /* Error closes input without pinning power or requesting host calibration.
   * Recovery reopens only the current user-requested session. */
  setup(true);touch=true;fingers=1;info_extra=IQS915X_ATI_ERROR;
  event_rdy=true;run(&d,1);
  assert(d.ati_error && !iqs915x_output_is_enabled(&d) && input_frames==0);
  assert(input_resets==1 && !writes);
  d.requested_enabled=0;d.requested_mode=IQS915X_POWER_LP2;d.request_generation++;
  iqs915x_apply_pending_power_request(&d);
  for(int i=0;i<8 && d.work_state!=WORK_READ_DATA;i++) iqs915x_mode_step(&d);
  assert(d.confirmed_mode==IQS915X_MODE_LP2 && d.ati_error);
  info_extra=IQS915X_REATI_OCCURRED;event_rdy=true;run(&d,1);
  assert(!d.ati_error && !iqs915x_output_is_enabled(&d));
  /* No host ATI command is issued during an error. */
  setup(false); info_extra=IQS915X_ATI_ERROR; event_rdy=true; run(&d,1);
  assert(d.ati_error_seen && !writes);
  /* Runtime Re-ATI resets input; cached debounce frames do not
   * replay it. This uses the actual runtime loop prefix. */
  setup(true); d.finger_tracker.count_change_pending=true;
  d.finger_tracker.candidate_since_ms=0; info_extra=IQS915X_REATI_OCCURRED;
  now=20; next_rdy=20; ic_cfg&=~IQS915X_EVENT_MODE;
  run(&d,1); assert(input_resets==1 && !input_frames && d.output_enabled);
  assert(d.pointer_resume_guard_frames==IQS915X_POINTER_RESUME_GUARD_FRAMES && !writes);
  setup(false); info_extra=IQS915X_REATI_OCCURRED|IQS915X_ATI_ERROR; event_rdy=true; run(&d,1);
  assert(input_resets==1 && d.ati_error_seen && !d.output_enabled && !writes);
  info_extra=IQS915X_REATI_OCCURRED; event_rdy=true; run(&d,1);
  assert(input_resets==2 && !d.ati_error_seen && !writes);
  /* SHOW_RESET still starts reinitialization with the input gate closed. */
  setup(true); info_extra=IQS915X_SHOW_RESET; event_rdy=true; run(&d,1);
  assert(!d.initialized && !d.output_enabled && d.init_step==INIT_CHECK_SHOW_RESET);
  /* PM can finish an already-stable LP2 request without any I2C transaction. */
  setup(false); d.pm_suspended=1; d.request_generation++;
  iqs915x_apply_pending_power_request(&d);
  assert(!txns && d.transition_sem.count && d.work_state==WORK_READ_DATA);
  /* Fresh validated reads reach the calibration stage. */
  setup(false); fingers=1; coords[0]=coords[1]=100;
  struct iqs915x_stream_data sample;
  assert(!iqs915x_read_stream(&dev,&sample));
  assert(sample.abs_x==150);
  /* Pre-patching enforces the communication/ATI policy without modifying tuning data. */
  uint8_t profile[IQS915X_INIT_DATA_TOTAL_SIZE];
  memset(profile, 0xff, sizeof(profile));
  config.init_data=profile; config.init_data_len=sizeof(profile);
  config.report_rate_ms=20;
  /* Distinct tuning bytes prevent an accidental hard-coded override passing. */
  for (uint16_t reg=0x119C;reg<=0x11A0;reg++) profile[reg-IQS915X_INIT_DATA_BASE_ADDR]=reg & 0xff;
  profile[0x1196-IQS915X_INIT_DATA_BASE_ADDR]=0x34; /* ATI target */
  profile[0x1197-IQS915X_INIT_DATA_BASE_ADDR]=0x12;
  uint8_t patched[IQS915X_INIT_DATA_TOTAL_SIZE];
  for (uint16_t offset=0; offset<sizeof(profile);) {
    uint16_t addr, count;
    assert(!iqs915x_prepare_init_chunk(&dev,offset,&addr,&count,patched+offset,false));
    offset+=count;
  }
  #define INDEX(reg) ((reg)-IQS915X_INIT_DATA_BASE_ADDR)
  assert(!(patched[INDEX(IQS915X_SYSTEM_CONTROL)] & 0xf8));
  assert(patched[INDEX(IQS915X_REATI_RETRY_TIME)]==1);
  for (int i=0;i<4;i++) assert(patched[INDEX(IQS915X_ALP_SETUP+i)]==profile[INDEX(IQS915X_ALP_SETUP+i)]);
  /* Passing ALP Enable through also preserves an export that disables it. */
  profile[INDEX(IQS915X_ALP_SETUP+3)] &= ~BIT(7);
  for (uint16_t offset=0;offset<sizeof(profile);) {
    uint16_t addr,count;
    assert(!iqs915x_prepare_init_chunk(&dev,offset,&addr,&count,patched+offset,false));
    offset+=count;
  }
  for (int i=0;i<4;i++) assert(patched[INDEX(IQS915X_ALP_SETUP+i)]==profile[INDEX(IQS915X_ALP_SETUP+i)]);
  assert(!(patched[INDEX(IQS915X_OTHER_SETTINGS)] & (BIT(4)|BIT(5))));
  assert((patched[INDEX(IQS915X_LP2_MODE_REPORT_RATE)] |
         patched[INDEX(IQS915X_LP2_MODE_REPORT_RATE+1)]<<8)==500);
  assert(patched[INDEX(IQS915X_ACTIVE_MODE_REPORT_RATE)]==20);
  for (uint16_t reg=0x119C;reg<=0x11A0;reg++) assert(patched[INDEX(reg)]==profile[INDEX(reg)]);
  assert(patched[INDEX(0x1196)]==0x34 && patched[INDEX(0x1197)]==0x12);
  uint16_t patched_cfg=patched[INDEX(IQS915X_CONFIG_SETTINGS)] |
                       patched[INDEX(IQS915X_CONFIG_SETTINGS+1)]<<8;
  assert(patched_cfg & IQS915X_TP_REATI_ENABLE);
  assert(!(patched_cfg & (IQS915X_EVENT_MODE|IQS915X_ALP_REATI_ENABLE|
                         IQS915X_ALP_EVENT|IQS915X_SWITCH_EVENT|IQS915X_SNAP_EVENT|
                         IQS915X_TP_TOUCH_EVENT|IQS915X_FORCE_COMMS_METHOD)));
  puts("IQS915x runtime: LP2 Event Mode, forced enable/disable, watchdog, requests, ALP preservation, ATI and PM passed");
}
'''

def main():
    defs = '\n'.join(re.findall(r'^#define IQS915X_(?:POWER_TRANSITION_MAX_RETRIES|INIT_TOTAL_TIMEOUT_MS|INIT_MAX_RESTARTS|INIT_REATI_MAX_WAIT|INIT_BUSY|INIT_BUSY_RETRY_MS|LP2_SAMPLING_PERIOD_MS|POINTER_RESUME_GUARD_FRAMES)\b.*$', SOURCE, re.M))
    names = ['iqs915x_channels_cancel_pending', 'iqs915x_restart_initialization', 'iqs915x_request_generation', 'iqs915x_output_is_enabled',
             'iqs915x_power_retry_backoff_ms', 'iqs915x_complete_transition',
             'iqs915x_apply_config_settings_policy', 'iqs915x_config_settings_for_mode',
             'iqs915x_config_settings_without_event_mode',
             'iqs915x_init_wait_window', 'iqs915x_read_block', 'iqs915x_write_reg16', 'iqs915x_read_reg16', 'iqs915x_validate_stream_coordinates',
             'iqs915x_read_stream', 'iqs915x_channel_profile_valid', 'iqs915x_prepare_init_chunk']
    functions = []
    for name in names:
        matches = list(re.finditer(r'^static [^\n]*\b'+name+r'\(', SOURCE, re.M))
        m = matches[-1]
        functions.append(block(SOURCE[m.start():], m[0]))
    gestures = (ROOT/'drivers/input/iqs915x_gestures.c').read_text()
    functions.insert(0, "static int iqs915x_write_reg16(const struct device *,uint16_t,uint16_t);")
    functions.insert(0, block(gestures, 'bool iqs915x_get_finger_coordinates('))
    a = SOURCE.index('static void iqs915x_terminal(')
    b = SOURCE.index('/* ============================================================\n * ボタンリリース遅延処理', a)
    functions.append(block(SOURCE, 'static void iqs915x_end_contact_session('))
    functions.append(block(SOURCE, 'static bool iqs915x_contact_probe_required('))
    functions.append(block(SOURCE, 'static int64_t iqs915x_contact_refresh_deadline('))
    functions.append(block(SOURCE, 'static bool iqs915x_deliver_contacts('))
    functions.append((ROOT/'drivers/input/iqs915x_fault_diagnostics.inc').read_text())
    functions.append(SOURCE[a:b])
    functions.append('static void iqs915x_apply_pending_power_request(struct iqs915x_data *);')
    functions.append((ROOT/'drivers/input/iqs915x_channel_diagnostics.inc').read_text())
    functions.append(block(SOURCE, 'static void iqs915x_apply_pending_power_request('))
    power = (ROOT / 'drivers/input/iqs915x_power.c').read_text()
    for marker in ['static void iqs915x_set_pm_suspended(', 'static int iqs915x_request_enabled(',
                   'static int iqs915x_pm_action_locked(', 'int iqs915x_pm_action(',
                   'int iqs915x_set_mode(', 'int iqs915x_set_enabled(',
                   'int iqs915x_get_status(', 'bool iqs915x_get_enabled(']:
        functions.append(block(power, marker))
    a = SOURCE.index('    iqs915x_apply_pending_power_request(data);', SOURCE.index('static void iqs915x_thread_main('))
    b = SOURCE.index('    // =========================================================\n    // ドラッグ解除チェック', a)
    boot_start = SOURCE.index('  while (true)', SOURCE.index('static void iqs915x_thread_main('))
    boot_end = SOURCE.index('    iqs915x_apply_pending_power_request(data);', boot_start)
    boot = "static void boot_run(struct iqs915x_data *data, int steps) { const struct device *dev=data->dev; const struct iqs915x_config *config=dev->config; int ret;\n" + SOURCE[boot_start:boot_end].replace('while (true)', 'while (steps-- > 0)', 1) + "}}"
    code = HARNESS.replace('DEFINITIONS', defs).replace('FUNCTIONS', '\n'.join(functions)).replace('PREFIX', SOURCE[SOURCE.index('    iqs915x_apply_pending_power_request(data);', SOURCE.index('static void iqs915x_thread_main(')):SOURCE.index('    // =========================================================\n    // ドラッグ解除チェック', boot_end)]).replace('BOOT', boot)

    with tempfile.TemporaryDirectory(prefix='iqs915x-runtime-') as tmp:
        directory = Path(tmp)
        (directory/'stub.h').write_text(STUB)
        for name in ['device.h','drivers/gpio.h','drivers/i2c.h','kernel.h','sys/atomic.h','input/input.h']:
            path = directory/'zephyr'/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "stub.h"\n')
        (directory/'test.c').write_text(code)
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function',
                        '-Wno-unused-parameter','-Wno-unused-variable','-I'+tmp,
                        '-I'+str(ROOT/'include'),'-I'+str(ROOT/'drivers/input'),
                        str(directory/'test.c'),'-o',str(directory/'test')],check=True)
        subprocess.run([str(directory/'test')],check=True, timeout=10)

if __name__ == '__main__':
    main()
