#!/usr/bin/env python3
"""Exercise the actual runtime maintenance code with a deterministic IC/clock.

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
#define K_FOREVER INT64_MAX
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
static bool atomic_cas(atomic_t *p,int old,int n) { if (*p!=old) return false; *p=n; return true; }
static void k_sem_reset(struct k_sem *s) { s->count=0; }
static void k_sem_give(struct k_sem *s) { s->count=1; }
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w) {
  return CONTAINER_OF(w,struct k_work_delayable,work);
}
'''
HARNESS = r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include "iqs915x_internal.h"
DEFINITIONS
static int64_t now, next_rdy;
static uint16_t ic_mode, ic_cfg, info_extra, fingers;
static bool no_rdy, touch, event_rdy;
static int input_resets, input_frames, reseeds, reads, writes, schedules;
static int reject_cfg, io_errors;
static int sem_waits, force_stretch_ms, forced_transactions;
static uint16_t txn_reg[1024];
static bool txn_write[1024];
static int64_t txn_time[1024];
static int txns;
static uint16_t coords[8];
static uint16_t period(void) { return ic_mode==IQS915X_MODE_LP2 ? 500 : 10; }
static int64_t k_uptime_get(void) { return now; }
static int gpio_pin_get_dt(const struct gpio_dt_spec *g) {
  (void)g; return !no_rdy && (event_rdy ||
      (!(ic_cfg & IQS915X_EVENT_MODE) && now >= next_rdy));
}
static int k_sem_take(struct k_sem *s,k_timeout_t timeout) {
  sem_waits++;
  if (s->count) { s->count=0; return 0; }
  int64_t end=timeout==K_FOREVER ? INT64_MAX : now+timeout;
  if (!no_rdy && !(ic_cfg & IQS915X_EVENT_MODE) && next_rdy <= end) {
    now=MAX(now,next_rdy); return 0;
  }
  assert(end!=INT64_MAX); now=end; return -1;
}
static void k_sleep(int64_t delay) { now+=delay; }
static void k_work_cancel_delayable(struct k_work_delayable *w) { w->scheduled=0; }
static void k_work_reschedule(struct k_work_delayable *w,int64_t delay) {
  assert(delay==60000); w->scheduled=1; schedules++;
}
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
static int i2c_write_dt(const struct i2c_dt_spec *i,const uint8_t *b,size_t len) {
  (void)i; assert(len==4); uint16_t reg=b[0]|b[1]<<8, value=b[2]|b[3]<<8;
  record(reg,true); writes++;
  if (reg==IQS915X_CONFIG_SETTINGS) {
    if (reject_cfg) reject_cfg--; else ic_cfg=value;
  } else {
    assert(reg==IQS915X_SYSTEM_CONTROL);
    ic_mode=value & IQS915X_CHARGING_MODE_MASK;
    if (value & IQS915X_TP_RESEED) { reseeds++; info_extra|=IQS915X_REATI_OCCURRED; }
  }
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
  else if (reg==IQS915X_INFO_FLAGS) { assert(len==2); put16(b,info); info_extra=0; }
  else {
    assert(reg==IQS915X_REL_X && len==44); memset(b,0,len);
    put16(b+12,info); put16(b+14,fingers);
    for (int j=0;j<4;j++) { put16(b+16+j*8,coords[j*2]); put16(b+18+j*8,coords[j*2+1]); }
    info_extra=0;
  }
  return 0;
}
static void iqs915x_reset_input_session(struct iqs915x_data *d) {
  input_resets++; d->buttons_pressed=0; d->active_tap_hold=false;
  d->finger_tracker.count_change_pending=false;
}
static void iqs915x_mark_initialized(struct iqs915x_data *d,bool init) { d->initialized=init; }
static void iqs915x_correct_stream_coordinates(const struct iqs915x_config *c,
                                               const struct iqs915x_data *d,
                                               struct iqs915x_stream_data *s) {
  (void)c; (void)d; s->abs_x+=50; /* proves stuck capture precedes LUT correction */
}
static void iqs915x_log_stream_coordinates(const struct iqs915x_stream_data *s) { (void)s; }
FUNCTIONS
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
  no_rdy=touch=event_rdy=false; input_resets=input_frames=reseeds=reads=writes=schedules=0;
  sem_waits=force_stretch_ms=forced_transactions=0;
  reject_cfg=io_errors=info_extra=fingers=0;
  for (int j=0;j<8;j++) coords[j]=UINT16_MAX;
  memset(&d,0,sizeof(d)); memset(&config,0,sizeof(config));
  dev=(struct device){&d,&config}; d.dev=&dev; d.initialized=true;
  d.stuck.enabled=true; d.stuck.threshold=100;
  d.swipe_resolution_x=d.swipe_resolution_y=1000;
  d.active_sampling_period_ms=10;
  d.enabled=d.requested_enabled=d.output_enabled=output;
  ic_mode=d.confirmed_mode=output?IQS915X_MODE_ACTIVE:IQS915X_MODE_LP2;
  d.streaming_expected=!output;
  ic_cfg=d.confirmed_config_settings=output ? iqs915x_apply_config_settings_policy(0) :
                                              iqs915x_config_settings_without_event_mode(0);
}
static void until_idle(void) {
  for (int i=0;i<40 && (d.work_state!=WORK_READ_DATA || d.reseed_state!=RESEED_IDLE);i++) run(&d,1);
  assert(d.initialized && d.work_state==WORK_READ_DATA && d.reseed_state==RESEED_IDLE);
}
int main(void) {
  /* Fresh LP2 data only; no periodic Force Comms while RDY arrives. */
  setup(false); run(&d,3);
  assert(reads==3 && writes==0 && now==1500 && !d.comm_fallback_active);
  for (int i=0;i<3;i++) assert(txn_reg[i]==IQS915X_INFO_FLAGS && !txn_write[i]);
  /* A stopped stream waits precisely 3T from the previous STOP, repeatedly. */
  setup(false); no_rdy=true; run(&d,1); assert(now==1500 && reads==1 && d.comm_fallback_active);
  run(&d,1); assert(now==3000 && reads==2);
  no_rdy=false; run(&d,1); assert(!d.comm_fallback_active);
  /* Actual Active period, spurious wakes, and a transition use the right 3T. */
  setup(false); no_rdy=true; d.confirmed_mode=IQS915X_MODE_ACTIVE;
  d.active_sampling_period_ms=37;
  k_sem_give(&d.rdy_sem);
  assert(iqs915x_wait_window(&d,false,INT64_MAX) && now==111);
  d.work_state=WORK_SET_POWER; d.power_target_mode=IQS915X_MODE_LP2;
  now=120; d.comm_completed_ms=111;
  k_sem_give(&d.rdy_sem);
  assert(iqs915x_wait_window(&d,false,INT64_MAX) && now==1611);
  /* A probe round trip cannot move an existing periodic deadline. */
  setup(false); iqs915x_schedule_lp2_reseed(&d); iqs915x_schedule_lp2_reseed(&d);
  assert(schedules==1 && d.reseed_timer_armed);
  iqs915x_reseed_work_handler(&d.reseed_work.work);
  assert(d.reseed_due && !d.reseed_timer_armed);
  iqs915x_schedule_lp2_reseed(&d); assert(schedules==1);
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
    /* Steady-state input returns to event RDY, rather than forced polling. */
    no_rdy=false; event_rdy=true; now+=10; run(&d,1);
    assert(input_frames==1 && forced_transactions==4 && sem_waits==0);
  }
  /* An unconfirmed configuration is repaired and verified, still without RDY. */
  setup(false); no_rdy=true; force_stretch_ms=7;
  d.confirmed_config_settings=0;
  d.requested_enabled=1; d.request_generation++;
  run(&d,6);
  assert(d.output_enabled && forced_transactions==6 && sem_waits==0 && now==42);
  assert(txn_reg[0]==IQS915X_CONFIG_SETTINGS && txn_write[0]);
  assert(txn_reg[1]==IQS915X_CONFIG_SETTINGS && !txn_write[1]);
  assert(txn_reg[2]==IQS915X_SYSTEM_CONTROL && txn_write[2]);
  /* Failed/mismatched Event Mode confirmation never opens the output gate. */
  setup(false); no_rdy=true; d.requested_enabled=1; d.request_generation++;
  run(&d,2); reject_cfg=10;
  run(&d,6); assert(!d.initialized && !d.output_enabled && !input_frames && sem_waits==0);
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
  /* An enable during an issued reseed first drains the next fresh TP scan.
   * Force controls must not consume its indication or duplicate the request. */
  setup(false); ic_mode=d.confirmed_mode=IQS915X_MODE_ACTIVE; next_rdy=10;
  iqs915x_begin_reseed(&d,true); run(&d,1);
  d.requested_enabled=1; d.request_generation++;
  run(&d,1); assert(txn_reg[1]==IQS915X_REL_X && !txn_write[1] && !d.output_enabled);
  assert(d.power_force_comms);
  no_rdy=true; until_idle(); assert(d.output_enabled && reseeds==1);
  /* Runtime no-touch path: four fresh Active scans, then reseed, then a read.
   * No configuration or power write can consume the post-reseed indication. */
  setup(false); d.reseed_due=1; run(&d,1); assert(d.reseed_state==RESEED_OBSERVE_ACTIVE);
  while (d.work_state!=WORK_READ_DATA) run(&d,1);
  assert(ic_mode==IQS915X_MODE_ACTIVE && !(ic_cfg & IQS915X_EVENT_MODE));
  int before=reads;
  run(&d,3); assert(!reseeds && d.no_touch_scans==3);
  run(&d,1); assert(!reseeds && d.no_touch_scans==4 && reads==before+4);
  run(&d,1); assert(reseeds==1 && d.reseed_state==RESEED_WAIT_TP_SCAN);
  int seed_txn=txns-1;
  run(&d,1); assert(txn_reg[seed_txn+1]==IQS915X_REL_X && !txn_write[seed_txn+1]);
  until_idle(); assert(ic_mode==IQS915X_MODE_LP2 && !d.reseed_due && !input_frames && schedules==1);
  /* A busy or unreadable sample cannot complete a consecutive no-touch run. */
  setup(false); ic_mode=d.confirmed_mode=IQS915X_MODE_ACTIVE; next_rdy=10;
  d.reseed_state=RESEED_OBSERVE_ACTIVE; d.no_touch_scans=3;
  info_extra=0xEEEE; run(&d,1); assert(d.no_touch_scans==0 && !reseeds);
  run(&d,3); assert(d.no_touch_scans==3 && !reseeds);
  io_errors=1; run(&d,1); assert(d.no_touch_scans==0 && !reseeds);
  /* Contact during confirmation aborts the routine reseed, retains due, and
   * records all valid coordinates even when NUM_FINGERS is greater than four. */
  setup(false); d.reseed_due=1; run(&d,1);
  while (d.work_state!=WORK_READ_DATA) run(&d,1);
  touch=true; fingers=5;
  for (int j=0;j<8;j++) coords[j]=100+(j/2)*200;
  run(&d,1); until_idle();
  assert(!reseeds && d.reseed_due && !input_frames);
  for (int j=0;j<4;j++) assert(d.stuck.candidate[j].active);
  /* Slot permutations in LP2 retain ages; one mature candidate forces reseed. */
  int64_t age=d.stuck.candidate[0].since_ms;
  uint32_t id=d.stuck.candidate[0].id;
  coords[0]=coords[1]=700; coords[6]=coords[7]=100;
  while (now<age+10000) run(&d,1);
  while (d.work_state!=WORK_READ_DATA) run(&d,1);
  run(&d,1); assert(d.reseed_state==RESEED_ISSUE_TP_RESEED && d.reseed_forced);
  assert(d.stuck.candidate[0].id==id && d.stuck.candidate[0].slot==3);
  run(&d,1); run(&d,1); until_idle(); assert(reseeds==1);
  /* Due contact clears on the next LP2 no-touch sample, without another minute. */
  setup(false); d.reseed_due=1; touch=true; fingers=1; coords[0]=coords[1]=100;
  run(&d,1); while (d.work_state!=WORK_READ_DATA) run(&d,1); run(&d,1); until_idle();
  touch=false; fingers=0; run(&d,1); assert(d.reseed_state==RESEED_OBSERVE_ACTIVE);
  /* Enable during a ten-second wait preserves candidate identity. */
  setup(false); touch=true; fingers=1; coords[0]=coords[1]=100; d.reseed_due=1;
  run(&d,1); while (d.work_state!=WORK_READ_DATA) run(&d,1); run(&d,1); until_idle();
  id=d.stuck.candidate[0].id; age=d.stuck.candidate[0].since_ms;
  d.requested_enabled=1; d.request_generation++; k_sem_give(&d.rdy_sem);
  run(&d,1); until_idle(); assert(d.enabled && d.output_enabled && (ic_cfg & IQS915X_EVENT_MODE));
  assert(d.stuck.candidate[0].id==id);
  run(&d,1); assert(now>=age+10000 && d.reseed_state==RESEED_ISSUE_TP_RESEED);
  until_idle(); assert(d.enabled && d.output_enabled && reseeds==1);
  /* Reverse an enable before opening the output gate. */
  setup(false); d.requested_enabled=1; d.request_generation++; run(&d,1);
  d.requested_enabled=0; d.request_generation++; run(&d,1); until_idle();
  assert(!d.output_enabled && ic_mode==IQS915X_MODE_LP2);
  /* A reported write error may still queue reseed; never issue it twice. */
  setup(false); ic_mode=d.confirmed_mode=IQS915X_MODE_ACTIVE; next_rdy=10;
  iqs915x_begin_reseed(&d,true); io_errors=1; run(&d,1);
  assert(d.reseed_state==RESEED_WAIT_TP_SCAN && reseeds==1);
  run(&d,1); until_idle(); assert(reseeds==1);
  /* Errors reading the post-reseed scan reset after three attempts. */
  setup(false); ic_mode=d.confirmed_mode=IQS915X_MODE_ACTIVE; next_rdy=10;
  iqs915x_begin_reseed(&d,true); run(&d,1); io_errors=3;
  run(&d,3); assert(!d.initialized && reseeds==1 && input_resets);
  /* Config writes that don't latch are verified and bounded. */
  setup(true); d.requested_enabled=0; d.request_generation++; reject_cfg=10;
  run(&d,15); assert(!d.initialized && !d.output_enabled);
  /* ATI Error is informational: the driver never requests a software ATI. */
  setup(false); info_extra=IQS915X_ATI_ERROR; run(&d,1);
  assert(d.ati_error_seen && !writes && !reseeds);
  /* Runtime Re-ATI resets input and candidates; cached debounce frames do not
   * replay it. This uses the actual runtime loop prefix. */
  setup(true); d.finger_tracker.count_change_pending=true;
  d.finger_tracker.candidate_since_ms=0; info_extra=IQS915X_REATI_OCCURRED;
  now=20; next_rdy=20; ic_cfg&=~IQS915X_EVENT_MODE;
  run(&d,1); assert(input_resets==1 && !input_frames && d.output_enabled);
  /* PM can finish an already-stable LP2 request without any I2C transaction. */
  setup(false); d.pm_suspended=1; d.request_generation++;
  iqs915x_apply_pending_power_request(&d);
  assert(!txns && d.transition_sem.count && d.work_state==WORK_READ_DATA);
  /* Raw coordinates are recorded before the optional correction. */
  setup(false); config.coordinate_correction=true; fingers=1; coords[0]=coords[1]=100;
  struct iqs915x_stream_data sample;
  assert(!iqs915x_read_stream(&dev,&sample));
  assert(sample.abs_x==150 && sample.raw_point[0].x==100 && sample.raw_point[0].valid);
  /* Pre-patching enforces maintenance policy without modifying tuning data. */
  uint8_t profile[IQS915X_INIT_DATA_TOTAL_SIZE];
  memset(profile, 0xff, sizeof(profile));
  config.init_data=profile; config.init_data_len=sizeof(profile);
  config.report_rate_ms=20;
  uint8_t patched[IQS915X_INIT_DATA_TOTAL_SIZE];
  for (uint16_t offset=0; offset<sizeof(profile);) {
    uint16_t addr, count;
    assert(!iqs915x_prepare_init_chunk(&dev,offset,&addr,&count,patched+offset,false));
    offset+=count;
  }
  #define INDEX(reg) ((reg)-IQS915X_INIT_DATA_BASE_ADDR)
  assert(patched[INDEX(IQS915X_REATI_RETRY_TIME)]==1);
  assert(!(patched[INDEX(IQS915X_ALP_SETUP+3)] & BIT(7)));
  assert(!(patched[INDEX(IQS915X_OTHER_SETTINGS)] & (BIT(4)|BIT(5))));
  assert((patched[INDEX(IQS915X_LP2_MODE_REPORT_RATE)] |
         patched[INDEX(IQS915X_LP2_MODE_REPORT_RATE+1)]<<8)==500);
  assert(patched[INDEX(IQS915X_ACTIVE_MODE_REPORT_RATE)]==20);
  assert(patched[INDEX(0x11A0)]==0xff); /* reference drift limit unchanged */
  uint16_t patched_cfg=patched[INDEX(IQS915X_CONFIG_SETTINGS)] |
                       patched[INDEX(IQS915X_CONFIG_SETTINGS+1)]<<8;
  assert(patched_cfg & IQS915X_TP_REATI_ENABLE);
  assert(!(patched_cfg & (IQS915X_EVENT_MODE|IQS915X_ALP_REATI_ENABLE|
                         IQS915X_TP_TOUCH_EVENT|IQS915X_FORCE_COMMS_METHOD)));
  puts("IQS915x maintenance: streaming, watchdog, forced enable, probes, reseed, requests, ATI and PM passed");
}
'''

def main():
    defs = '\n'.join(re.findall(r'^#define IQS915X_(?:POWER_TRANSITION_MAX_RETRIES|INIT_REATI_MAX_WAIT|LP2_RESEED_INTERVAL_MS|LP2_SAMPLING_PERIOD_MS|POINTER_RESUME_GUARD_FRAMES)\b.*$', SOURCE, re.M))
    names = ['iqs915x_request_generation', 'iqs915x_output_is_enabled',
             'iqs915x_power_retry_backoff_ms', 'iqs915x_complete_transition',
             'iqs915x_apply_config_settings_policy', 'iqs915x_config_settings_without_event_mode',
             'iqs915x_write_reg16', 'iqs915x_read_reg16', 'iqs915x_validate_stream_coordinates',
             'iqs915x_read_stream', 'iqs915x_prepare_init_chunk']
    functions = []
    for name in names:
        m = re.search(r'^static [^\n]*\b'+name+r'\(', SOURCE, re.M)
        functions.append(block(SOURCE, m[0]))
    gestures = (ROOT/'drivers/input/iqs915x_gestures.c').read_text()
    functions.insert(0, block(gestures, 'bool iqs915x_get_finger_coordinates('))
    a = SOURCE.index('static void iqs915x_clear_stuck(')
    b = SOURCE.index('/* ============================================================\n * ボタンリリース遅延処理', a)
    functions.append(SOURCE[a:b])
    functions.append(block(SOURCE, 'static void iqs915x_apply_pending_power_request('))
    a = SOURCE.index('    iqs915x_apply_pending_power_request(data);', SOURCE.index('static void iqs915x_thread_main('))
    b = SOURCE.index('    // =========================================================\n    // ドラッグ解除チェック', a)
    code = HARNESS.replace('DEFINITIONS', defs).replace('FUNCTIONS', '\n'.join(functions)).replace('PREFIX', SOURCE[a:b])
    with tempfile.TemporaryDirectory(prefix='iqs915x-maintenance-') as tmp:
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
        subprocess.run([str(directory/'test')],check=True)

if __name__ == '__main__':
    main()
