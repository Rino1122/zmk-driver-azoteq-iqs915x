#!/usr/bin/env python3
"""Compile production request/reset paths; mock Zephyr work and hardware only."""
from pathlib import Path
import subprocess
import re
import sys
import tempfile
sys.dont_write_bytecode = True
from init_event_mode import block
root = Path(__file__).resolve().parents[1]
core = (root / 'drivers/input/iqs915x.c').read_text()
power = (root / 'drivers/input/iqs915x_power.c').read_text()
def definition(source, marker):
    name = re.search(r'iqs915x_\w+', marker)[0]
    match = re.search(r'static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', source)
    assert match, name
    return block(source[match.start():], "static ")

functions = '\n'.join(definition(core, marker) for marker in (
    'static uint32_t iqs915x_request_generation',
    'static bool iqs915x_output_is_enabled',
    'static bool iqs915x_work_session_is_current',
    'static void iqs915x_stop_scroll_inertia_locked',
    'static void iqs915x_reset_scroll_inertia',
    'static void iqs915x_reset_input_session(struct iqs915x_data *data)\n{',
    'static void iqs915x_apply_pending_power_request',
))
functions += '\n' + block(power, 'static int iqs915x_request_enabled')
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include "iqs915x_scroll_motion.h"
#define BIT(n) (1U << (n))
#define K_FOREVER -1
#define K_MSEC(n) (n)
#define IQS915X_POWER_TRANSITION_WAIT_MS 2000
#define IQS915X_MODE_LP2 2
#define WORK_READ_DATA 0
#define WORK_SET_POWER 1
#define INPUT_BTN_0 272
#define LEFT_BUTTON_CODE INPUT_BTN_0
#define LOG_INF(...) ((void)0)
typedef int atomic_t;
typedef int atomic_val_t;
struct k_work_sync { int unused; };
struct k_work_delayable { bool scheduled; };
struct k_mutex { bool held; };
struct k_sem { int count; };
struct iqs915x_data;
struct device { struct iqs915x_data *data; };
struct iqs915x_data {
 const struct device *dev;
 struct k_work_delayable tap_and_hold_release_work, single_tap_work, tap_and_hold_start_work, button_release_work, scroll_inertia_work;
 struct k_mutex settings_lock;
 struct k_sem rdy_sem, transition_sem;
 atomic_t requested_enabled, output_enabled, request_generation, pm_suspended, transition_result;
 bool initialized, active_pending, lp2_pending, enabled, power_force_comms;
 uint32_t applied_generation, transition_generation;
 uint8_t buttons_pressed;
 bool active_tap_hold, tap_and_hold_release_pending, single_tap_pending, tap_sequence_second_touch, tap_and_hold_start_pending, is_touching;
 int last_touch_down_time, gesture_pointer_suppress_ticks, pointer_resume_guard_frames;
 int inertia_scroll_x_acc, inertia_scroll_y_acc, scroll_x_acc, scroll_y_acc, scroll_contact_fingers;
 struct { int velocity, remainder; } scroll_inertia_state;
 struct iqs915x_motion_history scroll_motion_history;
 int confirmed_mode, work_state;
};
static unsigned releases, cancellations, tracking_resets, gesture_resets, restores;
static int atomic_get(const int *p) { return *p; }
static void atomic_set(int *p, int v) { *p=v; }
static void atomic_clear(int *p) { *p=0; }
static void atomic_inc(int *p) { (*p)++; }
static bool atomic_cas(int *p, int old, int value) { if (*p!=old) return false; *p=value; return true; }
static void k_sem_reset(struct k_sem *s) { s->count=0; }
static void k_sem_give(struct k_sem *s) { s->count++; }
static int k_sem_take(struct k_sem *s, int wait) { (void)wait; return s->count ? 0 : -1; }
static void k_mutex_lock(struct k_mutex *m, int wait) { (void)wait; assert(!m->held); m->held=true; }
static void k_mutex_unlock(struct k_mutex *m) { m->held=false; }
static void k_work_cancel_delayable(struct k_work_delayable *w) { w->scheduled=false; cancellations++; }
static void k_work_cancel_delayable_sync(struct k_work_delayable *w, struct k_work_sync *sync) { (void)sync; k_work_cancel_delayable(w); }
static int input_report_key(const struct device *dev, int code, int value, bool sync, int wait) {
 (void)dev; assert(code>=INPUT_BTN_0 && code<=INPUT_BTN_0+2 && !value && sync && wait==K_FOREVER); releases++; return 0;
}
static void iqs915x_reset_absolute_tracking(struct iqs915x_data *data) { (void)data; tracking_resets++; }
static void iqs915x_reset_runtime_gesture_state(struct iqs915x_data *data) { (void)data; gesture_resets++; }
static void iqs915x_complete_transition(struct iqs915x_data *data, int result) { (void)data; assert(!result); }
static void iqs915x_restore_mode(struct iqs915x_data *data) { restores++; data->work_state=WORK_SET_POWER; }
FUNCTIONS
int main(void) {
 struct iqs915x_data data={0}; struct device dev={&data}; data.dev=&dev;
 data.initialized=data.enabled=true; data.requested_enabled=data.output_enabled=1;
 data.request_generation=data.applied_generation=20; data.confirmed_mode=IQS915X_MODE_LP2;
 data.buttons_pressed=3; data.active_tap_hold=true;
 data.scroll_inertia_state.velocity=100; data.scroll_inertia_state.remainder=45;
 data.inertia_scroll_x_acc=data.inertia_scroll_y_acc=data.scroll_x_acc=data.scroll_y_acc=12;
 data.scroll_contact_fingers=2;
 data.tap_and_hold_release_work.scheduled=data.single_tap_work.scheduled=data.tap_and_hold_start_work.scheduled=true;
 data.button_release_work.scheduled=data.scroll_inertia_work.scheduled=true;
 assert(iqs915x_request_enabled(&dev, false, false)==0);
 assert(!data.output_enabled && !data.requested_enabled && data.request_generation==21);
 assert(!iqs915x_work_session_is_current(&data, 20));
 iqs915x_apply_pending_power_request(&data);
 assert(releases==2 && !data.buttons_pressed && !data.active_tap_hold);
 assert(!data.scroll_inertia_work.scheduled && !data.scroll_inertia_state.velocity && !data.scroll_inertia_state.remainder);
 assert(!data.inertia_scroll_x_acc && !data.inertia_scroll_y_acc && !data.scroll_x_acc && !data.scroll_y_acc && !data.scroll_contact_fingers);
 assert(cancellations==6 && tracking_resets==1 && gesture_resets==1);
 assert(iqs915x_request_enabled(&dev, true, false)==0);
 assert(data.requested_enabled && !data.output_enabled && data.request_generation==22);
 assert(!iqs915x_work_session_is_current(&data, 21));
 iqs915x_apply_pending_power_request(&data); assert(restores==1 && !data.enabled);
 /* HOLD may exist without a remembered button bit: it must release too. */
 data.active_tap_hold=true; data.buttons_pressed=0;
 iqs915x_reset_input_session(&data); assert(releases==3 && !data.active_tap_hold);
 puts("PASS request closes output immediately; reset releases buttons/hold and stops inertia; re-enable rejects old work sessions");
}
'''.replace('FUNCTIONS', functions)
with tempfile.TemporaryDirectory(prefix='iqs915x-session-reset-') as tmp:
    source = Path(tmp) / 'test.c'; source.write_text(harness)
    binary = Path(tmp) / 'test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(root / 'drivers/input'), str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
