#!/usr/bin/env python3
"""Compile production power-control code and exercise failures and request races."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root/'drivers/input/iqs915x_power.c').read_text()
source = '\n'.join(s for s in source.splitlines() if not s.startswith('#include'))
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#define IS_ENABLED(x) (x)
#define CONFIG_PM_DEVICE 1
#define K_FOREVER -1
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define WORK_READ_DATA 0
typedef int atomic_val_t;
struct k_mutex { bool locked; };
struct k_sem { int count; };
struct iqs915x_data {
    struct k_mutex settings_lock, power_lock;
    struct k_sem transition_sem, rdy_sem;
    int pm_suspended, requested_enabled, output_enabled, request_generation, transition_result;
    bool initialized, active_pending, lp2_pending, enabled, pm_saved_enabled;
    int work_state;
    int mode_transitioning, requested_mode, actual_mode, last_error, lifecycle, pm_saved_mode;
};
struct device { struct iqs915x_data *data; };
static bool device_is_ready(const struct device *dev) { return dev != NULL; }
PUBLIC_TYPES
enum pm_device_action { PM_DEVICE_ACTION_SUSPEND, PM_DEVICE_ACTION_RESUME, OTHER };
static int atomic_get(const int *p) { return *p; }
static void atomic_set(int *p, int v) { *p=v; }
static void atomic_clear(int *p) { *p=0; }
static bool atomic_cas(int *p,int old,int v) { if(*p!=old)return false; *p=v; return true; }
static void atomic_inc(int *p) { (*p)++; }
static int k_mutex_lock(struct k_mutex *m, int wait) {
    if(m->locked) { assert(wait==K_NO_WAIT); return -EBUSY; }
    m->locked=true; return 0;
}
static void k_mutex_unlock(struct k_mutex *m) { assert(m->locked); m->locked=false; }
static void k_sem_reset(struct k_sem *s) { s->count=0; }
static void k_sem_give(struct k_sem *s) { s->count++; }
int iqs915x_set_enabled(const struct device *, bool);
static struct iqs915x_data data;
static struct device dev={&data};
static int worker_result, wait_result, raced;
static bool inject_request;
static int k_sem_take(struct k_sem *sem, int timeout) {
    assert(sem==&data.transition_sem && timeout==2000);
    if(inject_request) {
        int gen=data.request_generation;
        assert(iqs915x_set_enabled(&dev,true)==-EBUSY);
        assert(gen==data.request_generation && !data.requested_enabled);
        raced++;
    }
    if(wait_result<0) return wait_result;
    data.transition_result=worker_result;
    if(worker_result==0) {
        data.enabled=data.requested_enabled;
        data.output_enabled=data.enabled && !data.pm_suspended;
    }
    return 0;
}
'''
public = (root/'include/iqs915x.h').read_text()
public_types = '\n'.join(re.search(pattern, public, re.S)[0] for pattern in (r'enum iqs915x_lifecycle.*?};', r'enum iqs915x_mode.*?};', r'struct iqs915x_status.*?};'))
stub = stub.replace('PUBLIC_TYPES', public_types)
main = r'''
static void reset(bool enabled) {
    memset(&data,0,sizeof(data));
    data.initialized=true;
    data.requested_mode=enabled ? IQS915X_POWER_ACTIVE : IQS915X_POWER_LP2;
    data.enabled=data.requested_enabled=data.output_enabled=enabled;
    worker_result=wait_result=raced=0; inject_request=false;
}
int main(void) {
    reset(true); inject_request=true;
    assert(iqs915x_pm_action(&dev,PM_DEVICE_ACTION_SUSPEND)==0);
    assert(data.pm_saved_enabled && data.pm_suspended && !data.enabled && raced==1);
    int gen=data.request_generation;
    assert(iqs915x_set_enabled(&dev,true)==-EBUSY && gen==data.request_generation);
    inject_request=false;
    assert(iqs915x_pm_action(&dev,PM_DEVICE_ACTION_RESUME)==0);
    assert(!data.pm_suspended && data.requested_enabled && data.enabled && data.output_enabled);
    puts("PASS suspend confirms completion, excludes concurrent/runtime requests, resume restores enabled state");

    for(int enabled=0;enabled<2;enabled++) {
        for(int timeout=0;timeout<2;timeout++) {
            reset(enabled);
            if(timeout) wait_result=-EAGAIN; else worker_result=-EIO;
            assert(iqs915x_pm_action(&dev,PM_DEVICE_ACTION_SUSPEND)==(timeout?-ETIMEDOUT:-EIO));
            assert(!data.pm_suspended && data.requested_enabled==enabled);
            assert(data.pm_saved_enabled==enabled && data.request_generation >= (enabled ? 2 : 1));
            assert(!data.output_enabled); /* Never reopen output before hardware verification. */
            int recovery_generation=data.request_generation;
            worker_result=wait_result=0;
            assert(iqs915x_pm_action(&dev,PM_DEVICE_ACTION_SUSPEND)==0);
            assert(data.pm_saved_enabled==enabled && data.request_generation>recovery_generation);
            assert(iqs915x_pm_action(&dev,PM_DEVICE_ACTION_RESUME)==0);
            assert(data.requested_enabled==enabled && data.enabled==enabled);
        }
    }
    puts("PASS error/timeout restore original on/off intent with a superseding generation; retry preserves it");
    reset(true);
    assert(iqs915x_pm_action(&dev,OTHER)==-ENOTSUP);
    assert(!data.power_lock.locked && !data.settings_lock.locked);
    puts("PASS unsupported PM action releases locks without changing state");
}
'''
with tempfile.TemporaryDirectory(prefix='iqs915x-pm-test-') as tmp:
    p=Path(tmp)
    (p/'test.c').write_text(stub+source+main)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True)
    subprocess.run([str(p/'test')],check=True)
