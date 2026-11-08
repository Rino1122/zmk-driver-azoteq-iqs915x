#!/usr/bin/env python3
"""Exercise production sensor RPC/readback with deterministic host I/O stubs."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
code=r"""
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <iqs915x_sensor_tuning.h>
#define BIT(n) (1u<<(n))
#define K_FOREVER 0
#define IQS915X_READY 2
#define IQS915X_TRACKPAD_SETTINGS 0x11E2
#define IQS915X_ACTIVE_MODE_REPORT_RATE 0x1160
struct iqs915x_data {uint8_t sensor_initializing[12],sensor_desired[12],sensor_applied[12];bool sensor_verified;uint16_t active_sampling_period_ms;int sensor_request,lifecycle,pm_suspended,trace_active,channel_running,channel_pending,processing_lock,rdy_sem;};
struct device {struct iqs915x_data *data;};
static int wakes,reads,error;static uint8_t regs[8],bits;static uint16_t period;
static int atomic_get(int *p){return *p;}
static void atomic_set(int *p,int v){*p=v;}
static void k_mutex_lock(int *p,int t){(void)p;(void)t;}
static void k_mutex_unlock(int *p){(void)p;}
static void k_sem_give(int *p){(void)p;wakes++;}
static int iqs915x_read_block(const struct device *dev,uint16_t addr,uint8_t *b,uint16_t n){(void)dev;reads++;if(error)return error;if(addr==0x11EA&&n==8)memcpy(b,regs,n);else {assert(addr==IQS915X_TRACKPAD_SETTINGS&&n==1);*b=bits;}return 0;}
static int iqs915x_read_reg16(const struct device *dev,uint16_t addr,uint16_t *p){(void)dev;assert(addr==IQS915X_ACTIVE_MODE_REPORT_RATE);reads++;*p=period;return error;}
#include "iqs915x_sensor_tuning.inc"
int main(void){
 const uint8_t defaults[12]={6,60,0,0,0xd5,0x14,0,0,0x75,0x8f,3,0};
 struct iqs915x_data d={.active_sampling_period_ms=15,.lifecycle=IQS915X_READY,.sensor_verified=true};struct device dev={&d};
 memcpy(d.sensor_initializing,defaults,12);memcpy(d.sensor_desired,defaults,12);memcpy(d.sensor_applied,defaults,12);
 assert(iqs915x_sensor_valid(defaults,15));assert(iqs915x_sensor_cycle(100,15)==2);
 uint8_t q[21]={7},out[96];unsigned n=0;memcpy(q+9,defaults,12);
 assert(!iqs915x_sensor_command(&dev,q,21,out,&n)&&n==34&&!out[1]&&reads==0&&wakes==0);
 q[9]=0;assert(!iqs915x_sensor_command(&dev,q,21,out,&n)&&out[1]&&wakes==1&&d.sensor_request);
 assert(d.sensor_applied[0]==6&&d.sensor_desired[0]==0); /* atomic staging */
 assert(!iqs915x_sensor_command(&dev,q,21,out,&n)&&wakes==1); /* retry */
 d.trace_active=1;q[9]=2;assert(iqs915x_sensor_command(&dev,q,21,out,&n)==-EBUSY);d.trace_active=0;
 d.lifecycle=4;assert(iqs915x_sensor_command(&dev,q,21,out,&n)==-ENODEV);d.lifecycle=IQS915X_READY;
 q[12]=1;assert(iqs915x_sensor_command(&dev,q,21,out,&n)==-ERANGE);q[12]=0;
 q[1]=1;assert(iqs915x_sensor_command(&dev,q,21,out,&n)==-EINVAL);q[1]=0;
 assert(iqs915x_sensor_command(&dev,q,20,out,&n)==-EINVAL);
 for(unsigned i=0;i<8;i++)regs[i]=sensor_patch_byte(&dev,0x11EA+i,0);period=15;bits=0;
 assert(!sensor_check_registers(&dev));regs[7]^=1;assert(sensor_check_registers(&dev)==1);regs[7]^=1;
 regs[0]^=1;assert(sensor_check_registers(&dev)==1);regs[0]^=1;
 bits=BIT(3);assert(sensor_check_registers(&dev)==1);bits=0;period=16;assert(sensor_check_registers(&dev)==1);period=15;
 error=-EIO;assert(sensor_check_registers(&dev)==-EIO);error=0;
 d.sensor_initializing[2]=1;bits=BIT(3);assert(!sensor_check_registers(&dev));bits|=BIT(4);assert(sensor_check_registers(&dev)==1);
 puts("Sensor tuning: rounding, staged/idempotent RPC, busy/rejected requests, readback corruption, I/O errors and retained policy: PASS");
}
"""
with tempfile.TemporaryDirectory() as tmp:
 p=Path(tmp);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(root/'include'),'-I',str(root/'drivers/input'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
