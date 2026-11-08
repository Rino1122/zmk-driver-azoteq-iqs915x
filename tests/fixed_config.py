#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
code=r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "IQS9150_init.h"
#define BIT(n) (1u<<(n))
#define IQS915X_INIT_DATA_BASE_ADDR 0x115C
#define IQS915X_TRACKPAD_SETTINGS 0x11E2
#define IQS915X_ACTIVE_MODE_REPORT_RATE 0x1176
struct iqs915x_config {const uint8_t *init_data;uint16_t report_rate_ms;};
struct device {const struct iqs915x_config *config;};
static uint8_t retained[8],bits=0x28;static uint16_t period=15;static int error;
static int iqs915x_read_block(const struct device *d,uint16_t addr,uint8_t *out,uint16_t n){(void)d;if(error)return error;if(addr==0x11EA){assert(n==8);memcpy(out,retained,n);}else{assert(addr==IQS915X_TRACKPAD_SETTINGS&&n==1);*out=bits;}return 0;}
static int iqs915x_read_reg16(const struct device *d,uint16_t addr,uint16_t *out){(void)d;assert(addr==IQS915X_ACTIVE_MODE_REPORT_RATE);*out=period;return error;}
#include "iqs915x_fixed_config.inc"
int main(void){uint8_t data[256]={0};uint8_t expected[]={0x98,0x3A,0x74,0x40,100,0,0,5};memcpy(data+0x11EA-0x115C,expected,8);memcpy(retained,expected,8);
struct iqs915x_config c={data,15};struct device d={&c};assert(!iqs915x_check_fixed_sensor_config(&d));
for(unsigned i=0;i<8;i++)if(i<5||i==7){retained[i]^=1;assert(iqs915x_check_fixed_sensor_config(&d)==1);retained[i]^=1;}
bits=0x20;assert(iqs915x_check_fixed_sensor_config(&d)==1);bits=0x38;assert(iqs915x_check_fixed_sensor_config(&d)==1);bits=0x28;
period=16;assert(iqs915x_check_fixed_sensor_config(&d)==1);period=15;error=-5;assert(iqs915x_check_fixed_sensor_config(&d)==-5);
puts("Fixed init config: cold/warm readback, IIR bits, speed/beta/split/period mismatch and I/O failures PASS");}
"""
with tempfile.TemporaryDirectory() as tmp:
 p=Path(tmp);(p/'test.c').write_text(code)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+str(r/'drivers/input'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
