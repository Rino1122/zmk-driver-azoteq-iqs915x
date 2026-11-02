#!/usr/bin/env python3
"""Run actual init chunk preparation: preserve export tuning, apply driver policy."""
from pathlib import Path
import re
import subprocess
import tempfile
from init_event_mode import block

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "drivers/input/iqs915x.c").read_text()
registers = (ROOT / "drivers/input/iqs915x_regs.h").read_text()
definitions = "\n".join(re.findall(r"^#define IQS915X_.*$", registers, re.M))
policy = block(source, "static uint16_t iqs915x_apply_config_settings_policy")
policy += "\n" + block(source, "static uint16_t iqs915x_config_settings_without_event_mode")
prepare = block(source, "static bool iqs915x_channel_profile_valid") + "\n" + block(source, "static int iqs915x_prepare_init_chunk")
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define BIT(n) (1U << (n))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
DEFINITIONS
#include "iqs915x_init_array.h"
#include "iqs915x_sensor_config.h"
struct iqs915x_config {
  uint8_t channel_profile;
  const uint8_t *init_data;
  uint16_t init_data_len, report_rate_ms;
  bool flip_x, flip_y, switch_xy;
};
struct device { const struct iqs915x_config *config; };
static uint8_t retained[88];
static unsigned mask_reads;
static int mask_read_error;
static int iqs915x_read_block(const struct device *dev,uint16_t address,uint8_t *out,uint16_t n) {
 (void)dev;mask_reads++;
 assert(address>=IQS915X_CHANNEL_DISABLE_BASE && address+n<=IQS915X_CHANNEL_DISABLE_BASE+88);
 assert(n>0 && n<=IQS915X_INIT_WRITE_CHUNK_SIZE);
 if(mask_read_error)return mask_read_error;
 memcpy(out,retained+address-IQS915X_CHANNEL_DISABLE_BASE,n);return 0;
}
POLICY
PREPARE
CHECK_MASK
static uint16_t read16(const uint8_t *bytes, int address) {
  int offset = address - 0x115C;
  return bytes[offset] | ((uint16_t)bytes[offset + 1] << 8);
}
int main(void) {
  struct iqs915x_config config = {
    .init_data = iqs915x_init_data, .init_data_len = sizeof(iqs915x_init_data),
    .report_rate_ms = IQS915X_ACTIVE_SAMPLING_PERIOD_MS,
    .flip_x = IQS915X_SENSOR_FLIP_X, .flip_y = IQS915X_SENSOR_FLIP_Y,
    .switch_xy = IQS915X_SENSOR_SWITCH_XY,
  };
  struct device dev = { .config = &config };
  uint8_t written[1174], verified[1174], original[1174];
  memcpy(original, iqs915x_init_data, sizeof(original));
  uint16_t addr = 0, chunk = 0;
  assert(iqs915x_prepare_init_chunk(&dev, 1174, &addr, &chunk, written, false) == -EINVAL);
  for (int pass = 0; pass < 2; pass++) {
    uint16_t offset = 0;
    uint8_t *result = pass ? verified : written;
    while (offset < 1174) {
      assert(iqs915x_prepare_init_chunk(&dev, offset, &addr, &chunk, result + offset, pass == 0) == 0);
      assert(addr == (offset < 1168 ? 0x115C + offset : 0x2000 + offset - 1168));
      assert(chunk > 0 && chunk <= IQS915X_INIT_WRITE_CHUNK_SIZE);
      offset += chunk;
    }
    assert(offset == 1174);
  }
  assert(memcmp(written, verified, 1174) == 0);
  assert(memcmp(original, iqs915x_init_data, 1174) == 0);
  for (int offset = 0; offset < 1174; offset++) {
    int address = offset < 1168 ? 0x115C + offset : 0x2000 + offset - 1168;
    switch (address) {
      case IQS915X_SYSTEM_CONTROL:
      case IQS915X_CONFIG_SETTINGS: case IQS915X_CONFIG_SETTINGS + 1:
      case IQS915X_LP2_MODE_REPORT_RATE: case IQS915X_LP2_MODE_REPORT_RATE + 1:
      case IQS915X_OTHER_SETTINGS:
      case IQS915X_REATI_RETRY_TIME:
      case IQS915X_ACTIVE_MODE_REPORT_RATE: case IQS915X_ACTIVE_MODE_REPORT_RATE + 1:
      case IQS915X_TRACKPAD_SETTINGS:
      case IQS915X_SINGLE_FINGER_GESTURES_ENABLE: case IQS915X_TWO_FINGER_GESTURES_ENABLE:
        break;
      default: assert(written[offset] == original[offset]);
    }
  }
  assert(read16(written, IQS915X_ACTIVE_MODE_REPORT_RATE) == 15);
  assert(read16(written, IQS915X_LP2_MODE_REPORT_RATE) == 500);
  assert(written[IQS915X_REATI_RETRY_TIME - 0x115C] == 1);
  /* Export tuning differs from the retired firmware DTS and must survive. */
  assert(read16(written, 0x1196) == 700);
  assert(read16(written, 0x119C) == 250);
  assert(read16(written, 0x119E) == 4096);
  assert(written[0x11A0 - 0x115C] == 100);
  assert(!(written[IQS915X_SYSTEM_CONTROL - 0x115C] & 0xF8));
  uint16_t cfg = read16(written, IQS915X_CONFIG_SETTINGS);
  assert((cfg & (IQS915X_MANUAL_CONTROL | IQS915X_TP_EVENT | IQS915X_TP_REATI_ENABLE | IQS915X_REATI_EVENT)) ==
         (IQS915X_MANUAL_CONTROL | IQS915X_TP_EVENT | IQS915X_TP_REATI_ENABLE | IQS915X_REATI_EVENT));
  assert(!(cfg & (IQS915X_FORCE_COMMS_METHOD | IQS915X_TERMINATE_COMMS | IQS915X_EVENT_MODE |
                 IQS915X_GESTURE_EVENT | IQS915X_TP_TOUCH_EVENT | IQS915X_ALP_REATI_ENABLE | IQS915X_ALP_EVENT)));
  assert(read16(written, IQS915X_ALP_SETUP) == read16(original, IQS915X_ALP_SETUP));
  assert(read16(written, IQS915X_ALP_SETUP + 2) == read16(original, IQS915X_ALP_SETUP + 2));
  assert(!(written[IQS915X_OTHER_SETTINGS - 0x115C] & (BIT(5) | BIT(4))));
  assert(!(written[IQS915X_TRACKPAD_SETTINGS - 0x115C] & (IQS915X_FLIP_X | IQS915X_FLIP_Y | IQS915X_SWITCH_XY_AXIS)));
  assert(!(written[IQS915X_SINGLE_FINGER_GESTURES_ENABLE - 0x115C] & (IQS915X_SINGLE_TAP | IQS915X_PRESS_AND_HOLD)));
  assert(!(written[IQS915X_TWO_FINGER_GESTURES_ENABLE - 0x115C] & (IQS915X_TWO_FINGER_TAP | IQS915X_SCROLL)));
  assert(sizeof(iqs915x_coord_lut_x) / sizeof(uint16_t) == 17);
  assert(sizeof(iqs915x_coord_lut_y) / sizeof(uint16_t) == 17);
  uint8_t baseline[1174];memcpy(baseline,written,sizeof(baseline));
  unsigned mask_offset=IQS915X_CHANNEL_DISABLE_BASE-IQS915X_INIT_DATA_BASE_ADDR;
  for(unsigned selected=0;selected<=2;selected++) {
    config.channel_profile=selected;
    for(unsigned offset=0;offset<1174;offset+=chunk) {
      assert(!iqs915x_prepare_init_chunk(&dev,offset,&addr,&chunk,written+offset,false));
      assert(!iqs915x_prepare_init_chunk(&dev,offset,&addr,&chunk,verified+offset,false));
    }
    assert(!memcmp(written,verified,sizeof(written)));
    unsigned disabled=0;
    for(unsigned i=0;i<88;i++)for(unsigned bit=0;bit<8;bit++)
      if(written[mask_offset+i]&(1u<<bit)) {
        unsigned row=i/4,col=(i%4)*8+bit;disabled++;
        assert(row>=9 && row<12);
        assert(selected==2 ? col>=12 && col<18 : col<6);
      }
    assert(disabled==18);
    for(unsigned i=0;i<1174;i++)if(i<mask_offset || i>=mask_offset+88)
      assert(written[i]==baseline[i]);
    if(selected!=2)assert(!memcmp(written,baseline,sizeof(written)));
    memcpy(retained,written+mask_offset,88);mask_reads=0;
    assert(iqs915x_check_channel_profile(&dev)==0);
    assert(mask_reads==(88+IQS915X_INIT_WRITE_CHUNK_SIZE-1)/IQS915X_INIT_WRITE_CHUNK_SIZE);
    retained[87]^=1;assert(iqs915x_check_channel_profile(&dev)==1);
    retained[87]^=1;mask_read_error=-EIO;assert(iqs915x_check_channel_profile(&dev)==-EIO);
    mask_read_error=0;
  }
  /* A warm left profile must reject retained right-side bytes with same X resolution. */
  memcpy(retained,baseline+mask_offset,88);config.channel_profile=2;
  assert(iqs915x_check_channel_profile(&dev)==1);
  uint8_t wrong_geometry[1174];memcpy(wrong_geometry,iqs915x_init_data,sizeof(wrong_geometry));
  wrong_geometry[IQS915X_PROFILE_TOTAL_RXS-IQS915X_INIT_DATA_BASE_ADDR]=17;
  config.init_data=wrong_geometry;
  assert(iqs915x_prepare_init_chunk(&dev,0,&addr,&chunk,written,false)==-EINVAL);
  puts("Channel profiles: 18x12 bottom3 x left/right6, write/readback, retained mask and I/O errors: PASS");
  puts("IQS915x init-data: 1174 export bytes, driver overrides and read-back preparation verified");
}
'''
for marker, code in (("DEFINITIONS", definitions), ("POLICY", policy), ("PREPARE", prepare), ("CHECK_MASK", block(source,"static int iqs915x_check_channel_profile"))):
    harness = harness.replace(marker, code)
with tempfile.TemporaryDirectory() as temporary:
    directory = Path(temporary)
    subprocess.run(["python3", str(ROOT / "scripts/generate_init_array.py"), str(ROOT / "drivers/input/IQS9150_init.h"), "-o", str(directory / "iqs915x_init_array.h")], check=True)
    (directory / "test.c").write_text(harness)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "drivers/input"), "-I", str(directory), str(directory / "test.c"), "-o", str(directory / "test")], check=True)
    subprocess.run([str(directory / "test")], check=True)
