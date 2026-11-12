#!/usr/bin/env python3
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
registers = (root / "drivers/input/iqs915x_regs.h").read_text()
definitions = "\n".join(re.findall(r"^#define IQS915X_.*$", registers, re.M))
code = r"""
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
DEFINITIONS
struct iqs915x_config {const uint8_t *init_data;};
struct device {const struct iqs915x_config *config;};
static uint8_t retained[IQS915X_INIT_DATA_MAIN_SIZE];
static int error;
static int iqs915x_read_block(const struct device *dev, uint16_t addr, uint8_t *out, uint16_t len)
{
  (void)dev;
  if (error) { return error; }
  memcpy(out, retained + (addr - IQS915X_INIT_DATA_BASE_ADDR), len);
  return 0;
}
static int iqs915x_read_reg16(const struct device *dev, uint16_t addr, uint16_t *out)
{
  uint8_t value[2];
  int ret = iqs915x_read_block(dev, addr, value, sizeof(value));
  if (!ret) { *out = value[0] | ((uint16_t)value[1] << 8); }
  return ret;
}
#include "iqs915x_fixed_config.inc"
int main(void)
{
  uint8_t profile[sizeof(retained)];
  for (unsigned i = 0; i < sizeof(profile); i++) { profile[i] = i; }
  memcpy(retained, profile, sizeof(profile));
  struct iqs915x_config config = {profile};
  struct device dev = {&config};
  assert(iqs915x_check_fixed_sensor_config(&dev) == 0);
  const uint16_t addresses[] = {
    0x11EA, 0x11EC, 0x11EE, 0x11F1, IQS915X_TRACKPAD_SETTINGS,
    IQS915X_ACTIVE_MODE_REPORT_RATE, IQS915X_LP2_MODE_REPORT_RATE,
    IQS915X_OTHER_SETTINGS, IQS915X_REATI_RETRY_TIME,
    IQS915X_SINGLE_FINGER_GESTURES_ENABLE, IQS915X_TWO_FINGER_GESTURES_ENABLE,
  };
  for (unsigned i = 0; i < ARRAY_SIZE(addresses); i++) {
    unsigned offset = addresses[i] - IQS915X_INIT_DATA_BASE_ADDR;
    retained[offset] ^= 1;
    assert(iqs915x_check_fixed_sensor_config(&dev) == 1);
    retained[offset] ^= 1;
  }
  error = -5;
  assert(iqs915x_check_fixed_sensor_config(&dev) == -5);
  puts("Fixed init config: retained settings compared against export values");
}
""".replace("DEFINITIONS", definitions)
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / "test.c").write_text(code)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "drivers/input"), str(path / "test.c"),
                    "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
