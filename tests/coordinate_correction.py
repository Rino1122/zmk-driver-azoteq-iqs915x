#!/usr/bin/env python3
"""Check the actual C correction path and init-data resolution loading."""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
from init_event_mode import block

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "drivers/input/iqs915x.c").read_text()
registers = (ROOT / "drivers/input/iqs915x_regs.h").read_text()
definitions = "\n".join(re.findall(r"^#define IQS915X_.*$", registers, re.M))
for name in ("IQS915X_COORD_LUT_Q15_SCALE", "IQS915X_DEFAULT_SWIPE_THRESHOLD_FALLBACK"):
    definitions += "\n" + re.search(r"^#define " + name + r"\b.*$", source, re.M)[0]
functions = "\n".join(block(source, marker) for marker in (
    "static uint16_t iqs915x_correct_half_block_distance",
    "static uint16_t iqs915x_correct_axis_coordinate",
    "static void iqs915x_correct_stream_coordinates",
    "static bool iqs915x_get_init_data_reg16",
    "static void iqs915x_configure_swipe_thresholds",
))
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define BIT(n) (1U << (n))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define LOG_INF(...) ((void)0)
#define LOG_WRN(...) ((void)0)
DEFINITIONS
#include "iqs915x_init_array.h"
#include "iqs915x_sensor_config.h"
struct iqs915x_config {
  const uint8_t *init_data;
  uint16_t init_data_len, swipe_threshold_numerator, swipe_threshold_denominator;
  uint16_t swipe_step;
  const uint16_t *coord_lut_x_q15, *coord_lut_y_q15;
  size_t coord_lut_x_len, coord_lut_y_len;
  uint8_t coord_x_blocks, coord_y_blocks;
};
struct iqs915x_data {
  uint16_t swipe_resolution_x, swipe_resolution_y;
  uint16_t swipe_threshold_x, swipe_threshold_y;
};
struct iqs915x_stream_data {
  uint16_t abs_x, abs_y, finger2_x, finger2_y;
  uint16_t finger3_x, finger3_y, finger4_x, finger4_y;
};
FUNCTIONS
static void check_axis(uint16_t resolution, uint8_t blocks, const uint16_t *lut) {
  uint16_t previous = 0;
  for (uint32_t raw = 0; raw <= resolution; raw++) {
    uint16_t corrected = iqs915x_correct_axis_coordinate(raw, resolution, blocks, lut, 17);
    assert(corrected >= previous && corrected <= resolution);
    previous = corrected;
    uint32_t b = MIN(raw * blocks / resolution, blocks - 1U);
    uint32_t start = (uint32_t)resolution * b / blocks;
    uint32_t end = (uint32_t)resolution * (b + 1U) / blocks;
    uint32_t center = (start + end) / 2U;
    if (raw == start || raw == end || raw == center) assert(corrected == raw);
    else if (raw < center) assert(corrected <= raw && corrected >= start);
    else assert(corrected >= raw && corrected <= end);
  }
  assert(iqs915x_correct_axis_coordinate(UINT16_MAX, resolution, blocks, lut, 17) == UINT16_MAX);
  assert(iqs915x_correct_axis_coordinate(resolution + 1U, resolution, blocks, lut, 17) == UINT16_MAX);
}
static void write16(uint8_t *bytes, uint16_t reg, uint16_t value) {
  bytes[reg - IQS915X_INIT_DATA_BASE_ADDR] = value;
  bytes[reg - IQS915X_INIT_DATA_BASE_ADDR + 1U] = value >> 8;
}
int main(void) {
  for (unsigned i = 0; i <= 16; i++) {
    /* Exact Q15 samples of u + 0.5*u*(1-u), u=i/16. */
    unsigned expected = 3072U * i - 64U * i * i;
    assert(iqs915x_coord_lut_x[i] == expected);
    assert(iqs915x_coord_lut_y[i] == expected);
  }
  struct iqs915x_config cfg = {
    .init_data = iqs915x_init_data, .init_data_len = sizeof(iqs915x_init_data),
    .coord_lut_x_q15 = iqs915x_coord_lut_x, .coord_lut_y_q15 = iqs915x_coord_lut_y,
    .coord_lut_x_len = 17, .coord_lut_y_len = 17,
    .coord_x_blocks = IQS915X_COORD_X_BLOCKS, .coord_y_blocks = IQS915X_COORD_Y_BLOCKS,
  };
  struct iqs915x_data data = {0};
  iqs915x_configure_swipe_thresholds(&cfg, &data);
  uint16_t expected;
  assert(iqs915x_get_init_data_reg16(&cfg, IQS915X_X_RESOLUTION, &expected));
  assert(data.swipe_resolution_x == expected && expected > 0);
  assert(iqs915x_get_init_data_reg16(&cfg, IQS915X_Y_RESOLUTION, &expected));
  assert(data.swipe_resolution_y == expected && expected > 0);
  check_axis(data.swipe_resolution_x, 6, cfg.coord_lut_x_q15);
  check_axis(data.swipe_resolution_y, 4, cfg.coord_lut_y_q15);
  /* A different export must scale both axes, even with a swipe override. */
  uint8_t changed[sizeof(iqs915x_init_data)];
  memcpy(changed, iqs915x_init_data, sizeof(changed));
  write16(changed, IQS915X_X_RESOLUTION, 12000);
  write16(changed, IQS915X_Y_RESOLUTION, 8000);
  cfg.init_data = changed;
  cfg.swipe_step = 123;
  iqs915x_configure_swipe_thresholds(&cfg, &data);
  assert(data.swipe_resolution_x == 12000 && data.swipe_resolution_y == 8000);
  struct iqs915x_stream_data stream = {
    .abs_x = 500, .abs_y = 500, .finger2_x = 1500, .finger2_y = 1500,
    .finger3_x = 2500, .finger3_y = 2500,
    .finger4_x = UINT16_MAX, .finger4_y = UINT16_MAX,
  };
  iqs915x_correct_stream_coordinates(&cfg, &data, &stream);
  assert(stream.abs_x == 375 && stream.abs_y == 375);
  assert(stream.finger2_x == 1625 && stream.finger2_y == 1625);
  assert(stream.finger3_x == 2375 && stream.finger3_y == 2375);
  assert(stream.finger4_x == UINT16_MAX && stream.finger4_y == UINT16_MAX);
  check_axis(6000, 6, cfg.coord_lut_x_q15);
  check_axis(4000, 4, cfg.coord_lut_y_q15);
  check_axis(6011, 6, cfg.coord_lut_x_q15);
  check_axis(4007, 4, cfg.coord_lut_y_q15);
  check_axis(65534, 6, cfg.coord_lut_x_q15);
  assert(iqs915x_correct_axis_coordinate(250, 0, 6, cfg.coord_lut_x_q15, 17) == 250);
  puts("IQS915x coordinates: a=0.5, fixed points, monotonicity, all slots and init-data scaling passed");
}
'''.replace("DEFINITIONS", definitions).replace("FUNCTIONS", functions)
with tempfile.TemporaryDirectory(prefix="iqs915x-coordinates-") as temporary:
    directory = Path(temporary)
    subprocess.run(["python3", str(ROOT / "scripts/generate_init_array.py"),
                    str(ROOT / "drivers/input/IQS9150_init.h"),
                    "-o", str(directory / "iqs915x_init_array.h")], check=True)
    (directory / "test.c").write_text(harness)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "drivers/input"), "-I", str(directory),
                    str(directory / "test.c"), "-o", str(directory / "test")], check=True)
    subprocess.run([str(directory / "test")], check=True)
