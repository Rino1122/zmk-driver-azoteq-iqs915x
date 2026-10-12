#!/usr/bin/env python3
"""Exercise pointer frame submission with disable injected between X and Y."""
from pathlib import Path
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
from init_event_mode import block

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "drivers/input/iqs915x.c").read_text()
functions = "\n".join(block(source, marker) for marker in (
    "static uint32_t iqs915x_request_generation",
    "static bool iqs915x_output_is_enabled",
    "static bool iqs915x_work_session_is_current",
    "static void iqs915x_report_pointer_pair",
))
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define INPUT_REL_X 0
#define INPUT_REL_Y 1
#define K_FOREVER -1
#define LOG_WRN(...) ((void)0)
struct device { int unused; };
struct iqs915x_data {
  const struct device *dev;
  int output_enabled, request_generation;
};
static struct iqs915x_data data;
static int pending_x, pending_y, reports, last_x, last_y, submissions;
static bool disable_after_x, fail_x;
static int atomic_get(const int *p) { return *p; }
/* Model a listener that accumulates REL axes until a sync event arrives. */
static int input_report_rel(const struct device *dev, int code, int value,
                            bool sync, int timeout) {
  (void)dev; assert(timeout == K_FOREVER); submissions++;
  if (code == INPUT_REL_X && fail_x) { return -5; }
  if (code == INPUT_REL_X) {
    assert(!sync); pending_x += value;
    if (disable_after_x) { data.output_enabled=0; data.request_generation++; }
  } else {
    assert(code == INPUT_REL_Y && sync); pending_y += value;
  }
  if (sync) {
    reports++; last_x=pending_x; last_y=pending_y;
    pending_x=pending_y=0;
  }
  return 0;
}
FUNCTIONS
static void setup(void) {
  data=(struct iqs915x_data){.output_enabled=1, .request_generation=10};
  pending_x=pending_y=reports=last_x=last_y=submissions=0;
  disable_after_x=fail_x=false;
}
int main(void) {
  setup(); iqs915x_report_pointer_pair(&data,10,-88,30);
  assert(reports==1 && last_x==-88 && last_y==30 && submissions==2);
  setup(); disable_after_x=true;
  iqs915x_report_pointer_pair(&data,10,-88,30);
  assert(reports==1 && last_x==-88 && last_y==30 && submissions==2);
  assert(!pending_x && !pending_y && !data.output_enabled);
  /* Further movement is suppressed; no delta leaks into a later sync. */
  iqs915x_report_pointer_pair(&data,10,500,500);
  assert(submissions==2 && !pending_x && !pending_y);
  setup(); data.output_enabled=0;
  iqs915x_report_pointer_pair(&data,10,500,500); assert(!submissions);
  /* A rapid disable/enable must reject a frame from the older session. */
  setup(); data.request_generation=12;
  iqs915x_report_pointer_pair(&data,10,500,500); assert(!submissions);
  setup(); fail_x=true;
  iqs915x_report_pointer_pair(&data,10,-88,30);
  assert(submissions==1 && !reports && !pending_x && !pending_y);
  puts("IQS915x pointer: disable between axes, stale session and admission failure passed");
}
'''.replace("FUNCTIONS", functions)
with tempfile.TemporaryDirectory(prefix="iqs915x-pointer-") as temporary:
    directory = Path(temporary)
    (directory / "test.c").write_text(harness)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    str(directory / "test.c"), "-o", str(directory / "test")], check=True)
    subprocess.run([str(directory / "test")], check=True)
