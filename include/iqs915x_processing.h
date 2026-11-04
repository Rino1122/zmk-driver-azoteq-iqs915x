/* SPDX-License-Identifier: MIT */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define IQS915X_PROCESSING_SETTINGS_SIZE 12U
struct iqs915x_processing_settings {
    int16_t coefficient_permille; /* -900..900; g(u)=u+a*u*(1-u), a=value/1000 */
    uint16_t bottom_beta; /* 0..256; beta/256 is the gain at the 15 ms reference */
    uint32_t bottom_speed, top_speed; /* corrected sensor pixels/second */
};
#define IQS915X_PROCESSING_DEFAULTS {500,60,5333,233333}
static inline bool iqs915x_processing_valid(const struct iqs915x_processing_settings *p) {
    return p && p->coefficient_permille>=-900 && p->coefficient_permille<=900 &&
        p->bottom_beta<=256 && p->bottom_speed<p->top_speed && p->top_speed<=1000000;
}
struct device;
/* q: op:u8, reserved:u32=0, reserved:u32=0, settings:12 LE bytes for SET.
 * op=0 GET, op=1 SET. Reply: version:u8=1, pending:u8, profile:u8,
 * lifecycle:u8, maxX:u16, maxY:u16, applied settings:12, generation:u32.
 * SET is atomic and becomes active at all-up or while no output is active.
 * No sensor I/O; callable from a non-sensor thread. */
int iqs915x_processing_command(const struct device *dev,const uint8_t *q,unsigned n,
    uint8_t *out,unsigned *length);
