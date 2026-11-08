/* SPDX-License-Identifier: MIT */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define IQS915X_PROCESSING_SETTINGS_SIZE 36U
struct iqs915x_processing_settings {
    int16_t coefficient_permille; /* X upper, -900..900 */
    int16_t x_middle, x_lower, y_upper, y_lower; /* -900..900 */
    int16_t y_center; /* -500..500; full-pitch displacement = value/8000 */
    uint16_t bottom_beta; /* 0..256; beta/256 is the gain at the 15 ms reference */
    uint32_t bottom_speed, top_speed;
    uint16_t stop_enabled,stop_arm_ms,stop_quiet_ms,stop_radius,stop_samples;
    uint32_t stop_fast_speed;
};
#define IQS915X_PROCESSING_DEFAULTS {500,500,500,500,500,0,60,5333,233333,1,120,30,2,3,5333}
static inline bool iqs915x_processing_valid(const struct iqs915x_processing_settings *p) {
    return p && p->coefficient_permille>=-900 && p->coefficient_permille<=900 &&
        p->x_middle>=-900 && p->x_middle<=900 && p->x_lower>=-900 && p->x_lower<=900 &&
        p->y_upper>=-900 && p->y_upper<=900 && p->y_lower>=-900 && p->y_lower<=900 &&
        p->y_center>=-500 && p->y_center<=500 && p->bottom_beta<=256 && p->bottom_speed<p->top_speed && p->top_speed<=1000000 && p->stop_enabled<=1 && p->stop_fast_speed>=1 && p->stop_fast_speed<=1000000 &&
        p->stop_arm_ms>=1 && p->stop_arm_ms<=2000 && p->stop_quiet_ms>=1 && p->stop_quiet_ms<=250 &&
        p->stop_radius>=1 && p->stop_radius<=100 && p->stop_samples>=2 && p->stop_samples<=32;
}
struct device;
/* q: op:u8, reserved:u32=0, reserved:u32=0, settings:36 LE bytes for SET.
 * op=0 GET, op=1 SET. Reply: version:u8=3, pending:u8, profile:u8,
 * lifecycle:u8, maxX:u16, maxY:u16, applied settings:36, generation:u32.
 * SET is atomic and becomes active at all-up or while no output is active.
 * No sensor I/O; callable from a non-sensor thread. */
/* Diagnostic ops 2..5: see README Multi-contact trace monitor. */
int iqs915x_processing_command(const struct device *dev,const uint8_t *q,unsigned n,
    uint8_t *out,unsigned *length);
