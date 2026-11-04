/* SPDX-License-Identifier: MIT */
#ifndef IQS915X_CALIBRATION_MATH_H
#define IQS915X_CALIBRATION_MATH_H
#include <stdbool.h>
#include <stdint.h>
#define IQS915X_CAL_COLS 17U
#define IQS915X_CAL_ROWS 13U
#define IQS915X_CAL_NODES (IQS915X_CAL_COLS * IQS915X_CAL_ROWS)
/* Host serialization must encode members explicitly, never memcpy this struct. */
struct iqs915x_calibration_grid {
    int16_t dx[IQS915X_CAL_NODES];
    int16_t dy[IQS915X_CAL_NODES];
};
/* Q12 weights. Products fit int64_t: 32768 * 4096^2 < 2^40.
 * Round the final signed sum to nearest, ties away from zero. */
static inline int32_t iqs915x_cal_interpolate(const int16_t *v, unsigned i,
                                             uint32_t fx, uint32_t fy) {
    int64_t a = 4096U - fx, b = 4096U - fy;
    int64_t sum = v[i]*a*b + (int64_t)v[i+1]*fx*b
        + (int64_t)v[i+IQS915X_CAL_COLS]*a*fy
        + (int64_t)v[i+IQS915X_CAL_COLS+1]*fx*fy;
    return sum < 0 ? -(int32_t)((-sum + (1LL<<23)) >> 24)
                   : (int32_t)((sum + (1LL<<23)) >> 24);
}
/* Invalid slots stay invalid. NULL is the exact identity operation. */
static inline bool iqs915x_cal_apply(const struct iqs915x_calibration_grid *g,
    uint16_t max_x, uint16_t max_y, uint16_t *x, uint16_t *y) {
    if (!max_x || !max_y || *x == UINT16_MAX || *y == UINT16_MAX
        || *x > max_x || *y > max_y) return false;
    if (!g) return true;
    uint32_t px = (uint32_t)*x * 16U * 4096U / max_x;
    uint32_t py = (uint32_t)*y * 12U * 4096U / max_y;
    unsigned col = px >> 12, row = py >> 12;
    if (col == 16) col = 15;
    if (row == 12) row = 11;
    unsigned i = row * IQS915X_CAL_COLS + col;
    int32_t cx = *x + iqs915x_cal_interpolate(g->dx, i, px-col*4096U, py-row*4096U);
    int32_t cy = *y + iqs915x_cal_interpolate(g->dy, i, px-col*4096U, py-row*4096U);
    *x = cx < 0 ? 0 : cx > max_x ? max_x : cx;
    *y = cy < 0 ? 0 : cy > max_y ? max_y : cy;
    return true;
}
#endif
