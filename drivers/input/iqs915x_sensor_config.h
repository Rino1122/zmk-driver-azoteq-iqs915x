/* Driver-owned overrides and coordinate calibration for the IQS9150 export. */
#ifndef IQS915X_SENSOR_CONFIG_H
#define IQS915X_SENSOR_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* Applied before init-data transfer; the Azoteq export remains unmodified. */
#define IQS915X_ACTIVE_SAMPLING_PERIOD_MS 15U
#define IQS915X_LP2_SAMPLING_PERIOD_MS 500U
#define IQS915X_REATI_RETRY_SECONDS 1U
#define IQS915X_SENSOR_FLIP_X false
#define IQS915X_SENSOR_FLIP_Y false
#define IQS915X_SENSOR_SWITCH_XY false

#define IQS915X_COORD_X_BLOCKS 6U
#define IQS915X_COORD_Y_BLOCKS 4U
/* Q15 samples at u = i/16 of g(u) = u + 0.5*u*(1-u), where u is
 * distance from a key center divided by the half-block width. Mirror the
 * curve on each side of every key: centers and gaps stay fixed, while
 * intermediate coordinates move toward the gaps. Both axes use a = 0.5;
 * block widths come from their respective init-data resolutions.
 */
static const uint16_t iqs915x_coord_lut_x[] = {
    0, 3008, 5888, 8640, 11264, 13760, 16128, 18368, 20480,
    22464, 24320, 26048, 27648, 29120, 30464, 31680, 32768,
};
static const uint16_t iqs915x_coord_lut_y[] = {
    0, 3008, 5888, 8640, 11264, 13760, 16128, 18368, 20480,
    22464, 24320, 26048, 27648, 29120, 30464, 31680, 32768,
};

#endif
