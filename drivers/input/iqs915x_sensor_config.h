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
static const uint16_t iqs915x_coord_lut_x[] = {
    0, 3596, 4314, 6980, 9152, 11723, 14282, 15850, 17002,
    19860, 21795, 23755, 25945, 27475, 29719, 30583, 32768,
};
static const uint16_t iqs915x_coord_lut_y[] = {
    0, 3901, 3933, 6509, 9466, 10858, 13425, 14987, 17489,
    19561, 21659, 23583, 25359, 27318, 29356, 31054, 32768,
};

#endif
