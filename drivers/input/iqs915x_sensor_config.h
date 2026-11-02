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

/* Profile indices match channel-profile in the common DTS binding.
 * Bit positions are logical Rx columns, rows are logical Tx (not XY-flipped).
 * Bretagne has 18 Rx columns x 12 Tx rows; its missing corner is 6 x 3. */
#define IQS915X_CHANNEL_PROFILE_EXPORT 0U
#define IQS915X_CHANNEL_PROFILE_BRETAGNE_RIGHT 1U
#define IQS915X_CHANNEL_PROFILE_BRETAGNE_LEFT 2U
#define IQS915X_CHANNEL_DISABLE_BASE 0x1246U
#define IQS915X_CHANNEL_DISABLE_BYTES 88U
#define IQS915X_PROFILE_TOTAL_RXS 0x11E3U
#define IQS915X_PROFILE_TOTAL_TXS 0x11E4U
static inline uint8_t iqs915x_channel_disable_byte(uint8_t profile, unsigned index,
                                                 uint8_t original)
{
    if (profile == IQS915X_CHANNEL_PROFILE_EXPORT) return original;
    unsigned row=index/4, byte=index%4;
    uint32_t mask=0;
    if (row>=9 && row<12)
        mask=profile==IQS915X_CHANNEL_PROFILE_BRETAGNE_LEFT ? 0x0003F000U:0x0000003FU;
    return (uint8_t)(mask>>(8*byte));
}

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
