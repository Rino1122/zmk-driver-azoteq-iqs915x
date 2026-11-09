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
 * LandsFall has 18 Rx columns x 12 Tx rows; its missing corner is 6 x 3. */
#define IQS915X_CHANNEL_PROFILE_EXPORT 0U
#define IQS915X_CHANNEL_PROFILE_LANDSFALL_RIGHT 1U
#define IQS915X_CHANNEL_PROFILE_LANDSFALL_LEFT 2U
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
        mask=profile==IQS915X_CHANNEL_PROFILE_LANDSFALL_LEFT ? 0x0003F000U:0x0000003FU;
    return (uint8_t)(mask>>(8*byte));
}

#endif
