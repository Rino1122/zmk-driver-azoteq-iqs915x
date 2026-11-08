/* SPDX-License-Identifier: MIT */
#ifndef IQS915X_SENSOR_TUNING_H
#define IQS915X_SENSOR_TUNING_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#define IQS915X_SENSOR_TUNING_SIZE 12
/* Wire: split, beta, enabled, reserved=0, bottom_px_s LE32, top_px_s LE32. */
static inline uint32_t iqs915x_sensor_u32(const uint8_t *p) {
 return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;
}
static inline uint32_t iqs915x_sensor_cycle(uint32_t speed,uint16_t period) {
 return ((uint64_t)speed*period+500)/1000;
}
static inline bool iqs915x_sensor_valid(const uint8_t *p,uint16_t period) {
 if(!period||p[2]>1||p[3])return false;
 uint32_t b=iqs915x_sensor_u32(p+4),t=iqs915x_sensor_u32(p+8);
 if(b>=t||t>65535000U/period)return false;
 return iqs915x_sensor_cycle(b,period)<iqs915x_sensor_cycle(t,period)&&iqs915x_sensor_cycle(t,period)<=65535;
}
#endif
