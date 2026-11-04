/* SPDX-License-Identifier: MIT */
#pragma once
#include <stdint.h>
#define IQS915X_CAL_BLOB_SIZE 900U
#define IQS915X_CAL_PACKET_SIZE 96U
/* Commands: INFO=0, START=1, KEEPALIVE=2, END=3, READ=4,
 * UPLOAD_BEGIN=5, UPLOAD_CHUNK=6, APPLY=7, CLEAR=8.
 * All multibyte fields little endian. Packet is op:u8, token:u32,
 * offset:u32, data:bytes. Response layouts are documented by firmware RPC. */
struct device;
int iqs915x_calibration_command(const struct device *dev, const uint8_t *request,
    unsigned length, uint8_t *reply, unsigned *reply_length);
