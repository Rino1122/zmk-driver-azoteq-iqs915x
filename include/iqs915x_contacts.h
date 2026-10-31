/* SPDX-License-Identifier: MIT */
#ifndef IQS915X_CONTACTS_H_
#define IQS915X_CONTACTS_H_
#include <stdbool.h>
#include <stdint.h>
#define IQS915X_MAX_CONTACTS 4
#define IQS915X_FRAME_SESSION_END 1U
#define IQS915X_FRAME_TRACKING_LOST 2U
/* Coordinates are validated, driver-corrected sensor units, not HID units.
 * count includes final tip=false records. slot is the physical sensor slot;
 * id is stable during contact and changes on reuse within a generation. */
struct iqs915x_contact { uint8_t slot, id; uint16_t x, y; bool tip, confidence; };
struct iqs915x_contact_frame {
  uint64_t sample_time_us; /* MCU read-completion time; not sensor hardware time. */
  uint32_t sequence, generation;
  uint8_t count, flags;
  struct iqs915x_contact contacts[IQS915X_MAX_CONTACTS];
};
#endif
