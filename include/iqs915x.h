/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVER_IQS915X_H_
#define ZMK_DRIVER_IQS915X_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>
#include <iqs915x_contacts.h>
#include <iqs915x_processing.h>


/** Single synchronous subscriber per sensor. Called from the sensor thread,
 * outside I2C communication, under the subscription mutex. Must not block,
 * perform I2C, retain frame pointers or call driver APIs: copy/queue the frame.
 * SESSION_END closes the supplied generation even when count=0. Frames from
 * older generations must be discarded by consumers after an end marker.
 * Switch to MOUSE and wait for the power/request boundary before NULL detach
 * (-EBUSY until then); success guarantees the old callback has finished.
 * Thread context only; -EBUSY means retry after an in-flight callback finishes.
 * Replacing a live subscriber requires unsubscribe first (-EALREADY otherwise).
 */
typedef void (*iqs915x_contact_callback_t)(const struct device *dev,
    const struct iqs915x_contact_frame *frame, void *user_data);
enum iqs915x_output_mode { IQS915X_OUTPUT_MOUSE, IQS915X_OUTPUT_CONTACTS };
int iqs915x_subscribe_contacts(const struct device *dev,
    iqs915x_contact_callback_t callback, void *user_data);
/** Asynchronous request, independent of power selection. Default MOUSE.
 * Closes output immediately, ends the old session in the sensor thread and
 * restarts only after all fingers lift. CONTACTS requires a subscriber.
 * Existing delayed mouse work is canceled before the new output starts.
 * Returns -EBUSY during PM/request contention and -ENODEV on a terminal device.
 * Does not configure a faster scan period (rate tuning is a separate stage).
 */
/* Re-requesting CONTACTS explicitly starts a fresh tracking session and waits
 * for all-up. Ordinary power-mode changes preserve an existing barrier but do
 * not add one; resuming CONTACTS can report fingers already on the surface. */
int iqs915x_set_output_mode(const struct device *dev, enum iqs915x_output_mode mode);
int iqs915x_get_output_mode(const struct device *dev, enum iqs915x_output_mode *mode);
/** Cached logical coordinate maxima after initialization; never performs I2C. */
int iqs915x_get_contact_resolution(const struct device *dev, uint16_t *max_x, uint16_t *max_y);

/** Raw-contact refresh policy; true is the backward-compatible default.
 * false retains event waiting, including while stationary. No I2C performed. */
struct iqs915x_contact_statistics { uint32_t forced_reads, event_reads; };
int iqs915x_get_contact_statistics(const struct device *dev, struct iqs915x_contact_statistics *out);
int iqs915x_set_contact_periodic(const struct device *dev, bool periodic);
struct iqs915x_gesture_profile {
  uint16_t touch_ms, air_ms, distance, max_x, max_y, drag_release_ms;
  uint16_t scroll_divisor, trigger_ms, interval_ms, stop_threshold;
  uint16_t swipe_numerator, swipe_denominator, swipe_settle, lock_numerator, lock_denominator;
  bool one_tap, two_tap, drag, swipe3, swipe4;
};
/** Immutable DTS/init-data policy, available even when sensor is absent. */
int iqs915x_get_gesture_profile(const struct device *dev, struct iqs915x_gesture_profile *out);

#define IQS915X_SETTINGS_VERSION_1 1U
#define IQS915X_SETTINGS_VERSION_2 2U

/** Lifecycle is independent of Zephyr device readiness. ABSENT/FAULT latch until MCU boot. */
enum iqs915x_lifecycle { IQS915X_PROBING, IQS915X_INITIALIZING, IQS915X_READY,
                       IQS915X_RECOVERING, IQS915X_ABSENT, IQS915X_FAULT };
enum iqs915x_mode { IQS915X_POWER_ACTIVE, IQS915X_POWER_LP2, IQS915X_POWER_SUSPEND };
struct iqs915x_status {
  enum iqs915x_lifecycle lifecycle;
  enum iqs915x_mode requested_mode;
  enum iqs915x_mode actual_mode; /* Applied driver policy. Active permits automatic IC modes. */
  bool transitioning;
  uint32_t ati_sequence; /* Incremented once per fresh TP Re-ATI notification. */
  bool ati_error; /* Cached TP calibration health; independent of power/lifecycle. */
  int last_error;
  uint32_t generation;
};
/** Nonblocking mode request; does not retry a terminal device. Suspend is the
 * IC's data-retaining Suspend bit, not LP2. Terminal devices record the
 * requested mode for UI indication only and never perform hardware I/O. It closes output immediately.
 * Waking from Suspend triggers the IC's automatic reseed. Device PM uses
 * Suspend and restores the previous requested mode on resume.
 * Active selects Active in Manual Control, then restores automatic IC mode
 * switching. LP2 enables Manual Control and holds the IC in LP2. */
int iqs915x_set_mode(const struct device *dev, enum iqs915x_mode mode);
/** Cached MCU state only: never initiates I2C or wakes a suspended sensor. */
int iqs915x_get_status(const struct device *dev, struct iqs915x_status *status);

/** Lifetime subscribers, invoked on the sensor thread outside driver locks.
 * Callback must only queue work: do not block or call driver mutating APIs.
 * Registration schedules an initial notification; no hardware I/O is added.
 * Two independent consumers are supported; repeating a registration is harmless. */
typedef void (*iqs915x_status_callback_t)(const struct device *dev, void *user_data);
int iqs915x_subscribe_status(const struct device *dev, iqs915x_status_callback_t callback,
                            void *user_data);

/** Runtime pointer acceleration controls. */
struct iqs915x_pointer_settings {
  bool enabled;
  uint16_t sensitivity_percent;
  uint16_t threshold;
  uint16_t saturation;
  uint16_t max_percent;
};

/** Runtime scroll inertia controls. Times are in milliseconds.
 * threshold_start is the release-time 100 ms average motion in coordinate
 * units per 10 ms; threshold_stop applies to decayed velocity in the same units.
 * initial_velocity_percent (v2) multiplies average release velocity: 100–1000.
 */
struct iqs915x_scroll_inertia_settings {
  bool enabled;
  uint16_t trigger_ms;
  uint16_t decay_factor_percent;
  uint16_t interval_ms;
  uint16_t threshold_start;
  uint16_t threshold_stop;
  uint16_t max_duration_ms; /* 0 preserves the legacy unlimited duration. */
  uint16_t initial_velocity_percent;
};

/** Versioned settings controlled by the Harbour trackpad settings UI. */
struct iqs915x_settings {
  uint16_t version;
  struct iqs915x_pointer_settings pointer;
  struct iqs915x_scroll_inertia_settings scroll_inertia;
};

struct iqs915x_setting_range {
  uint16_t min;
  uint16_t max;
};

/** Supported numeric ranges for version 2 settings. */
struct iqs915x_settings_limits {
  uint16_t version;
  struct iqs915x_setting_range pointer_sensitivity_percent;
  struct iqs915x_setting_range pointer_threshold;
  struct iqs915x_setting_range pointer_saturation;
  struct iqs915x_setting_range pointer_max_percent;
  struct iqs915x_setting_range inertia_trigger_ms;
  struct iqs915x_setting_range inertia_decay_factor_percent;
  struct iqs915x_setting_range inertia_interval_ms;
  struct iqs915x_setting_range inertia_threshold_start;
  struct iqs915x_setting_range inertia_threshold_stop;
  struct iqs915x_setting_range inertia_max_duration_ms;
  struct iqs915x_setting_range inertia_initial_velocity_percent;
};

/** Convenience request: true enters Active then enables automatic IC mode
 * switching, false holds the IC in LP2 using Manual Control.
 * Use set_mode(SUSPEND) for an unselected sensor. Returns -ENODEV after a
 * latched absence/fault and -EBUSY during Device PM. Requests are asynchronous;
 * get_status distinguishes acceptance from actual completion. Active/LP2
 * transitions use Force Comms and retain the two-frame pointer baseline guard.
 * Device PM always enters IC Suspend and restores the previous mode on resume.
 */
int iqs915x_set_enabled(const struct device *dev, bool enabled);

/**
 * @brief トラックパッドの現在の有効/無効状態を取得する
 *
 * @param dev  IQS915xデバイスインスタンス
 * @return true=有効(Active要求中を含む), false=無効(LP2/Suspend要求中を含む)
 */
bool iqs915x_get_enabled(const struct device *dev);

/**
 * @brief Get the currently applied runtime settings.
 *
 * Settings can be read while the trackpad is disabled. Returns -EAGAIN before
 * initialization completes or while the device is recovering from a reset,
 * -EBUSY while Device PM has suspended the device, and -ENODEV for an invalid
 * or unready device.
 */
int iqs915x_get_settings(const struct device *dev,
                         struct iqs915x_settings *settings);

/**
 * Get supported version 2 setting ranges. For max_duration_ms, zero is also a
 * special accepted value meaning no duration limit; otherwise the range is
 * 50–5000 ms.
 */
int iqs915x_get_settings_limits(struct iqs915x_settings_limits *limits);

/** Validate settings without changing device state. Version 1 is accepted and
 * ignores initial_velocity_percent, using 100%. Reads always return version 2.
 */
int iqs915x_validate_settings(const struct iqs915x_settings *settings);

/**
 * @brief Atomically apply a complete runtime settings object.
 *
 * The call is synchronous. On success, all subsequent pointer and scroll
 * processing uses the new values. Existing touch/button/gesture state is
 * preserved, while pointer remainders and active scroll inertia are cleared.
 * Returns -EINVAL for unsupported versions or invalid settings, -EAGAIN before
 * initialization completes or during reset recovery, -EBUSY during PM suspend,
 * and -ENODEV for an invalid or unready device.
 */
int iqs915x_apply_settings(const struct device *dev,
                           const struct iqs915x_settings *settings);


#include <iqs915x_channel_capture.h>
/* ATI Compensation (10-bit values), explicit nonblocking capture request. Requires READY, requested/actual Active.
 * The sensor thread closes input while capturing; layer/power changes cancel it.
 * No automatic/periodic capture is performed. IDs fence stale paged reads. */
struct iqs915x_channel_page {
  uint32_t id, started_ms, ended_ms;
  int status;
    int ati_status, restore_status;
  uint16_t info_before, info_after, config_before;
  uint16_t offset, count, values[IQS915X_CHANNEL_PAGE];
  uint8_t disabled_mask;
  bool mapping_valid;
};
int iqs915x_diagnostic_start(const struct device *dev, bool run_ati, uint32_t *id);
int iqs915x_channels_start(const struct device *dev, uint32_t *id);
int iqs915x_channels_page(const struct device *dev, uint32_t id, unsigned offset,
                           struct iqs915x_channel_page *out);

/* Cached terminal-failure evidence, captured once before best-effort Suspend.
 * valid_mask bits 0/1/2 mark meaningful Info/Config/Control values; busy 0xEEEE
 * is retained in info but must never be interpreted as ordinary flags. */
struct iqs915x_fault_snapshot {
  int error, stop_status;
  uint32_t captured_ms, wait_count;
  uint16_t info, config, control;
  int read_status[3];
  uint8_t valid_mask, init_step, restart_count, lifecycle;
  bool busy, rdy_active;
};
int iqs915x_fault_get(const struct device *dev, struct iqs915x_fault_snapshot *out);
/* First initial Re-ATI timeout, captured before reset; cached until MCU reboot. */
int iqs915x_ati_timeout_get(const struct device *dev, struct iqs915x_fault_snapshot *out);

/* First CONFIG_SETTINGS mismatch per MCU boot, retained across IC retries and
 * terminal failure. raw[] is one 16-byte read from 0x11BC, not live registers. */
struct iqs915x_mismatch_snapshot {
  struct iqs915x_fault_snapshot evidence;
  uint16_t expected, observed;
  int raw_status;
  uint8_t raw[16];
};
int iqs915x_mismatch_get(const struct device *dev, struct iqs915x_mismatch_snapshot *out);

#endif /* ZMK_DRIVER_IQS915X_H_ */
