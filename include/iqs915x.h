/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVER_IQS915X_H_
#define ZMK_DRIVER_IQS915X_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#define IQS915X_SETTINGS_VERSION_1 1U
#define IQS915X_SETTINGS_VERSION_2 2U

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

/**
 * @brief トラックパッドの有効/無効を設定する
 *
 * enabled=false: 出力ゲートを即座に閉じ、専用スレッドで操作状態を解除して
 *                IQS915xをLP2へ遷移させる
 * enabled=true:  専用スレッドでIQS915xをActive modeへ戻し、Event Modeの
 *                再ラッチ完了後に新しい入力セッションを開始する
 *
 * Manual Controlは初期化時に有効化される。有効化時はActiveへ遷移する。
 * モード変更とEvent Mode再ラッチはForce Commsで行い、指イベントのRDYを
 * 待たない。ICの通信可能な時点まではクロックストレッチによる待ちが発生する。
 * LP2のサンプリング周期はドライバで150 msに固定する。
 * LP2中はTP_EVENTを無効化し、Active復帰時に再度有効化する。
 * 無効化後はLP2で10秒ごとに接触状態を確認し、無接触を確認できた場合だけ
 * 一時的にIdleへ移ってTP Reseedを行い、LP2へ戻る。Reseed中は入力出力を
 * 閉じたままにする。接触中は周期を越えて延期する。Device PM suspend中は
 * 定期Reseedを停止する。
 *
 * @param dev  IQS915xデバイスインスタンス
 * @param enabled  true=有効(Active), false=無効(LP2)
 * 本関数は要求を非同期に登録する。短時間に要求が反転した場合は最新の要求が
 * 優先され、古い入力セッションのイベントは破棄される。
 *
 * @return 0 on success, negative errno on failure
 */
int iqs915x_set_enabled(const struct device *dev, bool enabled);

/**
 * @brief トラックパッドの現在の有効/無効状態を取得する
 *
 * @param dev  IQS915xデバイスインスタンス
 * @return true=有効(Active要求中を含む), false=無効(LP2要求中を含む)
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

#endif /* ZMK_DRIVER_IQS915X_H_ */
