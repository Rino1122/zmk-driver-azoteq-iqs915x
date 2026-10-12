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
 *                初期化データのALP Enable設定に従うLP2 Event Modeへ移行する
 * enabled=true:  RDYを待たずForce CommsでActive／Event Modeを設定・確認し、
 *                新しい入力セッションを開始する
 *
 * Active／LP2ともEvent Modeを使用し、直接モード切り替えへ進む。
 * ICが通信可能になるまでのクロックストレッチは発生し得る。
 * Active／Event Mode確認後も、座標の基準取得に2フレームを使う。
 *
 * 通常のActive／LP2通信はイベントによるRDYを待つ。
 * 有効化・無効化・Device PMのモード設定と読み戻し確認は、全段階で
 * RDYを待たずForce Commsを使う。LP2への遷移もEvent Mode確認後に完了する。
 * 各段階は1 transactionでSTOPにより通信窓を閉じる。
 * LP2ではTP Eventも無効にし、Active復帰時に再び有効にする。
 * 無効化要求時にX送信が始まっていた場合、そのフレームのY/syncまで送信する。
 * 以降のポインタフレームは破棄し、未同期のXを後続イベントに残さない。
 *
 * ホスト起動時（soft off復帰を含む）はSHOW_RESETを確認し、0なら保持済みの
 * init-dataを使い、1ならinit-dataを転送する。両経路ともTP Re-ATIを要求する。
 * ドライバからATIを要求するのは初期化時（ICリセット後の再初期化を含む）のみ。
 * 通常動作中はICの自動Re-ATIに任せ、Reseedや指スタック判定は行わない。
 * 自動Re-ATIは共通の有効ビットで、reference drift／negative delta／positive delta
 * の各条件が適用される。ATI targetと各閾値はprofileの値を保持する。
 * Negative DeltaによるRe-ATIは閾値超過の15サイクル連続判定で発動する。
 * Positive Deltaを使わない用途では、profileで発動しない閾値を設定する。
 * ATI Errorの再試行はICに任せる（retry timeは1秒）。
 *
 * 自動Re-ATI検出時は入力状態を解除し、座標の基準を取り直す。
 * Device PM suspend中はLP2移行後に通信を停止し、復帰時は保存した有効状態へ戻す。
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
