/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVER_IQS915X_H_
#define ZMK_DRIVER_IQS915X_H_

#include <zephyr/device.h>
#include <stdbool.h>

/**
 * @brief トラックパッドの有効/無効を設定する
 *
 * enabled=false: イベント破棄を即座に開始し、IQS915xをLP2へ遷移させる
 * enabled=true:  イベント処理を再開し、IQS915xをActive modeへ戻す
 *
 * runtimeのpower遷移ではTP Reseedは行わない。Manual Controlは
 * 初期化時に有効化され、Mode Selectのみを切り替える。
 *
 * @param dev  IQS915xデバイスインスタンス
 * @param enabled  true=有効(Active), false=無効(LP2)
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

#endif /* ZMK_DRIVER_IQS915X_H_ */
