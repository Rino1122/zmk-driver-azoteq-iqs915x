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
 * enabled=false: 出力ゲートを即座に閉じ、専用スレッドで操作状態を解除して
 *                IQS915xをLP2へ遷移させる
 * enabled=true:  専用スレッドでIQS915xをActive modeへ戻し、Event Modeの
 *                再ラッチ完了後に新しい入力セッションを開始する
 *
 * runtimeのpower遷移ではTP Reseedは行わない。Manual Controlは
 * 初期化時に有効化され、Mode Selectのみを切り替える。
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

#endif /* ZMK_DRIVER_IQS915X_H_ */
