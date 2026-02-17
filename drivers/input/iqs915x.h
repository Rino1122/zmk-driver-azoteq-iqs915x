/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 *
 * IQS9150/IQS9151 トラックパッドICのレジスタ定義
 *
 * 参照: IQS9150_IQS9151_datasheet.pdf
 *   - Section 15: I2C Memory Map - Register Descriptions
 *   - Appendix A: レジスタビットフィールド詳細
 *
 * 注意事項:
 *   - IQS9150はリトルエンディアン（IQS5xxはビッグエンディアン）
 *   - I2Cデフォルトアドレス: 0x56
 *   - メモリアドレスは16bit幅
 *   - デフォルトではI2C STOPで通信ウィンドウが閉じるため、
 *     1回のRDY期間中に実行できるI2Cトランザクションは1つのみ。
 */

#ifndef IQS915X_H_
#define IQS915X_H_

#include <zephyr/device.h>

/* ============================================================
 * プロダクト情報レジスタ (Read-Only)
 * ============================================================ */

// プロダクト番号 (2 bytes)
// IQS9150: 0x076A, IQS9151: 0x09BC
#define IQS915X_PRODUCT_NUMBER 0x1000

/* ============================================================
 * データ出力レジスタ (Read-Only)
 * ============================================================ */

// 相対移動量 (各2 bytes, signed int16)
#define IQS915X_REL_X 0x1014
#define IQS915X_REL_Y 0x1016

// ジェスチャー座標 (各2 bytes)
// タップ位置 or ズームデルタ値
#define IQS915X_GESTURE_X 0x1018
#define IQS915X_GESTURE_Y 0x101A

// ジェスチャーイベントフラグ (各2 bytes, Read-Only)
#define IQS915X_SINGLE_FINGER_GESTURES 0x101C
#define IQS915X_TWO_FINGER_GESTURES 0x101E

// システムステータスフラグ (2 bytes)
#define IQS915X_INFO_FLAGS 0x1020

// トラックパッドステータスフラグ (2 bytes)
// 指の数、移動検知、各指の信頼度
#define IQS915X_TRACKPAD_FLAGS 0x1022

// 指1の座標データ (各2 bytes)
// 指Nのベースアドレス: 0x1024 + (N-1) * 8
#define IQS915X_FINGER1_X 0x1024
#define IQS915X_FINGER1_Y 0x1026
#define IQS915X_FINGER1_STRENGTH 0x1028
#define IQS915X_FINGER1_AREA 0x102A

// 指2〜7の座標データ（8バイト間隔で連続配置）
#define IQS915X_FINGER2_X 0x102C
#define IQS915X_FINGER3_X 0x1034
#define IQS915X_FINGER4_X 0x103C
#define IQS915X_FINGER5_X 0x1044
#define IQS915X_FINGER6_X 0x104C
#define IQS915X_FINGER7_X 0x1054

// 指Nの座標データを算出するためのマクロ
// finger_num: 1〜7
#define IQS915X_FINGER_X(finger_num) (0x1024 + ((finger_num) - 1) * 8)
#define IQS915X_FINGER_Y(finger_num) (0x1026 + ((finger_num) - 1) * 8)
#define IQS915X_FINGER_STRENGTH(finger_num) (0x1028 + ((finger_num) - 1) * 8)
#define IQS915X_FINGER_AREA(finger_num) (0x102A + ((finger_num) - 1) * 8)

// 1回の読み取りで指のデータサイズ (bytes)
#define IQS915X_FINGER_DATA_SIZE 8
// 最大タッチ数
#define IQS915X_MAX_FINGERS 7

/* ============================================================
 * Single Finger Gesture ビットフラグ
 * レジスタ: 0x101C (イベント), 0x11F6 (有効化設定)
 * ============================================================ */
#define IQS915X_SINGLE_TAP BIT(0)
#define IQS915X_DOUBLE_TAP BIT(1)
#define IQS915X_TRIPLE_TAP BIT(2)
#define IQS915X_PRESS_AND_HOLD BIT(3)
#define IQS915X_PALM_GESTURE BIT(4)
#define IQS915X_SWIPE_X_POS BIT(8)   // 右スワイプ
#define IQS915X_SWIPE_X_NEG BIT(9)   // 左スワイプ
#define IQS915X_SWIPE_Y_POS BIT(10)  // 下スワイプ
#define IQS915X_SWIPE_Y_NEG BIT(11)  // 上スワイプ
#define IQS915X_SWIPE_HOLD_X_POS BIT(12) // 右スワイプ＆ホールド
#define IQS915X_SWIPE_HOLD_X_NEG BIT(13) // 左スワイプ＆ホールド
#define IQS915X_SWIPE_HOLD_Y_POS BIT(14) // 下スワイプ＆ホールド
#define IQS915X_SWIPE_HOLD_Y_NEG BIT(15) // 上スワイプ＆ホールド

/* ============================================================
 * Two Finger Gesture ビットフラグ
 * レジスタ: 0x101E (イベント), 0x11F8 (有効化設定)
 * ============================================================ */
#define IQS915X_TWO_FINGER_TAP BIT(0)
#define IQS915X_TWO_FINGER_DOUBLE_TAP BIT(1)
#define IQS915X_TWO_FINGER_TRIPLE_TAP BIT(2)
#define IQS915X_TWO_FINGER_PRESS_AND_HOLD BIT(3)
#define IQS915X_ZOOM_IN BIT(4)
#define IQS915X_ZOOM_OUT BIT(5)
#define IQS915X_SCROLL_V BIT(6)  // 縦スクロール
#define IQS915X_SCROLL_H BIT(7)  // 横スクロール
// スクロール判定用の複合マスク
#define IQS915X_SCROLL (IQS915X_SCROLL_V | IQS915X_SCROLL_H)

/* ============================================================
 * Info Flags ビットフラグ (0x1020)
 * ============================================================ */
#define IQS915X_SHOW_RESET BIT(7)           // リセット発生の通知
#define IQS915X_ATI_ERROR BIT(3)            // ATIエラー
#define IQS915X_REATI_OCCURRED BIT(4)       // Re-ATI実行通知
#define IQS915X_ALP_PROX_TOGGLED BIT(12)    // ALP Proximity変化
#define IQS915X_TP_TOUCH_TOGGLED BIT(13)    // タッチ状態変化
#define IQS915X_SWITCH_TOGGLED BIT(14)      // スイッチ状態変化
#define IQS915X_SNAP_TOGGLED BIT(15)        // スナップ状態変化

/* ============================================================
 * Trackpad Flags ビットフラグ (0x1022)
 * ============================================================ */
// 下位バイト: 指の数と状態
#define IQS915X_NUM_FINGERS_MASK 0x000F     // Bit 0-3: アクティブな指の数(0-7)
#define IQS915X_TP_MOVEMENT BIT(4)          // 指の移動を検知
// 上位バイト: 各指の信頼度フラグ
#define IQS915X_FINGER1_CONFIDENCE BIT(8)
#define IQS915X_FINGER2_CONFIDENCE BIT(9)
#define IQS915X_FINGER3_CONFIDENCE BIT(10)
#define IQS915X_FINGER4_CONFIDENCE BIT(11)
#define IQS915X_FINGER5_CONFIDENCE BIT(12)
#define IQS915X_FINGER6_CONFIDENCE BIT(13)
#define IQS915X_FINGER7_CONFIDENCE BIT(14)

/* ============================================================
 * 設定レジスタ (Read-Write)
 * ============================================================ */

// System Control (2 bytes)
// 注意: ACK Resetは Bit 7 で行う
#define IQS915X_SYSTEM_CONTROL 0x11BC
// System Control ビットフラグ
#define IQS915X_MODE_SELECT_MASK 0x0007     // Bit 2-0: モード選択
#define IQS915X_MODE_ACTIVE 0x00            // アクティブモード
#define IQS915X_MODE_IDLE_TOUCH 0x01        // アイドルタッチモード
#define IQS915X_MODE_IDLE 0x02              // アイドルモード
#define IQS915X_MODE_LP1 0x03               // 低消費電力モード1
#define IQS915X_MODE_LP2 0x04               // 低消費電力モード2
#define IQS915X_REATI_TP BIT(5)             // トラックパッド再ATI
#define IQS915X_REATI_ALP BIT(6)            // ALP再ATI
#define IQS915X_ACK_RESET BIT(7)            // リセットフラグクリア
#define IQS915X_SW_RESET BIT(9)             // ソフトウェアリセット実行

// Config Settings (2 bytes)
#define IQS915X_CONFIG_SETTINGS 0x11BE
// Config Settings ビットフラグ
// 注意: TERMINATE_COMMS=0（デフォルト）ではI2C STOPで通信ウィンドウが閉じる。
// データシート推奨はI2C STOPでの自動終了のため、本ドライバでは
// TERMINATE_COMMSを設定せず、1トランザクション/RDYで動作する。
#define IQS915X_TERMINATE_COMMS BIT(6)      // 1: 手動終了（0xEEEE書込み必要）
#define IQS915X_EVENT_MODE BIT(8)           // 0: ストリーミング, 1: イベントモード
#define IQS915X_GESTURE_EVENT BIT(9)        // ジェスチャーイベント有効化

// Trackpad Settings (1 byte)
// XYのフリップ・スワップ設定
#define IQS915X_TRACKPAD_SETTINGS 0x11E2
// Trackpad Settings ビットフラグ
#define IQS915X_FLIP_X BIT(0)
#define IQS915X_FLIP_Y BIT(1)
#define IQS915X_SWITCH_XY_AXIS BIT(2)

// 最大同時タッチ数 (1 byte, 1-7)
#define IQS915X_MAX_MULTI_TOUCHES 0x11E5

// 解像度設定 (各2 bytes)
#define IQS915X_X_RESOLUTION 0x11E6
#define IQS915X_Y_RESOLUTION 0x11E8

// ジェスチャー有効化設定 (各2 bytes)
// ビットマスクはSingle/Two Finger Gestureのビットフラグと共通
#define IQS915X_SINGLE_FINGER_GESTURES_ENABLE 0x11F6
#define IQS915X_TWO_FINGER_GESTURES_ENABLE 0x11F8

// ジェスチャータイミング設定 (各2 bytes, ms単位)
#define IQS915X_TAP_TIME 0x11FA             // タップ判定時間
#define IQS915X_HOLD_TIME 0x1200            // プレス＆ホールド判定時間

// スクロール設定 (各2 bytes)
#define IQS915X_SCROLL_INITIAL_DIST 0x1212  // スクロール開始に必要な移動量
#define IQS915X_SCROLL_CONS_DIST 0x1214     // 連続スクロール判定の移動量

/* ============================================================
 * マウスボタンヘルパー
 * ============================================================ */
#define LEFT_BUTTON_BIT BIT(0)
#define RIGHT_BUTTON_BIT BIT(1)
#define MIDDLE_BUTTON_BIT BIT(2)
#define LEFT_BUTTON_CODE INPUT_BTN_0
#define RIGHT_BUTTON_CODE (INPUT_BTN_0 + 1)
#define MIDDLE_BUTTON_CODE (INPUT_BTN_0 + 2)

/* ============================================================
 * ステートマシン定義
 *
 * IQS9150はデフォルトでI2C STOPにより通信ウィンドウが閉じるため、
 * 1回のRDY信号に対して1回のI2Cトランザクションしか実行できない。
 * そのため、複数レジスタの読み書きは複数のRDYサイクルにわたって
 * ステートマシンで管理する。
 * ============================================================ */

// 初期化ステップ: iqs915x_setup中に1ステップずつ進行
enum iqs915x_init_step {
    INIT_ACK_RESET,                // リセットACK
    INIT_CONFIG_SETTINGS,          // イベントモード・ジェスチャーイベント設定
    INIT_SINGLE_FINGER_GESTURES,   // 1本指ジェスチャー有効化
    INIT_HOLD_TIME,                // プレス＆ホールド判定時間
    INIT_TWO_FINGER_GESTURES,      // 2本指ジェスチャー有効化
    INIT_TRACKPAD_SETTINGS,        // 軸設定（FlipX/Y, SwitchXY）
    INIT_COMPLETE,                 // 初期化完了
};

// 通常動作時のワークハンドラステート
enum iqs915x_work_state {
    WORK_READ_INFO_FLAGS,          // Info Flags読み取り（リセット検知）
    WORK_ACK_RESET,                // リセットACK書き込み
    WORK_READ_DATA,                // トラックパッドデータ一括読み取り
};

/* ============================================================
 * データ構造体
 * ============================================================ */

// デバイス設定（DTSから読み込まれる定数値）
struct iqs915x_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec reset_gpio;

    // ジェスチャー設定
    bool one_finger_tap;
    bool press_and_hold;
    bool two_finger_tap;
    uint16_t press_and_hold_time;

    // スクロール設定
    bool scroll;
    bool natural_scroll_x;
    bool natural_scroll_y;

    // 軸設定
    bool switch_xy;
    bool flip_x;
    bool flip_y;
};

// ランタイムデータ（実行時に変化する状態）
struct iqs915x_data {
    const struct device *dev;
    struct gpio_callback rdy_cb;
    struct k_work work;
    struct k_work_delayable button_release_work;

    // ステートマシン
    enum iqs915x_init_step init_step;    // 初期化進行状態
    enum iqs915x_work_state work_state;  // 通常動作ステート
    bool initialized;                    // 初期化完了フラグ

    // 前回読み取ったInfo Flags（リセット判定用、RDYをまたいで保持）
    uint16_t last_info_flags;

    // 直前のサイクルで押されたボタンのビットマスク
    uint8_t buttons_pressed;
    // プレス＆ホールドが有効かどうか
    bool active_hold;
    // スクロールアキュムレータ
    int16_t scroll_x_acc;
    int16_t scroll_y_acc;
};

#endif /* IQS915X_H_ */
