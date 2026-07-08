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

#ifndef IQS915X_REGS_H_
#define IQS915X_REGS_H_

#include <zephyr/device.h>
#include <stdint.h>

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
#define IQS915X_SWIPE_X_POS BIT(8)       // 右スワイプ
#define IQS915X_SWIPE_X_NEG BIT(9)       // 左スワイプ
#define IQS915X_SWIPE_Y_POS BIT(10)      // 下スワイプ
#define IQS915X_SWIPE_Y_NEG BIT(11)      // 上スワイプ
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
#define IQS915X_SCROLL_V BIT(6) // 縦スクロール
#define IQS915X_SCROLL_H BIT(7) // 横スクロール
// スクロール判定用の複合マスク
#define IQS915X_SCROLL (IQS915X_SCROLL_V | IQS915X_SCROLL_H)

/* ============================================================
 * Info Flags ビットフラグ (0x1020)
 * データシート Section 15: INFO_FLAGS レジスタ定義
 * ============================================================ */
// Bit 2-0: 現在の動作モード
#define IQS915X_CHARGING_MODE_MASK 0x0007
#define IQS915X_MODE_ACTIVE 0x0000     // Activeモード
#define IQS915X_MODE_IDLE_TOUCH 0x0001 // Idle-Touchモード
#define IQS915X_MODE_IDLE 0x0002       // Idleモード
#define IQS915X_MODE_LP1 0x0003        // LP1モード
#define IQS915X_MODE_LP2 0x0004        // LP2モード
// Bit 3: ATI Error - トラックパッドATIエラー
#define IQS915X_ATI_ERROR BIT(3)
// Bit 4: Re-ATI Occurred - トラックパッドRe-ATI完了
#define IQS915X_REATI_OCCURRED BIT(4)
// Bit 5: ALP ATI Error - ALPチャネルATIエラー
#define IQS915X_ALP_ATI_ERROR BIT(5)
// Bit 6: ALP Re-ATI Occurred - ALPチャネルRe-ATI完了
#define IQS915X_ALP_REATI_OCCURRED BIT(6)
// Bit 7: Show Reset - リセット発生通知（ACK Resetで解除）
#define IQS915X_SHOW_RESET BIT(7)
// Bit 8: ALP Prox Status - ALPチャネルのProx/Touch検出状態
#define IQS915X_ALP_PROX_STATUS BIT(8)
// Bit 9: Global TP Touch - いずれかのTPチャネルでタッチ検出中
#define IQS915X_GLOBAL_TP_TOUCH BIT(9)
// Bit 10: Switch Pressed - スイッチ押下検出中
#define IQS915X_SWITCH_PRESSED BIT(10)
// Bit 11: Global Snap - いずれかのSnapチャネルでスナップ検出中
#define IQS915X_GLOBAL_SNAP BIT(11)
// Bit 12: ALP Prox Toggled - ALPチャネルProx状態変化
#define IQS915X_ALP_PROX_TOGGLED BIT(12)
// Bit 13: TP Touch Toggled - TPチャネルごとのタッチ状態変化
// ドライバの挙動判定には使わず、レジスタ定義としてのみ保持する。
#define IQS915X_TP_TOUCH_TOGGLED BIT(13)
// Bit 14: Switch Toggled - スイッチ状態変化
#define IQS915X_SWITCH_TOGGLED BIT(14)
// Bit 15: Snap Toggled - Snapチャネル状態変化
#define IQS915X_SNAP_TOGGLED BIT(15)

/* ============================================================
 * Trackpad Flags ビットフラグ (0x1022)
 * ============================================================ */
// 下位バイト: 指の数と状態
#define IQS915X_NUM_FINGERS_MASK 0x000F // Bit 0-3: アクティブな指の数(0-7)
#define IQS915X_TP_MOVEMENT BIT(4)      // 指の移動を検知
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
#define IQS915X_MODE_SELECT_MASK 0x0007 // Bit 2-0: モード選択
#define IQS915X_MODE_ACTIVE 0x00        // アクティブモード
#define IQS915X_MODE_IDLE_TOUCH 0x01    // アイドルタッチモード
#define IQS915X_MODE_IDLE 0x02          // アイドルモード
#define IQS915X_MODE_LP1 0x03           // 低消費電力モード1
#define IQS915X_MODE_LP2 0x04           // 低消費電力モード2
#define IQS915X_TP_RESEED BIT(3)        // トラックパッドリシード
#define IQS915X_REATI_TP BIT(5)         // トラックパッド再ATI
#define IQS915X_REATI_ALP BIT(6)        // ALP再ATI
#define IQS915X_ACK_RESET BIT(7)        // リセットフラグクリア
#define IQS915X_SW_RESET BIT(9)         // ソフトウェアリセット実行
#define IQS915X_SUSPEND BIT(11)         // Suspend（処理停止、<3µA）

// Config Settings (2 bytes)
#define IQS915X_CONFIG_SETTINGS 0x11BE
// Config Settings ビットフラグ
// 注意: TERMINATE_COMMS=0（デフォルト）ではI2C STOPで通信ウィンドウが閉じる。
// データシート推奨はI2C STOPでの自動終了のため、本ドライバでは
// TERMINATE_COMMSを設定せず、1トランザクション/RDYで動作する。
#define IQS915X_FORCE_COMMS_METHOD BIT(4) // 0: クロックストレッチ, 1: コマンド要求方式
#define IQS915X_TERMINATE_COMMS BIT(6)    // 1: 手動終了（0xEEEE書込み必要）
#define IQS915X_MANUAL_CONTROL BIT(7)     // 手動モード制御
#define IQS915X_EVENT_MODE BIT(8)         // 0: ストリーミング, 1: イベントモード
#define IQS915X_GESTURE_EVENT BIT(9)   // ジェスチャーイベント有効化
#define IQS915X_TP_EVENT BIT(10)       // 指移動および指up/downイベント有効化
#define IQS915X_TP_TOUCH_EVENT BIT(13) // diamond pattern各チャネル状態変化イベント

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
#define IQS915X_TAP_TOUCH_TIME 0x11FA // タップ接触上限時間
#define IQS915X_TAP_WAIT_TIME 0x11FC  // 連続タップ待ち時間（air time）
#define IQS915X_TAP_DISTANCE 0x11FE   // タップ許容移動量
#define IQS915X_HOLD_TIME 0x1200      // プレス＆ホールド判定時間

// スクロール設定 (各2 bytes)
#define IQS915X_SCROLL_INITIAL_DIST 0x1212 // スクロール開始に必要な移動量
#define IQS915X_SCROLL_CONS_DIST 0x1214    // 連続スクロール判定の移動量

// サンプリング周期レジスタ (各2 bytes, ms単位)
// データシート Section 6.1 参照
#define IQS915X_ACTIVE_MODE_REPORT_RATE 0x11A2 // Active Mode (デフォルト: ~10ms)
#define IQS915X_IDLE_TOUCH_REPORT_RATE 0x11A4  // Idle-Touch Mode
#define IQS915X_IDLE_MODE_REPORT_RATE 0x11A6   // Idle Mode
#define IQS915X_LP1_MODE_REPORT_RATE 0x11A8    // LP1 Mode
#define IQS915X_LP2_MODE_REPORT_RATE 0x11AA    // LP2 Mode

/* ============================================================
 * 初期化データ（init-data）メモリマップ定数
 *
 * IQS9150/IQS9151はNVMを搭載しないため、起動時に全設定レジスタを
 * I2Cで書き込む必要がある。init-dataはメモリマップ上の2つの連続領域
 * （メイン領域 + エンジニアリング領域）から構成される。
 * ============================================================ */

// メイン領域: 0x115C〜0x15EB（ALP補償〜仮想ホイール）
#define IQS915X_INIT_DATA_BASE_ADDR 0x115C
#define IQS915X_INIT_DATA_MAIN_SIZE 1168 // 0x15EB - 0x115C + 1 = 0x0490

// エンジニアリング領域: 0x2000〜0x2005
#define IQS915X_INIT_DATA_ENG_ADDR 0x2000
#define IQS915X_INIT_DATA_ENG_SIZE 6

// init-dataの合計サイズ（バイト）
#define IQS915X_INIT_DATA_TOTAL_SIZE (IQS915X_INIT_DATA_MAIN_SIZE + IQS915X_INIT_DATA_ENG_SIZE)

// 1回のRDYサイクルで書き込むチャンクサイズ（バイト）
// nRF52のTWIM EasyDMAバッファ上限（255バイト）以内で、
// 2バイトのレジスタアドレスを考慮した保守的な値
#define IQS915X_INIT_WRITE_CHUNK_SIZE 128

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
enum iqs915x_init_step
{
    INIT_CHECK_SHOW_RESET,    // SHOW_RESETフラグ確認
    INIT_SOFTWARE_RESET,      // ソフトウェアリセット発行
    INIT_WAIT_SOFTWARE_RESET, // ソフトウェアリセット完了待機
    INIT_WRITE_INIT_DATA,     // init-dataブロック書き込み（複数RDYサイクル、DTS設定を事前適用済み）
    INIT_VERIFY_INIT_CHUNK,   // 書き込み失敗チャンクのread-back確認
    INIT_ACK_RESET,           // リセットACK
    INIT_VERIFY_SHOW_RESET_CLEAR, // ACK_RESET後のSHOW_RESETクリア確認
    INIT_REQUEST_REATI,       // TP/ALP Re-ATIリクエスト
    INIT_WAIT_REATI,          // TP Re-ATI完了待機
    INIT_PREPARE_EVENT_MODE,  // CONFIG_SETTINGS読み取りとEvent Mode書き込み値準備
    INIT_SET_EVENT_MODE,      // Event Mode + Manual Control明示設定
    INIT_CONFIRM_EVENT_MODE,  // Event Mode設定後の再確認
    INIT_COMPLETE,            // 初期化完了
    INIT_FAILED,              // 初期化失敗（上限超過）
};

// 通常動作時のワークハンドラステート
enum iqs915x_work_state
{
    WORK_READ_INFO_FLAGS, // Info Flags読み取り（リセット検知）
    WORK_ACK_RESET,       // リセットACK書き込み
    WORK_READ_DATA,       // トラックパッドデータ一括読み取り
    WORK_RELATCH_EVENT_MODE_DISABLE, // Event Mode再ラッチ: EVENT_MODE clear
    WORK_RELATCH_EVENT_MODE_ENABLE,  // Event Mode再ラッチ: EVENT_MODE set
};

#define IQS915X_INERTIA_MOTION_HISTORY_SIZE 12

enum iqs915x_two_finger_mode
{
    IQS915X_2F_MODE_NONE = 0,
    IQS915X_2F_MODE_SCROLL,
    IQS915X_2F_MODE_PINCH,
};

struct iqs915x_inertia_profile
{
    bool enabled;
    uint16_t interval_ms;
    uint16_t decay_x1000;
    uint16_t recent_window_ms;
    uint16_t stale_gap_ms;
    uint8_t min_samples;
    uint16_t min_avg_speed;
};

struct iqs915x_scroll_inertia_profile
{
    bool enabled;
    uint16_t trigger_ms;
    uint16_t decay_factor_int;
    uint16_t interval_ms;
    uint16_t threshold_start;
    uint16_t threshold_stop;
};

struct iqs915x_motion_sample
{
    int64_t ms;
    int16_t x;
    int16_t y;
};

struct iqs915x_motion_history
{
    struct iqs915x_motion_sample samples[IQS915X_INERTIA_MOTION_HISTORY_SIZE];
    uint8_t head;
    uint8_t count;
};

struct iqs915x_inertia_state
{
    bool active;
    int32_t velocity_x_fp;
    int32_t velocity_y_fp;
    int32_t accum_x_fp;
    int32_t accum_y_fp;
    int64_t last_ms;
    uint32_t elapsed_ms;
};

struct iqs915x_scroll_inertia_state
{
    int16_t vx;
    int16_t vy;
    int16_t ema_vx;
    int16_t ema_vy;
    int16_t remainder_x_q8;
    int16_t remainder_y_q8;
    uint8_t zero_output_ticks;
    bool active;
    bool is_inertial;
};

struct iqs915x_finger_tracker
{
    uint8_t current_count;
    uint8_t previous_count;
    uint8_t stable_count;
    uint8_t sequence_max_count;
    bool tail_suppressed;
    bool awaiting_zero_contact;
    bool sequence_active;
    bool sequence_seen_one;
    bool sequence_seen_two;
    bool completed_one_tap_path;
    bool completed_two_tap_path;
};

struct iqs915x_two_finger_session
{
    bool active;
    bool rebaseline_pending;
    enum iqs915x_two_finger_mode mode;
    int32_t centroid_dx;
    int32_t centroid_dy;
    int32_t distance_delta;
    int32_t centroid_last_x;
    int32_t centroid_last_y;
    int32_t centroid_start_x;
    int32_t centroid_start_y;
    uint32_t max_centroid_movement;
    int32_t distance_last;
    int32_t pinch_wheel_remainder;
};

/* ============================================================
 * データ構造体
 * ============================================================ */

// デバイス設定（DTS + ドライバ内固定値）
struct iqs915x_config
{
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec reset_gpio;

    // 初期化データ（NVM非搭載デバイス用）
    const uint8_t *init_data; // ドライバ内蔵バイト配列
    uint16_t init_data_len;   // バイト配列の長さ（IQS915X_INIT_DATA_TOTAL_SIZE固定）

    // ジェスチャー設定
    bool one_finger_tap;
    bool tap_and_hold;
    bool two_finger_tap;
    uint16_t tap_and_hold_release_timeout_ms;

    // スクロール設定
    bool scroll;
    uint16_t scroll_divisor; // スクロール感度除数

    // ポインタ加速度設定
    bool pointer_accel;
    uint16_t pointer_sensitivity_percent;
    uint16_t pointer_accel_threshold;
    uint16_t pointer_accel_saturation;
    uint16_t pointer_accel_max_percent;

    struct iqs915x_scroll_inertia_profile scroll_inertia;
    struct iqs915x_inertia_profile pinch_inertia;

    // 3/4本指スワイプ設定（専用gesture input event）
    bool three_finger_swipe;
    bool four_finger_swipe;
    uint16_t swipe_step;
    uint16_t swipe_threshold_numerator;
    uint16_t swipe_threshold_denominator;
    uint16_t swipe_direction_settle_frames;
    uint16_t swipe_direction_lock_numerator;
    uint16_t swipe_direction_lock_denominator;

    // タイミング設定
    uint16_t report_rate_ms; // Active Modeサンプリング周期(ms), 0=NVMデフォルト

    // 座標補正
    bool coordinate_correction; // true=絶対座標にブロック内LUT補正を適用

    // 軸設定
    bool switch_xy;
    bool flip_x;
    bool flip_y;

    // Power mode制御
    bool disabled_by_default; // 起動時にトラックパッドを無効（初期化完了後にLP2）状態で開始
};

// ランタイムデータ（実行時に変化する状態）
struct iqs915x_data
{
    const struct device *dev;
    struct gpio_callback rdy_cb;

    // 専用スレッド用
    struct k_sem rdy_sem;
    struct k_thread thread;
    K_KERNEL_STACK_MEMBER(thread_stack, 2048);

    struct k_work_delayable button_release_work;
    struct k_work_delayable tap_and_hold_release_work;
    struct k_work_delayable single_tap_work;
    struct k_work_delayable tap_and_hold_start_work;

    // ステートマシン
    enum iqs915x_init_step init_step;   // 初期化進行状態
    enum iqs915x_work_state work_state; // 通常動作ステート
    bool initialized;                   // 初期化完了フラグ
    uint16_t init_data_offset;          // init-data書き込み進捗（バイトオフセット）
    uint8_t wait_count;                 // SW ResetやRe-ATIなどの待機リトライカウンタ
    uint8_t init_chunk_retry_count;      // 現在のinit-dataチャンク書き込みリトライ回数
    uint8_t init_restart_count;          // 初期化リカバリのためのsoftware reset回数
    uint16_t init_pending_cfg;           // Event Mode強制設定で次RDYに持ち越すCONFIG_SETTINGS値
    uint16_t confirmed_config_settings;  // 初期化時にread-back確認したCONFIG_SETTINGS値
    uint8_t event_mode_relatch_retry_count; // EVENT_MODE再有効化確認リトライ回数

    // 前回読み取ったInfo Flags（リセット判定用、RDYをまたいで保持）
    uint16_t last_info_flags;

    // 直前のサイクルで押されたボタンのビットマスク
    uint8_t buttons_pressed;
    // Tap-and-Holdでドラッグ状態かどうか
    bool active_tap_hold;
    bool tap_and_hold_release_pending;
    bool single_tap_pending;
    bool tap_sequence_second_touch;
    bool tap_and_hold_start_pending;
    int64_t pending_tap_up_time;

    // 生のタッチ状態トラッキング（Rapid Tap-and-Drag判定用）
    bool is_touching;
    int64_t last_touch_down_time;
    uint8_t tap_drag_raw_max_fingers;
    bool tap_drag_raw_gesture_seen;
    bool raw_single_tap_reported;
    bool raw_two_finger_tap_reported;
    bool tap_start_valid;
    int32_t tap_start_x;
    int32_t tap_start_y;
    uint32_t tap_max_movement;
    uint32_t completed_two_finger_movement;
    uint16_t tap_touch_time_ms;
    uint16_t tap_air_time_ms;
    uint16_t tap_distance;
    uint16_t last_abs_x; // 直前に報告したabsolute X座標
    uint16_t last_abs_y; // 直前に報告したabsolute Y座標
    bool last_abs_valid; // absolute座標の直前報告値が有効か
    int32_t pointer_x_acc; // ポインタ倍率適用時のX剰余（percent単位）
    int32_t pointer_y_acc; // ポインタ倍率適用時のY剰余（percent単位）

    // スクロールアキュムレータ
    int32_t scroll_x_acc;
    int32_t scroll_y_acc;

    struct iqs915x_finger_tracker finger_tracker;
    struct iqs915x_two_finger_session two_finger;
    struct iqs915x_motion_history scroll_motion_history;
    struct iqs915x_motion_history pinch_motion_history;
    struct iqs915x_scroll_inertia_state scroll_inertia_state;
    struct iqs915x_inertia_state pinch_inertia_state;

    // 3/4本指スワイプ（重心追跡）
    int32_t swipe_last_centroid_x;
    int32_t swipe_last_centroid_y;
    bool swipe_centroid_valid;
    uint8_t swipe_active_fingers;
    uint16_t swipe_valid_frames;
    bool swipe_triggered;
    bool multifinger_swipe_latched;
    bool scroll_sequence_active;
    bool scroll_blocked_until_low_contact;
    uint16_t swipe_threshold_x;
    uint16_t swipe_threshold_y;
    uint16_t swipe_resolution_x;
    uint16_t swipe_resolution_y;

    // スクロール慣性用
    struct k_work_delayable scroll_inertia_work; // 慣性スクロールタイマー
    uint8_t gesture_pointer_suppress_ticks;      // gesture終了後のポインタ抑止残りtick数

    // Power mode制御
    bool enabled;        // トラックパッド有効フラグ（falseでイベント破棄）
    bool lp2_pending;    // IQS915xへのLP2遷移待ち
    bool active_pending; // IQS915xのActive mode復帰待ち
};

#endif /* IQS915X_REGS_H_ */
