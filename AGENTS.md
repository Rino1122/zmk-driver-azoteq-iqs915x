# AGENTS.md

このリポジトリは ZMK/Zephyr 用の Azoteq IQS9150/IQS9151 トラックパッド入力ドライバです。主実装は `azoteq,iqs915x` compatible の `drivers/input/iqs915x.c` です。

## 現状の実装状況

- Zephyr モジュールとしての組み込みは実装済みです。
  - `zephyr/module.yml` で CMake/Kconfig/DTS root を公開します。
  - `CMakeLists.txt` は `include/` と `drivers/` を Zephyr に追加します。
  - `drivers/input/Kconfig` は `CONFIG_INPUT_AZOTEQ_IQS915X` を定義します。
- DTS binding は `dts/bindings/input/azoteq,iqs915x-i2c.yaml` と `azoteq,iqs915x-common.yaml` にあります。
  - 必須: `rdy-gpios`
  - 任意: `reset-gpios`
  - 機能フラグ: `one-finger-tap`, `press-and-hold`, `two-finger-tap`, `scroll`, `scroll-inertia`, `three-finger-swipe`, `four-finger-swipe`, `report-absolute`, `disabled-by-default`
  - 調整値: スクロール慣性、スワイプ閾値、タップ/ホールド時間、report rate、軸入れ替え/反転など
- IQS915x 向け初期化データはビルトイン方式です。
  - 入力: `drivers/input/IQS9150_init.h`
  - 変換: `scripts/convert_init_header.py`
  - 生成物: build directory 配下の `generated/iqs915x_init_data_bretagne_array.h`
  - `drivers/input/CMakeLists.txt` がビルド時に変換を走らせ、`iqs915x.c` が生成ヘッダーを include します。
- `drivers/input/iqs915x.c` は IQS915x 専用の実装です。
  - 16-bit little-endian register read/write を実装しています。
  - RDY GPIO 割り込みで専用スレッドを起こし、1 RDY 期間に 1 I2C transaction で処理する設計です。
  - 初期化は state machine で進みます。SHOW_RESET 確認、必要時 SW reset、init-data chunk write、ACK reset/Re-ATI、Event Mode/Manual Control 確認、通常動作へ遷移します。
  - init-data 書き込み中に、DTS 設定値とドライバ必須ビットをバッファへ事前パッチします。
  - runtime reset は `SHOW_RESET` で検出し、操作状態を解除して再初期化します。
- 入力イベント機能はかなり実装済みです。
  - 相対ポインタ移動: IQS915x の `REL_X/REL_Y` を `INPUT_REL_X/Y` として出します。
  - `report-absolute`: `FINGER1_X/Y` から差分を作って `INPUT_REL_X/Y` を出します。
  - 1本指 tap: left button。
  - 2本指 tap: right button。
  - software press-and-hold / tap-and-drag: left button を保持し、遅延解除も持ちます。
  - 2本指 vertical/horizontal scroll: `GESTURE_X/Y` を wheel/hwheel に変換します。
  - scroll inertia: Q8 fixed-point 風の減衰、remainder 保持、delayable work による継続出力を持ちます。
  - 3本指/4本指 swipe: 指群の centroid 移動から方向を判定し、F13-F20 既定の one-shot key tap を出します。
  - `disabled-by-default` と `iqs915x_set_enabled()`/`iqs915x_get_enabled()` による Active/LP2 runtime 切り替え API があります。
- `drivers/input/iqs915x_regs.h` は IQS915x の register map、bit flag、state/data/config 構造体をまとめています。
- `include/iqs915x.h` は runtime enable/disable API の公開ヘッダーです。
- `drivers/input/iqs5xx.c` と `iqs5xx.h` は旧 IQS5xx 系の実装として残っていますが、現在の IQS915x ドライバとは別 compatible です。

## 未完了または注意が必要な点

- `pinch-inertia` 系の DTS プロパティと構造体はありますが、現状の `iqs915x.c` では pinch 出力処理は実質未接続です。binding の説明にも「fully wired まで disabled by default」とあります。
- `iqs915x.c` 冒頭コメントには「raw read を使う」とある一方、実装の `iqs915x_read_stream()` は `i2c_write_read_dt()` で `IQS915X_REL_X` から 44 bytes を読んでいます。I2C シーケンス前提を変更する場合はコメントと実装を必ず同期してください。
- `INIT_FINAL_ACK_RESET` と `INIT_VERIFY_RESET` 付近には古いステップが残っています。初期化 sequence を触るときは、到達可能な state と実機での SHOW_RESET/Re-ATI 挙動を確認してください。
- `.gitignore` で `docs/` は無視されています。ローカルには datasheet や調整メモがある場合がありますが、追跡済みファイルとしては扱わないでください。
- Zephyr/ZMK のビルド環境はこのリポジトリ単体には含まれていません。変更確認は、利用側 firmware workspace から west/ZMK build で行う前提です。

## 作業時の指針

- まず `README.md`, `drivers/input/iqs915x.c`, `drivers/input/iqs915x_regs.h`, `dts/bindings/input/azoteq,iqs915x-common.yaml` を確認してください。
- DTS プロパティを追加・変更するときは、少なくとも次の 3 箇所を同期してください。
  - `dts/bindings/input/azoteq,iqs915x-common.yaml`
  - `struct iqs915x_config` in `drivers/input/iqs915x_regs.h`
  - `IQS915X_INIT(n)` in `drivers/input/iqs915x.c`
- init-data register の既定値を変える変更では、`drivers/input/IQS9150_init.h` を置き換えるか、`iqs915x.c` の init-data pre-patch に明示的に追加してください。
- 1 RDY 期間中の I2C transaction 数に注意してください。複数 register 操作が必要な処理は state machine で RDY cycle をまたがせる設計に合わせてください。
- pointer, tap, scroll, swipe の競合を触る場合は、`iqs915x_update_finger_state()`, `iqs915x_handle_multifinger_swipe()`, `gesture_pointer_suppress_ticks`, `allow_pointer_report` の関係を先に読んでください。
- ボタン押下を追加する場合は、必ず解除経路も同時に確認してください。`button_release_work`, `hold_release_work`, runtime reset, `iqs915x_set_enabled(false)` が関連します。
- power mode を触る場合は `include/iqs915x.h` の公開 API コメントも更新してください。

## 変更後の確認

- 少なくとも `scripts/convert_init_header.py drivers/input/IQS9150_init.h` が 1174 bytes を抽出できることを確認してください。
- 可能なら利用側 ZMK workspace で firmware build を実行してください。
- 実機確認では、起動時 init 完了ログ、tap/drag、scroll、scroll inertia、3/4 finger swipe、runtime disable/enable、runtime reset 復帰を分けて確認してください。
