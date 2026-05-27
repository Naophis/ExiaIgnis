# CLAUDE.md

このファイルは、リポジトリ内のコードを扱う Claude Code (claude.ai/code) に対してガイダンスを提供します。

## プロジェクト概要

ExiaIgnis は RP2350 (Raspberry Pi Pico2) 向けマウスロボットのファームウェアです。センシング・自己位置推定・軌道生成・PID 制御・モーター出力まで一貫して実装しています。

## ビルド・フラッシュコマンド

CMake 設定（初回またはCMakeLists変更後のみ必要）:
```bash
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
```

ビルド:
```bash
./compile.sh
# または: cmake --build build -- -j$(nproc)
```

デバイスへのフラッシュ（BOOTSELモード経由）:
```bash
./flash.sh
```

シリアルモニター:
```bash
./serial_monitor.sh        # デフォルト /dev/ttyACM0 @ 115200
./serial_monitor.sh /dev/ttyACM0 115200
```

**注意:** クローン後の初回ビルドでは、CMake configure 時に cJSON (v1.7.18) と LittleFS (v2.5.0) を FetchContent でダウンロードします。ネットワーク接続が必要です。

## アーキテクチャ

### マルチコア設計

エントリポイントは `src/main.cpp`。

- **コア0** (`src/main/main_task.cpp`): パラメータロード・UI・ボタン処理・シリアル printf 表示
- **コア1**: `SensingTask` (TIMER0 IRQ) + `PlanningTask` (TIMER1 IRQ) を両方登録し、`__wfi()` でスリープ

コア1 の rt_core1_entry で `sensing->start_irq()` と `planning->start_irq()` を順に呼び、IRQ をコア1 に登録します。TinyUSB がコア0 固定のため printf はコア0 からのみ呼べます。

### `.time_critical` セクション属性

パフォーマンスが重要な関数には `__attribute__((noinline, section(".time_critical.<module>")))` を付与し、SRAM に配置しています。モジュール名は `search` / `path_creator` / `main` など。IRQ ハンドラやホットパスに適用してください。

### SensingTask IRQ 構造 (`src/sensing_task.cpp`)

TIMER0 のハードウェアアラームを2本使用（alarm_pool オーバーヘッドなし）:
- **Alarm 2** (`timer_a_irq_handler`): `interval_us_` 周期（デフォルト1kHz）の定周期タイマー。Alarm 1 を即座にスケジュール。
- **Alarm 1** (`timer_b_irq_handler`): センサー読み取りシーケンス全体（ADC LED → diff 計算 → ジャイロ/エンコーダ/バッテリ + タイムスタンプ）。

ドリフト防止のため絶対時刻方式を採用: `next_alarm_a_ += interval_us_`

### PlanningTask IRQ 構造 (`src/planning/planning_task.cpp`)

TIMER1 のハードウェアアラームを1本使用:
- **Alarm 0** (`timer_irq_handler`): 1kHz 定周期。`tick(dt_us)` を呼び出し、`EgoEstimator → SensorProcessor → TrajectoryGenerator → ControlLaw` の順で実行。
- `send_command(shared_ptr<motion_tgt_val_t>)` で Core0 から目標値を投入（`__dmb()` で cross-core 安全）。

PlanningTask は以下のサブシステムを内包:

| クラス | ファイル | 役割 |
|--------|---------|------|
| `EgoEstimator` | `include/planning/ego_estimator.hpp` | センサー→速度・角度推定、カルマンフィルタ管理 |
| `SensorProcessor` | `include/planning/sensor_processor.hpp` | センサー LP 値→mm 距離変換、補間テーブル管理 |
| `TrajectoryGenerator` | `include/planning/trajectory_generator.hpp` | 台形速度プロファイル生成 |
| `ControlLaw` | `include/planning/control_law.hpp` | PID 制御・デューティ計算・モーター出力 |
| `MotorActuator` | `include/planning/motor_actuator.hpp` | モーター/吸引 PWM 出力 |

### MainTask 構造 (`src/main/`)

ファイルが機能ごとに分割されています:

| ファイル | 内容 |
|---------|------|
| `main_task.cpp` | `create()` / `run()` / パラメータロード / コンポーネント初期化 |
| `main_task_run.cpp` | `run_main_mode()` / `select_run_mode()` / `path_run()` / `sim_run_time()` |
| `main_task_run_profile.cpp` | `exec_param_prof()` / プロファイル読み込み関連 |
| `main_task_test.cpp` | `run_test_mode()` ルーティング |
| `main_task_test_misc.cpp` | 雑多なテストモード実装 |
| `main_task_test_pivot.cpp` | ピボットターンテスト |
| `main_task_test_run.cpp` | 走行テスト |
| `main_task_test_sla.cpp` | スラロームテスト |
| `main_task_usb.cpp` | USB シリアルコマンド処理 |
| `main_task_util.cpp` | `load_exec_params()` / `load_turn_param_profiles()` / `load_slalom_param()` など |

起動フロー:
1. `run()` でブザー/UI 初期化
2. LittleFS からパラメータロード (`load_params()`)
3. ボタン押し待ちループ（この間 USB 経由でファイル受信可能）
4. `sys_.user_mode != 0` → `run_test_mode()` / `== 0` → `run_main_mode()`

#### `run_main_mode()` サブモード

エンコーダーで番号を選択し、長押しで決定:

| mode_num | 動作 |
|----------|------|
| 0 | 探索走行 (SearchMode::ALL) |
| 1 | 片側探索 / 帰還探索 (Kata/Return) |
| 2 〜 N+1 | FastRun (exec_param_list[mode-2] のパラメータ) |
| N+2 | keep_pivot（その場旋回） |
| N+3 | 吸引テスト |
| N+4 | sim_run_time_all（全プロファイルのタイム計算） |
| N+5 | 迷路データ・探索ログ消去 |

### SearchController / Adachi (`src/search/`)

迷路探索を担当するサブシステム:

| クラス | ファイル | 役割 |
|--------|---------|------|
| `MazeSolverBaseLgc` | `include/search/logic.hpp`, `src/search/logic.cpp` | 迷路マップ・BFS 歩数マップ・ベクター距離マップ管理 |
| `Adachi` | `include/search/adachi.hpp`, `src/search/adachi.cpp` | 足立法ベースの次移動方向決定アルゴリズム |
| `SearchController` | `include/search/search_controller.hpp`, `src/search/search_controller.cpp` | 探索走行全体のオーケストレーション |

#### MazeSolverBaseLgc の迷路マップ形式

`map[x + y * maze_size]` の1バイト:
- 下位4bit (0x0f): 壁の有無（North=0x01, East=0x02, West=0x04, South=0x08）
- 上位4bit (0xf0): 踏破済みフラグ（North=0x10, East=0x20, West=0x40, South=0x80）
- 全方向踏破済み = `(map & 0xf0) == 0xf0`

`isStep(x, y, dir)` = その方向から踏み込んだことがある  
`existWall(x, y, dir)` = 壁がある  
`isProceed(x, y, dir)` = 壁なし かつ 踏破済み

#### ベクター距離マップ (`vector_dist`)

斜め経路コストを格納する `vector_map_t` 配列。`n/e/w/s` に各方向のコスト、`N1/NE/E1/SE/S1/SW/W1/NW` に8方向の通過カウントを格納。`updateVectorMap()` で priority_queue を使って Dijkstra 的に更新します。

#### Adachi アルゴリズム

`exec()` が1歩分の次移動方向 (`Motion`) を返します。  
`detect_next_direction()` で前進方向を最優先し、左右を `setNextDirection2()`（= 歩数が低い同値でも更新しない）で評価、後退は `enable_back` 条件下のみ。  
`subgoal_list` に未踏マスをキャッシュし、ゴール到達後は帰還目的地を動的切り替えします。

### Action サブシステム (`src/action/`)

走行アクションを組み立てるレイヤー:

| クラス | ファイル | 役割 |
|--------|---------|------|
| `PathCreator` | `include/action/path_creator.hpp`, `src/action/path_creator.cpp` | ベクター距離マップから `path_s` / `path_t` 配列を生成 |
| `MotionPlanning` | `include/action/motion_planning.hpp`, `src/action/motion_planning.cpp` | 直進・ピボット・スラロームの実行オーケストレーション |
| `TrajectoryCreator` | `include/action/trajectory_creator.hpp`, `src/action/trajectory_creator.cpp` | `path_t` 値 → TurnType / TurnDirection 変換ヘルパー |
| `WallOffController` | `include/action/wall_off_controller.hpp`, `src/action/wall_off_controller.cpp` | 壁補正制御ロジック |

#### `path_s` / `path_t` エンコーディング

`path_s[i]` = セグメントの直線距離（1セル = 2単位）  
`path_t[i]` = ターン種別の整数コード:

| 値 | 意味 |
|----|------|
| 1 | 右ターン (Normal Right) |
| 2 | 左ターン (Normal Left) |
| 3 | Large Right |
| 4 | Large Left |
| 5 | Orval Right |
| 6 | Orval Left |
| 7 | Dia45 Right |
| 8 | Dia45 Left |
| 9 | Dia135 Right |
| 10 | Dia135 Left |
| 11 | Dia90 Right |
| 12 | Dia90 Left |
| 254 | スキップ（pathOffset で削除対象） |
| 255 | ゴール / 終端 |

生成パイプライン: `path_create()` → `convert_large_path()` → `diagonalPath()` → `pathOffset()`

#### PathCreator の経路最適化

`timebase_path_create()` では `other_route_map` に候補分岐マスを記録し、`exec_param` の1〜5パターンで `path_create_with_change()` を試して最短タイムの経路を `path_set_map` (priority_queue) から取得します。

### LoggingTask (`include/logging/logging_task.hpp`)

PSRAM バンプアロケータ (`psram_heap::alloc`) を使い、`std::vector<LogEntry, PsramAllocator<LogEntry>>` にデータを蓄積。

- Core1 IRQ 内から `append_from_irq()` を呼ぶ（active_ が false なら no-op）
- Core0 から `start()` / `stop()` / `dump_csv()` を呼ぶ
- デフォルト最大 60,000 サンプル（1kHz × 60 秒）

### SPI バス共有

2本の SPI バスに複数デバイスが異なるモードで接続:

| バス  | デバイス | ピン |
|-------|---------|------|
| SPI1 | ASM330LHH ジャイロ (mode 3) + AS5147P 右エンコーダ (mode 1) | MISO=12, CS_gyro=13, CLK=14, MOSI=15, CS_enc=9 |
| SPI0 | AS5147P 左エンコーダ (mode 1) + ADS7042I バッテリADC (mode 0) | MISO=0, CS_enc=1, CLK=2, MOSI=3, CS_bat=5 |

**重要:** ARM PL022 は CPOL/CPHA 変更前に SSE=0 が必要です。同一バス上でモードを切り替える場合は、`spi_set_format()` の代わりに必ず `include/driver/spi_util.hpp` の `spi_set_format_safe()` を使用してください。

### センサー LED シーケンス

R45・L45 はそれぞれ LED が2本あり、3パターンで読み取ります:
1. LED1 のみ → `r45_1` / `l45_1`
2. LED1 + LED2 同時点灯 → `r45_both` / `l45_both`
3. LED2 のみ → `r45_2` / `l45_2`

R90・L90 は LED1本。全読み取りは `led_settle_us_` のビジーウェイト後に実施。先に ambient（暗）値を取得し、`diff = lit - dark`（負の場合は0にクランプ）。

### GPIO ピンマップ（センシング）

| GPIO | 機能 |
|------|------|
| 18 | R90_LED |
| 20 | R45_LED2 |
| 21 | R45_LED1 |
| 23 | L45_LED1 |
| 24 | L45_LED2 |
| 25 | L90_LED |
| 26 | R90_SEN (ADC0) |
| 27 | R45_SEN (ADC1) |
| 28 | L45_SEN (ADC2) |
| 29 | L90_SEN (ADC3) |

## 設定システム（ConfigLoader）

`ConfigLoader`（`include/config_loader.hpp`, `src/config_loader.cpp`）は起動時にフラッシュ末尾 256KB の LittleFS から各 JSON ファイルを読み込みます。初回起動時はフォーマットしてデフォルト値を自動生成します。

```cpp
ConfigLoader::init();  // multicore_launch_core1 より前に呼ぶこと
int val = ConfigLoader::get_int("sensing.led_settle_us", 12);
```

### LittleFS ファイル一覧

| ファイル | 読み込み先 | 内容 |
|---------|-----------|------|
| `/config.json` | `ConfigLoader::get_int/float` | sensing.led_settle_us / interval_us |
| `/hardware.json` | `input_param_t` | タイヤ径・ギア比・バッテリゲイン等 |
| `/sensor.json` | `input_param_t` | センサーゲイン・参照値 |
| `/offset.json` | `input_param_t` | クリアアングル等オフセット |
| `/system.json` | `system_t` | user_mode / maze_size / goals / test_mode_t / circuit_mode |
| `/exec.json` | `exec_pram_t[]` | fast/normal/slow インデックスのリスト |
| `/profiles.hf` or `/profiles.cl` | `turn_param_profile_t` | TurnType ごとのファイルインデックス |
| `/vel_prof.hf` or `/vel_prof.cl` | `straight_param_t` | 速度プロファイル (v_prof[]) |
| `/<slalom_file>` | `slalom_param2_t` | スラロームパラメータ |

`sys_.hf_cl == 0` なら `.hf` 、`1` なら `.cl` を使用します。  
`sys_.circuit_mode == 1` の場合、`path_run()` はサーキットパス (`load_circuit_path()`) を使います。

## USB シリアルコマンド

stdio は USB のみ（UART 無効）。起動後のボタン待ちループ中に以下のコマンドを受け付けます（`src/main/main_task_usb.cpp`）:

| コマンド形式 | 動作 |
|------------|------|
| `filename@json_content\n` | LittleFS に `/filename` として書き込み、パラメータを即再ロード |
| `LIST` | ファイル一覧を `name:size` 形式で出力 |
| `DELETE:filename` | `/filename` を削除 |
| `READ:filename` | `size\n content OK\n` 形式で内容を出力 |

flash_range_erase/prog は USB CDC を ~100ms 切断するため、書き込み前に `OK\n` を送信してから 80ms 待機します。

## データ構造

### 主要な共有エンティティ（`include/structs.hpp`）

| 型 | 説明 |
|---|------|
| `sensing_result_entity_t` | センシング結果（ego / led_sen / encoder / gyro 等）。PlanningTask が毎 tick 更新 |
| `input_param_t` | 全走行パラメータ（PID ゲイン・センサーゲイン・各種しきい値等）。MainTask が LittleFS から読み込み |
| `motion_tgt_val_t` | 動作目標値（`new_motion_req_t nmr` 等）。MainTask → PlanningTask 方向の指示 |
| `param_set_t` | スラローム/直線パラメータセット（`map` / `map_slow` / `map_fast` / `str_map` / `circuit_mode`） |
| `turn_param_profile_t` | プロファイルファイルリストと TurnType→インデックスマップ |
| `slalom_param2_t` | スラロームパラメータ（v / ang / rad / rad2 / time / front / back オフセット等） |
| `straight_param_t` | 直線パラメータ（v_max / accl / decel / w_max / alpha） |

`sensing_result_entity_t`, `input_param_t`, `motion_tgt_val_t` は `main.cpp` で `make_shared` し、sensing / planning / main_task 間で shared_ptr で共有します。

### SensingTask::Data（`include/sensing_task.hpp`）

タイミング（`dt_us`, `sense_duration_us`）、センサー値（`SensorDark`, `SensorLit`, `SensorDiff`）、ジャイロ（`gz` + タイムスタンプ）、エンコーダ（`enc_r`, `enc_l` + タイムスタンプ）、`battery`。

**注意:** `volatile` からコピーするため `Data(const volatile Data &o)` コピーコンストラクタを明示実装しています。`Data` にフィールドを追加した場合は必ずコピーコンストラクタも更新してください。

タイムスタンプの記録パターン（各センサー共通）:
```cpp
self->data.gz_ts_z = self->data.gz_ts;   // 前回時刻を退避
self->data.gz_ts   = time_us_64();        // 今回時刻を記録
self->data.gz      = gyro_.read_gyro_z();
self->data.gz_dt   = self->data.gz_ts_z ? (self->data.gz_ts - self->data.gz_ts_z) : 0;
```
`ts_z ? ... : 0` のガードは初回実行時の巨大な dt を防ぎます。

### PlanningTask::Command / State

`Command` は Core0 → Core1 方向の動作指示（MotionMode, v_max, dist, ang 等）。
`State` は Core1 → Core0 方向の現在状態（img_v, img_dist, v_est, duty_l/r 等）。
どちらも `volatile` 経由でコピーするためコピーコンストラクタを明示実装しています。

## ドライバークラス（`include/driver/` + `src/driver/`）

- **ASM330LHH**: ジャイロ、SPI mode 3。`init()` で SPI バスを初期化。`setup()` でソフトウェアリセット + 設定シーケンスを実行。Z 軸角速度のみ取得。
- **AS5147P**: 磁気エンコーダ、SPI mode 1。`init()` は初期化済み SPI バス + CS ピンのみ受け取る。14bit 角度値 [0–16383] を返す。
- **ADS7042**: バッテリ電圧 ADC、SPI mode 0。結果 = `(rx >> 2) & 0x0FFF`。

## ユーティリティ（`include/utils/`）

- **KalmanFilter** (`kalman_filter.hpp`): 1次元カルマンフィルタ。速度・角速度・角度・距離・バッテリ等で使用。
- **KalmanFilterMatrix** (`kalman_filter_matrix.hpp`): 行列形式カルマンフィルタ。位置推定 (pos) で使用。
- **irq_log** (`irq_log.hpp`): IRQ 内から安全に使えるリングバッファログ。

## Enum 一覧

### `Direction` （`include/maze_solver.hpp`）

North=1, East=2, NorthEast=3, West=4, NorthWest=5, SouthEast=6, SouthWest=7, South=8, Undefined=255, Null=0

対角方向（NE/SE/SW/NW）はベクター距離マップと `diagonalPath()` で使用。  
`static_cast<int>(now_dir) * static_cast<int>(dir) == 8` は逆方向チェック（North×South, East×West）。

### `TurnType` / `StraightType` / `ExecParamType` （`include/maze_solver.hpp`）

主要な TurnType: `None / Normal / Large / Orval / Dia45 / Dia45_2 / Dia135 / Dia135_2 / Dia90 / Kojima / Finish`  
主要な StraightType: `Search / FastRun / FastRunDia`  
ExecParamType: `Fast / Normal / Slow`

### `SearchMode` / `MotionResult` / `SearchResult` （`include/enums.hpp`）

SearchMode: `ALL=0 / Kata=1 / Return=2`  
MotionResult: `NONE=0 / ERROR=1 / WALL_OFF_DETECTED=2`  
SearchResult: `SUCCESS=0 / FAIL=1`
