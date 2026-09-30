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

### ファイル単位の SRAM 配置（`memmap_custom.ld`）

属性の付け忘れ対策として、ホットなモジュールは**オブジェクトファイル単位**で SRAM に配置しています。`memmap_custom.ld` の `.text` と `.rodata` の `EXCLUDE_FILE(...)` に挙げたファイルは flash から除外され、`.data` 内の `*(.text*)` / `*(.rodata*)` で RAM に入ります（2 箇所のリストは同一に保つこと）。

- 対象: `src/planning/` `src/action/` `src/search/` `src/utils/` `src/logging/` `src/sensing_task.cpp` `src/ui.cpp` `src/main/main_task_run.cpp` `gen_code_*/`、センサー/PWM/DShot ドライバ、そこから呼ばれる SDK（`pico_time` `hardware_timer/spi/gpio/irq/...`）、`libstdc++` の `tree.o`/`hashtable_c++0x.o`、libc の `malloc/free` と `mem*`
- 対象外（flash のまま）: `src/main/` の UI・テスト・USB・パラメータロード、`config_loader`、`am32_*`、`psram_check`、LittleFS、cJSON、printf、TinyUSB
- SDK の float/double 実装は `CMakeLists.txt` の `PICO_FLOAT_IN_RAM=1` / `PICO_DOUBLE_IN_RAM=1` で RAM に配置
- 新しいホットなソースを上記ディレクトリ外に置いた場合は、リストへの追加が必要です。配置の確認は `arm-none-eabi-nm -n build/ExiaIgnis.elf | awk '$1 ~ /^20/ && /veneer/'`（RAM 上のコードから flash への呼び出し一覧）が手早いです。
- libc の除外パターンは `*lib*_a-mem*.o`。SDK 標準の `*lib_a-mem*.o` は現行ツールチェーンのオブジェクト名 `libc_a-memcpy.o` に一致しません。

### SensingTask IRQ 構造 (`src/sensing_task.cpp`)

TIMER0 のハードウェアアラームを使用（alarm_pool オーバーヘッドなし）。1ms を 4 つの枠に分けて読みます（2026-09-29、`SensingTask::kSlotOffsetUs`）:

| 枠 | 時刻（tick 基準） | 中身 |
|----|------|------|
| S0 | +0us | 45 系（環境光、R45・L45。WALL_OFF 等は LED1 / 両方 / LED2 の 3 通り） |
| S1 | +220us | 90 系（環境光、R90・L90）→ 差分の計算（`finalize_sensing`） |
| S2 | +440us | IMU（1 点読み + FIFO）と角速度（`read_imu`） |
| S3 | +600us | エンコーダー・バッテリー、車輪速度、距離・角度の積分（`read_enc_bat`） |

WALL_OFF / WALL_OFF_DIA 中は、左右の 45° LED1 も読む（`read_wo_extra`、環境光もその場で読んで差分、約 40us）。S2・S3 は枠の最初、S1 は 90 系を読み終えて LED の待ち時間の 2 倍空けてから（45° を先に読むと直後の 90° の生値が 0〜4 → 6〜14 に上がった。LED を消した直後の受光素子の尾か ADC の前の値が残るため。**別の LED の読みを続けるときは、間を空けて、読むたびに暗い値をその場で取ること**）。S0 の 45 系シーケンスの LED1 と合わせて 4 サンプル / tick。1 tick 分（値と読んだ時刻）を S3 の最後に `sensing_result->wo`（`wo_hf_t`）へまとめて写すので、ログや planning が見る 1 組の中で tick が混ざらない。ログ列は `wo_l0..3` / `wo_r0..3`（差分 raw）、`wo_tl0..3` / `wo_tr0..3`（tick 開始からの時刻 [us]）、`wo_n`（4 = WALL_OFF 中、1 = S0 の分だけ）、`wo_seq`（`gyro_fifo_seq` と同じ tick 番号）。

`offset.yaml` の `wall_off_hf_mode: 1`(2026-09-30)では、最短走行の直進(STRAIGHT / SLA_FRONT_STR / SLA_BACK_STR)でも同じ読み方(各枠の最初、左 → 右の順)で読み、検知器(`SensorProcessor::update_wall_edge`)には S1〜S3 だけを入れる(S0 は入れない)。省電力のため読むのは使う側だけ: WALL_OFF 中は曲がる側(`motion_dir`)、直進中は Core0 が経路から指令に付けた側(`nmr.hf_side`: 0 = 左右とも、1 = 左、2 = 右、3 = 読まない。`MotionPlanning::hf_side_hint_`、`exec_path_running()` が次のターンの側、`slalom()` が SLA_FRONT_STR に曲がる側・SLA_BACK_STR に `next_motion.next_turn_dir` を入れる。ゴール後の直進は 3)。読まなかった側は時刻が 0 で、値は最後に読んだものを保持(ログの見やすさのため)。右だけ読んだ右の読みは、左の直後に読んだ右と同じ形(093556 と 030005)。壁なしで柱を見る WALL_OFF は柱の約 2 tick 手前で始まるので、WALL_OFF 中だけ読んでも柱の手前側が取れないため。

**読みは、そのセンサーを最後に光らせてからの時間と直前の LED の並びで変わる**(原因は未確定)。毎枠同じ並びで読む S1〜S3 どうしは 1 raw 以内で揃うが、S0(tick の最初、前の点灯から 380us 以上)は低く出る(右の柱 raw 約 120 で −3〜−17)。S0 を混ぜた今日の壁ありのログ 29 件の再生では、S1〜S3 だけにすると同じコースの基準位置のばらつきが 左 0.79 → 0.58 / 右 0.64 → 0.37mm になった。**枠ごとに読む側を変えてはいけない**(片側ずつ交互に読む版を入れたら、右の柱で S1 が +9、S0 が 1 tick おきに −10 になり、1 tick 1 点の谷底が約 1mm ずれた。20260930_093556。読む並びを変えるときは S1〜S3 が揃うこと・S0 に 1 tick おきの段が出ないことをログで確かめる)。左の直後に右を読んでも右は変わらない(右だけ読んだときと同じ形、030005 と 093556 の WALL_OFF 中)。

S1 は S2・S3 と読みの大きさが少し合わない(2026-09-30、`hf_mode` 1 の 15630 件の再生): 平らな区間で、左は S1−S2 が raw 0〜60 で +2.6、60〜150 で +1.8、150〜300 で +0.6、300 以上で −0.9〜−1.3、右は 150〜450 で ±0.1(S1〜S3 が揃う)。傾き・レベル・時刻ずれを同時に回帰すると、時刻ずれは ±8us で 0 と区別できず(急な区間で「S1 が約 50us 早い」に見えたのはレベルの偏りの見かけ)、S1 は S2・S3 より 加算 +2〜5 raw / ゲイン −0.8〜−1.3%(左は raw 370、右は 260 付近が境目)。原因は直前の LED の余韻(2026-09-30 確認): 暗い値(約 12 raw、ログ `wo_bl0..3` / `wo_br0..3`。点灯時の値 = 差分 + 暗い値)は、直前に LED を消してから約 40us 後の読みで +3〜4 raw 高く、約 100us 後では 0(S1 の前の待ち `wall_off_hf_s1_guard_us` を 32 → 100us にすると、超過が S1 から S2 に移った: S1 の暗い値 左 +2.5 → −0.4、S2 の暗い値 −0.5 → +2.8 / 右 S2 +3.7。S1 の読んだ時刻 305 → 377us)。**読みの偏り(低いレベルで高く・高いレベルで低い差分)も余韻のある枠に付いて移る**(0〜40 raw で S2 が +10、120〜250 で +5、250〜400 で +1、400 以上で 0。待ち 32us のときの S1 は +2.6 / +1.8 / +0.6 / −1.3)。余韻はセンサーの動作点を上げて小さい信号のゲインを増やす形。待ちを伸ばしても S1〜S3 は 220us の中に収まらず、余韻が別の枠へ移るだけ(S1 の 90 系の後の待ち 100us なら S2 が 40us)なので、既定(0 = 約 32us)のまま。柱の谷底(raw 230 付近)への影響は約 0.2〜0.5mm、肩(raw 100 付近)で約 0.7〜1.7mm。`wall_off_hf_mode: 2` は省電力の側の指定を無視して常に左右とも読む調査用(結果は 1 と同じ形)。
| planning | +720us | 最大 247us（09-26〜29 のログ）→ 次の S0（+1000us）までに終わる |

- **Alarm 1** (`timer_b_irq_handler`): 枠の予約と振り分け。各枠の入口で次の枠を予約する。S0 の予約時刻がその tick の基準（ドリフトなし）で、Core1 が止まって 1ms 以上遅れたときだけ今に取り直す。S0 で planning を基準 + `PlanningTask::kPhaseAfterSensingUs`(720us) に予約する。
- **Alarm 2** (`led_seq_irq_handler`): LED 点灯シーケンスの非同期継続（LED ON → 待ち → ADC）。45 系（S0）と 90 系（S1）の 2 本。
- planning が動く時間帯（+720〜+1000us）にセンシングの処理を置かないので、同じ優先度のまま重ならない（受け渡しは今までどおり `sensing_result` を直接書く）。重なりの確認はログ列 `slot_late_us`（枠の開始の遅れの最大）・`pln_margin_us`（planning 終了から次の S0 までの余裕、負なら重なり）・`led_overrun`（LED シーケンスが次の枠まで残って打ち切った回数）。**枠の中身や時刻を変えるときは、planning の時間帯（最大 247us + 余裕）にかからないことを確かめること。**

#### ジャイロ FIFO（`read_gyro_fifo()` / `update_gyro_fifo()`）

ASM330LHH はジャイロを実 ODR（個体ごと、本機 3508.5Hz）で FIFO に入れ続け、毎 tick の SPI Phase A の直後に FIFO_STATUS1/2 で個数を読み、その数だけ 0x78 から 1 回の DMA で読みます（7 バイト/ワード、通常 3〜4 ワード）。8 ワード超・あふれ・タグ不一致のときは FIFO を捨て（`fifo_flush()`）、その tick は 1 点読みで代用します。

- 使い方は `hardware.yaml` の `gyro_param.fifo_mode`。0 は従来の 1 点読みのまま（FIFO は計算してログに出すだけ）、1/2/3 は最新・直近 3 サンプル平均・tick 内平均を `w_raw` に使い、角度を Σw·T_odr で積分します。4 は直近 3 サンプル平均を、直近 `fifo_alpha_win` サンプルに当てた直線の傾き(角加速度)で 1.5 サンプル + `fifo_lead_extra_us` 先読みします（平均の遅れ 1 サンプル + 最新サンプルの古さの平均 0.5 サンプルを打ち消す）。
- ログ列: `gyro_fifo_n`（-1/-2 は flush）、`w_snap`、`w_fifo_last/ma3/mean/pred`、`alpha_fifo`、`ang_fifo_diff`（FIFO 角度 − 1 点読み角度の累積 [deg]）、`gyro_odr_err`（MCU 時間で数えた ODR の FF 由来値からのずれ [%]）、オフライン検証用の生サンプル `gyro_raw0..3`（古い順、n 個まで有効）・`gyro_fifo_seq`（tick 通し番号）・`gyro_fifo_t`（読んだ MCU 時刻 [us] 下位 16bit）。
- `gyro_param.fifo_plan_lead: 1` で、`fifo_mode` 1〜4 の w に「目標角加速度 × (FIFO を読んでから次の planning tick までの時間)」を足します。PlanningTask は S2（IMU）の約 275us 後に w を使うため。時間は `PlanningTask::next_tick_us()` から毎回読みます。角度の積分(FIFO の和)には入れません。ログ列 `plan_age_us`(読んでから planning までの時間)・`w_plan_lead`(足した量)。
- 1 点読み（9 バイト）は加速度の取得と比較用に残してあります。加速度はまだ FIFO に入れていません。

### PlanningTask IRQ 構造 (`src/planning/planning_task.cpp`)

TIMER1 のハードウェアアラームを1本使用:
- **Alarm 0** (`timer_irq_handler`): 1kHz 定周期。`tick(dt_us)` を呼び出し、`EgoEstimator → SensorProcessor → TrajectoryGenerator → ControlLaw` の順で実行。
- **位相はセンシングの tick（S0）+ 720us に固定**（`PlanningTask::kPhaseAfterSensingUs`、2026-09-29。最初は +600us で固定し、枠分けで +720us にした）。planning は自分で次回のアラームを決めず、SensingTask の `timer_b_irq_handler` が毎 tick の入口で `schedule_tick()` を呼んで予約する。以前は両方が自分で「1ms 以上遅れたら 今 + 1ms」と取り直していたため、パラメータ送信（`flash_safe_execute` で Core1 が止まる）のたびに位相が約 590us と 815us の間で変わっていた（再開時は IRQ 番号の小さい planning が先に動き、sensing はその処理の後になる）。ログ列 `plan_age_us`（FIFO を読んでから planning までの時間）で確認できる（枠分け後は約 275us）。
- `send_command(shared_ptr<motion_tgt_val_t>)` で Core0 から目標値を投入（`__dmb()` で cross-core 安全）。

#### 旋回の始まりを tick の途中へ合わせる(`sla_start_align`、2026-09-30)

従来は Core0 の `go_straight`(SLA_FRONT_STR)が目標位置 X を越えた tick で終わり、SLALOM の指令は次の planning の tick で効く。距離が進むのは 1ms に 1 回なので、越えた量(0〜1 tick、2200mm/s で 0〜2.2mm)がそのまま旋回位置のばらつきになっていた(±0.63mm)。Core0 を速く起こしても、指令が効くのは planning の 1kHz の tick だけなので消えない。

`hardware.yaml` の `sla_start_align: 1` で、Core0 は X の 0.5 tick 手前で SLA_FRONT_STR を終え、X(`global_pos.dist`)を SLALOM の指令(`nmr.sla_start_x` / `sla_align`)に付けて送る。planning(`include/planning/sla_start_align.hpp`、`TrajectoryGenerator::generate_sla_aligned()`)は受け取った tick で tau = (global_pos.dist − X)/(v·dt) − 1.5 を求め、旋回の出力列(角速度・角度・FF・点列)を tick の途中まで遅らせて出す(隣り合う 2 tick の直線補間)。生成器の中身(カウンタ・積分)は影の状態で従来のまま進めるので、生成コード `gen_code_mpc` は触っていない。

- tau の基準は従来の切り替えの平均と同じ位置なので、ターンの front/back はそのまま使える。
- 最短走行の SLAROM_RUN の旋回だけ。探索の Normal と角度で決める SLALOM_RUN2 は従来どおり。
- Core0 が 1 tick 遅れたとき(tau ≥ 0)は従来と同じ出力列、早すぎたとき(tau < −1)は直進を出して待つ(最大 3 tick)。
- ホスト検証は `tests/sla_align_host/run.sh`(生成コードをそのまま使う)。large90 v2200 で旋回後の直線の横位置のばらつきが幅 2.18 → 0.013mm、平均の差 0.002mm。1 tick 遅れは従来と出力列が完全一致。
- ログ列 `sla_tau`(決めたずれ [tick]、−1〜0 が正常、9 = 合わせていない旋回)・`sla_wait`(直進で待った tick 数)。
- 補間の起点は前の tick の**生成器の出力そのもの**(`raw_prev_`、FF・alpha2 を含む)。`ego_in` は FF を持たない(copy_tgt が写さない)ので起点にしてはいけない。最初の版はこれで旋回中の FF(逆起電力分 `ff_duty_rpm` を含む)が f 倍になっていた(20260930_020508〜020829 の 1 側、旋回終わりの角度が 3〜5° 変わった)。ホスト検証は全項目を「従来の出力列の補間」と比べる。
- **`copy_tgt()` が生成器の出力から `ego_in` へ写す項目を変えたら `sla_state_fields()` も合わせる。**

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

ゴール後(`SearchMode::ALL`)のサブゴールは `Adachi::update()` → `searchGoalPositionReuse()` で、未知を壁なしとみなした重みパターン 1 の最短経路 1 本の上の未知区画。見つけた未知区画を `subgoal_list` に足していく(古いものは `age_subgoal()` が 45 回で期限切れにする)。

`update()` まわりの計算は 2026-09-29 に、**結果を変えずに**軽くした(機体向けビルドの命令数。機体での時間は未計測):

- 表づくり(`updateVectorMap(bool, subgoal_list)` / `clear_vector_distmap(subgoal_list)`)を書き直し: 137 万 → 62 万命令。区画の範囲確認と壁・既知の確認を進む先の区画ごとに 1 回にまとめ、向きごとの if の連鎖を表引き(`VECTOR_STEP`)にし、小さい関数(どれも noinline)の呼び出しをやめた。取り出す順番(`vq_list`)・書き込む値・書き込む順番は元と同じ。経路生成が使う `updateVectorMap(bool)` は元のまま。
- 歩数マップ(`update_dist_map`)を書き直し: 11.2 万 → 1.9 万命令。
- `searchGoalPositionReuse()`: 地図・ゴール・重みパターンが前回の表づくりから変わっておらず、ほかの誰も表を作り直していなければ(`vector_map_serial`)、表を作り直さずサブゴールの手入れ(期限切れ → 4 辺とも既知になった区画を外す → 経路の上の未知区画を足す)だけをやる: 5〜13 万命令。ゴール後の `update()` の 44 % がこれに当たる。`searchGoalPosition(true, …)` と同じ結果を返す。
- RAM: コード +1.6 KB、地図の写し +1 KB。
- **探索まわり(`logic.cpp` / `adachi.cpp`)を、結果を変えないつもりで直したら** `python3 tools/path_sim/check_search.py`(探索 1 本まるごとの出力が基準と同じか、27 迷路)を回す。基準は変更前のソース(`tools/path_sim/experiments/frozen2`)で取ったもので、走行パラメータも基準と一緒に保存してある。乱数の状態での比較は `experiments/emu/eq_test.py`、命令数は `experiments/emu/run.py`。

サブゴールの選び方は 2026-09-29 に search_sim でいくつか比べたが、**ファームには入れていない**(探索中に動く部分は変えない、というユーザー判断。経緯と数字は `tools/path_sim/experiments/README.md` の実験 4・5):

- パターン 1 が尽きたら 4 → 3 → 2 で探し直す予備: 探索の総時間 +7 %、最短走行がタイム最小にならないケース 25 → 15 件(21 迷路 × 5 モード中)。
- パターン 2 と 4 の経路を覚え、壁で塞がれたときだけ作り直す: 探索 −1.3 %、25 → 9 件。ただし迷路ごとの差が大きく(探索 −47〜+97 s)、最短走行が速くなるのは 21 本中 4 本。
- タイム最小の経路(未知は壁なし)の上の未知区画を選ぶ: 探索 −6 %、ほぼ 0 件。計算が重く、機体で回せるかは未確認。

ホスト版: `tools/path_sim` の search_sim(Param Console の迷路タブの「探索」)が `adachi.cpp` / `logic.cpp` をそのまま PC でビルドして探索を再現する。`SearchController::exec()` の探索ループ・`judge_wall`・`pivot()` の手順と `run_main_mode()` の mode_num == 0 の準備は `tools/path_sim/search_main.cpp` に写しがあるので、**それらを変えたら search_main.cpp も合わせる**。

### Action サブシステム (`src/action/`)

走行アクションを組み立てるレイヤー:

| クラス | ファイル | 役割 |
|--------|---------|------|
| `PathCreator` | `include/action/path_creator.hpp`, `src/action/path_creator.cpp` | ベクター距離マップから `path_s` / `path_t` 配列を生成 |
| `TimePathPlanner` | `include/action/time_path_planner.hpp`, `src/action/time_path_planner.cpp` | タイム最小の経路探索(最短走行の経路生成の本体) |
| `MotionPlanning` | `include/action/motion_planning.hpp`, `src/action/motion_planning.cpp` | 直進・ピボット・スラロームの実行オーケストレーション |
| `TrajectoryCreator` | `include/action/trajectory_creator.hpp`, `src/action/trajectory_creator.cpp` | `path_t` 値 → TurnType / TurnDirection 変換ヘルパー |
| `WallOffController` | `include/action/wall_off_controller.hpp`, `src/action/wall_off_controller.cpp` | 壁補正制御ロジック |

#### `path_s` / `path_t` エンコーディング

`path_s[i]` = セグメントの直線距離（1セル = 2単位。前のターンの出口の基準点から次のターンの入口の基準点までで、実際の直線は `0.5 * path_s − 1` 区画）  
`path_t[i]` = ターン種別の整数コード:

| 値 | 意味 |
|----|------|
| 1 | 右ターン (Normal Right) |
| 2 | 左ターン (Normal Left) |
| 3 | Orval Right (180°) |
| 4 | Orval Left (180°) |
| 5 | Large Right (大回り 90°) |
| 6 | Large Left (大回り 90°) |
| 7 | Dia45 Right |
| 8 | Dia45 Left |
| 9 | Dia135 Right |
| 10 | Dia135 Left |
| 11 | Dia90 Right |
| 12 | Dia90 Left |
| 254 | スキップ（pathOffset で削除対象） |
| 255 | ゴール / 終端 |

生成パイプライン: `path_create()` → `convert_large_path()` → `diagonalPath()` → `pathOffset()`

#### 壁切れ検知の柱の谷(下に凸)検知 (`include/planning/pillar_trough_detector.hpp`)

壁なし開始の `WALL_OFF` では注視側 45° LED1 が「下降→谷底→急上昇」の谷を見せる。`PillarTroughDetector` は Core1 の `SensorProcessor::update_pillar_trough()` で左右 2 本を毎 tick 更新し(旋回・超信地・停止中は再アーム)、結果を `sensing_result->pillar_l/r` に公開する。切れ目の判定は**走行距離で正規化した 2 階微分(曲率)が `curv_th` 以上を `curv_n` tick 連続**で行う。首振れや姿勢ドリフトは読みをほぼ直線に動かすので 2 階微分では符号が交互に振れるだけになり、1 階微分では紛らわしい緩い上昇を弾ける。1 階微分ルールと `far_th` 到達は保険として残してある。Core0 の `WallOffController::take_pillar_trough()` が exist=false の経路で最優先に拾い、発火位置ではなく谷底位置でアンカーして `ps_front.dist += pillar_str − (現在位置 − 谷底位置)` とする。パラメータは `offset.yaml` の `wall_off_pillar_*`。従来の絶対しきい値経路と 25mm 通過の安全網(`detect_pass_through_case2`)は残してある。ホスト検証は `tests/pillar_trough_host/run.sh`(CSV ログを渡すと全行再生)。

谷底の**位置**は、`wall_off_pillar_hf: 1`(かつ `wall_off_hf_mode: 1`)のとき細かいサンプルで求め直す(2026-09-30、`WallEdgeDetector::trough_vertex()`、±`pillar_hf_win` の最小二乗の放物線の頂点を窓を置き直して 3 回)。柱かどうかの判定は `PillarTroughDetector` のまま。Core1 は発火した tick から、窓の先の端までサンプルが来るまで毎 tick 試し、`pillar_l/r.bottom_x_hf` と `hf_tag` に出す。Core0 の `take_pillar_trough()` は求まっていればそれを使い(`wall_off_pillar_hf_str_l/r`)、求められなければ従来の谷底(`wall_off_pillar_str_l/r`)、まだなら**ほかの判定をせずに**次の tick を待つ(`pillar_wait_`。谷底から `pillar_hf_wait` まで)。

- 従来の谷底 `bottom_x` は planning の時刻の位置で、読んだ時刻の位置より 速度 × 約 0.6ms(2200mm/s で 1.3mm)先にずれている。`bottom_x_hf` は読んだ時刻の位置。`pillar_hf_str` の初期値はこの差を足したもの(`pillar_str` + 1.4)。
- 実測の谷の形とノイズ 1.5 raw での位置のばらつき(tick の位相・速度・WALL_OFF の開始位置を振った再生): 従来 std 0.18〜0.23mm・最大 ±0.9mm → 求め直し std 0.08〜0.10mm・最大 ±0.3mm。2200mm/s では発火の tick で求まり、1500mm/s では半分が 1 tick 待ち。
- ホスト検証は `tests/wall_edge_host/test_vertex.cpp`。ログ列 `pillar_hf_lag_l/r`(現在位置 − 求めた谷底、無ければ 0、求められなければ −1)。
- `wall_off_pillar_hold: 1` で、柱の谷を追跡中(検知器の shape_ok と同じ条件)は従来の判定(`wall_missing` 等)を待たせて柱の検知に譲る。谷底が WALL_OFF の開始より前に来た走行では柱の検知(谷底の 2〜3 tick 後)より先に `wall_missing` が抜け、別の補正値(`wall_off_hold_dist_str_l/r`)で旋回位置が決まっていた(20260930_121814: 同じ柱で 025937 より 3.7mm 手前)。
- 実機(左 → 右で毎枠読む版、121814 / 121834): 直進中も S1〜S3 は揃い(右の柱 22,23,25 | 28,30,31 | …)、S0 は S1〜S3 より 右 −10〜−17 / 左 −1〜−4 raw で一定(1 tick おきの段は無し)。右の柱の谷底(細かいサンプル)は 3 走行で 99.14〜99.32mm(スタートから)、旋回開始は 127.72〜127.93mm。

#### 壁ありで始まる WALL_OFF の壁の切れ目の形の検知 (`include/planning/wall_edge_detector.hpp`、2026-09-30)

従来の判定(`detect_wall_off`)は「45° が絶対しきい値 `noexist_th`(49mm)を越えて増えている」で、壁ありの普段の距離(46〜49mm)との余裕が 1〜3mm しかなく、壁から少し離れて走ったり暗い壁だったりすると壁が続いているのに発火した。`WallEdgeDetector` は 1 tick 4 サンプル(`sensing_result->wo`)で形を見る: 壁の距離 = 過去 [x−16, x−4]mm の中央値(60mm 以上は壁なし)、発火 = そこから +4mm 以上まで 10mm 以内に遠のいた、基準位置 = +3mm を越えた点(補間)。Core1 の `SensorProcessor::update_wall_edge()` が毎 tick 更新して `sensing_result->edge_l/r` に公開し(再アームは柱の谷と同じ区間)、Core0 の `WallOffController::take_wall_edge()` が exist=true の第二段階で拾って `ps_front.dist += edge_str − (現在位置 − 基準位置)` とする。一度使った発火は使わない。発火後は発火位置から `win_far` 進むまで発火しない(履歴を消して再アームすると、上昇の途中の読みで壁の距離を出し直して同じ上昇で 2 回発火した)。

- `offset.yaml` の `wall_off_edge_*`。`edge_enable` 0 = 検知してログに出すだけ、1 = 旋回位置に使う(従来の判定は 45° が `edge_fallback_dist` = 60mm を越えたときだけの保険)。
- サンプルの位置は `global_pos.dist`(S3 でエンコーダーを読んだ時刻の位置)+ 速度 × (読んだ時刻 − エンコーダーの時刻)。距離は `sensor_gain.l45/r45`。前のサンプルから 0.25mm 未満のサンプルは捨てる(低速でバッファ 128 点が 32mm 以上を覚えるように)。
- ホスト検証は `tests/wall_edge_host/`(Python 版と 713 件で最初の発火が一致。引数 `all` で全発火を出す)。ログ列 `edge_seq_l/r`・`edge_lag_l/r`・`edge_lvl_l/r`、`wo_dl0..3` / `wo_dr0..3`(wo の生値をダンプ時に距離にしたもの [mm]、無効は 0)。
- 実機(20260930_011650〜011852、v≈2200): Core1 の計算は `pln_t_sensor` で直進 +12us・WALL_OFF 中 +25〜30us、WALL_OFF 中の `pln_margin_us` は最小 70us。発火 tick・基準位置・壁の距離はオフラインの再生と一致(差 0.04mm 以内)。
- 壁切れのあとの直進(SLA_FRONT_STR)は、壁切れの判定で読んだ位置 + 1 tick 分の走行(`|ego_in.v|·dt`)から距離を数える(`param_straight_t::start_x`、`WallOffController::set_front_start()`、2026-09-30)。以前は `go_straight()` が送信後の最初の tick で読んだ位置を基準にしていて、平均は同じ(838 件で差 +0.025mm)だが、Core0 が tick を取りこぼすとその回だけ 1 tick(2.2mm)ずれた。`take_wall_edge` / `take_pillar_trough` は lag を計算した位置をそのまま使う。壁切れ → 旋回の始まりまでで tick の刻みが位置に入る所は、これと `sla_start_align` でなくなった。
- あわせて `MotionPlanning::wall_off_recheck_ok()`(SLA_FRONT_STR の後の再確認)を、入るときに壁があったなら入るときの距離 + `wall_off_recheck_delta`(5mm)以上遠のいたことも求める形にした。

#### タイム最小の経路探索 (`TimePathPlanner`、2026-09-29)

最短走行(`path_run()` で右を選んだとき)と `sim_run_time()` の経路は `MainTask::create_fast_path()` が作る。まず `TimePathPlanner::solve()` を使い、使えなかったときだけ下の「PathCreator の経路最適化」(重みパターン 1〜5 の比較)へ戻る。ボタンで中断したときは単純な経路(`path_create()` そのまま)。

- 経路を「直線 + ターン」の区間の列として、区間を辺にした最短経路問題を解く。辺の重みは `PathCreator::calc_segment_time()` = `calc_goal_time()` の 1 区間ぶんそのものなので、求めた経路の `calc_goal_time()` が、作れる経路の中で最小になる。重みパターンも分岐の総当たりも使わない。
- 節点は「ターンを終えた位置・向き・直進か斜めか・そのときの速度・次の区間への約束」。約束は `calc_segment_time()` の先読み(ターンの前後に直線があり、次が Large / Orval なら速いターン `map_fast`)を辺の重みへ入れるためのもの。
- 辺の作り方は `convert_large_path()` / `diagonalPath()` の規則の写し(ターン 1 個 = Large、同じ向き 2 個 = Orval、交互に続く組 = 斜めで入口・出口は Dia45 か Dia135、途中の同じ向き 2 個 = Dia90)。同じ向き 3 個(その場で 270° 回る形)は扱わない。**変換の規則を変えたらここも合わせる。**
- 出力は `path_create()` と同じ素の経路(`pc->path_s` / `path_t` / `path_size`)。呼んだ側が `convert_large_path()` / `diagonalPath()` を続けるので、走行側は変わらない。
- メモリは `solve()` の間だけ確保する。節点の上限は空きメモリ(ヒープの上端から続いている分 − 24 KB)から 1024〜8192 の 2 の累乗で決め、1 個あたり 20 バイト + 固定 17 KB(上限 4096 で約 97 KB)。足りない・あふれたときは `NoMemory` / `Overflow` を返して従来の方法へ戻る。大会迷路 27 本 × 5 モードの実績は節点最大 3333〜3601、ヒープ最大 546、覚えた区間最大 960。
- 実行するとコンソールに `[time_path] ok 3.419 s calc 35 ms nodes 2046/4096 heap 441 edges 11189 seg 738 mem 97 KB free 140 KB` の形で出る。`ok` 以外(`no memory` / `overflow` 等)なら従来の方法で作っている。
- ホストでの確認は `python3 tools/path_sim/check_time_path.py`(求めた経路を変換と `calc_goal_time()` に通して同じタイムになるか、従来の方法より遅くならないか。`--cap 4096` で機体の上限を模す)。検討の経緯と数字は `tools/path_sim/experiments/README.md`。
- 結果: 105 ケースで従来より速い経路が 11 件(最大 77 ms)、遅い経路は 0 件。計算は PC 上で 1 件 2 ms 前後(従来は 120〜280 ms)。**実機での計算時間・空きメモリは未計測。**

`calc_goal_time()` は `calc_segment_time()` を区間ごとに呼んで積み上げる形にしてある(速度・加速度の選び方は `calc_segment_time()` だけを直せば、見積もりと経路探索の両方に入る)。あわせて**最後の直線(最後のターン〜ゴール)も合計に入れた**(以前は Finish の直線を足した直後に break しており、返す値に入っていなかった。21 迷路で平均 45 ms、最大 514 ms 短く出ていた)。

`convert_large_path()` / `diagonalPath()` の `while (path_t[i] != 0)` は、終端が 255 で 0 が入らないため配列の先(他のメモリ)まで読み、値しだいで配列の外へ書いていた(ホストの AddressSanitizer で検出。ときどき異常終了する原因)。配列の中だけを見る形に直した(配列の中の結果は変わらない。105 ケースで変更前と一致)。

#### PathCreator の経路最適化(従来の方法。いまは予備)

`timebase_path_create()` では `other_route_map` に候補分岐マスを記録し、`exec_param` の1〜5パターンで `path_create_with_change()` を試して最短タイムの経路を `path_set_map` (priority_queue) から取得します。

近似コスト(`updateVectorMap`)は区画の手数を数えるだけでターンの種類(Dia135 は遅い・Large は速い等)を見ないため、手数は多いが実際は速い経路が「下り」にならない。`checkOtherRoot()` は分岐候補を「今の値 + `OTHER_ROUTE_MARGIN_CELLS`(0.5 区画)以下」で集める(2026-09-29。以前は「今の値より小さい」で同点も落ちていた)。幅は重みパターンごとの 1 区画のコスト `MazeSolverBaseLgc::cell_cost()` で測る(倍率だとパターンやゴールからの距離で幅が変わり候補が増えすぎる)。path_sim で 19 迷路 × 3 モード: 14 件速く(最大 −0.158 s)、1 件 +0.006 s。

`timebase_path_create()` が候補を 1 つ試すたびに `path_create()` が近似コストの表(`updateVectorMap`、迷路全体の Dijkstra)を作り直していたのをやめ、候補の評価中は直前の `path_create()` で作った表を使い回す(`reuse_vector_map`、2026-09-29)。前提は「同じ重みパターンで `path_create()` してから `timebase_path_create()` を呼ぶ」(`path_run` / `sim_run_time` とも)。57 件で経路・タイムは完全一致、PC 上の計算時間は候補を広げる前の約 1.4 倍(使い回し前は約 3.5 倍)。

`go_straight_dummy()`(`calc_goal_time` の直線を 1 ms 刻みで積み上げる)は、無限ループ対策のボタン確認を計算上 5 ms(ループ 5 回)ごとにしている(以前は毎回で、1 回の path_run で数千万回 GPIO を読んでいた)。加えて、速度が 0 以下のまま距離が残ったら、ボタンを待たずに失敗(ボタンと同じ 10000)を返す(パラメータの抜けで速度 0 だと人が押すまで止まらなかった。path_sim で確認)。どちらも正常な計算の結果は変わらない(57 件と探索 3 本で一致)。

ホスト版: `tools/path_sim`(Param Console の迷路タブの「経路」)が `logic.cpp` / `path_creator.cpp` / `time_path_planner.cpp` / `trajectory_creator.cpp` をそのまま PC でビルドして使う。MainTask の読込関数(`load_slalom_param` ほか)は `tools/path_sim/host_common.hpp`、`path_run()` の経路部分と `create_fast_path()` ほかは `tools/path_sim/main.cpp` に写しがあるので、**それらを変えたら main.cpp も合わせる**(詳細は `tools/param_tuner/webapp/CLAUDE.md` の「経路」)。

### 吸引 ESC（ESCape32 / DShot）

吸引モーターは外付け ESC（ESCape32 ファームウェア）が駆動します。ファームウェア側は
「スロットル指令を出すだけ」で、コミュテーションは ESC が担当します。

| クラス | ファイル | 役割 |
|--------|---------|------|
| `SuctionEscDshotActuator` | `include/planning/suction_esc_dshot_actuator.hpp` | 標準(非反転) DShot600 出力（既定） |
| `SuctionEscActuator` | `include/planning/suction_esc_actuator.hpp` | RC サーボ標準 PWM 出力（AM32 時代の経路、フォールバック） |
| `DshotTx` | `include/driver/dshot_tx.hpp`, `pio/dshot_tx_std.pio` | PIO + DMA による DShot フレームの自律反復送出 |

`include/planning/suction_esc.hpp` の `SuctionEsc` 型エイリアスがどちらを使うかを決め、
切り替えは `define.hpp` の `SUCTION_ESC_USE_DSHOT`（1=DShot / 0=サーボ PWM）で行います。

- **自律出力**: DMA が固定アドレス（`frame_word_`）を ENDLESS で読み PIO TX FIFO へ流すため、
  CPU が止まっていても最後のフレームが送出され続けます（`SUCTION_ESC_DSHOT_FRAME_HZ` = 2.5kHz）。
  `apply_us()` は 32bit ストア 1 回だけで、Core1 の 1kHz IRQ から呼べます。
- **パルス幅(us)互換**: `apply_us()` が受け取る 1000〜2000us は ESCape32 のサーボ PWM 入力と
  同じ式（`throt_min+50` のデッドバンド付き線形）で DShot スロットル値へ変換されます。
  そのため `system.yaml` の `suction_duty` 系（us 単位）はそのまま使えます。
- **回転方向**: `system.yaml` の `test.suction_dshot_reverse`（0/1）を、USB コマンド `DSHOTDIR`
  または テストモード 27 で ESC へ書き込みます（DShot コマンド 7/8 + 12 で ESC のフラッシュへ
  永続化）。起動時には送りません（ESC 通電＋DShot ロック待ちで 1.5 秒以上かかるため）。
  ESCape32 はコマンドを「telemetry 要求 bit が立っている・モーター停止中・同一コマンドが
  6 フレーム連続」の条件でのみ受け付けます。
- **テレメトリ**: 未実装。双方向 DShot 用の `DshotBidir`（`pio/dshot_bidir.pio`）は未結線・未検証で、
  受信側のビット周期と GCR のトグル復号に既知の誤りがあります（ヘッダーのコメント参照）。

### LoggingTask (`include/logging/logging_task.hpp`)

PSRAM バンプアロケータ (`psram_heap::alloc`) を使い、`std::vector<LogEntry, PsramAllocator<LogEntry>>` にデータを蓄積。

- Core1 IRQ 内から `append_from_irq()` を呼ぶ（active_ が false なら no-op）
- Core0 から `start()` / `stop()` / `dump_csv()` を呼ぶ
- デフォルト最大 60,000 サンプル（1kHz × 60 秒）

### SPI バス共有

SPI1 のみ使用。全デバイスが MISO=12 / CLK=14 / MOSI=15 を共有し、CS で切り替え:

| バス  | デバイス | ピン |
|-------|---------|------|
| SPI1 | ASM330LHH ジャイロ (mode 3) | CS_gyro=13 |
| SPI1 | AS5147P 右エンコーダ (mode 1) | CS_enc_r=17 |
| SPI1 | AS5147P 左エンコーダ (mode 1) | CS_enc_l=1 |
| SPI1 | ADS7042I バッテリADC (mode 0) | CS_bat=3 |

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
| `/enc_lut.hf` | `input_param_t` | エンコーダ角度補正テーブル (enc_lut_enable / enc_lut_l / enc_lut_r 各64点) |

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
| `AM32READ` / `AM32WRITE` | AM32 ESC 設定の読み出し/書き込み（AM32 ファームウェア搭載 ESC 用） |
| `DSHOTDIR` | `system.yaml` の `test.suction_dshot_reverse` を吸引 ESC へ書き込み、ESC のフラッシュへ永続化 |

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
  - 実 ODR はチップごとに公称値からずれる(内部クロックの製造ばらつき、本機は INTERNAL_FREQ_FINE=35 で +5.25% = 3508.5Hz)。`setup()` が工場校正値 INTERNAL_FREQ_FINE を読んで `gyro_odr_hz()` / `accel_odr_hz()` / `gyro_sample_period_us()` に個体の値を入れる。サンプル数×周期で積分する処理(FIFO 等)は公称の 3333Hz を決め打ちせずこれを使うこと。
  - 1kHz で最新値を 1 点読みしているため、ODR/2 未満の高周波も折り返して見える(w_lp に常在する 195/313Hz の対は約 1.19kHz の振動の折り返しで、2 本の和 = 実 ODR − 3000)。
- **AS5147P**: 磁気エンコーダ、SPI mode 1。`init()` は初期化済み SPI バス + CS ピンのみ受け取る。14bit 角度値 [0–16383] を返す。
- **ADS7042**: バッテリ電圧 ADC、SPI mode 0。結果 = `(rx >> 2) & 0x0FFF`。

### エンコーダ角度補正 (enc_lut)

磁石の芯ずれ・タイヤの振れによる角度依存誤差を、`SensingTask::correct_enc()` が `read_angle()` の直後に引きます(生角度の上位 6bit で 64 点テーブルを引き線形補間、補正後 = 生角度 − table、単位 count)。

- テーブルは `tools/param_tuner/enc_lut_fit.py` が低速直進ログ(v=400 前後、機体を置き直しながら 8 本以上)から同定し、`profile/hf/enc_lut.yaml` を生成します(手で編集しない)。機体へは `/enc_lut.hf` として送られ、ファイルが無い・点数が 64 でない場合は補正なしで動きます。
- 同定は左右差とジャイロだけを使います(v_c が速度 PID へ戻るため、片輪ごとの平滑化残差では左右の誤差が混ざる)。超信地は 4 輪のスクラブで使えず、ツールが自動で除外します。
- ログの `v_l_enc` / `v_r_enc` は補正前の生角度(`encoder.left_raw/right_raw`)なので、補正の有効/無効に関係なく校正し直せます。ファームの適用確認は `enc_lut_fit.py --check`。
- 磁石・タイヤを付け直したら取り直してください。

### 車輪速度を planning の時刻まで先読みする (enc_v_lead)

1ms の位置差分で求めた車輪速度はその 1ms の中央(読んだ時刻の 0.5ms 前)の値で、さらに PlanningTask はそれを後の tick で使います(枠分け前は約 600〜800us 後、枠分け後は約 80〜120us 後)。枠分け前の planning の時刻を基準にすると、加減速中は直進 −43mm/s、旋回の出入り −56mm/s 遅れていました。`hardware.yaml` の `enc_v_lead: 1` で、制御と推定に渡す `ego.v_l/v_r` に「車輪ごとの目標加速度(`ego_in.accl` ± 目標角加速度 × `tire_tread`/2) × (dt/2 + 読んでから次の planning tick までの時間)」を足します(次の tick の時刻は `PlanningTask::next_tick_us()`)。偏りは +1〜4mm/s になり、目標加速度はノイズが無いので巡航中のノイズは増えません。

- 同じ時刻のずれはジャイロにもあります(`fifo_mode` のどれも、読んだ時刻の値を約 588us 後に planning が使う)。オフラインでは mode 4 に目標角加速度 × 588us を足すと、旋回の出入りの偏りが −1.8 → −0.08 rad/s になりました(`gyro_param.fifo_plan_lead` として実装)。

- 距離(`ego_in.dist` / `global_pos.dist`)は位置の差分そのもの `ego.v_l_dist/v_r_dist` で積分し、先読みは入れません(入れると加減速のたびに速度変化 × dt/2 ずれる)。エンコーダー読み取り失敗の判定も `v_*_dist` で行います。
- `kim` の位置推定は `v_kf × dt` を積分しているので、先読みした速度が KF を通って入ります(加減速のたびに最大で速度変化 × 0.5ms、速度が戻れば消える)。
- ログの `v_l` / `v_r` は先読み後の値です。差分の速度は生角度 `v_l_enc` / `v_r_enc` から再計算できます。FIFO のワード数(3/4)でエンコーダーを読む時刻が約 6us 前後するので、オフラインで 1ms 固定で割ると約 12mm/s ずれます(ファームは実測 dt で割っている)。

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
