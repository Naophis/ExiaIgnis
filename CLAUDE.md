# CLAUDE.md

このファイルは、リポジトリ内のコードを扱う Claude Code (claude.ai/code) に対してガイダンスを提供します。

## プロジェクト概要

ExiaIgnis は RP2350 (Raspberry Pi Pico2) 向けマウスロボットのファームウェアです。センシング処理をコア1のハードウェアタイマー IRQ で実行し、コア0では UI・ボタン入力・シリアル出力を担います。

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

- **コア0** (`src/ExiaIgnis.cpp`): 初期化・UI・ボタン処理・シリアル printf 表示
- **コア1** (`src/sensing_task.cpp`): ハードウェアタイマー IRQ 駆動のセンサー読み取りループ

コア1 → コア0 のデータ受け渡しは `volatile SensingTask::Data data` + `volatile bool data_ready` をポーリングで行います。

### センシングタスク IRQ 構造

TIMER0 のハードウェアアラームを2本使用（alarm_pool オーバーヘッドなし）:
- **Alarm 2** (`timer_a_irq_handler`): `interval_us_` 周期（デフォルト1kHz）の定周期タイマー。Alarm 1 を即座にスケジュール。
- **Alarm 1** (`timer_b_irq_handler`): センサー読み取りシーケンス全体（ADC LED → diff 計算 → ジャイロ/エンコーダ/バッテリ + タイムスタンプ）。

ドリフト防止のため絶対時刻方式を採用: `next_alarm_a_ += interval_us_`

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

### 設定システム（ConfigLoader）

`ConfigLoader`（`include/config_loader.hpp`, `src/config_loader.cpp`）は起動時にフラッシュ末尾 256KB の LittleFS から `/config.json` を読み込みます。初回起動時はフォーマットしてデフォルト値を自動生成します。

使い方:
```cpp
ConfigLoader::init();  // multicore_launch_core1 より前に呼ぶこと
int val = ConfigLoader::get_int("sensing.led_settle_us", 13);
```

デフォルト設定:
```json
{"sensing": {"led_settle_us": 13, "interval_us": 1000}}
```

### ドライバークラス

センサードライバーは `include/driver/` + `src/driver/` に配置:

- **ASM330LHH**: ジャイロ、SPI mode 3。`init()` で SPI バスを初期化。`setup()` でソフトウェアリセット + 設定シーケンスを実行。Z 軸角速度のみ取得。
- **AS5147P**: 磁気エンコーダ、SPI mode 1。`init()` は初期化済み SPI バス + CS ピンのみ受け取る。14bit 角度値 [0–16383] を返す。
- **ADS7042**: バッテリ電圧 ADC、SPI mode 0。結果 = `(rx >> 2) & 0x0FFF`。

### データ構造

`SensingTask::Data` は以下を保持: タイミング（`dt_us`, `sense_duration_us`）、センサー値（`SensorDark`, `SensorLit`, `SensorDiff`）、ジャイロ（`gz` + タイムスタンプ）、エンコーダ（`enc_r`, `enc_l` + タイムスタンプ）、`battery`。

**注意:** `volatile` からコピーするため `Data(const volatile Data &o)` コピーコンストラクタを明示実装しています。`Data` にフィールドを追加した場合は必ずコピーコンストラクタも更新してください。デフォルトコピーコンストラクタは `volatile` を扱えません。

タイムスタンプの記録パターン（各センサー共通）:
```cpp
self->data.gz_ts_z = self->data.gz_ts;   // 前回時刻を退避
self->data.gz_ts   = time_us_64();        // 今回時刻を記録
self->data.gz      = gyro_.read_gyro_z();
self->data.gz_dt   = self->data.gz_ts_z ? (self->data.gz_ts - self->data.gz_ts_z) : 0;
```
`ts_z ? ... : 0` のガードは初回実行時の巨大な dt を防ぎます。

## USB シリアル出力

stdio は USB のみ（UART 無効）。ANSI エスケープシーケンスで画面をリフレッシュして表示:
```
=== ExiaIgnis sensor monitor ===
[timing]  dt=... sense=... gz->encR: ... encR->encL: ...
[sensor]  L90=... L45 2=... both=... 1=... R45 1=... both=... 2=... R90=...
[motion]  gz=... encL=... encR=...
[power]   bat=...
```
表示順は左から右（L90 → L45 → R45 → R90）。
