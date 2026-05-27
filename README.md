# ExiaIgnis

RP2350 (Raspberry Pi Pico2) 向けマイクロマウスファームウェア。  
センシング・自己位置推定・軌道生成・PID 制御・モーター出力・迷路探索・最速走行経路生成まで一貫して実装しています。

---

## ハードウェア構成

| 項目 | 内容 |
|------|------|
| MCU | RP2350 (Raspberry Pi Pico2) |
| ジャイロ | ASM330LHH (SPI1 / mode 3) |
| エンコーダ（右） | AS5147P (SPI1 / mode 1) |
| エンコーダ（左） | AS5147P (SPI0 / mode 1) |
| バッテリ ADC | ADS7042I (SPI0 / mode 0) |
| 赤外線センサー | R90 / L90 / R45×2 / L45×2 (GPIO ADC) |
| ストレージ | フラッシュ末尾 256KB (LittleFS) |
| PSRAM | ログバッファ用バンプアロケータ |

---

## ビルド

### 前提

- Raspberry Pi Pico SDK 2.2.0
- CMake 3.13+
- ARM GCC toolchain (15.2)

### 手順

```bash
# 初回またはCMakeLists.txt変更後のみ
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release

# ビルド
./compile.sh
# または
cmake --build build -- -j$(nproc)
```

初回ビルド時は CMake が cJSON (v1.7.18) と LittleFS (v2.5.0) を自動ダウンロードします（ネットワーク接続が必要）。

### フラッシュ

BOOTSEL ボタンを押しながら USB 接続してから:

```bash
./flash.sh
```

---

## シリアルモニター

```bash
./serial_monitor.sh                    # /dev/ttyACM0 @ 115200
./serial_monitor.sh /dev/ttyACM0 115200
```

stdio は USB CDC のみ（UART 無効）。

---

## アーキテクチャ概要

### マルチコア構成

```
Core0: MainTask
  - パラメータロード / UI / ボタン処理 / printf 出力
  - 探索走行・最速走行のオーケストレーション

Core1: SensingTask (TIMER0 IRQ) + PlanningTask (TIMER1 IRQ)
  - 1kHz 定周期センシング
  - EgoEstimator → SensorProcessor → TrajectoryGenerator → ControlLaw → MotorActuator
```

TinyUSB がコア0 固定のため `printf` はコア0 からのみ呼べます。

### レイヤー構成

```
MainTask (src/main/)
  └─ SearchController (src/search/)        ← 探索走行
       ├─ MazeSolverBaseLgc               ← 迷路マップ・歩数マップ・ベクター距離マップ
       └─ Adachi                           ← 足立法ベース次移動方向決定
  └─ MotionPlanning (src/action/)          ← 直進・スラローム・ピボット実行
       ├─ PathCreator                      ← 最速経路生成 (path_s / path_t 配列)
       ├─ TrajectoryCreator               ← path_t 値 → TurnType 変換
       └─ WallOffController               ← 壁補正制御
  └─ PlanningTask (src/planning/)          ← 1kHz IRQ 制御ループ
       ├─ EgoEstimator                    ← 速度・角度推定 (カルマンフィルタ)
       ├─ SensorProcessor                 ← センサー値 → mm 距離変換
       ├─ TrajectoryGenerator             ← 台形速度プロファイル
       ├─ ControlLaw                      ← PID 制御・デューティ計算
       └─ MotorActuator                   ← PWM 出力
  └─ SensingTask (src/sensing_task.cpp)    ← センサー読み取り IRQ
  └─ LoggingTask (src/logging/)            ← PSRAM ログバッファ
```

---

## パラメータ設定

パラメータは LittleFS 上の JSON ファイルで管理します。  
起動後のボタン待ち状態（LED 点灯中）に USB シリアル経由で書き込めます:

```bash
# ファイル書き込み
echo "sensor.json@{...json...}" > /dev/ttyACM0

# ファイル確認
echo "LIST" > /dev/ttyACM0
echo "READ:sensor.json" > /dev/ttyACM0
```

| ファイル | 内容 |
|---------|------|
| `/config.json` | センサー周期・LED 整定時間 |
| `/hardware.json` | タイヤ径・ギア比・バッテリゲイン |
| `/sensor.json` | センサーゲイン・参照値 |
| `/offset.json` | クリアアングル等オフセット |
| `/system.json` | 動作モード・迷路サイズ・ゴール座標 |
| `/exec.json` | 最速走行パラメータインデックスリスト |
| `/profiles.hf` / `/profiles.cl` | TurnType ごとのスラロームパラメータファイルインデックス |
| `/vel_prof.hf` / `/vel_prof.cl` | 直線速度プロファイル |

---

## 動作モード

### テストモード (`system.json` の `user_mode != 0`)

センサー確認・ピボットターン調整・スラローム調整など個別テストを実行します。

### 本走行モード (`user_mode == 0`)

エンコーダー操作でサブモードを選択、長押しで決定:

| 番号 | 動作 |
|------|------|
| 0 | 探索走行（全探索） |
| 1 | 片側探索 / 帰還探索 |
| 2〜N+1 | 最速走行（exec.json の各プロファイル） |
| N+2 | その場旋回（keep_pivot） |
| N+3 | 吸引テスト |
| N+4 | 全プロファイルのタイム計算 |
| N+5 | 迷路データ消去 |

---

## 開発者向け情報

詳細なアーキテクチャ・データ構造・IRQ 設計・enum 一覧は [CLAUDE.md](CLAUDE.md) を参照してください。
