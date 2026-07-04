# LittleFS パラメータ JSON スキーマ

フラッシュ末尾 256KB (LittleFS) に格納する設定ファイルの全フィールド定義。  
USB シリアルコマンド `filename@json_content\n` で書き込み、起動時に自動ロードされる。

---

## ファイル一覧

| ファイル | 読み込み先 | 説明 |
|---------|-----------|------|
| `/config.json` | `ConfigLoader` | センシングタイミング設定 |
| `/hardware.json` | `input_param_t` | 機体定数・PID・FF ゲイン |
| `/sensor.json` | `input_param_t` | センサー参照値・変換ゲイン |
| `/offset.json` | `input_param_t` | 壁補正距離・Kanayama・軌道テーブル |
| `/system.json` | `system_t` | ゴール・迷路サイズ・テストモード |
| `/exec.json` | `exec_pram_t[]` | 実行プロファイルインデックスリスト |
| `/profiles.hf` (or `.cl`) | `turn_param_profile_t` | スラロームファイルリストと TurnType→インデックスマップ |
| `/vel_prof.hf` (or `.cl`) | `straight_param_t` | 直線走行プロファイルテーブル |
| `/<slalom_file>` | `slalom_param2_t` | TurnType ごとのスラロームパラメータ |

`sys_.hf_cl == 0` → `.hf` / `== 1` → `.cl` を使用。

---

## /config.json

```json
{
  "sensing": {
    "led_settle_us": 13,
    "interval_us": 1000
  }
}
```

| キー | 型 | デフォルト | 説明 |
|------|-----|-----------|------|
| `sensing.led_settle_us` | int | 13 | LED 発光後の安定待ち時間 [µs] |
| `sensing.interval_us` | int | 1000 | センシング周期 [µs]（= 1 kHz） |

---

## /system.json

```json
{
  "goals": [[7, 7], [8, 8]],
  "maze_size": 16,
  "mode": 0,
  "circuit_mode": 0,
  "hf_cl": 0,
  "test": { ... }
}
```

| キー | 型 | 説明 |
|------|-----|------|
| `goals` | `[[x,y], ...]` | ゴールマスの座標リスト（左下原点、セル単位） |
| `maze_size` | int | 迷路の一辺サイズ（例: 16 = 16×16） |
| `mode` | int | 起動モード（0=探索、1=テスト） |
| `circuit_mode` | int | 1 のとき `load_circuit_path()` を使用 |
| `hf_cl` | int | 0=`.hf` ファイル、1=`.cl` ファイルを使用 |
| `test` | object | テストモードパラメータ（下記参照） |

### test オブジェクト

| キー | 型 | 説明 |
|------|-----|------|
| `v_max` | float | 最大速度 [m/s] |
| `end_v` | float | 終端速度 [m/s] |
| `accl` | float | 加速度 [m/s²] |
| `decel` | float | 減速度 [m/s²] |
| `dia_accl` | float | 斜め加速度 [m/s²] |
| `dia_decel` | float | 斜め減速度 [m/s²] |
| `dist` | float | 走行距離 [mm] |
| `w_max` | float | 最大角速度 [rad/s] |
| `alpha` | float | 角加速度 [rad/s²] |
| `ang` | float | 旋回角 [rad] |
| `suction_active` | int | 吸引有効フラグ (0/1/2) |
| `suction_duty` | float | 吸引デューティ [%] |
| `suction_duty_low` | float | 低吸引デューティ [%] |
| `suction_duty_burst` | float | バースト吸引デューティ |
| `suction_gain` | float | 吸引ゲイン |
| `sla_dist` | float | スラローム走行距離 |
| `file_idx` | int | スラロームファイルインデックス |
| `sla_type` | int | スラロームタイプ |
| `turn_times` | int | 旋回回数 |
| `search_mode` | int | 探索モード番号 |

---

## /exec.json

```json
[
  {"fast_idx": 0, "normal_idx": 1, "slow_idx": 2},
  {"fast_idx": 0, "normal_idx": 2, "slow_idx": 2}
]
```

各要素が1つの `exec_pram_t`。`run_main_mode()` の `mode_num` で選択される。

| キー | 型 | 説明 |
|------|-----|------|
| `fast_idx` | int | `profiles.hf` の `profile_idx` インデックス（高速用） |
| `normal_idx` | int | 同上（通常用） |
| `slow_idx` | int | 同上（低速用） |

---

## /profiles.hf / /profiles.cl

```json
{
  "list": ["sla_fast.json", "sla_mid.json", "sla_slow.json"],
  "profile_idx_size": 2,
  "profile_idx": [
    {
      "run_param": 0,
      "suction": 1,
      "normal": 0,
      "large": 0,
      "orval": 1,
      "dia45": 1,
      "dia45_2": 1,
      "dia135": 1,
      "dia135_2": 1,
      "dia90": 1
    }
  ]
}
```

| キー | 型 | 説明 |
|------|-----|------|
| `list` | `string[]` | スラロームファイル名リスト（`/` を除いたファイル名） |
| `profile_idx_size` | int | `profile_idx` の要素数 |
| `profile_idx[].run_param` | int | `list` へのインデックス（直線パラメータ用） |
| `profile_idx[].suction` | int | 吸引モード (0=OFF, 1=通常, 2=バースト) |
| `profile_idx[].normal` | int | Normal ターン用ファイルインデックス |
| `profile_idx[].large` | int | Large ターン用 |
| `profile_idx[].orval` | int | Orval ターン用 |
| `profile_idx[].dia45` | int | Dia45 ターン用 |
| `profile_idx[].dia45_2` | int | Dia45_2 ターン用 |
| `profile_idx[].dia135` | int | Dia135 ターン用 |
| `profile_idx[].dia135_2` | int | Dia135_2 ターン用 |
| `profile_idx[].dia90` | int | Dia90 ターン用 |

---

## /vel_prof.hf / /vel_prof.cl

```json
{
  "v_prof": [
    {
      "Search":     {"v": 0.5, "a": 3.0, "d": 3.0, "w0": 4.0, "w1": 0.0, "a2": 30.0},
      "FastRun":    {"v": 2.0, "a": 8.0, "d": 8.0, "w0": 8.0, "w1": 0.0, "a2": 80.0},
      "FastRunDia": {"v": 1.5, "a": 6.0, "d": 6.0, "w0": 6.0, "w1": 0.0, "a2": 60.0}
    }
  ]
}
```

`v_prof[i]` が1プロファイル。`profiles` の `run_param` インデックスで選択。

### straight_param_t フィールド（各 StraightType エントリ）

| キー | 別名 | 型 | 単位 | 説明 |
|------|------|-----|------|------|
| `v_max` | `v` | float | m/s | 最大直線速度 |
| `accl` | `a` | float | m/s² | 加速度 |
| `decel` | `d` | float | m/s² | 減速度 |
| `w_max` | `w0` | float | rad/s | 最大角速度 |
| `w_end` | `w1` | float | rad/s | 終端角速度 |
| `alpha` | `a2` | float | rad/s² | 角加速度 |

---

## /<slalom_file>

スラロームパラメータファイル。TurnType 名をキーとして持つ。

```json
{
  "Normal": {
    "v": 1.5, "end_v": 1.5, "ang": 90.0, "rad": 45.0, "rad2": 0.0, "pow_n": 4,
    "front": {"right": 5.0, "left": 5.0},
    "back":  {"right": 5.0, "left": 5.0}
  },
  "Large":   { ... },
  "Orval":   { ... },
  "Dia45":   { ... },
  "Dia45_2": { ... },
  "Dia135":  { ... },
  "Dia135_2":{ ... },
  "Dia90":   { ... }
}
```

### slalom_param2_t フィールド

| キー | 型 | 単位 | 説明 |
|------|-----|------|------|
| `v` | float | m/s | スラローム速度 |
| `end_v` | float | m/s | スラローム終端速度 |
| `ang` | float | **度** | 旋回角（内部でラジアンに変換） |
| `rad` | float | mm | クロソイド半径 |
| `rad2` | float | mm | 第2クロソイド半径（Orval のみ） |
| `pow_n` | int | — | クロソイド次数 (2/4/6)、Et 係数に影響 |
| `front.right` | float | mm | ターン進入前の右オフセット補正距離 |
| `front.left` | float | mm | ターン進入前の左オフセット補正距離 |
| `back.right` | float | mm | ターン脱出後の右オフセット補正距離 |
| `back.left` | float | mm | ターン脱出後の左オフセット補正距離 |

`time` は `(rad * ang_rad) / (2 * v * Et)` でコードが自動計算するため JSON には不要。

---

## /hardware.json

機体定数・PID ゲイン・FF パラメータを格納。`input_param_t` に展開される。

### 機体定数

| キー | 型 | デフォルト | 単位 | 説明 |
|------|-----|-----------|------|------|
| `tire` | float | 12 | mm | タイヤ半径 |
| `tire2` | float | 12 | mm | タイヤ半径2（別系統用） |
| `tread` / `tire_tread` | float | 38 | mm | トレッド幅 |
| `gear_a` | float | 37 | — | ギア歯数 A |
| `gear_b` | float | 8 | — | ギア歯数 B |
| `max_duty` | float | 99 | % | PWM 最大デューティ |
| `min_duty` | float | 8 | % | PWM 最小デューティ |
| `battery_gain` | float | 3.3 | — | バッテリー電圧換算ゲイン |
| `cell` | float | 90 | mm | 1セル長さ |
| `cell2` | float | 90 | mm | 1セル長さ（別用途） |
| `Ke` | float | 0 | — | 逆起電力定数 |
| `Km` | float | 0 | — | トルク定数 |
| `Resist` | float | 0 | Ω | モーター抵抗 |
| `Mass` | float | 0 | kg | 機体質量 |
| `Lm` | float | 0 | H | モーターインダクタンス |
| `coulomb_friction` | float | 0 | — | クーロン摩擦 |
| `viscous_friction` | float | 0 | — | 粘性摩擦 |

### フィードフォワード

| キー | 型 | 説明 |
|------|-----|------|
| `FF_front` | int | 前進 FF 有効フラグ |
| `FF_roll` | int | 旋回 FF 有効フラグ |
| `FF_keV` | int | 逆起電力 FF 有効フラグ |
| `ff_front_gain_14` | float | 前進 FF ゲイン（第1〜4区間） |
| `ff_roll_gain_before` | float | 旋回前 FF ゲイン |
| `ff_roll_gain_after` | float | 旋回後 FF ゲイン |
| `ff_front_gain_decel` | float | 減速時前進 FF ゲイン |

### PID オブジェクト（共通フィールド）

以下のキーで複数の `pid_param_t` オブジェクトが存在する:  
`motor_pid` / `motor_pid2` / `motor_pid3` / `gyro_pid` /  
`str_ang_pid` / `str_ang_dia_pid` / `angle_pid` /  
`front_ctrl_roll_pid` / `front_ctrl_angle_pid` / `front_ctrl_dist_pid` /  
`front_ctrl_keep_angle_pid` / `sensor_pid_dia` /  
`motor_pid_gain_limitter` / `motor2_pid_gain_limitter` / `gyro_pid_gain_limitter`

| キー | 型 | 説明 |
|------|-----|------|
| `p` | float | 比例ゲイン |
| `i` | float | 積分ゲイン |
| `d` | float | 微分ゲイン |
| `b` | float | 積分フィルタ係数 |
| `c` | float | 微分フィルタ係数 |
| `mode` | int | PID モード選択 |
| `antiwindup` | int | アンチワインドアップ有効 (0/1) |
| `windup_gain` | float | ワインドアップ抑制ゲイン |
| `th` | float | ゲイン適用閾値 |
| `theta_gate` | float | 角度ゲート閾値 |
| `omega_gate` | float | 角速度ゲート閾値 |
| `mpc_q_ang` | float | MPC 角度コスト重み |
| `mpc_q_vel` | float | MPC 速度コスト重み |
| `mpc_b` | float | MPC 入力行列 |
| `mpc_r` | float | MPC 入力コスト |
| `mpc_horizon` | int | MPC ホライズン長 |

### カルマンフィルタノイズ

ネスト形式（推奨）または フラット形式 どちらも受け付ける。

```json
{
  "battery_kalman_config": {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.35},
  "encoder_kalman_config": {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.035},
  "w_kalman_config":       {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.035},
  "v_kalman_config":       {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.035},
  "ang_kalman_config":     {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.035},
| "dist_kalman_config":    {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.035},
  "pos_kalman_config":     {"init_cov": 0.95, "p_noise": 0.05, "m_noise": 0.035}
}
```

### 軸退化・センサーリミッタテーブル

```json
{
  "axel_degenerate_x":      [0.0, 0.5, 1.0],
  "axel_degenerate_y":      [1.0, 0.8, 0.5],
  "axel_degenerate_dia_x":  [0.0, 0.5, 1.0],
  "axel_degenerate_dia_y":  [1.0, 0.8, 0.5],
  "sensor_deg_limitter_v":      [0.5, 1.0, 2.0],
  "sensor_deg_limitter_str":    [10.0, 5.0, 2.0],
  "sensor_deg_limitter_dia":    [10.0, 5.0, 2.0],
  "sensor_deg_limitter_piller": [10.0, 5.0, 2.0]
}
```

`interp1d(x_table, y_table, v)` で速度に応じたゲインを補間する。

### その他 hardware.json フィールド

| キー | 型 | 説明 |
|------|-----|------|
| `motor_driver_type` | int | モータードライバー方式 (0=EN1_PH1) |
| `motor_r_cw_ccw_type` | int | 右モーター回転方向 |
| `motor_l_cw_ccw_type` | int | 左モーター回転方向 |
| `pivot_angle_90` | float | ピボット90°目標角 [deg] |
| `pivot_angle_180` | float | ピボット180°目標角 [deg] |
| `offset_start_dist` | float | スタート位置オフセット [mm] |
| `offset_start_dist_search` | float | 探索スタート位置オフセット [mm] |
| `long_run_offset_dist` | float | 長走行時距離オフセット [mm] |
| `fail_check` | object | フェイルセーフカウント閾値 |
| `fail_check.duty` | int | デューティ異常検出カウント |
| `fail_check.v` | int | 速度異常検出カウント |
| `fail_check.w` | int | 角速度異常検出カウント |
| `fail_check.ang` | int | 角度異常検出カウント |
| `fail_check.wall_off` | int | 壁オフ異常検出カウント |
| `enable_kalman_gyro` | int | ジャイロカルマンフィルタ有効 |
| `enable_kalman_encoder` | int | エンコーダカルマンフィルタ有効 |
| `enable_mpc` | int | MPC 有効 |

---

## /sensor.json

センサー参照値と距離変換ゲインを格納。`input_param_t.sen_ref_p` と `sensor_gain` に展開。

### 構造

```json
{
  "normal": {
    "ref":    { "right45": 80, "left45": 80, "right90": 200, "left90": 200, "front": 100,
                "kireme_r": 50, "kireme_l": 50, "kireme_r_fast": 60, "kireme_l_fast": 60,
                "kireme_r_wall_off": 30, "kireme_l_wall_off": 30,
                "kireme_r_wall_off2": 25, "kireme_l_wall_off2": 25 },
    "ref_search": { ... },
    "exist":  { ... },
    "expand": { "dist": 10, "right45": 5, "left45": 5, "right45_2": 3, "left45_2": 3 }
  },
  "normal2": { ... },
  "dia":     { ... },
  "search": {
    "ref":   { "front": 100, "right45": 80, "left45": 80, "right90": 200, "left90": 200,
               "kireme_r": 50, "kireme_l": 50, "offset_r": 5, "offset_l": 5,
               "front_ctrl": 80, "front_ctrl_th": 50 },
    "exist": { ... }
  },
  "gain": {
    "L90_near": [1200.0, 10.0],
    "L45":      [800.0, 5.0],
    "F":        [600.0, 8.0],
    "R45":      [800.0, 5.0],
    "R90_near": [1200.0, 10.0],
    "L90_far":  [5000.0, 50.0],
    "R90_far":  [5000.0, 50.0],
    "L90_mid":  [2500.0, 25.0],
    "R90_mid":  [2500.0, 25.0],
    "L45_2":    [800.0, 5.0],
    "R45_2":    [800.0, 5.0],
    "L45_3":    [800.0, 5.0],
    "R45_3":    [800.0, 5.0]
  }
}
```

### sen_ref_param3_t フィールド（ref / ref_search / exist 共通）

| キー | 説明 |
|------|------|
| `right45` / `left45` | 45° センサー壁検出基準値 |
| `right90` / `left90` | 90° センサー壁検出基準値 |
| `front` | 前方センサー基準値 |
| `kireme_r` / `kireme_l` | 切れ目検出閾値（通常） |
| `kireme_r_fast` / `kireme_l_fast` | 切れ目検出閾値（高速） |
| `kireme_r_wall_off` / `kireme_l_wall_off` | Wall-off 時の切れ目閾値 |

### sensor_gain 配列 [a, b]

距離変換式: `dist = a / (lp_value - b)`（SensorProcessor 参照）  
`[a, b]` 配列形式または `{"a": ..., "b": ...}` オブジェクト形式 どちらも可。

---

## /offset.json

壁補正距離パラメータ・Kanayama 制御・軌道インデックステーブルを格納。  
`input_param_t` の対応フィールドに展開される。

### 壁補正距離（wall_off_dist）

`wall_off_dist` オブジェクトとして、またはルート直下のフラットキーとして記述可能。

```json
{
  "wall_off_dist": {
    "left_str": 40.0,
    "right_str": 40.0,
    "left_dia": 35.0,
    "right_dia": 35.0,
    "exist_dist_l": 20.0,
    "exist_dist_r": 20.0,
    "search_wall_off_enable": true
  }
}
```

主要フィールド:

| キー (short) | キー (long) | 説明 |
|-------------|------------|------|
| `left_str` | `wall_off_hold_dist_str_l` | 直進時左 Wall-off 保持距離 [mm] |
| `right_str` | `wall_off_hold_dist_str_r` | 直進時右 Wall-off 保持距離 [mm] |
| `left_dia` | `wall_off_hold_dist_dia_l` | 斜め時左 Wall-off 保持距離 |
| `right_dia` | `wall_off_hold_dist_dia_r` | 斜め時右 Wall-off 保持距離 |
| `left_diff_th` | `wall_off_hold_div_th_l` | 距離差分閾値（左） |
| `exist_dist_l` | `wall_off_hold_exist_dist_l` | 壁存在判定距離（左） |
| `noexist_th_l` | `wall_off_hold_noexist_th_l` | 壁なし判定閾値（左） |
| `search_wall_off_enable` | 同左 | 探索中 Wall-off 有効フラグ |
| `ctrl_exist_wall_th_l` | 同左 | 壁制御存在閾値（左） |
| `diff_check_dist` | `wall_off_diff_check_dist` | 差分チェック距離 [mm] |

### Kanayama 制御

```json
{
  "kanayama": {
    "kx": 5.0, "ky": 5.0, "k_theta": 10.0,
    "enable": 0, "windup": 0, "windup_deg": 10.0
  }
}
```

### 軌道インデックステーブル

```json
{
  "trj_idx_v":   [0, 500, 1000, 2000],
  "trj_idx_val": [0, 10,  50,   100]
}
```

速度 [mm/s] に応じた軌道バッファインデックスを `interp1d` で補間する。

### その他 offset.json フィールド

| キー | 型 | 説明 |
|------|-----|------|
| `clear_angle` | float | 壁当てクリアリング角度 [rad] |
| `clear_dist_order` | float | クリアリング距離 [mm] |
| `front_dist_offset` | float | 前方距離オフセット [mm] |
| `front_dist_offset0〜4` | float | 状況別前方距離オフセット |
| `offset_after_turn_l` / `_r` | float | ターン後の位置オフセット（通常） |
| `offset_after_turn_l2` / `_r2` | float | ターン後の位置オフセット（補正2） |
| `offset_after_turn_dia_l` / `_r` | float | 斜めターン後のオフセット |
| `dia_wall_off_ref_l` / `_r` | float | 斜め走行中の壁補正参照値 |
| `wall_off_pass_dist` | float | 壁通過補正距離 [mm] |
| `wall_off_wait_dist` | float | 壁補正待機距離 [mm] |
| `sla_wall_ref_l` / `_r` | float | スラローム中壁補正参照値 [mm] |
| `lim_angle` | float | 角度リミット [rad] |
| `logging_time` | float | ログ記録時間 [s]（デフォルト 4.0） |
| `search_log_enable` | int | 探索中ログ有効 |
| `fast_log_enable` | int | 高速走行中ログ有効 |
