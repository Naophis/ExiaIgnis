# STRAIGHT走行におけるKanayama制御則を用いた壁追従カスケード

対象: `src/planning/control_law.cpp` (`ControlLaw`)。STRAIGHT(直線)走行中の壁センサーに基づく中央追従制御に、Kanayamaの経路追従則を導入した設計・実装の記録。

## 1. 背景: 軌道生成のオープンループ性

`TrajectoryGenerator::generate()`(`trajectory_generator.hpp`)は毎tick、`tgt_val->ego_in`(=計画自身の前回出力)を起点に`mpc_tgt_calc`を再実行する、**実測位置を一切参照しないオープンループの自己伝播シミュレーション**である。実測とのズレを軌道側にフィードバックする機構(`TrajectoryGenerator::calc_kanayama()`)はコード上存在するが、`offset.yaml`の`kanayama.enable: 0`かつ`trj_length: 1`により、SLALOMを含め現状すべてのモーションで実質無効になっている。

STRAIGHTの中央追従は、この計画軌道とは独立に、壁センサー誤差(`check_sen_error()`、mm単位、右壁に寄ると正)を直接フィードバックする`ControlLaw`内の反応的ループ(`str_ang_pid_fast`)が担っていた。今回の作業は、この壁センサー誤差を**Kanayama則を介して計画的に軌道へ合流させる**ことで、中央への戻り方を滑らかにすることを目的とする。

## 2. 既存の3段カスケード構造

`ControlLaw`はSTRAIGHT系で以下の3段カスケードを持つ(内側ほど高帯域・実測に近い):

```
[Level 0] 壁センサー誤差ループ           calc_sensor_pid()
  ey = check_sen_error()  (mm)
    -> str_ang_pid_fast (P + D, I=0)
    -> duty_sen                              ... 角度[deg]相当のオフセット

[Level 1] 角度ループ                     calc_pid_val_ang()
  target_ang = ego_in.img_ang + duty_sen
  error_p = target_ang - ang_kf(実測)
    -> angle_pid (P/I/D)
    -> duty_roll_ang                         ... 角速度[deg/s]相当のオフセット

[Level 2] 角速度(ヨーレート)ループ        calc_pid_val_ang_vel()
  target_w = ego_in.w + duty_roll_ang [+ sen_kanayama_dw]
  error_p = target_w - w_lp / w_kf(実測)
    -> gyro_pid (P/I/B/D)
    -> duty_roll                             ... 最終的に左右duty差に反映
```

`duty_sen`という変数名は歴史的経緯で「duty」を名乗るが、実際にはLevel 1(角度ループ)の目標角オフセットとして使われており、モーターdutyへ直接注入されるわけではない。

## 3. Kanayama制御則

古典的なKanayama則(2輪ロボットの経路追従):

```
w_cmd = w_d + v_d ( ky・ey + kθ・sin(eθ) )
```

- `ey`: 経路に対する横偏差
- `eθ`: 経路に対する姿勢誤差(進行方向のずれ)
- `w_d`, `v_d`: 計画上の目標角速度・目標速度

本用途(STRAIGHT走行、v_dはほぼ一定)では速度項を陽に持たず、ky・kθを速度込みのゲインとして扱い、さらに定常偏差対策として積分項を追加した簡略形を採用している(`control_law.cpp:322-324`):

```cpp
const float ey = ee->sen.error_p;
const float e_theta = tgt_val_->ego_in.ang - sensing_result_->ego.kim_theta;
const float ki_gain = param_->kanayama_straight.ki * ee->sen.error_i;
const float delta_w = param_->kanayama_straight.ky * ey + ki_gain +
                       param_->kanayama_straight.k_theta * sinf(e_theta);
sen_kanayama_dw = delta_w;
```

- `ey`は`str_ang_pid_fast`と共有(壁センサー誤差そのもの)。
- `e_theta`には`ang_kf`ではなく`kim_theta`を使う。理由: `enable_kalman_gyro=0`(現行設定)では`ang_kf`は単に`ego_in.ang`(目標値そのもの)が代入されるだけで実測と独立しておらず、`e_theta`が構造的に常時ゼロになってしまう。`kim_theta`は`TrajectoryGenerator::calc_kanayama()`内で実測ジャイロKF(`w_kf`)を積分した値で、`kanayama.enable`の設定に関わらず毎tick更新される、真に独立した実測姿勢。
- 積分項`ki_gain`は`str_ang_pid_fast`用に条件付き積分・windup_i_maxでクランプ済みの`ee->sen.error_i`を再利用している。

## 4. カスケードへの合流点の設計

**`sen_kanayama_dw`はLevel 2(角速度ループ)のオフセットに直接加算する**(`calc_pid_val_ang_vel()`, `control_law.cpp:841`):

```cpp
float offset = 0;
if (param_->torque_mode == 2 && !(PIVOT || FRONT_CTRL)) {
  offset += duty_roll_ang;   // Level 1(角度ループ)からの出力
}
offset += sen_kanayama_dw;   // Kanayamaの補正はここで合流
ee->w.error_p = (tgt->ego_in.w + offset) - se->ego.w_lp;
```

Level 0(`str_ang_pid_fast`)がLevel 1の角度目標にオフセットを注入するのに対し、KanayamaのΔwはLevel 1をバイパスしてLevel 2(角速度目標)に直接合流する。これは恣意的な配線ではなく、**Kanayama則の出力`w_cmd`が定義上すでに角速度の次元を持つため**、次元的に整合する角速度ループへ直接注入するのが自然、という設計判断である。

また、`tgt_val_->ego_in.w`自体は書き換えない(後述の事故と再発防止のため)。`sen_kanayama_dw`は`duty_roll_ang`と同様、`calc_pid_val_ang_vel()`内のローカル変数`offset`にのみ加算される**1tick限りの一時的な補正**であり、次tickの計画軌道の自己伝播計算には一切影響しない。

## 5. 「いいとこ取り」構成: 排他から並行実行へ

### 5.1 経緯
当初はLevel 0を`str_ang_pid_fast`とKanayamaカスケードで**排他的にA/B切替**する設計だった。実機比較の結果:

| 方式 | 収束速度 | 収束の質 |
|---|---|---|
| `str_ang_pid_fast`単体 | 速い(約40ms) | やや荒い |
| Kanayamaカスケード単体 | 遅い(約240ms) | 滑らか |

速さと滑らかさはトレードオフの関係にあり、排他では両立しない。

### 5.2 「いいとこ取り」構成
排他をやめ、Level 0の`str_ang_pid_fast`(P+Dのみ、速い)とKanayama(ky/ki/kθ、Level 2へ合流、滑らか)を**常時並行実行**する構成に変更(`control_law.cpp:278-333`)。役割分担:

- 速い立ち上がり: `str_ang_pid_fast`のP+D(Level 1経由)
- 定常偏差の解消・向き調整: Kanayamaのki/kθ(Level 2経由)

二重積分(str_ang_pid_fastのIとKanayamaのkiが両方積分器として働き干渉する)を避けるため、`str_ang_pid_fast.i`は`0`に固定し、積分作用はKanayama側の`ki`のみが担う。

## 6. パラメータ (`offset.yaml`)

```yaml
kanayama_straight:
  kx: 0        # 未使用(kanayama_t型を流用)
  ky: 0.03
  ki: 0.006
  k_theta: 0.0005
  enable: 1
  windup: 1
  windup_deg: 2.5
```

`hardware.yaml`側: `str_ang_pid_fast.i = 0`(元 0.00375 から変更)。

### 6.1 チューニングの経過
- `ky=0.0005`(既存`kanayama`ブロックの値を流用)では ey が -35mm まで単調発散し収束せず → ky を大幅に引き上げ`0.03`に。
- `ky=0.03`のみでは ey が -8〜-9mm 付近で頭打ち(古典的なP-onlyの定常偏差) → 積分項`ki=0.006`を追加して解消。

## 7. 実装上の事故と修正

初期実装では`tgt_val_->ego_in.w += delta_w`として**計画軌道の自己伝播状態そのもの**を書き換えており、次tickの`TrajectoryGenerator::generate()`がこの汚染された値を新たな起点として再計算するため誤差が指数的に蓄積し、左壁への激突が継続する暴走を引き起こした。

修正方針: `duty_roll_ang`が採用している「一時的な1tickオフセット」パターンを踏襲し、`sen_kanayama_dw`という専用メンバに保持、`calc_pid_val_ang_vel()`内のローカル`offset`にのみ加算する形に変更(4節参照)。`ego_in.w`は一切書き換えない。

## 8. 実機検証結果

### 8.1 孤立オフセットテスト(左右から強制的にずらして開始)
ey の標準偏差 ≈ 0.6mm。左右どちらの初期オフセットからも速く滑らかに収束することを確認。

### 8.2 t2200ターンテストでの実走行検証
4本のログ(`20260823_002313/002413/002436/002501.csv`)で、SLALOM区間(本変更の対象外)とSTRAIGHT区間を比較。

| ログ | SLALOM sat% | SLALOM v_c2-ideal_v stdev | STRAIGHT ey stdev(mm) | ang_kf range(deg) |
|---|---|---|---|---|
| 002313 | 24.1 | 63.8 | 0.46 | [-0.62, 1.22] |
| 002413 | 31.5 | 87.3 | 0.69 | [-1.75, 0.87] |
| 002436 | 31.5 | 101.5 | 0.38 | [-0.59, 1.08] |
| 002501 | 33.3 | 111.1 | 1.63 | [-1.61, 0.68] |
| **平均** | **30.1** | **90.9** | **0.79** | — |

比較対象(過去ベースライン、STRAIGHT無関係の値):
- 初回ベースライン: SLALOM sat 35.2/29.6%, stdev 85.1/64.5
- Ke/Km revert確認 n=4: SLALOM sat平均31.9%, stdev平均95.6

**評価**:
- SLALOM(本カスケードの対象外の別コードパス)はベースラインとほぼ同水準 → リグレッションなし(想定通り無影響)。
- STRAIGHT区間のey stdevは4本中3本が1mm未満(0.38〜0.69mm)。孤立テストで確認した収束性(stdev≈0.6mm)が、実際のターン後直進という現実的な走行文脈でも再現されている。002501のみ1.63mmとやや外れるが絶対値としては小さい。

## 9. 既知の限界: 傾き開始時の振動

意図的に機体を傾けた状態からテストを開始すると「ゆらゆら揺れる」振動が発生する。原因切り分け(`k_theta=0`テスト、`kanayama_straight.enable=0`テスト)の結果、**本カスケードとは無関係の既存バグ**と判明した:

- `reset_gyro_ref_with_check()`によるジャイロゼロ点較正はテスト開始トリガー時に実行される。開始前に機体が物理的に傾いていると、「ゼロ角度」の基準がその傾きごと較正されてしまい、コリドー相対の姿勢誤差がジャイロ/角度ベースの制御(`e_theta`含む)から恒久的に見えなくなる。
- `str_ang_pid_fast`単体(Kanayama無効)でも同様の傾向が再現されるため、今回新規追加したロジックの不具合ではない。

### 9.1 対策案(保留)
`left45_dist_diff` / `right45_dist_diff`(既存の壁距離差分値)から、ジャイロに依存しない壁相対姿勢を推定する:

```
sin(θ_err) ≈ (壁距離の変化率) / v
```

ユーザー判断により、今回のセッションでは実装を保留(メモのみ)。

## 10. 今後の課題

1. 9節の壁センサーベース姿勢推定によるジャイロ零点較正盲点の恒久対策(保留中)。
2. `ky` / `ki` / `k_theta` の追加チューニング。
3. 意図的に傾けない通常条件でのn≥4反復トライアルによる再現性検証(孤立オフセットテストは実施済みだが未実施)。
