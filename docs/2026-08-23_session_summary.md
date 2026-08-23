# 2026-08-23 作業まとめ: 旋回後ヨー保持・FF/摩擦・減速・常時Kanayama

深夜〜夕方にかけての長時間セッションの記録。複数のテーマが絡み合っているため、テーマ別に「やったこと」「わかったこと」「今の状態」「次やるべきこと」を整理する。

## 1. 旋回後のヨー保持(`turn_angle_fb`)

### 背景
45°ターン(SLALOM/SLA_BACK_STR)終端でw_lpの追従が乱れ、kim_thetaが目標角に対して不足したまま直進に入り、そのままSTRAIGHT中も無補正でドリフトし続ける問題があった。

### わかったこと
- 既存`angle_pid`(p=4.5, d=4.5、コード中最大ゲイン)は、基準が`ang_kf`(`enable_kalman_gyro=0`では`ego_in.ang`そのもの=計画値)であるため実測と無関係、かつ出力がw目標へのオフセットとして`gyro_pid.p`(0.000325)経由で1/1000以下に希釈される、という二重の理由で実質何も収束させていなかった。
- `ee->ang.i_bias`(`= img_ang - kim.theta`、`calc_angle_i_bias()`)は実測基準の正しい信号。
- **duty_rollへの直接加算は長時間続く区間で無効化される**: 既存`gyro_pid.b`の積分(`w_error_i`)は「実測wが目標wからズレる外乱」とみなして正確に打ち消してしまうため、STRAIGHT中に定常的にduty_rollへ足し続ける形の補正は時間経過とともに完全にキャンセルされる(`20260823_062913.csv`等で確認、kb_gainが逆算したturn_angle_fb出力とほぼ完全に相殺)。
- 対策: **w目標のoffset自体をずらす経路**(`sen_kanayama_dw`と同じ、`calc_pid_val_ang_vel()`)を使えば、gyro_pid.bはこれと戦わずΔwへ実測wを追従させる側に回る。

### 実装(現状も有効)
`include/structs.hpp` `turn_angle_fb_t`、`control_law.cpp calc_angle_velocity_ctrl()` / `calc_pid_val_ang_vel()`:

- `gain`/`gain_i`/`gain_d`: duty_roll直接加算。**SLALOM/SLA_BACK_STR限定**(旋回直後の速い過渡用、gyro_pid.bが反応する前の短時間なら相殺されない)。
- `i_w_gate`: 積分の蓄積を`|w_lp|`に応じて連続的に絞る(旋回終端の残留角速度が実機ばらつきで2〜3倍変わり、無条件蓄積だとrun毎にオシレーション有無が変わる問題への対策)。当初on/offのハードゲートで実装したが、ゲート再開時のステップ的な再点火が2段目の発振を生んだため連続重み付けに変更した。
- `w_gain`: **w目標offsetへのΔw**(rad/s per rad)。STRAIGHT含む`angle_i_bias_active()`全区間で常時有効。定常保持はこちらが担当。

### 副産物のバグ修正(重要、恒久的な効果あり)
1. **`sen_kanayama_dw`の凍結バグ**: `calc_sensor_pid()`はsct==Straightの時しか呼ばれないため、SLALOM/SLA_BACK_STR中(sct==NONE)は`sen_kanayama_dw`(offsetに直接加算される値)が更新されず、直前のSTRAIGHT/SLA_FRONT_STRでの壁追従値(0.3〜0.4台)が凍結したまま旋回を跨いで持ち越されていた。これがoffsetを支配し、turn_angle_fbやangle_pidの効果を丸ごと隠していた。`calc_tgt_duty()`のsct==NONE/Dia分岐で`sen_kanayama_dw = 0`にリセットして解決(`control_law.cpp`)。
2. **`ego_in.ang`の壁検出時リセット**: `ego_in.ang`は`SensingTask::calc_vel()`で毎tick生ジャイロ(w_raw/w_kf)を積分しているだけの実測値で、壁が見えない間は無補正でドリフトする。壁を新規検出した瞬間(`SensingControlType::Wall`への立ち上がりエッジ)に`ego_in.ang`/`global_pos.ang`をゼロへスナップするようにした(`calc_sensor_pid()`、`wall_found_prev_`メンバ)。壁のある区間では効くが、壁が無い開けた区間には効かない(ジャイロのゼロ点バイアスによる純粋なセンサードリフトは制御則側では直せない)。

### 今の状態
`turn_angle_fb.enable: 1`で有効。`20260823_072506.csv`でkim_thetaが0°→0.045°(400+tick後)まで収束することを確認済み。

### 次やるべきこと
- n≥4の反復トライアルでの再確認(単発ログでの確認に留まっている)。
- `gain_i`は限界安定気味だった経緯があるため(0.75→0.45→0.35と下げてきた)、現在値での長期安定性を継続観察。

## 2. FF摩擦(吸引ON/OFF)

### わかったこと
- `coulomb_friction`/`viscous_friction`は**吸引OFF状態でチューニングされた値**。吸引ON時は荷重(押し付け力)が約250g増加し摩擦も増えるが、これが摩擦FFに反映されておらず、加速フェーズでFFが摩擦分を過小評価 → vel_pid(FB)がその穴埋めを背負って過大反応 → `duty_l`が最大99.9%に張り付き、v_cが目標の2倍近く跳ねる"羽"状のオーバーシュートが発生していた(`latest.csv`= `20260823_071223.csv`前後で確認)。

### 実装
`coulomb_friction_suction`/`viscous_friction_suction`(`input_param_t`、hardware.yaml)を新設。`tgt_val->duty_suction`(実際の吸引パルス幅、閾値1100us)で吸引ON/OFFを二値判定し、`TrajectoryGenerator::copy_tgt()`内で`dynamics.coulomb_friction`/`viscous_friction`を切り替える。

### 今の状態
初期値はOFF側と同じ(未チューニング、`hardware.yaml`参照)。

### 次やるべきこと
- 吸引ON状態での加速テストをしながら`viscous_friction_suction`から上げていく(vel_pid.iの効きすぎ・duty_lピークが下がる方向)。

## 3. バッテリー電圧のログ

### わかったこと
- 制御に使う`batt_kf`(duty%換算の分母)は、バタつきによるduty算出の不安定化を避けるため**強いLPF/KF済み**。加速・減速時の速い電圧降下(実測で最大0.9V、7-8%相当の振れ)を全く捉えられない。

### 実装
`battery_raw`(フィルタ無しの生ADC値、`ego_estimator.cpp`の`sr->ego.battery_raw`)をログに追加(`structs.hpp` `log_data_t2`/`LogStruct11`、`logging_task.cpp`)。

### わかったこと(続き)
- 生電圧は確かに大きく振れているが、L/R両輪で共有のため`batt_kf`のズレは両輪に同じ比率で効くはずで、片輪だけの非対称(ヨーキック)を単独で説明できる仕組みではない。**「電圧推定の遅れが両輪の制動トルクを一律に不足させ、もともとあった片輪グリップ差を顕在化させるトリガーになっている」**という間接的な関与が濃厚(単独犯ではない)。

### 次やるべきこと
- `batt_kf`の追従を速くする、または減速時だけ別フィルタにする、等の根本対策は未着手(設計要検討)。

## 4. 減速時の片輪スリップ・位置ずれ

### 経緯
吸引ON状態で高速(v_max~6000)からの急減速時、片輪スリップによるヨーキック(-2〜3°級)と位置ずれを確認(`20260823_150751.csv`/`152414.csv`)。低速(v_max~3700)では軽微(-0.53°)。

### v_max→decel絶対値LUT(`decel_v_max_x/y`)は効果なしと判明
- `mpc_tgt_calc.cpp`の生成コードを解析した結果、`arg_tgt->decel`(このLUTで上書きしていた値)は「**いつ減速フェーズに切り替えるか**」の閾値判定にしか使われておらず、減速フェーズに入った後の実際の適用減速度は毎tick`(v²-v_end²)/(2×残り距離)`で**距離から逆算再計算**されている。decel入力を小さくしても早めにブレーキを開始するだけで、開始後は結局その時々の(v, 残り距離)から要求される値に収束していくため、**片輪スリップ対策としては効果が無い**ことを実測で確認(`20260823_155443.csv`)。それでいて「ノミナルのdecelだけ弱まって見た目上ブレーキが緩くなる」という副作用だけが残る。

### 実装状態
- `decel_v_max_enable`(明示的なint、デフォルト0)を追加。**`system.yaml`の`test:`ブロック**(`sys_.test.decel_v_max_*`、accl_v_x/yと同じ置き場所)で管理し、`main_task_test_run.cpp`の`test_run()`開始時に`input_param_t`へコピーする形(hardware.yamlには値を置かない)。
- 当初「配列が空(size<2)なら無効」としていたが、`from_json_vector()`はJSONキーが存在しない場合`dst.clear()`まで到達せず前回ロードされた値が残ってしまうバグ(他の配列パラメータ全般に共通の仕様、部分push用途のため)があり、yamlから行を削除/コメントアウトしてpushしても無効化できない実害を確認した。配列の空/非空に頼らず、明示的な`decel_v_max_enable`フラグで確実にON/OFFする形に修正済み。
- 現在`decel_v_max_enable: 0`(無効)。

### 次やるべきこと(根本対策は未着手)
`copy_tgt()`で`mpc_next_ego.v`を受け取った後、**tick毎の実際の減速レート|Δv/dt|の上限を出力側でクランプする**方式が本命(生成コードは触らず、その出力に対して事後的に制限をかける)。ただし懸念点として、MPCモデル自身が次tickの入力として使う内部状態(`arg_ego->dist`/`v`)が、クランプ後の値ではなくモデル自身の未クランプ値をそのまま使っている可能性があり、出力だけクランプしても内部の距離/速度の帳尻がズレて後続tickでさらに過大な補正要求を生むリスクがある。**この内部状態の伝播経路を確認してから実装すること**(今回は未確認のまま終了)。

## 5. 常時Kanayama化 → 実機で悪化・revert済み

### 実装したこと
- `TrajectoryGenerator::calc_kanayama()`の対象motion_typeをSLALOM/SLA_BACK_STR限定から`angle_i_bias_active()`と同じ「実質全モーション」に拡張。
- `v_cmd`/`w_cmd`を`control_law.cpp calc_pid_val_ang_vel()`の`ee->w.error_p`/`ee->w_kf.error_p`(ヨーの主力ゲインkp/kb/kcの入力)に配線。従来`w_cmd`は`ee->w_kf`(D項のみ)にしか効いておらず、実質無力化されていた。
- `kanayama_straight.k_theta`を0に(2D Kanayamaの向き補正と役割が重複するため、二重補正回避)。`ky`/`ki`(壁センサー基準の横方向)は独立情報源なので維持。
- `kanayama.enable: 0→1`。

### 実機結果: 明確に悪化、revert済み
`20260823_165521.csv`(enable=0)と`20260823_165611.csv`(enable=1)の比較:

| | 平均\|sen_error_p\| | 平均\|kim_theta\| |
|---|---|---|
| enable=0 | 0.35mm | 0.34° |
| enable=1 | 2.71mm | 5.42° |

8〜16倍悪化。**`kanayama.enable: 0`にrevert済み**(コード側の配線はそのまま残置、yamlの`enable`フラグのみ)。

### 疑われる根本原因: 追従ロジック自体の不具合の可能性
`odm_y`(計画上の横位置、直進なので常に0)に対し`kim_y`(実測)が-6.82mmまでドリフトして**戻ってこない**現象を確認(`20260823_165611.csv`)。Kanayamaが正しく追従できていれば`ey`が大きくなるほど強く補正がかかって0に戻ろうとするはずが、実際は片方向に伸び続けているだけに見える。単なるゲイン不足や`kanayama_straight`との干渉というより、`ex`/`ey`/`e_theta`の符号・座標変換のどこかが合っていない可能性が高い。

### 次やるべきこと
1. `ex`/`ey`/`e_theta`(`calc_kanayama()`内のローカル変数)を直接ログに出す専用デバッグフィールドを追加し、符号・大きさが物理的に妥当か確認する。
2. `dx = std::clamp(dx, 0.0f, ABS(dx))`(`trajectory_generator.cpp`)というdxの片側クランプの意図・妥当性を再確認する(kimがodmより前方にいる場合dxが強制的に0になる仕様で、これが本当に正しいか未検証)。
3. `kx=10`/`ky=0.0005`/`k_theta=0.0005`はSLALOM用にチューニングされた値のまま。STRAIGHT等新しい適用範囲用には値が合っていない可能性がある(ただし追従ロジック自体が壊れているなら、ゲイン調整の前にそちらが先)。
4. `kanayama_straight`との同時実行が本当に安全か(両方が同じ物理量を別の参照系から補正しようとして、どちらかが優位/劣位になっていないか)を切り分ける。

## 6. 今回のコードクリーンアップ

以下は今夜の試行錯誤で導入し、その後不要と判断して削除済み:
- `turn_end_brake_t`(旋回終端ブレーキ、発振して断念)
- `turn_w_pid_t`(角速度PID積分ブースト、発散して断念)
- `dbg_off_ang`/`dbg_off_wgain`/`dbg_off_kny`(offset内訳デバッグ用、原因特定後に削除)

## ファイル一覧(今回変更した主なファイル)

- `include/structs.hpp`: `turn_angle_fb_t`、摩擦FF(`coulomb_friction_suction`等)、`decel_v_max_*`、`test_mode_t`拡張、`battery_raw`ログフィールド
- `include/config_mapping.hpp` / `include/config_dump.hpp`: 上記の配線
- `src/planning/control_law.cpp`: `calc_angle_velocity_ctrl()` / `calc_pid_val_ang_vel()` / `calc_sensor_pid()` / `calc_tgt_duty()`
- `src/planning/trajectory_generator.cpp`: `copy_tgt()`(摩擦FF切替)、`calc_kanayama()`(対象範囲拡張)
- `src/main/main_task_util.cpp`: `apply_decel_v_max_lut()`
- `src/main/main_task_test_run.cpp`: test_run()でのLUTコピー
- `src/logging/logging_task.cpp`: `battery_raw`ログ追加
- `tools/param_tuner/profile/hardware.yaml` / `hf/offset.yaml` / `system.yaml`: 各種パラメータ
