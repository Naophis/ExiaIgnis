# 指示書: Kanayama 2D追従ロジックのデバッグ

このドキュメントは新しいセッション(プロンプト)への引き継ぎ用。前提知識ゼロで読めるように書いてある。背景の全文脈は`docs/2026-08-23_session_summary.md`を参照(特に5章「常時Kanayama化 → 実機で悪化・revert済み」)。

## タスク

`TrajectoryGenerator::calc_kanayama()`(`src/planning/trajectory_generator.cpp:55-127`)の2D姿勢追従ロジックが、実測位置のドリフトを正しく補正できていない疑いがある。原因を特定し、修正すること。

## 背景

このロジックは実測位置(`kim.x/y/theta`、実測ジャイロ+エンコーダのKF)と計画上の理想位置(`odm.x/y/theta`、`trajectory_points[idx]`から取得)を比較し、Kanayama則で`v_cmd`/`w_cmd`を計算する:

```cpp
float dx = ego.odm.x - ego.kim.x;
float dy = ego.odm.y - ego.kim.y;
dx = std::clamp(dx, 0.0f, ABS(dx));   // ← ここが怪しい(後述)

float d_theta = ego.odm.theta - (se->ego.ang_kf + last_tgt_angle);
float e_theta = d_theta;
const float cos_theta = std::cos(se->ego.ang_kf);
const float sin_theta = std::sin(se->ego.ang_kf);
const float ex = cos_theta * dx + sin_theta * dy;
const float ey = -sin_theta * dx + cos_theta * dy;

se->ego.knym_v = vd * cos_e_theta + kx * ex;
se->ego.knym_w = wd + vd * (ky * ey + k_theta * sin_e_theta);
```

これまで`kanayama.enable: 0`かつ`w_cmd`が実際の制御にほぼ配線されていなかったため(`ee->w_kf`のD項にしか効かない)、このロジックのバグが表面化していなかった。今回`w_cmd`を`ee->w.error_p`(ヨーの主力ゲインkp/kb/kcの入力、`control_law.cpp calc_pid_val_ang_vel()`)に配線し、`kanayama.enable: 1`にして実機テストしたところ、**壁追従・向き保持が明確に悪化した**(平均|sen_error_p|: 0.35mm→2.71mm、平均|kim_theta|: 0.34°→5.42°、`20260823_165521.csv` vs `20260823_165611.csv`)。

決定的な証拠: 直進走行なのに`odm_y`は常に0のはずが、`kim_y`(実測)が-6.82mmまでドリフトして**戻ってこない**(`20260823_165611.csv`)。Kanayamaが正しく追従できていれば`ey`が大きくなるほど強く補正がかかって0に戻ろうとするはずだが、実際は片方向に伸び続けているだけに見える。

現在は`kanayama.enable: 0`(`tools/param_tuner/profile/hf/offset.yaml`)にrevert済みで実害なし。コード配線(`w_cmd`→`ee->w.error_p`)はそのまま残してある。

## 疑わしい箇所(優先順に)

1. **`dx = std::clamp(dx, 0.0f, ABS(dx))`という片側クランプ**(`trajectory_generator.cpp:81`)。`dx`が負(kimがodmより前方=実測が計画より進んでいる)の場合、強制的に0にされる。この設計意図が不明。もし本来は符号付きのまま使うべきなら、ここでey/ex全体の符号関係が壊れている可能性がある。
2. **`e_theta`の基準**: `d_theta = odm.theta - (ang_kf + last_tgt_angle)`。`ang_kf`は`enable_kalman_gyro=0`では`ego_in.ang`そのもの(計画値のコピー、実測ではない)。一方`kanayama_straight`(既存の壁ベースKanayama、`control_law.cpp`)は`e_theta`の実測側に`kim_theta`を使っている(`ang_kf`は使わない、理由はコード内コメント参照)。この`calc_kanayama()`側だけ`ang_kf`(実質計画値)を使っているのは一貫性がなく、`e_theta`が意図通りの値になっていない可能性がある。
3. **`ky`/`kx`/`k_theta`のゲイン**: `kx=10, ky=0.0005, k_theta=0.0005`はSLALOM用にチューニングされた値のまま(`offset.yaml`の`kanayama:`ブロック)。STRAIGHT等の新しい適用範囲には値が合っていない可能性。ただし上記1・2が本当にバグなら、ゲイン調整より先にそちらを直すべき。
4. **`trj_length: 1`の影響**(`offset.yaml`)。`trajectory_points`配列が1要素しかなく、`idx`は常に0にクランプされる(`calc_kanayama()`の`idx = std::min(trj_length-1, idx_val)`)。1tick先の自己伝播予測点を毎回参照するだけなので致命的ではないはずだが、念のため確認候補に入れておく。

## 推奨する進め方

1. **まずデバッグログを足す**: `ex`/`ey`/`e_theta`/`dx`(クランプ前後両方)を直接ログに出す専用フィールドを追加する(今夜使った`dbg_off_*`パターンを再利用: `structs.hpp`の`aw_log_t`または類似箇所に一時フィールドを足し、`logging_task.cpp`のスキーマ宣言+ls11変換+ld代入を配線、`control_law.cpp`または`trajectory_generator.cpp`側で値を代入)。
2. **低速・直進・壁ありの単純なテスト**(`test_run()`、`sys_.test`)で1本ログを取り、`dx`(クランプ前)・`ey`・`e_theta`の符号と大きさが物理的に妥当か確認する。具体的には: ロボットが右に寄ったら`ey`は正負どちらになるべきか、それに対して`ky*ey`が実際にロボットを左に押し返す符号になっているかを手計算で追う。
3. **クランプの妥当性を検証**: `dx`の片側クランプを外してみて(あるいは意図をコード内コメントや設計文書から確認できないか探して)、挙動がどう変わるか比較する。
4. バグを直したら、**まず`kanayama.enable: 0`のまま**(=w_cmdが実際には使われない状態)でログ上の`ex`/`ey`/`e_theta`が妥当になったことを確認してから、`enable: 1`で実機テストする。
5. 実機テストは必ずn≥2〜3の反復で行う(このセッション全体を通じて、単発ログの比較は run-to-run ノイズに埋もれやすいことが繰り返し確認されている)。

## 触ってよい/触らない方が良い範囲

- **触ってよい**: `TrajectoryGenerator::calc_kanayama()`内のロジック、`offset.yaml`の`kanayama:`ブロックのゲイン。
- **既に配線済みで動作確認が要らない部分**: `w_cmd`→`ee->w.error_p`/`ee->w_kf.error_p`の配線自体(`control_law.cpp calc_pid_val_ang_vel()`)は今回のバグとは無関係な可能性が高いので、まずは触らない。
- **`kanayama_straight`(既存の壁ベース1D Kanayama、`control_law.cpp calc_sensor_pid()`)は正常に動いている**。今回の`kanayama`(2D)の`k_theta`を0にして向き補正を`kanayama_straight`と重複させないようにしてあるが、`kanayama`(2D)側のバグが直ってenable=1にする場合、この重複回避が引き続き妥当か再考すること。
- **FF系のパラメータ(Ke/Km/Mass/Resist等)は変更しないこと**。過去に何度もこの方向のチューニングで実機悪化・リバートを繰り返している経緯があるため、この件はFB(制御則)側だけで完結させる。

## 副次タスク(時間があれば、優先度低)

`docs/2026-08-23_session_summary.md`の各章「次やるべきこと」を参照:

- 摩擦FF(`coulomb_friction_suction`/`viscous_friction_suction`)のチューニング(3章)
- 減速時の片輪スリップ対策: `copy_tgt()`出力側でのΔv/dtクランプ実装(4章、mpc内部状態の伝播経路を先に確認すること)
- バッテリー電圧の速い追従化(2章)
