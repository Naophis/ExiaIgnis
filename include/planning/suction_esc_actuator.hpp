#pragma once
#include "hardware/pwm.h"
#include <stdint.h>

// 吸引モーター用 AM32 ESC への RC標準PWM(1000〜2000us)出力。
// MotorActuatorと同じ「PWM出力のみを担当」する薄いクラス。BLDCの自前
// コミュテーション(BldcActuator)は行わない — 実際の6-step commutationは
// 外付けAM32 ESC側がBEMFセンサレスで自律的に行うため、こちら側は
// コアレスモーター相当の単純なduty指令を出すだけでよい。
//
// GPIO11(SUCTION_PWM3)を流用し、ESCのスロットル信号線として使う
// (define.hpp の SUCTION_ESC_PWM 参照)。BldcActuatorが使っていた
// GPIO8/9/10は未使用のまま残る。
//
// [重要] apply(0%)="最小パルス(1000us)"は「停止」であって「信号なし」
// ではない。ESCがアーム状態を保つには継続的なパルス出力が必要なため、
// init()直後から常時PWMを出し続ける。
//
// enable()/disable()はフラグ管理のみで、PWM出力そのものは書き換えない。
// 実際のパルス幅は毎tick ControlLaw::set_next_duty()がsuction_ramp_us_
// per_sec_の速度で目標値へ滑らかに追従させる(有効化時は最小パルスから
// 目標へ、無効化時は目標から最小パルスへ)。まだ高速回転中のモーターに
// 対して急に最小パルス(停止)を送ると、急ブレーキ相当のコマンドとなり
// 大きな逆起電力/回生電流が生じ得るため、停止時も同じ滑らかなランプを
// 経由させる設計にしている。
class SuctionEscActuator {
public:
  // GPIOファンクション設定・PWMスライス初期化。Core0のmainから呼ぶ。
  // 呼び出し直後から最小パルス(0%duty)を出力し始める(ESCのアーム待ち状態)。
  void init();

  // duty[%] (0〜100) をパルス幅(1000〜2000us)に線形変換してPWMへ書き込む。
  void apply(float duty_pct);

  // パルス幅を us で直接指定してPWMへ書き込む(1000〜2000usへクランプ)。
  // ControlLawはこちらを使う(system.yamlのsuction_duty系はus直接指定)。
  void apply_us(float pulse_us);

  // フラグ管理のみ(クラスコメント参照)。実際の出力遷移はControlLaw側で行う。
  void enable();
  void disable();
  bool is_enabled() const { return enabled_; }

private:
  void write_ticks(float pulse_us);

  uint     slice_   = 0;
  uint     channel_ = 0;
  uint32_t wrap_    = 0;
  float    ticks_per_us_ = 0.0f;
  bool     enabled_ = false;
};
