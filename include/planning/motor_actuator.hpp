#pragma once

#include "define.hpp" // MOTOR_PWM_FREQ_HZ
#include "driver/dither_pwm.hpp"
#include "pico/types.h"
#include "hardware/pwm.h"
#include <stdint.h>

// 左右駆動モーター (コアレス) の PWM 出力のみを担当。
// 吸引 BLDC は BldcActuator / SuctionEscActuator が担当。
//
// 2026-09-14: 出力段を DitherPwm(Hardware PWM + DMA 時間方向ディザ)に置き換え。
//   * duty[%] を Q16.16 count に変換し、1 制御周期ぶんの N/N+1 列を DMA が
//     wrap ごとに CC へ流す。CPU が CC を直接書く経路は無い(docs/dither_pwm.md)。
//   * DitherPwm::init() が失敗する構成(MotorHz/制御周波数 がリングに収まらない等)
//     では従来の pwm_set_chan_level 直書きに自動で戻る(legacy_)。
//   * 呼び出しは全て Core1 の tick() 内(init は Core0、core1 起動前)。
class MotorActuator {
public:
  // GPIO ファンクション設定・PWM スライス初期化。Core0 の main から呼ぶ。
  // motor_hz  : PWM周波数[Hz](hardware.yamlのMotorHz)。省略時はdefine.hppの既定値。
  // control_hz: apply() を呼ぶ制御周波数[Hz]。DitherPwm の samples_per_tick に使う。
  void init(uint32_t motor_hz = MOTOR_PWM_FREQ_HZ, uint32_t control_hz = 1000);

  // duty_l/r [%] を出力段に渡す。制御周期ごとに 1 回呼ぶ(Core1)。
  void apply(float duty_l, float duty_r);

  void motor_enable();
  void motor_disable();

  // 診断(ログ用)。idx 0=左 slice, 1=右 slice。legacy 時は全 0。
  const dpwm::DitherStats& dither_stats(uint idx) const { return pwm_.stats(idx); }
  // 診断: リングに目印を書いてから CC レジスタと GPIO 出力に現れるまでの時間[us]を
  // 実測して printf する(Core0、モーター有効中に呼ぶ。左 slice ch A に 1 周期だけ
  // 約 1000 count のパルスが出る)。
  void probe_latency();
  bool dither_active() const { return !legacy_; }
  // 診断用 A/B: 走行間(モーター無効中)に DitherPwm ⇔ 従来 CC 直書きを切り替える。
  // DitherPwm::init() が失敗した個体では true にできない(戻り値 false)。
  bool set_dither(bool enable);

private:
  dpwm::DitherPwm pwm_;
  uint     slice_L_    = 0;
  uint     slice_R_    = 0;
  uint32_t motor_wrap_ = 2999;
  uint32_t motor_hz_   = MOTOR_PWM_FREQ_HZ;
  float    q16_per_pct_ = 0.0f;   // duty[%] -> Q16.16 count の係数 = (top+1)*65536/100
  bool     motor_en    = false;
  bool     legacy_     = false;   // true: 従来の CC 直書き(DitherPwm 無効)
  bool     dither_ok_  = false;   // DitherPwm::init() 成功

  void apply_legacy_(float duty_l, float duty_r);
};
