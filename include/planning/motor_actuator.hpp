#pragma once

#include "pico/types.h"
#include "hardware/pwm.h"
#include <stdint.h>

// 左右駆動モーター (コアレス) の PWM 出力のみを担当。
// 吸引 BLDC は BldcActuator が担当。
class MotorActuator {
public:
  // GPIO ファンクション設定・PWM スライス初期化。Core0 の main から呼ぶ。
  void init();

  // duty_l/r [%] を PWM ハードウェアに書き込む。
  void apply(float duty_l, float duty_r);

  void motor_enable();
  void motor_disable();

private:
  uint     slice_L_    = 0;
  uint     slice_R_    = 0;
  uint32_t motor_wrap_ = 2999;
  bool     motor_en    = false;
};
