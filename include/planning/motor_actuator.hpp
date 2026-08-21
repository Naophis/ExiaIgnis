#pragma once

#include "define.hpp" // MOTOR_PWM_FREQ_HZ
#include "pico/types.h"
#include "hardware/pwm.h"
#include <stdint.h>

// 左右駆動モーター (コアレス) の PWM 出力のみを担当。
// 吸引 BLDC は BldcActuator が担当。
class MotorActuator {
public:
  // GPIO ファンクション設定・PWM スライス初期化。Core0 の main から呼ぶ。
  // motor_hz: PWM周波数[Hz](hardware.yamlのMotorHz)。省略時はdefine.hppの既定値。
  void init(uint32_t motor_hz = MOTOR_PWM_FREQ_HZ);

  // duty_l/r [%] を PWM ハードウェアに書き込む。
  void apply(float duty_l, float duty_r);

  void motor_enable();
  void motor_disable();

private:
  uint     slice_L_    = 0;
  uint     slice_R_    = 0;
  uint32_t motor_wrap_ = 2999;
  uint32_t motor_hz_    = MOTOR_PWM_FREQ_HZ;
  bool     motor_en    = false;
};
