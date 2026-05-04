#pragma once

#include "pico/types.h"
#include <stdint.h>

class MotorActuator {
public:
  // GPIO ファンクション設定・PWM スライス初期化。Core0 の main から呼ぶ。
  void init();

  // duty_l/r/suction [%] を PWM ハードウェアに書き込む。
  void apply(float duty_l, float duty_r, float duty_suction);

private:
  uint slice_L_ = 0;
  uint slice_R_ = 0;
  uint slice_S_ = 0;
  uint32_t motor_wrap_ = 2999;
};
