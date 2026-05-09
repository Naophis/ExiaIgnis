#pragma once

#include "pico/types.h"
#include <stdint.h>

class MotorActuator {
public:
  // GPIO ファンクション設定・PWM スライス初期化。Core0 の main から呼ぶ。
  void init();

  // duty_l/r/suction [%] を PWM ハードウェアに書き込む。
  void apply(float duty_l, float duty_r, float duty_suction);
  void apply_suction(float duty_suction);

  void motor_enable() { motor_en = true; }
  void motor_disable() { motor_en = false; }
  void suction_enable() { suction_en = true; }
  void suction_disable() { suction_en = false; }
  void set_suction_gain(float gain) { suction_gain = gain; }

private:
  uint slice_L_ = 0;
  uint slice_R_ = 0;
  uint slice_S_ = 0;
  uint32_t motor_wrap_ = 2999;
  bool motor_en = false;
  bool suction_en = false;
  float gain_cnt = 0;
  float suction_gain = 200;
};
