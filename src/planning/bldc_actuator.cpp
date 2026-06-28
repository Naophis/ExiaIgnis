#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <algorithm>

void BldcActuator::init() {
  // PWM GPIO
  gpio_set_function(SUCTION_PWM1, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM2, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM3, GPIO_FUNC_PWM);

  // EN GPIO: 初期 LOW (ドライバ OFF)
  gpio_init(SUCTION_EN);
  gpio_set_dir(SUCTION_EN, GPIO_OUT);
  gpio_put(SUCTION_EN, 0);

  slice_s1_ = pwm_gpio_to_slice_num(SUCTION_PWM1);  // slice4: GPIO8(A)+GPIO9(B)
  slice_s2_ = pwm_gpio_to_slice_num(SUCTION_PWM3);  // slice5: GPIO10(A)

  wrap_ = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

  for (uint slice : {slice_s1_, slice_s2_}) {
    pwm_set_clkdiv_int_frac4(slice, 1, 0);
    pwm_set_wrap(slice, wrap_);
    pwm_set_enabled(slice, false);  // enable() が呼ばれるまで停止
  }

  set_duty(0.0f);
}

void BldcActuator::set_duty(float duty_percent) {
  uint16_t level = (uint16_t)((float)(wrap_ + 1u) *
                               std::clamp(duty_percent, 0.0f, 100.0f) / 100.0f);

  // TODO: 120° 位相シフト生成（現在は全 ch 同一デューティのプレースホルダ）
  // Phase1 (0°)  : slice_s1_ CHAN_A (GPIO8)
  // Phase2 (120°): slice_s1_ CHAN_B (GPIO9)
  // Phase3 (240°): slice_s2_ CHAN_A (GPIO10)
  pwm_set_chan_level(slice_s1_, PWM_CHAN_A, level);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_B, level);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_A, level);
}

void BldcActuator::enable() {
  enabled_ = true;
  pwm_set_enabled(slice_s1_, true);
  pwm_set_enabled(slice_s2_, true);
  gpio_put(SUCTION_EN, 1);
}

void BldcActuator::disable() {
  enabled_ = false;
  gpio_put(SUCTION_EN, 0);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
}
