#include "planning/motor_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>

void MotorActuator::init() {
  const uint motor_pins[] = {M_PWM_L1, M_PWM_L2, M_PWM_R1, M_PWM_R2,
                             SUCTION_PWM};
  for (uint pin : motor_pins) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
  }

  slice_L_ = pwm_gpio_to_slice_num(M_PWM_L1);
  slice_R_ = pwm_gpio_to_slice_num(M_PWM_R1);
  slice_S_ = pwm_gpio_to_slice_num(SUCTION_PWM);

  motor_wrap_ = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

  for (uint slice : {slice_L_, slice_R_, slice_S_}) {
    pwm_set_clkdiv_int_frac4(slice, 1, 0);
    pwm_set_wrap(slice, motor_wrap_);
    pwm_set_enabled(slice, true);
  }

  apply(0.0f, 0.0f, 0.0f);
}

void MotorActuator::apply(float duty_l, float duty_r, float duty_suction) {
  auto set_drive = [&](uint slice, float duty) {
    uint16_t level = (uint16_t)((float)motor_wrap_ * std::fabs(duty) / 100.0f);
    if (duty >= 0.0f) {
      pwm_set_chan_level(slice, PWM_CHAN_A, level);
      pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else {
      pwm_set_chan_level(slice, PWM_CHAN_A, 0);
      pwm_set_chan_level(slice, PWM_CHAN_B, level);
    }
  };

  set_drive(slice_L_, duty_l);
  set_drive(slice_R_, duty_r);

  uint16_t suction_level =
      (uint16_t)((float)motor_wrap_ * std::clamp(duty_suction, 0.0f, 100.0f) /
                 100.0f);
  pwm_set_chan_level(slice_S_, PWM_CHAN_A, suction_level);
  pwm_set_chan_level(slice_S_, PWM_CHAN_B, 0);
}
