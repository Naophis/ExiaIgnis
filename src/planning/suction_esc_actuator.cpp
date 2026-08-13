#include "planning/suction_esc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace {
// 整数分周器固定 + wrap可変方式 (ui.cpp UserInterface::set_pwm_freq と同型)。
// SUCTION_ESC_FREQ_HZ(250Hz)程度の低周波でも16bit wrapに収める。
constexpr uint8_t kClkDivInt = 16;
} // namespace

void SuctionEscActuator::init() {
  gpio_set_function(SUCTION_ESC_PWM, GPIO_FUNC_PWM);
  slice_   = pwm_gpio_to_slice_num(SUCTION_ESC_PWM);
  channel_ = pwm_gpio_to_channel(SUCTION_ESC_PWM);

  const uint32_t sys_clk = clock_get_hz(clk_sys);
  wrap_ = sys_clk / ((uint32_t)kClkDivInt * SUCTION_ESC_FREQ_HZ) - 1u;
  ticks_per_us_ = (float)sys_clk / (float)kClkDivInt / 1.0e6f;

  pwm_set_clkdiv_int_frac4(slice_, kClkDivInt, 0);
  pwm_set_wrap(slice_, wrap_);
  pwm_set_enabled(slice_, true);

  apply(0.0f); // 最小パルス(1000us)を即座に出し始める(ESCのアーム待ち状態)
}

void SuctionEscActuator::apply(float duty_pct) {
  if (duty_pct < 0.0f) duty_pct = 0.0f;
  if (duty_pct > 100.0f) duty_pct = 100.0f;

  const float pulse_us =
      (float)SUCTION_ESC_PULSE_MIN_US +
      ((float)SUCTION_ESC_PULSE_MAX_US - (float)SUCTION_ESC_PULSE_MIN_US) *
          duty_pct / 100.0f;
  const uint16_t level = (uint16_t)(pulse_us * ticks_per_us_);
  pwm_set_chan_level(slice_, channel_, level);
}

void SuctionEscActuator::enable() { enabled_ = true; }

void SuctionEscActuator::disable() {
  enabled_ = false;
  // MotorActuator::motor_disable()と同じ考え方: 無効化時は即座に最小
  // パルスを書き込む。PWM出力自体は止めない(信号が途切れるとESCが
  // アーム解除/フェイルセーフに入る可能性があるため)。
  apply(0.0f);
}
