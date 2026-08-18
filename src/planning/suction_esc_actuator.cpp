#include "planning/suction_esc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstdio>

#define SUCTION_ESC_FREQ_HZ   250u

namespace {
// 整数分周器固定 + wrap可変方式 (ui.cpp UserInterface::set_pwm_freq と同型)。
// PWM TOP(wrap)レジスタは実機のレジスタ定義上も16bit(最大65535)、
// pwm_set_wrap()の引数もuint16_tである。ここでのkClkDivInt/
// SUCTION_ESC_FREQ_HZの組み合わせは、単純な計算上の都合(粗い分解能で
// wrapを65535未満に収める)よりも実機での動作確認を優先して選んだ値
// (define.hppのSUCTION_ESC_FREQ_HZコメント参照: wrapがuint16_tへ暗黙変換
// で切り詰められて別の周波数になった状態がむしろ正常動作した、という
// 実機検証の経緯がある)。init()側でwrapが16bit範囲を超えていないか
// 明示的にチェックし、超えていれば標準出力に警告する(超過時に無言で
// 切り詰められて別の周波数になる、という今回ハマった事態を再発させない
// ため)。
constexpr uint8_t kClkDivInt = 16;
} // namespace

void SuctionEscActuator::init() {
  gpio_set_function(SUCTION_ESC_PWM, GPIO_FUNC_PWM);
  slice_   = pwm_gpio_to_slice_num(SUCTION_ESC_PWM);
  channel_ = pwm_gpio_to_channel(SUCTION_ESC_PWM);

  const uint32_t sys_clk = clock_get_hz(clk_sys);
  const uint32_t wrap32 = sys_clk / ((uint32_t)kClkDivInt * SUCTION_ESC_FREQ_HZ) - 1u;
  if (wrap32 > 0xFFFFu) {
    // pwm_set_wrap()はuint16_t引数のため、ここでチェックせずに渡すと
    // 無言で下位16bitに切り詰められ、意図と異なる周波数が出力される。
    printf("[suction_esc] WARN: wrap=%lu exceeds 16bit (freq=%uHz, clkdiv=%u) "
           "-- truncated value will be used, frequency will NOT match "
           "SUCTION_ESC_FREQ_HZ\n",
           (unsigned long)wrap32, (unsigned)SUCTION_ESC_FREQ_HZ, (unsigned)kClkDivInt);
  }
  wrap_ = wrap32;
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
  write_ticks(pulse_us);
}

void SuctionEscActuator::apply_us(float pulse_us) {
  if (pulse_us < (float)SUCTION_ESC_PULSE_MIN_US)
    pulse_us = (float)SUCTION_ESC_PULSE_MIN_US;
  if (pulse_us > (float)SUCTION_ESC_PULSE_MAX_US)
    pulse_us = (float)SUCTION_ESC_PULSE_MAX_US;
  write_ticks(pulse_us);
}

void SuctionEscActuator::write_ticks(float pulse_us) {
  const uint16_t level = (uint16_t)(pulse_us * ticks_per_us_);
  pwm_set_chan_level(slice_, channel_, level);
}

// enable/disableはどちらもフラグ管理のみ。実際のパルス幅は
// ControlLaw::set_next_duty()が毎tick suction_ramp_us_per_sec_で滑らかに
// 追従させる(有効化時は最小パルスから目標へ、無効化時は目標から最小
// パルスへ)。ここで即座に最小パルスへ叩き落とすと、まだ高速回転して
// いるモーターに急ブレーキ相当のコマンドを送ることになり、大きな
// 逆起電力/回生電流の原因になり得るため行わない。
void SuctionEscActuator::enable() { enabled_ = true; }

void SuctionEscActuator::disable() { enabled_ = false; }
