#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cmath>
#include <initializer_list>
#include <stdio.h>

static constexpr float TWO_PI    = 6.2831853f;
static constexpr float PHASE_120 = 2.0943951f;

// V/Hz パラメータ (sample/bldc.cpp の実証値)
static constexpr float AMP_BASE        = 0.06f;
static constexpr float AMP_BASE_HZ     = 120.0f;
static constexpr float MAX_AMP         = 0.35f;
// RAMP速度は BldcActuator::ramp_hz_per_sec_ メンバで管理 (set_ramp_rate()で変更可能)
static constexpr float START_ELEC_HZ   = 1.0f;

// PWM周波数はドライブモーター(MOTOR_PWM_FREQ_HZ=37500)と独立
static constexpr uint32_t BLDC_PWM_FREQ_HZ = 80000u;  // 80kHz: wrap=1874

// 600ms × 80kHz (sample/bldc.cpp の実証値)
static constexpr uint32_t ALIGN_TICKS = (uint32_t)(BldcActuator::CONTROL_HZ) * 600 / 1000;

static inline float amp_from_hz(float hz, float gain) {
  if (hz <= AMP_BASE_HZ) return AMP_BASE;
  const float a = AMP_BASE * (hz / AMP_BASE_HZ) * gain;
  return a < AMP_BASE ? AMP_BASE : (a > MAX_AMP ? MAX_AMP : a);
}

// PWM CC レジスタに3相サイン波を書く
// slice_s1 = slice4 (GPIO9=PWM4B=U相)
// slice_s2 = slice5 (GPIO10=PWM5A=V相, GPIO11=PWM5B=W相)
// reverse=true: V/W入れ替えで回転方向反転
static inline void write_phase(uint s1, uint s2, uint32_t wrap,
                                float theta, float amp, bool reverse) {
  const float   wu1 = (float)(wrap + 1u);
  const uint16_t lu = (uint16_t)(wu1 * (0.5f + amp * sinf(theta)));
  const uint16_t lv = (uint16_t)(wu1 * (0.5f + amp * sinf(theta - PHASE_120)));
  const uint16_t lw = (uint16_t)(wu1 * (0.5f + amp * sinf(theta + PHASE_120)));
  // CH_B(上位16bit) → GPIO9=U,  CH_A(下位16bit) → 未使用 (slice4)
  pwm_hw->slice[s1].cc = (uint32_t)lu << 16;
  // CH_B(上位16bit) → GPIO11=W相,  CH_A(下位16bit) → GPIO10=V相
  pwm_hw->slice[s2].cc = reverse
      ? (((uint32_t)lv << 16) | lw)   // V↔W入れ替え → 逆転
      : (((uint32_t)lw << 16) | lv);
}

// ============================================================
// init
// ============================================================
void BldcActuator::init() {
  gpio_set_function(SUCTION_PWM1, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM2, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM3, GPIO_FUNC_PWM);
  gpio_init(SUCTION_EN);
  gpio_set_dir(SUCTION_EN, GPIO_OUT);
  gpio_put(SUCTION_EN, 0);

  slice_s1_ = pwm_gpio_to_slice_num(SUCTION_PWM1);
  slice_s2_ = pwm_gpio_to_slice_num(SUCTION_PWM2);
  wrap_ = (uint32_t)(clock_get_hz(clk_sys) / BLDC_PWM_FREQ_HZ) - 1u;  // = 1874

  for (uint s : {slice_s1_, slice_s2_}) {
    pwm_set_clkdiv_int_frac4(s, 1, 0);
    pwm_set_wrap(s, wrap_);
    pwm_set_enabled(s, false);
  }
}

// ============================================================
// タイマーコールバック (Core0, 80kHz)
// ============================================================
bool BldcActuator::timer_cb(repeating_timer_t *rt) {
  static_cast<BldcActuator *>(rt->user_data)->step();
  return true;
}

void BldcActuator::step() {
  constexpr float dt = 1.0f / (float)CONTROL_HZ;

  switch (state_) {
  case State::STOP:
    break;

  case State::ALIGN:
    // ロータを theta=0 に磁気ロックしてから加速する
    write_phase(slice_s1_, slice_s2_, wrap_, 0.0f, AMP_BASE, reverse_);
    if (++state_cnt_ >= ALIGN_TICKS) {
      state_     = State::RAMP;
      state_cnt_ = 0u;
      theta_     = 0.0f;
      elec_hz_   = START_ELEC_HZ;
    }
    break;

  case State::RAMP: {
    elec_hz_ += ramp_hz_per_sec_ * dt;
    if (elec_hz_ > target_elec_hz_) elec_hz_ = target_elec_hz_;
    theta_ += TWO_PI * elec_hz_ * dt;
    if (theta_ >= TWO_PI) theta_ -= TWO_PI;
    write_phase(slice_s1_, slice_s2_, wrap_, theta_,
                amp_from_hz(elec_hz_, amp_gain_), reverse_);
    if (elec_hz_ >= target_elec_hz_) state_ = State::RUN;
    break;
  }

  case State::RUN:
    theta_ += TWO_PI * elec_hz_ * dt;
    if (theta_ >= TWO_PI) theta_ -= TWO_PI;
    write_phase(slice_s1_, slice_s2_, wrap_, theta_,
                amp_from_hz(elec_hz_, amp_gain_), reverse_);
    break;
  }
}

// ============================================================
// set_duty: Core1 IRQ から呼ばれる振幅ゲイン更新
// ALIGN/RAMP中はサンプル実証値(0.10)を保護するため無視する
// ============================================================
void BldcActuator::set_duty(float duty_pct) {
  if (state_ == State::ALIGN || state_ == State::RAMP) return;
  float gain = (duty_pct / 100.0f) * 0.10f;
  if      (gain < 0.06f) gain = 0.06f;
  else if (gain > 0.20f) gain = 0.20f;
  amp_gain_ = gain;
}

// ============================================================
// enable: 非同期起動 (すぐ返る, ALIGN→RAMP→RUN はタイマーで進行)
// ============================================================
void BldcActuator::enable() {
  if (enabled_) return;
  // sample/bldc.cpp と同じ順序: theta=0を先に出力してからドライバを起こす
  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));
  write_phase(slice_s1_, slice_s2_, wrap_, 0.0f, AMP_BASE, reverse_);
  gpio_put(SUCTION_EN, 1);
  sleep_ms(20);  // MP6540H nSLEEP 起動待ち
  state_     = State::ALIGN;
  theta_     = 0.0f;
  elec_hz_   = 0.0f;
  state_cnt_ = 0u;
  amp_gain_  = 0.10f;  // ファン負荷あり → 0.10(無負荷)の2倍が必要
  enabled_   = true;
  add_repeating_timer_us(1000000 / CONTROL_HZ, timer_cb, this, &timer_);
}

// ============================================================
// disable
// ============================================================
void BldcActuator::disable() {
  enabled_ = false;
  state_   = State::STOP;
  cancel_repeating_timer(&timer_);
  gpio_put(SUCTION_EN, 0);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_B, 0);
}

// ============================================================
// test_direct: 診断用。enable()→5秒運転→disable()
// ============================================================
void BldcActuator::test_direct(float amplitude_pct) {
  (void)amplitude_pct;
  if (enabled_) disable();
  enable();
  sleep_ms(5000);
  //   for (int i = 0; i < 25; ++i) {
  //   sleep_ms(200);
  //   printf("bldc: state=%d hz=%.0f amp=%.3f\n",
  //          (int)state_, (double)elec_hz_,
  //          (double)amp_from_hz(elec_hz_, amp_gain_));
  // }
  disable();
}
