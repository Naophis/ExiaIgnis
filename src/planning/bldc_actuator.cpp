#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <initializer_list>
#include <stdio.h>

static constexpr float TWO_PI = 6.2831853f;

// V/Hz パラメータ (sample/bldc_pio での実機検証値。ALIGN/低速域はAMP_BASE、
// それ以上はhzに比例させ、MAX_AMPでクランプする点はsine版と同じ考え方)
static constexpr float AMP_BASE        = 0.06f;
static constexpr float AMP_BASE_HZ     = 120.0f;
static constexpr float MAX_AMP         = 0.35f;
static constexpr float START_ELEC_HZ   = 1.0f;
static constexpr float RAMP_END_HZ     = 1200.0f;

static constexpr uint32_t BLDC_PWM_FREQ_HZ = 80000u;
static constexpr uint32_t ALIGN_TICKS = (uint32_t)(BldcActuator::CONTROL_HZ) * 600 / 1000;

// [実機知見] 1〜2相をGNDに固定する疑似6-step(旧方式)は実機で全く回転せず、
// 全相を常に50%中心(HIGH=0.5+amp/LOW=0.5-amp/FLOAT=0.5)でチョップし続ける
// 教科書通りの台形波駆動でのみ回転できることが sample/bldc_pio で確認できた。
// GND固定を一切作らないことが肝(sample/bldc.cppのsine駆動と同じ電気的性質)。
enum class PhaseRole : uint8_t { ROLE_LOW = 0, ROLE_FLOAT = 1, ROLE_HIGH = 2 };

// [sector][U,V,W] のロール表。standard 6-step commutation table
// (常にHIGH1相・LOW1相・FLOAT1相)。
static constexpr PhaseRole SECTOR_ROLE[6][3] = {
    {PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_LOW,   PhaseRole::ROLE_FLOAT},  // sector0
    {PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_LOW},    // sector1
    {PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_LOW},    // sector2
    {PhaseRole::ROLE_LOW,   PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_FLOAT},  // sector3
    {PhaseRole::ROLE_LOW,   PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_HIGH},   // sector4
    {PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_LOW,   PhaseRole::ROLE_HIGH},   // sector5
};

__attribute__((noinline, section(".time_critical.bldc_tick")))
static inline float amp_from_hz(float hz, float gain) {
  if (hz <= AMP_BASE_HZ) return AMP_BASE;
  const float a = AMP_BASE * (hz / AMP_BASE_HZ) * gain;
  return a < AMP_BASE ? AMP_BASE : (a > MAX_AMP ? MAX_AMP : a);
}

static inline int sector_of(float theta) {
  int s = (int)(theta / (TWO_PI / 6.0f));
  if (s < 0) s = 0;
  if (s > 5) s = 5;
  return s;
}

static inline float role_duty(PhaseRole role, float amp) {
  switch (role) {
  case PhaseRole::ROLE_HIGH: return 0.5f + amp;
  case PhaseRole::ROLE_LOW:  return 0.5f - amp;
  default:                   return 0.5f;
  }
}

// PWM CC レジスタに3相台形波を書く
// slice_s1 = slice4 (GPIO9=PWM4B=U相)
// slice_s2 = slice5 (GPIO10=PWM5A=V相, GPIO11=PWM5B=W相)
// reverse=true: V/W入れ替えで回転方向反転
__attribute__((noinline, section(".time_critical.bldc_tick")))
static inline void write_phase(uint s1, uint s2, uint32_t wrap,
                                float theta, float amp, bool reverse) {
  const PhaseRole *role = SECTOR_ROLE[sector_of(theta)];
  const float   wu1 = (float)(wrap + 1u);
  const uint16_t lu = (uint16_t)(wu1 * role_duty(role[0], amp));
  const uint16_t lv = (uint16_t)(wu1 * role_duty(role[1], amp));
  const uint16_t lw = (uint16_t)(wu1 * role_duty(role[2], amp));
  pwm_hw->slice[s1].cc = (uint32_t)lu << 16;
  pwm_hw->slice[s2].cc = reverse
      ? (((uint32_t)lv << 16) | lw)
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
// add_repeating_timer_us コールバック (sample/bldc.cpp と同方式)
// ============================================================
bool BldcActuator::timer_cb(repeating_timer_t *rt) {
  auto *self = static_cast<BldcActuator *>(rt->user_data);
  self->step();
  return true;  // 継続
}

__attribute__((noinline, section(".time_critical.bldc_tick")))
void BldcActuator::step() {
  constexpr float dt = 1.0f / (float)CONTROL_HZ;

  switch (state_) {
  case State::STOP:
    break;

  case State::ALIGN:
    write_phase(slice_s1_, slice_s2_, wrap_, 0.0f, AMP_BASE, reverse_);
    {
      uint32_t cnt = state_cnt_ + 1u;
      state_cnt_ = cnt;
      if (cnt >= ALIGN_TICKS) {
        state_     = State::RAMP;
        state_cnt_ = 0u;
        theta_     = 0.0f;
        elec_hz_   = START_ELEC_HZ;
      }
    }
    break;

  case State::RAMP: {
    elec_hz_ += ramp_hz_per_sec_ * dt;
    if (elec_hz_ > RAMP_END_HZ) elec_hz_ = RAMP_END_HZ;
    theta_ += TWO_PI * elec_hz_ * dt;
    if (theta_ >= TWO_PI) theta_ -= TWO_PI;
    write_phase(slice_s1_, slice_s2_, wrap_, theta_,
                amp_from_hz(elec_hz_, amp_gain_), reverse_);
    if (elec_hz_ >= RAMP_END_HZ) state_ = State::RUN;
    break;
  }

  case State::RUN:
    if (elec_hz_ < target_elec_hz_) {
      elec_hz_ += ramp_hz_per_sec_ * dt;
      if (elec_hz_ > target_elec_hz_) elec_hz_ = target_elec_hz_;
    } else if (elec_hz_ > target_elec_hz_) {
      elec_hz_ -= ramp_hz_per_sec_ * dt;
      if (elec_hz_ < target_elec_hz_) elec_hz_ = target_elec_hz_;
    }
    theta_ += TWO_PI * elec_hz_ * dt;
    if (theta_ >= TWO_PI) theta_ -= TWO_PI;
    write_phase(slice_s1_, slice_s2_, wrap_, theta_,
                amp_from_hz(elec_hz_, amp_gain_), reverse_);
    break;
  }
}

void BldcActuator::set_duty(float /*duty_pct*/) {}

// ============================================================
// enable
// ============================================================
void BldcActuator::enable() {
  if (enabled_) return;
  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));
  write_phase(slice_s1_, slice_s2_, wrap_, 0.0f, AMP_BASE, reverse_);
  gpio_put(SUCTION_EN, 1);
  sleep_ms(20);
  state_     = State::ALIGN;
  theta_     = 0.0f;
  elec_hz_   = 0.0f;
  state_cnt_ = 0u;
  amp_gain_  = 0.10f;
  enabled_   = true;
  // sample/bldc.cpp と同じ: add_repeating_timer_us(12, cb, this, &timer)
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
  printf("bldc: ALIGN start (600ms)...\n");
  sleep_ms(700);
  printf("bldc: RAMP start, hz=%.0f\n", (double)elec_hz_);
  sleep_ms(2000);
  printf("bldc: RUN, hz=%.0f target=%.0f\n", (double)elec_hz_, (double)target_elec_hz_);
  sleep_ms(2300);
  printf("bldc: done, hz=%.0f\n", (double)elec_hz_);
  disable();
}
