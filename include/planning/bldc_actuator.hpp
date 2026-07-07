#pragma once
#include "hardware/pwm.h"
#include "pico/time.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (alarm pool 台形波PWM, RP2350専用)
//
// add_repeating_timer_us 方式は sample/bldc.cpp と同じ。波形は当初サイン波
// (SPWM) だったが、sample/bldc_pio での実機検証の結果、1〜2相をGNDに固定する
// 疑似6-stepでは全く回転せず、全相を常に50%中心でチョップし続ける教科書通りの
// 台形波駆動(HIGH=0.5+amp/LOW=0.5-amp/FLOAT=0.5、60°ごとに役割を入れ替え)
// でのみ回転することが確認できたため、その方式に置き換えた。
//
// 起動シーケンス: ALIGN(600ms) → RAMP(1→1200Hz) → RUN(1200→target_elec_hz_ を追従)
// 振幅: V/Hz制御 (AMP_BASE=0.06, AMP_BASE_HZ=120, MAX_AMP=0.35)
//
// GPIO8=SUCTION_EN / GPIO9(PWM4B)=U相 / GPIO10(PWM5A)=V相 / GPIO11(PWM5B)=W相
class BldcActuator {
public:
  static constexpr int CONTROL_HZ = 80000;   // 12.5µs — 9500Hz時 8サンプル/回転

  void init();
  void set_duty(float duty_pct);
  void enable();
  void disable();
  void set_direction(bool reverse) { reverse_ = reverse; }
  void set_min_amplitude(float v)  { (void)v; }
  void set_ramp_rate(float hz_per_sec)  { ramp_hz_per_sec_ = hz_per_sec; }
  void set_target_hz(float hz)          { target_elec_hz_  = hz; }
  void test_direct(float amplitude_pct);

  bool  is_enabled()   const { return enabled_; }
  bool  is_ramping()   const {
    return state_ == State::ALIGN || state_ == State::RAMP ||
           (state_ == State::RUN && elec_hz_ < target_elec_hz_ - 1.0f);
  }
  bool  is_dma_busy()  const { return false; }
  float get_elec_hz()  const { return elec_hz_; }
  float get_target_hz() const { return target_elec_hz_; }

private:
  static bool timer_cb(repeating_timer_t *rt);
  void step();

  enum class State : uint8_t { STOP, ALIGN, RAMP, RUN };

  uint     slice_s1_ = 0;
  uint     slice_s2_ = 0;
  uint32_t wrap_     = 0;

  volatile State    state_           = State::STOP;
  volatile float    theta_           = 0.0f;
  volatile float    elec_hz_         = 0.0f;
  volatile float    amp_gain_        = 0.10f;
  volatile uint32_t state_cnt_       = 0u;
  volatile float    ramp_hz_per_sec_ = 2000.0f;

  float    target_elec_hz_ = 6000.0f;
  bool     enabled_        = false;
  bool     reverse_        = true;

  repeating_timer_t timer_ = {};
};
