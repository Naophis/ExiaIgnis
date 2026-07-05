#pragma once
#include "hardware/pwm.h"
#include "pico/time.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (タイマーIRQ SPWM, RP2350専用)
//
// sample/bldc.cpp と同じタイマーIRQ方式。電気周波数を可変にすることで
// DMA固定方式(1172Hz)では届かなかった高速域(60,000RPM相当)を実現する。
//
// 起動シーケンス: ALIGN(600ms) → RAMP(1→TARGET_ELEC_HZ @ ramp_hz_per_sec_) → RUN
// 振幅: V/Hz制御 (AMP_BASE=0.06, AMP_BASE_HZ=120, MAX_AMP=0.35)
//
// GPIO8=SUCTION_EN / GPIO9(PWM4B)=U相 / GPIO10(PWM5A)=V相 / GPIO11(PWM5B)=W相
class BldcActuator {
public:
  // 極対数 6 → 60,000RPM / 極対数 7 → 51,400RPM / 実機に合わせて変更
  static constexpr float TARGET_ELEC_HZ = 6000.0f;
  static constexpr int   CONTROL_HZ     = 80000;   // 12.5µs — 9500Hz時 8サンプル/回転

  void init();
  void set_duty(float duty_pct);   // Core1 IRQ から (振幅ゲイン調整)
  void enable();                   // Core0 から (非同期 — すぐ返る)
  void disable();                  // Core0 から
  void set_direction(bool reverse) { reverse_ = reverse; }
  void set_min_amplitude(float v)  { (void)v; }  // API互換用
  // enable() 前に呼ぶこと。デフォルト=2000Hz/s (sample/bldc.cpp 実証値)
  void set_ramp_rate(float hz_per_sec) { ramp_hz_per_sec_ = hz_per_sec; }
  void test_direct(float amplitude_pct);

  bool is_enabled()  const { return enabled_; }
  bool is_ramping()  const { return state_ == State::ALIGN || state_ == State::RAMP; }
  bool is_dma_busy() const { return false; }  // API互換用

private:
  static bool timer_cb(repeating_timer_t *rt);
  void step();

  enum class State : uint8_t { STOP, ALIGN, RAMP, RUN };

  uint     slice_s1_ = 0;
  uint     slice_s2_ = 0;
  uint32_t wrap_     = 0;

  // timer_cb (Core0) と set_duty (Core1 IRQ) の両方からアクセス
  volatile State    state_          = State::STOP;
  volatile float    theta_          = 0.0f;
  volatile float    elec_hz_        = 0.0f;
  volatile float    amp_gain_       = 0.10f;
  volatile uint32_t state_cnt_      = 0u;
  volatile float    ramp_hz_per_sec_ = 2000.0f;  // set_ramp_rate()で変更可

  bool enabled_ = false;
  bool reverse_ = true;

  repeating_timer_t timer_{};
};
