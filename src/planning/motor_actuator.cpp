#include "planning/motor_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cmath>
#include <stdio.h>
#include <initializer_list>

void MotorActuator::init(uint32_t motor_hz, uint32_t control_hz) {
  motor_hz_ = motor_hz;

  const uint pwm_pins[] = {M_PWM_L1, M_PWM_L2, M_PWM_R1, M_PWM_R2};
  for (uint pin : pwm_pins) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
  }

  slice_L_ = pwm_gpio_to_slice_num(M_PWM_L1);
  slice_R_ = pwm_gpio_to_slice_num(M_PWM_R1);

  motor_wrap_ = (uint32_t)(clock_get_hz(clk_sys) / motor_hz_) - 1u;
  if (motor_wrap_ > 0xFFFFu) motor_wrap_ = 0xFFFFu;
  q16_per_pct_ = (float)(motor_wrap_ + 1u) * 65536.0f / 100.0f;

  // ---- DitherPwm(Hardware PWM + DMA) ----
  //   samples_per_tick: 制御周期あたりの PWM 周期数(切り上げ)。100kHz/1kHz=100
  //   lead 4          : 指令レイテンシ = 4 周期 + 1 周期(latch) ≒ 50us @100kHz
  //   guard = M       : CPU が遅れても 1 制御周期ぶんは「最後の duty」を保持
  dpwm::DitherPwm::Config cfg;
  cfg.top              = (uint16_t)motor_wrap_;
  cfg.clkdiv_int       = 1;
  cfg.clkdiv_frac4     = 0;
  cfg.samples_per_tick = (uint16_t)((motor_hz_ + control_hz - 1u) / control_hz);
  cfg.lead_samples     = 4;
  cfg.guard_samples    = 0;
  cfg.max_level        = 0;      // top+1 = 100%
  cfg.high_priority    = true;

  const uint slices[2] = {slice_L_, slice_R_};
  dither_ok_ = pwm_.init(cfg, slices, 2);
  legacy_ = !dither_ok_;

  if (!legacy_) {
    // 従来と同じく init 直後から PWM を duty 0 で走らせる
    pwm_.set_levels_q16(0, 0, 0);
    pwm_.set_levels_q16(1, 0, 0);
    pwm_.start();
  } else {
    // フォールバック: 従来の CC 直書き
    for (uint slice : {slice_L_, slice_R_}) {
      pwm_set_clkdiv_int_frac4(slice, 1, 0);
      pwm_set_wrap(slice, motor_wrap_);
      pwm_set_enabled(slice, true);
    }
    legacy_all_low_();
  }
}

// duty[%] -> (A, B) の Q16.16 count。方向の割り当ては従来と同一:
//   duty >= 0: A=0, B=level / duty < 0: A=level, B=0
// MOTOR_DRIVE_SLOW_DECAY=1 のときは同じ方向割り当てのまま drive<->brake にする:
//   駆動側(従来 level を出していた側) = 100%(H 固定)
//   反対側(従来 0 だった側)           = 100% - level (反転 PWM)
//   → 両方 H の区間が brake、反対側が L の区間が drive。duty=0 は両方 H = brake。
__attribute__((noinline, section(".time_critical.motor_actuator")))
void MotorActuator::apply(float duty_l, float duty_r) {
  if (legacy_) { apply_legacy_(duty_l, duty_r); return; }

  auto to_q16 = [&](float duty) -> uint32_t {
    float q = std::fabs(duty) * q16_per_pct_ + 0.5f;
    if (q > 4294967040.0f) q = 4294967040.0f;   // uint32 上限(set_levels_q16 で max_level にクランプ)
    return (uint32_t)q;
  };
#if MOTOR_DRIVE_SLOW_DECAY
  const uint32_t full = (motor_wrap_ + 1u) << 16;   // 100% (H 固定)
  auto clamp_full = [&](uint32_t q) -> uint32_t { return q > full ? full : q; };
  const uint32_t ql = clamp_full(to_q16(duty_l));
  const uint32_t qr = clamp_full(to_q16(duty_r));
  if (duty_l <= 0.0f) pwm_.set_levels_q16(0, full - ql, full); else pwm_.set_levels_q16(0, full, full - ql);
  if (duty_r >= 0.0f) pwm_.set_levels_q16(1, full - qr, full); else pwm_.set_levels_q16(1, full, full - qr);
#else
  const uint32_t ql = to_q16(duty_l);
  const uint32_t qr = to_q16(duty_r);
  if (duty_l <= 0.0f) pwm_.set_levels_q16(0, 0u, ql); else pwm_.set_levels_q16(0, ql, 0u);
  if (duty_r >= 0.0f) pwm_.set_levels_q16(1, 0u, qr); else pwm_.set_levels_q16(1, qr, 0u);
#endif

  // 左右の指令を揃えてから 1 回だけ update: 両 slice のリングに同じ read 位置基準で
  // commit されるので、同じ wrap で反映される。
  pwm_.update();
}

__attribute__((noinline, section(".time_critical.motor_actuator")))
void MotorActuator::apply_legacy_(float duty_l, float duty_r) {
  // drive: 従来 level を出していた側 / other: 従来 0 だった側
  auto set_drive = [&](uint slice, uint drive, uint other, float duty) {
    const uint32_t full = motor_wrap_ + 1u;
    uint32_t level = (uint32_t)((float)full * std::fabs(duty) / 100.0f);
    if (level > full) level = full;
#if MOTOR_DRIVE_SLOW_DECAY
    pwm_set_chan_level(slice, drive, (uint16_t)full);
    pwm_set_chan_level(slice, other, (uint16_t)(full - level));
#else
    pwm_set_chan_level(slice, drive, (uint16_t)level);
    pwm_set_chan_level(slice, other, 0);
#endif
  };

  if (duty_l >= 0.0f) set_drive(slice_L_, PWM_CHAN_A, PWM_CHAN_B, duty_l);
  else                set_drive(slice_L_, PWM_CHAN_B, PWM_CHAN_A, duty_l);
  if (duty_r >= 0.0f) set_drive(slice_R_, PWM_CHAN_B, PWM_CHAN_A, duty_r);
  else                set_drive(slice_R_, PWM_CHAN_A, PWM_CHAN_B, duty_r);
}

// 従来経路の全出力 LOW(= MPQ6612A では coast)。slice を止める前・経路切替時用。
// slow decay では duty=0 が「両方 H(brake)」なので apply_legacy_(0,0) では LOW にならない。
__attribute__((noinline, section(".time_critical.motor_actuator")))
void MotorActuator::legacy_all_low_() {
  for (uint slice : {slice_L_, slice_R_}) {
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);
  }
}

__attribute__((noinline, section(".time_critical.motor_actuator")))
bool MotorActuator::set_dither(bool enable) {
  if (enable && !dither_ok_) return false;
  if (enable == !legacy_) return true;   // no change
  if (!enable) {
    // DMA を止めて CC=0(次の wrap で LOW)。slice は動かしたままにして従来経路へ。
    pwm_.stop(false);
    legacy_ = true;
    for (uint slice : {slice_L_, slice_R_}) {
      pwm_set_clkdiv_int_frac4(slice, 1, 0);
      pwm_set_wrap(slice, motor_wrap_);
      pwm_set_enabled(slice, motor_en);
    }
    legacy_all_low_();
  } else {
    // 従来経路 → DitherPwm。モーター有効中なら start() で DMA を回す。
    legacy_all_low_();
    legacy_ = false;
    pwm_.set_levels_q16(0, 0, 0);
    pwm_.set_levels_q16(1, 0, 0);
    if (motor_en) pwm_.start();
  }
  return true;
}

__attribute__((noinline, section(".time_critical.motor_actuator")))
void MotorActuator::probe_latency() {
  if (legacy_) { printf("[dprobe] legacy path (dither inactive)\n"); return; }
  const uint32_t level = (motor_wrap_ + 1u) / 4u;             // 25%duty: 1 周期だけ HIGH
  const uint32_t word  = level & 0xFFFFu;                       // A=level, B=0 (GPIO M_PWM_L1)
  printf("[dprobe] top=%lu period=%lu us running=%d M=%u lead=%u ring=%u\n", (unsigned long)motor_wrap_,
         (unsigned long)pwm_.period_us(), (int)pwm_.running(), (unsigned)0, (unsigned)4, (unsigned)dpwm::DitherPwm::kRingSamples);
  for (int trial = 0; trial < 6; ++trial) {
    const uint32_t r0   = pwm_.dma_read_index(0);
    const uint32_t ctr0 = pwm_.ctr_reg(0);
    const uint64_t t0   = time_us_64();
    const uint32_t pos  = pwm_.probe_write(0, 4, word);
    uint32_t t_cc = 0, t_pin = 0, r_cc = 0, r_pin = 0, cc_seen = 0;
    while (time_us_64() - t0 < 4000) {
      const uint32_t cc = pwm_.cc_reg(0);
      if (!t_cc && cc == word) { t_cc = (uint32_t)(time_us_64() - t0); r_cc = pwm_.dma_read_index(0); cc_seen = cc; }
      if (!t_pin && gpio_get(M_PWM_L1)) { t_pin = (uint32_t)(time_us_64() - t0); r_pin = pwm_.dma_read_index(0); }
      if (t_cc && t_pin) break;
    }
    const uint32_t ring_now = pwm_.ring_word(0, pos);
    printf("[dprobe] #%d r0=%lu ctr0=%lu wrote@%lu | cc_seen=%s t_cc=%lu us (r=%lu) | pin_high=%s t_pin=%lu us (r=%lu) | ring[pos] now=0x%08lx%s\n",
           trial, (unsigned long)r0, (unsigned long)ctr0, (unsigned long)(pos & 0xFF),
           t_cc ? "yes" : "NO", (unsigned long)t_cc, (unsigned long)r_cc,
           t_pin ? "yes" : "NO", (unsigned long)t_pin, (unsigned long)r_pin,
           (unsigned long)ring_now, (ring_now != word) ? " (overwritten by update)" : "");
    (void)cc_seen;
    busy_wait_us(2500);
  }
  const dpwm::DitherStats& s = pwm_.stats(0);
  printf("[dprobe] stats: ticks=%lu consumed=%lu last_consumed=%lu last_lead=%lu late=%lu early=%lu underrun=%lu stall=%lu backlog_max=%lu skew_max=%lu\n",
         (unsigned long)s.ticks, (unsigned long)s.consumed_total, (unsigned long)s.last_consumed, (unsigned long)s.last_lead,
         (unsigned long)s.late_samples, (unsigned long)s.early_samples, (unsigned long)s.underrun_events, (unsigned long)s.stall_ticks,
         (unsigned long)s.dreq_backlog_max, (unsigned long)s.skew_max);
}


__attribute__((noinline, section(".time_critical.motor_actuator")))
void MotorActuator::motor_enable() {
  motor_en = true;
  if (!legacy_) {
    pwm_.start();   // 既に動いていれば no-op
    return;
  }
  pwm_set_enabled(slice_L_, true);
  pwm_set_enabled(slice_R_, true);
}

__attribute__((noinline, section(".time_critical.motor_actuator")))
void MotorActuator::motor_disable() {
  motor_en = false;
  if (!legacy_) {
    // DMA abort -> CC=0 -> 2 周期待って slice 停止。出力が LOW に落ちてから止める
    // (途中で止めると位相によっては HIGH で固定される)のは従来と同じ。
    pwm_.stop(true);
    return;
  }
  // pwm_set_enabled(false) はカウンタを即座に止めるため、直前の duty の
  // 位相によっては出力ピンが HIGH のまま固定されてしまう（duty=0 の CC 値は
  // 次の wrap まで反映されない）。先に duty=0 を書き込み、1 周期分待って
  // 出力が確実に LOW に落ちてから PWM を止める。
  legacy_all_low_();
  busy_wait_us(2u * 1000000u / motor_hz_);
  pwm_set_enabled(slice_L_, false);
  pwm_set_enabled(slice_R_, false);
}
