#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>

// ============================================================
// init: GPIO / PWM / DMA チャンネル確保 + ctrl ch 設定 (Core0)
// ============================================================
void BldcActuator::init() {
  gpio_set_function(SUCTION_PWM1, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM2, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM3, GPIO_FUNC_PWM);
  gpio_init(SUCTION_EN);
  gpio_set_dir(SUCTION_EN, GPIO_OUT);
  gpio_put(SUCTION_EN, 0);

  slice_s1_ = pwm_gpio_to_slice_num(SUCTION_PWM1);  // slice4: GPIO9(B)=U
  slice_s2_ = pwm_gpio_to_slice_num(SUCTION_PWM2);  // slice5: GPIO10(A)=V, GPIO11(B)=W
  wrap_     = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

  for (uint s : {slice_s1_, slice_s2_}) {
    pwm_set_clkdiv_int_frac4(s, 1, 0);
    pwm_set_wrap(s, wrap_);
    pwm_set_enabled(s, false);
  }

  amplitude_ = 0.0f;
  rebuild_tables();

  ch_uv_      = dma_claim_unused_channel(true);
  ch_ctrl_uv_ = dma_claim_unused_channel(true);
  ch_w_       = dma_claim_unused_channel(true);
  ch_ctrl_w_  = dma_claim_unused_channel(true);
  ctrl_n_     = TABLE_SIZE;

  {
    dma_channel_config c = dma_channel_get_default_config(ch_ctrl_uv_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(ch_ctrl_uv_, &c,
        &dma_hw->ch[ch_uv_].al1_transfer_count_trig, &ctrl_n_, 1, false);
  }
  {
    dma_channel_config c = dma_channel_get_default_config(ch_ctrl_w_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(ch_ctrl_w_, &c,
        &dma_hw->ch[ch_w_].al1_transfer_count_trig, &ctrl_n_, 1, false);
  }
}

// ============================================================
// rebuild_tables: sine テーブル再計算
// ============================================================
void BldcActuator::rebuild_tables() {
  constexpr float TWO_PI   = 2.0f * (float)M_PI;
  constexpr float TWO_PI_3 = 2.0943951f;
  const float half_amp = amplitude_ * 0.5f;
  const float wu1      = (float)(wrap_ + 1u);
  for (int i = 0; i < TABLE_SIZE; i++) {
    const float theta = TWO_PI * i / TABLE_SIZE;
    const auto u = (uint16_t)((0.5f + half_amp * sinf(theta))             * wu1);
    const auto v = (uint16_t)((0.5f + half_amp * sinf(theta - TWO_PI_3)) * wu1);
    const auto w = (uint16_t)((0.5f + half_amp * sinf(theta + TWO_PI_3)) * wu1);
    table_uv_[i] = (uint32_t)u << 16;
    // reverse_=true で V/W を入れ替えると回転方向が逆になる
    table_w_[i] = reverse_ ? (((uint32_t)v << 16) | w)
                            : (((uint32_t)w << 16) | v);
  }
}

// ============================================================
// do_startup_ramp: 低周波から目標周波数まで直接 PWM CC を書いて引き込む
//
// DMA chain に引き渡す前にロータを目標速度まで加速する。
// 1172 Hz / 586 Hz などから突然スタートすると脱調するため必要。
// amp: 引き込みに使う振幅 (0.0〜1.0)。通常 ≥ 0.4 推奨。
// ============================================================
void BldcActuator::do_startup_ramp(float amp) {
  gpio_put(SUCTION_EN, 1);
  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));

  // V/f 一定ランプ: f とともに振幅を比例増加させてトルクを維持する。
  // 振幅固定で f だけ上げると高周波域でトルク不足 → 脱調 → ロータ固着 → 「ピ~」音。
  constexpr float F_START   = 10.0f;  // 低い極対数でも引き込めるよう低く
  const float     f_target  = (float)MOTOR_PWM_FREQ_HZ / TABLE_SIZE;
  constexpr int   RAMP_N    = 300;
  const float     df        = (f_target - F_START) / RAMP_N;
  constexpr float AMP_FLOOR = 0.4f;   // 低周波域の最低振幅 (引き込みトルク確保)

  float f = F_START;
  for (int cyc = 0; cyc < RAMP_N; cyc++) {
    // V/f: 振幅 = amp × (f / f_target)、ただし AMP_FLOOR 以上を保証
    const float vf_amp = amp * (f / f_target);
    amplitude_ = (vf_amp > AMP_FLOOR) ? vf_amp : AMP_FLOOR;
    rebuild_tables();

    const uint32_t step_us = (uint32_t)(1000000.0f / (f * TABLE_SIZE));
    for (int i = 0; i < TABLE_SIZE; i++) {
      pwm_hw->slice[slice_s1_].cc = table_uv_[i];
      pwm_hw->slice[slice_s2_].cc = table_w_[i];
      busy_wait_us_32(step_us < 2u ? 2u : step_us);
    }
    f += df;
  }

  // ランプ完了。目標周波数で数サイクル定速運転してロータ位相を table[0] 基準に揃える。
  // direct-write の最後が table[31] で終わり、DMA が table[0] から始めるため
  // 位相連続性がないと同期外れ(即時脱調)が起きる。
  amplitude_ = amp;
  rebuild_tables();

  const uint32_t step_us  = (uint32_t)(1000000.0f * TABLE_SIZE / MOTOR_PWM_FREQ_HZ);
  for (int cyc = 0; cyc < 5; cyc++) {
    for (int i = 0; i < TABLE_SIZE; i++) {
      pwm_hw->slice[slice_s1_].cc = table_uv_[i];
      pwm_hw->slice[slice_s2_].cc = table_w_[i];
      busy_wait_us_32(step_us < 2u ? 2u : step_us);
    }
  }
  // 最後の書き込みは table[TABLE_SIZE-1]。
  // DMA は table[0] から始まるため、次のステップとして自然に続く。
}

// ============================================================
// set_duty: 振幅更新 (Core1 IRQ から)
// ============================================================
void BldcActuator::set_duty(float duty_pct) {
  const float d = std::clamp(duty_pct, 0.0f, 100.0f) / 100.0f;
  // 定常運転中は min_amplitude_ を下限として脱調を防ぐ
  amplitude_ = (enabled_ && d < min_amplitude_) ? min_amplitude_ : d;
  if (!enabled_) return;
  rebuild_tables();
}

// ============================================================
// enable: 起動ランプ → DMA chain 引き渡し (Core0)
// ============================================================
void BldcActuator::enable() {
  // 引き込み振幅: 最低 90% を保証。ファン負荷は RPM² で増えるため高振幅が必要。
  const float pull_amp = (amplitude_ > 0.9f) ? amplitude_ : 0.9f;
  do_startup_ramp(pull_amp);
  // ここでモーターは目標速度に到達済み。amplitude_ は Core1 が更新した値になっている。

  // DMA chain 起動 (PWM/EN は do_startup_ramp 内で有効化済み)
  {
    dma_channel_config c = dma_channel_get_default_config(ch_uv_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pwm_get_dreq(slice_s1_));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, RING_BITS);
    channel_config_set_chain_to(&c, ch_ctrl_uv_);
    dma_channel_configure(ch_uv_, &c,
        &pwm_hw->slice[slice_s1_].cc, table_uv_, TABLE_SIZE, false);
  }
  {
    dma_channel_config c = dma_channel_get_default_config(ch_w_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pwm_get_dreq(slice_s2_));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, RING_BITS);
    channel_config_set_chain_to(&c, ch_ctrl_w_);
    dma_channel_configure(ch_w_, &c,
        &pwm_hw->slice[slice_s2_].cc, table_w_, TABLE_SIZE, false);
  }

  dma_start_channel_mask((1u << ch_uv_) | (1u << ch_w_));
  enabled_ = true;
}

// ============================================================
// disable
// ============================================================
void BldcActuator::disable() {
  enabled_ = false;
  gpio_put(SUCTION_EN, 0);
  dma_channel_abort(ch_uv_);
  dma_channel_abort(ch_ctrl_uv_);
  dma_channel_abort(ch_w_);
  dma_channel_abort(ch_ctrl_w_);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_B, 0);
}

// ============================================================
// test_direct: V/f ランプで引き込んでから 2 秒間定速運転 (DMA なし)
// Phase1: do_startup_ramp で V/f 一定引き込み
// Phase2: amplitude_pct で 2 秒定速
// ============================================================
void BldcActuator::test_direct(float amplitude_pct) {
  if (enabled_) disable();

  const float amp = std::clamp(amplitude_pct, 0.0f, 100.0f) / 100.0f;

  // Phase 1: V/f ランプ。引き込み振幅は amp と 70% の大きい方。
  do_startup_ramp((amp > 0.7f) ? amp : 0.7f);

  // Phase 2: amplitude_pct で 2 秒定速運転
  amplitude_ = amp;
  rebuild_tables();

  const float    f_target = (float)MOTOR_PWM_FREQ_HZ / TABLE_SIZE;
  const uint32_t step_us  = (uint32_t)(1000000.0f / (f_target * TABLE_SIZE));
  const int      n_cyc    = (int)(f_target * 2.0f);

  for (int cyc = 0; cyc < n_cyc; cyc++) {
    for (int i = 0; i < TABLE_SIZE; i++) {
      pwm_hw->slice[slice_s1_].cc = table_uv_[i];
      pwm_hw->slice[slice_s2_].cc = table_w_[i];
      busy_wait_us_32(step_us);
    }
  }

  gpio_put(SUCTION_EN, 0);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
}
