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
// 初期化 (Core0 から呼ぶ)
// ============================================================
void BldcActuator::init() {
  gpio_set_function(SUCTION_PWM1, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM2, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM3, GPIO_FUNC_PWM);

  gpio_init(SUCTION_EN);
  gpio_set_dir(SUCTION_EN, GPIO_OUT);
  gpio_put(SUCTION_EN, 0);

  slice_s1_ = pwm_gpio_to_slice_num(SUCTION_PWM1);  // slice4: GPIO8(A)+GPIO9(B)
  slice_s2_ = pwm_gpio_to_slice_num(SUCTION_PWM3);  // slice5: GPIO10(A)

  wrap_ = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

  for (uint s : {slice_s1_, slice_s2_}) {
    pwm_set_clkdiv_int_frac4(s, 1, 0);
    pwm_set_wrap(s, wrap_);
    pwm_set_enabled(s, false);
  }

  // テーブルを振幅 0 (全相 50% = ゼロ電圧) で初期化
  rebuild_tables();

  // DMA チャンネルを確保 (init 時のみ)
  ch_uv_ = dma_claim_unused_channel(true);
  ch_w_  = dma_claim_unused_channel(true);
}

// ============================================================
// サインテーブル再計算
// ============================================================
void BldcActuator::rebuild_tables() {
  constexpr float TWO_PI   = 2.0f * (float)M_PI;
  constexpr float TWO_PI_3 = 2.0943951f;  // 2π/3 = 120°
  const float half_amp = amplitude_ * 0.5f;
  const float wu1      = (float)(wrap_ + 1u);

  for (int i = 0; i < TABLE_SIZE; i++) {
    const float theta = TWO_PI * i / TABLE_SIZE;
    const auto  u = (uint16_t)((0.5f + half_amp * sinf(theta))          * wu1);
    const auto  v = (uint16_t)((0.5f + half_amp * sinf(theta - TWO_PI_3)) * wu1);
    const auto  w = (uint16_t)((0.5f + half_amp * sinf(theta + TWO_PI_3)) * wu1);
    table_uv_[i] = ((uint32_t)v << 16) | u;  // PWM4 CC: [31:16]=V, [15:0]=U
    table_w_[i]  = w;                         // PWM5 CC: [15:0]=W (上位=0, chan B未使用)
  }
}

// ============================================================
// デューティ更新 (Core1 の 1kHz IRQ から呼ばれる)
// ============================================================
void BldcActuator::set_duty(float duty_pct) {
  amplitude_ = std::clamp(duty_pct, 0.0f, 100.0f) / 100.0f;
  if (enabled_) rebuild_tables();
}

// ============================================================
// 有効化: DMA を設定して SPWM 出力を開始
// ============================================================
void BldcActuator::enable() {
  rebuild_tables();

  // PWM4 DREQ トリガで slice_s1_ の CC レジスタを DMA 更新
  const uint dreq = pwm_get_dreq(slice_s1_);

  dma_channel_config c = dma_channel_get_default_config(ch_uv_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, dreq);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, RING_SIZE_BITS);  // read ring: 2^7 = 128 bytes
  dma_channel_configure(ch_uv_, &c,
      &pwm_hw->slice[slice_s1_].cc,  // dst: PWM4 CC (固定)
      table_uv_,                     // src: UV テーブル (ring)
      0x0FFFFFFF,                    // count: 実質無限 (37.5 kHz × ~7000 s)
      true);                         // trigger: DREQ 待ちで即開始

  dma_channel_config cw = dma_channel_get_default_config(ch_w_);
  channel_config_set_transfer_data_size(&cw, DMA_SIZE_32);
  channel_config_set_dreq(&cw, dreq);  // 同じ DREQ (slice4 と slice5 は同期)
  channel_config_set_read_increment(&cw, true);
  channel_config_set_write_increment(&cw, false);
  channel_config_set_ring(&cw, false, RING_SIZE_BITS);
  dma_channel_configure(ch_w_, &cw,
      &pwm_hw->slice[slice_s2_].cc,  // dst: PWM5 CC (固定)
      table_w_,                      // src: W テーブル (ring)
      0x0FFFFFFF,
      true);

  // 両スライスを同時に有効化してフェーズを揃える
  gpio_put(SUCTION_EN, 1);
  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));
  enabled_ = true;
}

// ============================================================
// 無効化
// ============================================================
void BldcActuator::disable() {
  enabled_ = false;
  gpio_put(SUCTION_EN, 0);
  dma_channel_abort(ch_uv_);
  dma_channel_abort(ch_w_);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_A, 0);
}
