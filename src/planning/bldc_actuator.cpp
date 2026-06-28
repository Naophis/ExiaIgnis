#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/regs/dma.h"   // DMA_CH0_TRANS_COUNT_MODE_* defines
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>

// RP2350: TRANS_COUNT [31:28] = TRIGGER_SELF (0x1) → DMA が自動再起動する
static constexpr uint32_t TRANS_COUNT_TRIGGER_SELF =
    (DMA_CH0_TRANS_COUNT_MODE_VALUE_TRIGGER_SELF << DMA_CH0_TRANS_COUNT_MODE_LSB);

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

  rebuild_tables();  // amplitude_=0: 全エントリを 50% (ゼロ電圧) で初期化

  ch_uv_ = dma_claim_unused_channel(true);
  ch_w_  = dma_claim_unused_channel(true);
}

// ============================================================
// サインテーブル再計算 (amplitude_ から 3 相 PWM レベルを生成)
// ============================================================
void BldcActuator::rebuild_tables() {
  constexpr float TWO_PI   = 2.0f * (float)M_PI;
  constexpr float TWO_PI_3 = 2.0943951f;  // 2π/3 = 120°
  const float half_amp = amplitude_ * 0.5f;
  const float wu1      = (float)(wrap_ + 1u);

  for (int i = 0; i < TABLE_SIZE; i++) {
    const float theta = TWO_PI * i / TABLE_SIZE;
    const auto u = (uint16_t)((0.5f + half_amp * sinf(theta))          * wu1);
    const auto v = (uint16_t)((0.5f + half_amp * sinf(theta - TWO_PI_3)) * wu1);
    const auto w = (uint16_t)((0.5f + half_amp * sinf(theta + TWO_PI_3)) * wu1);
    table_uv_[i] = ((uint32_t)v << 16) | u;  // PWM4 CC: [31:16]=V, [15:0]=U
    table_w_[i]  = w;                         // PWM5 CC: [15:0]=W (上位=0 未使用)
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
// 有効化: DMA を設定・起動して SPWM 出力を開始
// ============================================================
void BldcActuator::enable() {
  rebuild_tables();

  // --- ch_uv_: PWM4 wrap DREQ → table_uv_ → PWM4 CC (U+V) ---
  dma_channel_config c = dma_channel_get_default_config(ch_uv_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pwm_get_dreq(slice_s1_));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, RING_SIZE_BITS);  // read ring: 128 bytes
  dma_channel_configure(ch_uv_, &c,
      &pwm_hw->slice[slice_s1_].cc,  // dst: PWM4 CC (固定)
      table_uv_,                     // src: ring バッファ先頭
      TABLE_SIZE,                    // count (TRIGGER_SELF で上書き)
      false);
  // TRIGGER_SELF モードに切り替え: 読了後に自動再起動 (CPU 割り込み不要)
  dma_hw->ch[ch_uv_].transfer_count = TRANS_COUNT_TRIGGER_SELF | TABLE_SIZE;

  // --- ch_w_: PWM5 wrap DREQ → table_w_ → PWM5 CC (W) ---
  c = dma_channel_get_default_config(ch_w_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pwm_get_dreq(slice_s2_));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, RING_SIZE_BITS);
  dma_channel_configure(ch_w_, &c,
      &pwm_hw->slice[slice_s2_].cc,
      table_w_,
      TABLE_SIZE,
      false);
  dma_hw->ch[ch_w_].transfer_count = TRANS_COUNT_TRIGGER_SELF | TABLE_SIZE;

  // 両チャンネルを同時に起動 (DREQ 待機へ)
  dma_start_channel_mask((1u << ch_uv_) | (1u << ch_w_));

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
