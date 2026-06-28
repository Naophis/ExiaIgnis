#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/regs/dma.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>

// RP2350: TRANS_COUNT [31:28] = 0x1 → TRIGGER_SELF (自動再起動)
static constexpr uint32_t TRIGGER_SELF_BITS =
    (DMA_CH0_TRANS_COUNT_MODE_VALUE_TRIGGER_SELF << DMA_CH0_TRANS_COUNT_MODE_LSB);

// ============================================================
// V/f ステップ: duty [%] → TABLE_SIZE
// ============================================================
int BldcActuator::duty_to_n(float duty_pct) {
  if (duty_pct < 33.0f) return 128;  // 293 Hz: 2PP ≈  8,800 RPM (低速起動)
  if (duty_pct < 67.0f) return 64;   // 586 Hz: 2PP ≈ 17,600 RPM
  return 32;                          //1172 Hz: 2PP ≈ 35,200 RPM (定格)
}

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

  slice_s1_ = pwm_gpio_to_slice_num(SUCTION_PWM1);
  slice_s2_ = pwm_gpio_to_slice_num(SUCTION_PWM3);
  wrap_     = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

  for (uint s : {slice_s1_, slice_s2_}) {
    pwm_set_clkdiv_int_frac4(s, 1, 0);
    pwm_set_wrap(s, wrap_);
    pwm_set_enabled(s, false);
  }

  n_ = MAX_TABLE_SIZE;  // 起動時は最低速ステップ
  rebuild_tables();

  ch_uv_ = dma_claim_unused_channel(true);
  ch_w_  = dma_claim_unused_channel(true);
}

// ============================================================
// サインテーブル再計算
// ============================================================
void BldcActuator::rebuild_tables() {
  constexpr float TWO_PI   = 2.0f * (float)M_PI;
  constexpr float TWO_PI_3 = 2.0943951f;
  const float half_amp = amplitude_ * 0.5f;
  const float wu1      = (float)(wrap_ + 1u);

  for (int i = 0; i < n_; i++) {
    const float theta = TWO_PI * i / n_;
    const auto u = (uint16_t)((0.5f + half_amp * sinf(theta))             * wu1);
    const auto v = (uint16_t)((0.5f + half_amp * sinf(theta - TWO_PI_3)) * wu1);
    const auto w = (uint16_t)((0.5f + half_amp * sinf(theta + TWO_PI_3)) * wu1);
    table_uv_[i] = ((uint32_t)v << 16) | u;
    table_w_[i]  = w;
  }
}

// ============================================================
// DMA 設定・起動 (n_ に合わせた ring_bits で構成)
// ============================================================
void BldcActuator::restart_dma() {
  // ring_bits = log2(n_ * 4 bytes):
  //   n=32  → 128 bytes → bits=7
  //   n=64  → 256 bytes → bits=8
  //   n=128 → 512 bytes → bits=9
  // alignas(512) のテーブルはどの ring サイズも満たす
  int ring_bits = 2;
  while ((1 << ring_bits) < n_ * (int)sizeof(uint32_t)) ring_bits++;

  dma_channel_abort(ch_uv_);
  dma_channel_abort(ch_w_);

  // ch_uv_: PWM4 wrap DREQ → table_uv_ → PWM4 CC (U+V)
  dma_channel_config c = dma_channel_get_default_config(ch_uv_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pwm_get_dreq(slice_s1_));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, ring_bits);
  dma_channel_configure(ch_uv_, &c, &pwm_hw->slice[slice_s1_].cc, table_uv_, n_, false);
  dma_hw->ch[ch_uv_].transfer_count = TRIGGER_SELF_BITS | (uint32_t)n_;

  // ch_w_: PWM5 wrap DREQ → table_w_ → PWM5 CC (W)
  c = dma_channel_get_default_config(ch_w_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pwm_get_dreq(slice_s2_));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, ring_bits);
  dma_channel_configure(ch_w_, &c, &pwm_hw->slice[slice_s2_].cc, table_w_, n_, false);
  dma_hw->ch[ch_w_].transfer_count = TRIGGER_SELF_BITS | (uint32_t)n_;

  dma_start_channel_mask((1u << ch_uv_) | (1u << ch_w_));
}

// ============================================================
// デューティ更新 (Core1 の 1kHz IRQ から呼ばれる)
// ============================================================
void BldcActuator::set_duty(float duty_pct) {
  const float d = std::clamp(duty_pct, 0.0f, 100.0f);
  amplitude_    = d / 100.0f;

  if (!enabled_) return;

  const int new_n = duty_to_n(d);
  if (new_n != n_) {
    // V/f ステップ切り替え: テーブル再計算 + DMA 瞬時再起動 (~10 µs)
    n_ = new_n;
    rebuild_tables();
    restart_dma();
  } else {
    // 同ステップ内: テーブルをその場で更新 (DMA 停止不要、< 1ms で反映)
    rebuild_tables();
  }
}

// ============================================================
// 有効化
// ============================================================
void BldcActuator::enable() {
  n_ = MAX_TABLE_SIZE;  // 最低速ステップから開始
  amplitude_ = 0.0f;
  rebuild_tables();
  restart_dma();

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
