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

  slice_s1_ = pwm_gpio_to_slice_num(SUCTION_PWM1);
  slice_s2_ = pwm_gpio_to_slice_num(SUCTION_PWM3);
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

  // ctrl ch の設定は固定 (以降変更不要)。
  // ch_ctrl_uv_: ctrl_n_ を ch_uv_.al1_transfer_count_trig へ書くことで ch_uv_ を再起動する。
  // DREQ_FORCE で即時転送 (1 ワードだけ)。chain_to = self でそのまま停止。
  {
    dma_channel_config c = dma_channel_get_default_config(ch_ctrl_uv_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    // chain_to はデフォルト (= self = no chain)
    dma_channel_configure(ch_ctrl_uv_, &c,
        &dma_hw->ch[ch_uv_].al1_transfer_count_trig,
        &ctrl_n_, 1, false);
  }
  {
    dma_channel_config c = dma_channel_get_default_config(ch_ctrl_w_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(ch_ctrl_w_, &c,
        &dma_hw->ch[ch_w_].al1_transfer_count_trig,
        &ctrl_n_, 1, false);
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
    table_uv_[i] = ((uint32_t)v << 16) | u;
    table_w_[i]  = w;
  }
}

// ============================================================
// set_duty: 振幅更新 (Core1 IRQ から)
// ============================================================
void BldcActuator::set_duty(float duty_pct) {
  amplitude_ = std::clamp(duty_pct, 0.0f, 100.0f) / 100.0f;
  if (!enabled_) return;
  rebuild_tables();
}

// ============================================================
// enable: DMA chain 起動 + PWM 有効化 (Core0)
// ============================================================
void BldcActuator::enable() {
  amplitude_ = 0.0f;
  rebuild_tables();

  // data ch_uv_: ring でテーブルを循環読み出し → PWM4 CC、完了後 ch_ctrl_uv_ へ chain
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
  // data ch_w_: ring でテーブルを循環読み出し → PWM5 CC、完了後 ch_ctrl_w_ へ chain
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

  // data ch を起動。TABLE_SIZE 転送後に ctrl ch が自動で data ch を再起動し続ける。
  dma_start_channel_mask((1u << ch_uv_) | (1u << ch_w_));
  gpio_put(SUCTION_EN, 1);
  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));
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
  pwm_set_chan_level(slice_s1_, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_A, 0);
}

// ============================================================
// test_direct: DMA を使わず Core0 から直接 PWM CC へ sine 波を書く (~3秒)
//
// 100 Hz からスタートして 1172 Hz まで周波数をランプアップし、
// その後 2 秒間定速運転する。
// 1172 Hz (= 35K RPM for 1PP) から突然スタートすると引き込み失敗するため
// 低周波数スタートが必要。
//
// 回れば → PWM/MP6540H は正常、DMA が問題
// 回らなければ → 配線 / EN ピン / MP6540H FAULT / モーター断線の問題
// ============================================================
void BldcActuator::test_direct(float amplitude_pct) {
  if (enabled_) disable();

  // テスト振幅でテーブルを事前計算 (sinf をループ内で呼ばない)
  amplitude_ = std::clamp(amplitude_pct, 0.0f, 100.0f) / 100.0f;
  rebuild_tables();

  gpio_put(SUCTION_EN, 1);
  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));

  // Phase 1: 周波数ランプ 100 Hz → 1172 Hz (約 300 サイクル)
  // 各サイクルで step_us = 1e6 / (f * TABLE_SIZE) を計算して待機
  constexpr float F_START    = 100.0f;
  constexpr float F_END      = 1172.0f;
  constexpr int   RAMP_N     = 300;
  const float df = (F_END - F_START) / RAMP_N;

  float f = F_START;
  for (int cyc = 0; cyc < RAMP_N; cyc++) {
    uint32_t step_us = (uint32_t)(1000000.0f / (f * TABLE_SIZE));
    if (step_us < 2) step_us = 2;
    for (int i = 0; i < TABLE_SIZE; i++) {
      pwm_hw->slice[slice_s1_].cc = table_uv_[i];
      pwm_hw->slice[slice_s2_].cc = table_w_[i];
      busy_wait_us_32(step_us);
    }
    f += df;
  }

  // Phase 2: 1172 Hz で 2 秒間定速運転
  for (int cycle = 0; cycle < 2344; cycle++) {
    for (int i = 0; i < TABLE_SIZE; i++) {
      pwm_hw->slice[slice_s1_].cc = table_uv_[i];
      pwm_hw->slice[slice_s2_].cc = table_w_[i];
      busy_wait_us_32(27);
    }
  }

  gpio_put(SUCTION_EN, 0);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
}
