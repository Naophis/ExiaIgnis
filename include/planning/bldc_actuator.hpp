#pragma once

#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/types.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (DMA SPWM, RP2350専用)
//
// DMA chain 方式: data ch → (TABLE_SIZE 転送後) → ctrl ch → data ch へ count を書き戻し再起動
// ENDLESS / TRIGGER_SELF と違い、ring + chain の組み合わせは動作が保証されている。
//
// GPIO8 (PWM4A)=U相 / GPIO9 (PWM4B)=V相 / GPIO10 (PWM5A)=W相 / GPIO11=SUCTION_EN
// f_elec = MOTOR_PWM_FREQ_HZ / TABLE_SIZE = 37500/32 ≈ 1172 Hz
class BldcActuator {
public:
  static constexpr int TABLE_SIZE = 32;
  static constexpr int RING_BITS  = 7;   // 2^7=128 bytes = 32 entries × 4 bytes

  void init();
  void set_duty(float duty_pct);   // Core1 IRQ から呼ぶ
  void enable();                   // Core0 から呼ぶ
  void disable();                  // Core0 から呼ぶ

  // ハードウェア診断: DMA を使わず Core0 から直接 PWM CC に sine 波を書く (~1秒)
  // 回れば PWM/MP6540H は正常 → DMA 問題。回らなければ配線/ドライバ問題。
  void test_direct(float amplitude_pct);

  bool is_enabled()  const { return enabled_; }
  bool is_dma_busy() const {
    return dma_channel_is_busy(ch_uv_) || dma_channel_is_busy(ch_w_);
  }

private:
  void rebuild_tables();

  uint     slice_s1_ = 0;
  uint     slice_s2_ = 0;
  uint32_t wrap_     = 0;
  bool     enabled_  = false;
  float    amplitude_ = 0.0f;

  // data ch: sine table (ring) → PWM CC, chain_to ctrl ch
  // ctrl ch: ctrl_n_ → data ch の al1_transfer_count_trig (data ch を再起動)
  int      ch_uv_      = -1;
  int      ch_ctrl_uv_ = -1;
  int      ch_w_       = -1;
  int      ch_ctrl_w_  = -1;
  uint32_t ctrl_n_     = TABLE_SIZE;   // ctrl ch が data ch に書く値 (= TABLE_SIZE)

  alignas(1 << RING_BITS) uint32_t table_uv_[TABLE_SIZE];
  alignas(1 << RING_BITS) uint32_t table_w_[TABLE_SIZE];
};
