#pragma once

#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/types.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (DMA-driven Center-aligned SPWM)
//
// 動作原理:
//   PWM4 (slice_s1_) が wrap するたびに DREQ_PWM_WRAP が発火し、DMA が sine
//   テーブルの次エントリを PWM CC レジスタに書き込む。CPU 割り込みは不要。
//   電気角周波数 = MOTOR_PWM_FREQ_HZ / TABLE_SIZE
//
// ピン割り当て:
//   GPIO8  (PWM4A) = U 相
//   GPIO9  (PWM4B) = V 相
//   GPIO10 (PWM5A) = W 相
//   GPIO11         = SUCTION_EN (HW イネーブル)
//
// TABLE_SIZE を変えると電気角周波数が変わる (power-of-2 必須):
//   32 → 37500/32 ≈ 1172 Hz: 1PP ≈ 70,300 RPM, 2PP ≈ 35,150 RPM
//   64 → 37500/64 ≈  586 Hz: 1PP ≈ 35,150 RPM, 2PP ≈ 17,580 RPM
class BldcActuator {
public:
  // power-of-2 必須。ring_size_bits = log2(TABLE_SIZE * 4) = 7 (for 32 entries)
  static constexpr int TABLE_SIZE      = 32;
  static constexpr int RING_SIZE_BITS  = 7;   // 2^7 = 128 bytes = TABLE_SIZE * sizeof(uint32_t)

  // GPIO/PWM/DMA 初期化。Core0 の main から呼ぶ。
  void init();

  // デューティ比 [%] → 振幅を更新しサインテーブルを再計算。
  // enable 中は即座に DMA 出力に反映される。
  void set_duty(float duty_pct);

  // DMA・PWM スライスを有効化して SPWM 出力を開始。
  void enable();

  // DMA を停止し PWM スライスを無効化。
  void disable();

  bool is_enabled() const { return enabled_; }

private:
  // amplitude_ を使ってサインテーブルを再計算する
  void rebuild_tables();

  uint     slice_s1_ = 0;   // PWM4: GPIO8(A)=U, GPIO9(B)=V
  uint     slice_s2_ = 0;   // PWM5: GPIO10(A)=W
  uint32_t wrap_     = 0;
  bool     enabled_  = false;
  float    amplitude_ = 0.0f;  // 0.0 〜 1.0

  int ch_uv_ = -1;  // DMA: PWM4 CC レジスタ (U+V 32bit 同時書き込み)
  int ch_w_  = -1;  // DMA: PWM5 CC レジスタ (W 相)

  // DMA ring mode 要件: TABLE_SIZE * 4 バイト境界にアライン
  // table_uv_: bits[15:0]=U level, bits[31:16]=V level → pwm_hw->slice[s1].cc
  // table_w_:  bits[15:0]=W level, bits[31:16]=0       → pwm_hw->slice[s2].cc
  alignas(1 << RING_SIZE_BITS) uint32_t table_uv_[TABLE_SIZE];
  alignas(1 << RING_SIZE_BITS) uint32_t table_w_[TABLE_SIZE];
};
