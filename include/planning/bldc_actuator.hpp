#pragma once

#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/types.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (DMA SPWM, RP2350専用)
//
// RP2350 DMA の TRIGGER_SELF モードを使い、PWM carrier wrap DREQ のたびに
// sine テーブルのエントリを PWM CC レジスタへ書き込む。
// テーブルを読み終えると DMA が自動再起動するため CPU 割り込み不要。
//
// GPIO8 (PWM4A) = U相 / GPIO9 (PWM4B) = V相 / GPIO10 (PWM5A) = W相
// GPIO11 = SUCTION_EN (MP6540H HW イネーブル)
//
// 電気角周波数 = MOTOR_PWM_FREQ_HZ / TABLE_SIZE
//   TABLE_SIZE=32 → 37500/32 ≈ 1172 Hz
//   1PP ≈ 70,300 RPM / 2PP ≈ 35,150 RPM
//
// 速度制御: set_duty(%) で振幅を変化させると、モーターはファン負荷と
//   トルクが釣り合う回転数に自然に落ち着く (V/f オープンループ制御)。
class BldcActuator {
public:
  // power-of-2 必須: ring_size_bits = log2(TABLE_SIZE * sizeof(uint32_t))
  static constexpr int      TABLE_SIZE     = 32;
  static constexpr int      RING_SIZE_BITS = 7;    // 2^7 = 128 bytes

  // GPIO/PWM 初期化・DMA チャンネル確保。Core0 の main から呼ぶ。
  void init();

  // デューティ比 [%] で振幅を更新 → サインテーブルをその場で再計算。
  // enable 中は次の DMA パス (< 1ms) から新しい振幅が反映される。
  void set_duty(float duty_pct);

  // DMA 起動・PWM 有効化。Core0 の suction_enable() 経由で呼ぶ。
  void enable();

  // DMA 停止・PWM 無効化。
  void disable();

  bool is_enabled() const { return enabled_; }

private:
  void rebuild_tables();

  uint     slice_s1_ = 0;   // PWM4: GPIO8(A)=U, GPIO9(B)=V
  uint     slice_s2_ = 0;   // PWM5: GPIO10(A)=W
  uint32_t wrap_     = 0;
  bool     enabled_  = false;
  float    amplitude_ = 0.0f;  // 0.0 〜 1.0

  int ch_uv_ = -1;  // DMA: table_uv_ → PWM4->CC (U+V 32bit 同時)
  int ch_w_  = -1;  // DMA: table_w_  → PWM5->CC (W 相)

  // DMA ring mode 要件: 2^RING_SIZE_BITS バイト境界アライン
  // table_uv_: bits[31:16]=V level, bits[15:0]=U level
  // table_w_:  bits[15:0]=W level (bits[31:16]=0, PWM5 chan B 未使用)
  alignas(1 << RING_SIZE_BITS) uint32_t table_uv_[TABLE_SIZE];
  alignas(1 << RING_SIZE_BITS) uint32_t table_w_[TABLE_SIZE];
};
