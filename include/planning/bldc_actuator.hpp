#pragma once

#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/types.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (DMA SPWM, RP2350専用)
//
// RP2350 DMA の TRIGGER_SELF モードで sine テーブルを PWM CC レジスタへ書き込む。
// テーブル読了後は DMA が自動再起動するため CPU 割り込み不要。
//
// GPIO8 (PWM4A) = U相 / GPIO9 (PWM4B) = V相 / GPIO10 (PWM5A) = W相
// GPIO11 = SUCTION_EN (MP6540H HW イネーブル)
//
// 速度制御 (V/f 制御):
//   set_duty(%) で振幅と電気角周波数を同時スケール。duty が閾値を越えた際に
//   TABLE_SIZE (= N) を切り替え、DMA を瞬時再起動 (~10 µs) する。
//   CPU 割り込みは増えない。
//
//   duty  0- 33% → N=128 → ≈ 293 Hz: 2PP ≈  8,800 RPM (低速起動)
//   duty 33- 67% → N= 64 → ≈ 586 Hz: 2PP ≈ 17,600 RPM
//   duty 67-100% → N= 32 → ≈1172 Hz: 2PP ≈ 35,200 RPM (定格)
class BldcActuator {
public:
  // 最大テーブルサイズ (最低速ステップ / power-of-2 必須)
  static constexpr int MAX_TABLE_SIZE  = 128;
  static constexpr int MAX_RING_BITS   = 9;    // 2^9 = 512 bytes = 128 * 4

  // GPIO/PWM 初期化・DMA チャンネル確保。Core0 の main から呼ぶ。
  void init();

  // デューティ比 [%] で振幅と周波数を更新。
  // N が変わった場合のみ DMA を瞬時再起動する (enable 中のみ)。
  void set_duty(float duty_pct);

  // DMA 起動・PWM 有効化。
  void enable();

  // DMA 停止・PWM 無効化。
  void disable();

  bool is_enabled() const { return enabled_; }

private:
  // duty [%] から TABLE_SIZE を決定 (V/f ステップ)
  static int duty_to_n(float duty_pct);

  // table_uv_ / table_w_ を現在の amplitude_ と n_ で再計算
  void rebuild_tables();

  // DMA を abort → 設定 → 起動 (n_ が変わった時のみ呼ぶ)
  void restart_dma();

  uint     slice_s1_ = 0;
  uint     slice_s2_ = 0;
  uint32_t wrap_     = 0;
  bool     enabled_  = false;
  float    amplitude_ = 0.0f;   // 0.0 〜 1.0
  int      n_         = MAX_TABLE_SIZE;  // 現在のテーブルサイズ

  int ch_uv_ = -1;
  int ch_w_  = -1;

  // 最大サイズで確保。DMA は [0..n_-1] だけ使う。
  // ring mode: n_ * 4 バイト境界アライン (最大 512 B)
  alignas(1 << MAX_RING_BITS) uint32_t table_uv_[MAX_TABLE_SIZE];
  alignas(1 << MAX_RING_BITS) uint32_t table_w_[MAX_TABLE_SIZE];
};
