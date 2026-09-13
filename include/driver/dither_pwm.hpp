#pragma once
// ============================================================================
// DitherPwm — RP2350 Hardware PWM + DMA による時間方向ディザ(1次誤差拡散)
//
// 目的: PWM カウンタの 1 count 未満の duty 指令を、複数 PWM 周期の平均として
//       表現する(STM32 HRTIM のようなエッジ分解能の向上ではない)。
//       例: clk_sys 150 MHz / PWM 100 kHz → 1500 count(約 10.5 bit)。
//           指令 500.36 count → 「500 を 16 回, 501 を 9 回」を均等に混ぜて
//           25 周期平均で 500.36 を出す。モータの L/R による LPF で平均化される。
//
// 役割分担 ------------------------------------------------------------------
//   [PWM hardware]  キャリア生成。TOP/CC は double-buffer で wrap 時に latch
//                   (RP2350 datasheet 12.5.2.3)。wrap ごとに 1 サイクルの DREQ を
//                   DMA に出す (12.5.2.7)。EN レジスタで複数 slice を同一サイクルで
//                   開始すると lockstep で回る (12.5.2.8)。
//   [DMA]           wrap DREQ で 1 周期 1 転送。SRAM リング(1024 B 整列) → slice.cc
//                   (32 bit: A | B<<16、A/B は同一転送で同時更新)。
//                   RING_SIZE で read アドレスが自動ラップし、TRANS_COUNT=ENDLESS
//                   (12.6.2.2) で無限に走る。起動後、CPU は DMA に一切触らない。
//                   ★ここがリアルタイムクリティカル。CPU は関与しない。
//   [CPU / 制御周期] update() を制御周期ごとに 1 回呼ぶだけ。DMA の read 位置の
//                   lead_samples 先から、次の制御周期ぶん(samples_per_tick)の
//                   サンプル列を誤差拡散で生成してリングに書く。IRQ なし、
//                   DMA 再起動なし。遅れても DMA は止まらない。
//
// fail-static ----------------------------------------------------------------
//   * CPU が遅れる/止まる → DMA はリングを回し続ける。commit 領域の後ろに同じ
//     指令で guard_samples ぶん書き足してあるので、その間は「最後の duty」を保持。
//     さらに遅れると 1 周前(kRingSamples 周期前)のデータが再生される(=少し古い
//     duty。PWM は途切れない)。
//   * DMA が遅れる(バス競合) → DREQ クレジットが溜まり後でまとめて転送される。
//     PWM は前回 CC のまま継続。同一周期内の複数書き込みは最後の 1 つだけ latch
//     されるので、数サンプルが飛ぶ(平均に 1 count 未満の誤差)。
//   * GPIO が HIGH に張り付くのは CC >= TOP+1 の時だけ。本ドライバは max_level
//     (既定 TOP+1 = 100%)でクランプした値しかリングに書かない。
//
// 制約 -----------------------------------------------------------------------
//   * RP2350 専用(TRANS_COUNT MODE=ENDLESS は RP2040 に無い)。RP2040 では
//     bldc_actuator.cpp の chained control block 方式で代替する。
//   * 位相補正モード(phase-correct)非対応(DREQ/latch タイミングが異なる)。
//   * リングは SRAM に置くこと(PSRAM/flash 不可)。本実装は .cpp 内の static 配列。
//   * 1 インスタンスのみ(リング実体が static)。
//   * DMA が CC を毎周期上書きするので、動作中に pwm_set_chan_level() 等で
//     CC を直接書いても次の wrap で消される。duty 指令は必ず本クラス経由。
// ============================================================================

#include <stdint.h>
#include <stddef.h>
#include "pico/types.h"

namespace dpwm {

constexpr uint32_t kQ16One = 1u << 16;   // Q16.16 の 1.0

// ---- 変調器(差し替え可能) -----------------------------------------------
// 要件: uint16_t next(uint32_t cmd_q16) が「整数部」または「整数部+1」を返し、
//       長期平均が cmd_q16 に一致すること。状態はコピー可能であること
//       (update() が commit 境界で状態を snapshot/restore するため)。
//
// ErrorDiffusion1: 1次誤差拡散(= Bresenham = 1次ΣΔ)。
//   acc += frac; if (acc >= 1) { out = int+1; acc -= 1; } else out = int;
//   を Q0.16 の整数演算で行う。浮動小数点は使わない。
//
// 2次ΣΔ に拡張する場合は同じインターフェースの struct を作り、
// DitherPwm::Modulator の using を差し替える(出力が int-1..int+2 に広がるので
// クランプ幅に注意)。
struct ErrorDiffusion1 {
  uint32_t acc = 0;  // Q0.16 残差 (0..0xFFFF)

  inline uint16_t next(uint32_t cmd_q16) {
    acc += cmd_q16 & 0xFFFFu;
    const uint32_t carry = acc >> 16;  // 0 or 1
    acc &= 0xFFFFu;
    return (uint16_t)((cmd_q16 >> 16) + carry);
  }
  inline void reset() { acc = 0; }
};

// ---- 診断カウンタ(slice ごと) --------------------------------------------
struct DitherStats {
  uint32_t ticks            = 0;  // update() 回数
  uint32_t consumed_total   = 0;  // DMA が消費したサンプル総数
  uint32_t late_samples     = 0;  // commit 開始前に DMA が guard 領域まで進んでいたサンプル数(CPU 遅延)
  uint32_t late_max         = 0;  // その 1 tick あたり最大値
  uint32_t early_samples    = 0;  // 前回 commit 末尾が read+lead より先だった量(tick が早い/DMA が遅い)
  uint32_t underrun_events  = 0;  // guard を超えて 1 周前の古いデータが再生された回数
  uint32_t underrun_samples = 0;  // その総サンプル数
  uint32_t stall_ticks      = 0;  // DMA read index が動いていなかった tick(PWM 停止 / DMA 停止)
  uint32_t dreq_backlog_max = 0;  // DMA の DREQ クレジット残(DBG_CTDREQ)最大値。>1 なら DMA が周期内に間に合っていない
  uint32_t skew_max         = 0;  // slice 間の DMA read index ずれ最大値(0 なら全 slice 同一 wrap で反映)
  // 直近 update() の瞬時値(ログ用)
  uint32_t last_consumed    = 0;  // この tick で DMA が消費したサンプル数(期待: samples_per_tick)
  uint32_t last_lead        = 0;  // commit 開始 − DMA read 位置 [サンプル](期待: lead_samples 〜 +1)
  uint32_t last_backlog     = 0;  // DBG_CTDREQ の現在値(期待: 0)
  uint32_t last_cc          = 0;  // update() 開始時点の CC レジスタ生値(DMA が最後に書いた = いま出力中の値)
};

class DitherPwm {
public:
  static constexpr uint kMaxSlices    = 4;
  static constexpr uint kRingSizeBits = 10;                          // 1024 byte(DMA RING_SIZE)
  static constexpr uint kRingSamples  = (1u << kRingSizeBits) / 4u;  // 256 サンプル(uint32)
  static constexpr uint kRingMask     = kRingSamples - 1u;

  struct Config {
    uint16_t top              = 1499;  // wrap 値。PWM 周波数 = clk_sys / ((top+1) * clkdiv)
    uint8_t  clkdiv_int       = 1;
    uint8_t  clkdiv_frac4     = 0;
    uint16_t samples_per_tick = 25;    // 制御周期あたりの PWM 周期数(切り上げ)。100kHz/4kHz=25、100kHz/1kHz=100
    uint16_t lead_samples     = 4;     // DMA read 位置から commit 開始までの余裕。= 指令レイテンシ(周期数)
    uint16_t guard_samples    = 0;     // commit の後ろに同じ指令で書き足す長さ(fail-static 保持長)。0 なら samples_per_tick
    uint16_t max_level        = 0;     // CC 上限(クランプ)。0 なら top+1 (=100% duty)
    bool     high_priority    = true;  // DMA チャネルを高優先度にする
  };

  // PWM slice(停止状態で設定)と DMA チャネルを準備する。まだ出力しない。
  // slices: 使う PWM slice 番号の配列(最大 kMaxSlices)。全 slice 同一周波数。
  bool init(const Config& cfg, const uint* slices, uint n_slices);

  // DMA を待機状態にしてから全 slice を同一サイクルで開始する(lockstep)。
  void start();

  // DMA を abort し CC=0 を直接書く(次の wrap で出力 LOW)。
  // disable_slices=true なら 2 周期待ってから slice を停止する。
  void stop(bool disable_slices);

  // 次の update() で commit する指令(Q16.16 count)。A/B 各チャネル。
  // max_level を超える値はクランプされる。RT 安全(整数演算のみ)。
  void set_levels_q16(uint idx, uint32_t a_q16, uint32_t b_q16);

  // 制御周期ごとに 1 回呼ぶ(Core1 の制御 IRQ 内を想定)。
  // 全 slice の DMA 位置を同一時点で読み、各 slice のリングに
  // [read+lead, read+lead+samples_per_tick) を指令で埋め、その後ろに guard を書き足す。
  void update();

  // ディザ無しの固定値をリング全体と CC に即時書く(安全側への移行/停止用)。
  void force_static(uint idx, uint16_t a, uint16_t b);

  const DitherStats& stats(uint idx) const { return st_[idx]; }
  void reset_stats();

  // デバッグ/検証用
  uint32_t dma_read_index(uint idx) const;                  // DMA が次に読むリング index
  uint32_t commit_end(uint idx) const { return commit_end_[idx]; }  // 単調増加のサンプル番号
  uint32_t ring_word(uint idx, uint32_t sample_no) const;   // sample_no(単調番号)に対応するリング語
  uint     dma_channel(uint idx) const { return ch_[idx]; }
  // 診断プローブ: DMA read 位置 + offset のリング語を直接書き、書いた単調番号を返す。
  // (次の update() で上書きされ得る。レイテンシ計測用)
  uint32_t probe_write(uint idx, uint32_t offset, uint32_t word);
  uint32_t cc_reg(uint idx) const;     // slice の CC レジスタ生値(DMA が書いた最新値)
  uint32_t ctr_reg(uint idx) const;    // slice のカウンタ現在値
  uint     slice(uint idx) const { return slice_[idx]; }
  bool     running() const { return running_; }
  uint32_t max_level() const { return max_level_; }
  uint32_t period_us() const;

  // 変換ヘルパ(非 RT。float 使用)
  static uint32_t q16_from_counts(float counts);
  static uint32_t q16_from_percent(float duty_percent, uint16_t top);

private:
  using Modulator = ErrorDiffusion1;
  struct Cmd { uint32_t a = 0, b = 0; };

  Config    cfg_{};
  uint      n_ = 0;
  uint      slice_[kMaxSlices]{};
  uint      ch_[kMaxSlices]{};
  uint32_t* ring_[kMaxSlices]{};
  Cmd       cmd_[kMaxSlices]{};        // 次の commit 用指令
  Cmd       guard_cmd_[kMaxSlices]{};  // guard 領域に書いた指令(fast-forward 用)
  Modulator snap_a_[kMaxSlices]{};     // commit_end_ 時点の変調器状態
  Modulator snap_b_[kMaxSlices]{};
  uint32_t  commit_end_[kMaxSlices]{}; // commit 済み末尾(単調サンプル番号)= guard 先頭
  uint32_t  last_read_idx_[kMaxSlices]{};
  bool      resync_[kMaxSlices]{};     // 次の update() で commit_end_ を read+lead に合わせ直す
  DitherStats st_[kMaxSlices]{};
  uint32_t  read_count_ = 0;           // 全 slice 共通の単調 read サンプル番号
  uint32_t  max_level_  = 0;
  bool      running_    = false;
  bool      inited_     = false;

  void fill_static_(uint idx, uint16_t a, uint16_t b);
  void update_slice_(uint idx, uint32_t consumed);
};

}  // namespace dpwm
