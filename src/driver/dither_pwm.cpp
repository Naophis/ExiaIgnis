// DitherPwm 実装。設計・仕様確認・検証手順は docs/dither_pwm.md 参照。
#include "driver/dither_pwm.hpp"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/structs/dma_debug.h"
#include "hardware/sync.h"
#include "pico/time.h"

namespace dpwm {

// リアルタイム経路(制御 IRQ から呼ばれる)は SRAM に置く(プロジェクト規約)。
#define DPWM_RT __attribute__((noinline, section(".time_critical.dither_pwm")))

// ---------------------------------------------------------------------------
// リング実体。DMA が読む SRAM。
//   * DMA RING_SIZE(2^10 byte) は「アドレスの下位 10 bit だけが変化する」機能なので、
//     リング先頭は 1024 byte 境界に整列していなければならない(datasheet 12.6.2.3)。
//     行サイズ == リングサイズなので各行が自動的に 1024 byte 整列になる。
//   * PSRAM(0x11xxxxxx)/flash に置くと DMA 読み出しが XIP 経由になり遅延・停止の
//     原因になる。init() で SRAM 範囲を検査する。
// ---------------------------------------------------------------------------
alignas(1u << DitherPwm::kRingSizeBits) static
    uint32_t g_ring[DitherPwm::kMaxSlices][DitherPwm::kRingSamples];

static inline uint32_t pack_cc(uint32_t a, uint32_t b) {
  return (a & 0xFFFFu) | ((b & 0xFFFFu) << 16);
}

#ifndef DPWM_HOST_TEST  // ホストシミュレーション時(SDK スタブ)はアドレス検査を外す
static inline bool in_sram(const void* p) {
  const uintptr_t x = (uintptr_t)p;
  return x >= 0x20000000u && x < 0x20082000u;  // RAM 512k + SCRATCH
}
#else
static inline bool in_sram(const void*) { return true; }
#endif

// ---------------------------------------------------------------------------
// 設定(非 RT)
// ---------------------------------------------------------------------------
bool DitherPwm::init(const Config& cfg, const uint* slices, uint n_slices) {
  if (n_slices == 0 || n_slices > kMaxSlices) return false;
  cfg_ = cfg;
  n_   = n_slices;
  if (cfg_.guard_samples == 0) cfg_.guard_samples = cfg_.samples_per_tick;
  if (cfg_.lead_samples == 0 || cfg_.samples_per_tick == 0) return false;

  max_level_ = cfg_.max_level ? cfg_.max_level : (uint32_t)cfg_.top + 1u;
  if (max_level_ > (uint32_t)cfg_.top + 1u) max_level_ = (uint32_t)cfg_.top + 1u;

  // 1 tick で書く範囲 [read+lead, read+lead+M+guard) がリングを一周して
  // DMA の背後(まだ読まれていない領域)に回り込まないこと。
  const uint32_t span = (uint32_t)cfg_.lead_samples + cfg_.samples_per_tick + cfg_.guard_samples;
  if (span >= kRingSamples) return false;

  for (uint i = 0; i < n_; ++i) {
    slice_[i] = slices[i];
    ring_[i]  = g_ring[i];
    if (!in_sram(ring_[i])) return false;
    if (((uintptr_t)ring_[i] & ((1u << kRingSizeBits) - 1u)) != 0) return false;
  }

  // ---- PWM slice: 停止状態で設定 ---------------------------------------
  // free-running(非 phase-correct)。pwm_init() は CSR/DIV/TOP を書き、
  // CTR=0, CC=0 にする(start=false なので EN は立てない)。
  for (uint i = 0; i < n_; ++i) {
    pwm_config pc = pwm_get_default_config();
    pwm_config_set_clkdiv_int_frac4(&pc, cfg_.clkdiv_int, cfg_.clkdiv_frac4);
    pwm_config_set_wrap(&pc, cfg_.top);
    pwm_config_set_phase_correct(&pc, false);
    pwm_init(slice_[i], &pc, false);
  }

  // ---- DMA: slice ごとに 1 チャネル ------------------------------------
  //   read : リング(増分 + RING_SIZE ラップ)
  //   write: slice.cc(固定アドレス、32 bit = A|B<<16 を 1 転送で同時更新)
  //   pace : その slice の wrap DREQ(1 周期 1 転送)
  //   count: ENDLESS(無限。完了も IRQ も無い → CPU が再起動する経路が存在しない)
  for (uint i = 0; i < n_; ++i) {
    ch_[i] = (uint)dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ch_[i]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, kRingSizeBits);     // read アドレスを 1024 B でラップ
    channel_config_set_dreq(&c, pwm_get_dreq(slice_[i]));  // DREQ_PWM_WRAPn
    channel_config_set_high_priority(&c, cfg_.high_priority);
    channel_config_set_irq_quiet(&c, true);
    channel_config_set_chain_to(&c, ch_[i]);               // 自分自身 = chain 無し
    dma_channel_configure(ch_[i], &c,
                          &pwm_hw->slice[slice_[i]].cc,     // write
                          ring_[i],                          // read
                          dma_encode_endless_transfer_count(),
                          false);
    fill_static_(i, 0, 0);
    resync_[i] = true;
  }

  read_count_ = 0;
  reset_stats();
  inited_  = true;
  running_ = false;
  return true;
}

void DitherPwm::fill_static_(uint idx, uint16_t a, uint16_t b) {
  if (a > max_level_) a = (uint16_t)max_level_;
  if (b > max_level_) b = (uint16_t)max_level_;
  const uint32_t w = pack_cc(a, b);
  for (uint32_t k = 0; k < kRingSamples; ++k) ring_[idx][k] = w;
  cmd_[idx].a = (uint32_t)a << 16;
  cmd_[idx].b = (uint32_t)b << 16;
  guard_cmd_[idx] = cmd_[idx];
  snap_a_[idx].reset();
  snap_b_[idx].reset();
}

// ---------------------------------------------------------------------------
// 開始 / 停止(非 RT)
// ---------------------------------------------------------------------------
void DitherPwm::start() {
  if (!inited_ || running_) return;

  uint32_t dma_mask = 0, pwm_mask = 0;
  for (uint i = 0; i < n_; ++i) {
    // stop() の後も Core1 の update() がリングに書き続けている可能性がある
    // (running_=false で早期 return するが stop() と同時刻の tick は書き切る)。
    // 起動時のリングは必ず静的 0 にし、直前に set_levels_q16() された指令は
    // 保持して最初の update() で反映させる。
    {
      const Cmd keep = cmd_[i];
      fill_static_(i, 0, 0);
      cmd_[i] = keep;
    }
    // 全 slice を位相 0 から。停止中の CC 書き込みは即時 latch されるので、
    // 最初の周期はリング先頭の値で出る(DMA はその後 wrap ごとに続きを流す)。
    pwm_set_counter(slice_[i], 0);
    pwm_hw->slice[slice_[i]].cc = ring_[i][0];
    // stop()→start() の再開時、DMA の read_addr は abort 時点の位置のまま残るので
    // リング先頭へ戻す(帳尻: last_read_idx_=0 と一致させる)。ENDLESS モードは
    // trigger でアドレスを再ロードしない(datasheet 12.6.2.2)。
    dma_channel_set_read_addr(ch_[i], ring_[i], false);
    // 溜まっていた DREQ クレジットをクリアし、ハンドシェイクを初期化する
    // (datasheet: DBG_CTDREQ に任意値を書くとカウンタクリア + 再初期化)。
    dma_debug_hw->ch[ch_[i]].dbg_ctdreq = 0;
    last_read_idx_[i] = 0;
    resync_[i]        = true;
    dma_mask |= 1u << ch_[i];
    pwm_mask |= 1u << slice_[i];
  }
  read_count_ = 0;
  reset_stats();

  // DMA を先に走らせる(DREQ 待ちで止まっている)。その後 EN レジスタの
  // 該当ビットだけを同一書き込みで立て、全 slice を同一サイクルで開始する。
  // pwm_set_mask_enabled() は EN 全体を上書きして他 slice(ブザー/ESC 等)を
  // 止めてしまうため使わない。
  dma_start_channel_mask(dma_mask);
  __dmb();
  hw_set_bits(&pwm_hw->en, pwm_mask);
  running_ = true;
}

void DitherPwm::stop(bool disable_slices) {
  if (!inited_) return;
  for (uint i = 0; i < n_; ++i) dma_channel_abort(ch_[i]);
  running_ = false;
  // DMA が止まったので CC は CPU の書き込みが残る。0 は次の wrap で latch され出力 LOW。
  for (uint i = 0; i < n_; ++i) pwm_hw->slice[slice_[i]].cc = 0;
  for (uint i = 0; i < n_; ++i) fill_static_(i, 0, 0);
  if (disable_slices) {
    // CC=0 が確実に latch されて出力が LOW に落ちるまで 2 周期待ってから止める
    // (途中で止めると位相によっては HIGH で固定される)。
    busy_wait_us(2u * period_us() + 2u);
    for (uint i = 0; i < n_; ++i) pwm_set_enabled(slice_[i], false);
  }
}

uint32_t DitherPwm::period_us() const {
  const uint32_t sys = clock_get_hz(clk_sys);
  const uint32_t div16 = (uint32_t)cfg_.clkdiv_int * 16u + cfg_.clkdiv_frac4;  // /16
  const uint64_t ticks = (uint64_t)(cfg_.top + 1u) * div16;                     // sysclk*16
  uint32_t us = (uint32_t)((ticks * 1000000ull) / ((uint64_t)sys * 16ull));
  return us ? us : 1u;
}

void DitherPwm::reset_stats() {
  for (uint i = 0; i < kMaxSlices; ++i) st_[i] = DitherStats{};
}

// ---------------------------------------------------------------------------
// 指令(RT 安全)
// ---------------------------------------------------------------------------
DPWM_RT void DitherPwm::set_levels_q16(uint idx, uint32_t a_q16, uint32_t b_q16) {
  const uint32_t lim = max_level_ << 16;  // max_level ちょうど(frac=0)までは許可
  if (a_q16 > lim) a_q16 = lim;
  if (b_q16 > lim) b_q16 = lim;
  cmd_[idx].a = a_q16;
  cmd_[idx].b = b_q16;
}

void DitherPwm::force_static(uint idx, uint16_t a, uint16_t b) {
  fill_static_(idx, a, b);
  resync_[idx] = true;
  if (running_) {
    // DMA がリングの新しい値に到達する前に、直接 CC にも書いて次の wrap から効かせる。
    // 以後 DMA は同じ値を書き続けるので矛盾しない。
    pwm_hw->slice[slice_[idx]].cc = ring_[idx][0];
  }
}

// ---------------------------------------------------------------------------
// 制御周期ごとの更新(RT)。ここが CPU の担当範囲のすべて。
//
//   サンプル番号(単調増加)の並び:
//     ... | 消費済み | read | lead | commit(前 tick) | guard(前 tick) | 1周前の古いデータ ...
//   この tick では
//     start = max(commit_end(前 tick の commit 末尾), read + lead)
//     end   = read + lead + samples_per_tick
//     [start, end) を新しい指令で埋め、[end, end+guard) にも同じ指令で書き足す。
//   変調器の状態は commit_end 時点の snapshot から再開するので、guard を上書きしても
//   誤差拡散の連続性は保たれる(DMA が guard を食った分は fast-forward する)。
// ---------------------------------------------------------------------------
DPWM_RT void DitherPwm::update() {
  if (!running_) return;

  // 1) 全 slice の DMA read index を同一時点で読む。
  //    READ_ADDR は「次に発行する転送」のアドレス(発行時に増える)。
  uint32_t ridx[kMaxSlices];
  uint32_t consumed[kMaxSlices];
  uint32_t consumed_max = 0, rmin = 0xFFFFFFFFu, rmax = 0;
  for (uint i = 0; i < n_; ++i) {
    const uint32_t ra = dma_hw->ch[ch_[i]].read_addr;
    ridx[i] = ((ra - (uint32_t)(uintptr_t)ring_[i]) >> 2) & kRingMask;
    consumed[i] = (ridx[i] - last_read_idx_[i]) & kRingMask;  // < kRingSamples 前提
    last_read_idx_[i] = ridx[i];
    if (consumed[i] > consumed_max) consumed_max = consumed[i];
    if (ridx[i] < rmin) rmin = ridx[i];
    if (ridx[i] > rmax) rmax = ridx[i];
  }
  // slice 間ずれ(lockstep なら常に 0。DMA が片方だけ遅れると 1 になる)
  {
    uint32_t skew = rmax - rmin;
    if (skew > kRingSamples / 2) skew = kRingSamples - skew;  // ラップ跨ぎ
    for (uint i = 0; i < n_; ++i)
      if (skew > st_[i].skew_max) st_[i].skew_max = skew;
  }
  // 共通の read カウンタは「最も進んでいる slice」に合わせる。
  // 遅れている slice に対しても start >= その read + lead が保たれる(安全側)。
  read_count_ += consumed_max;

  for (uint i = 0; i < n_; ++i) update_slice_(i, consumed[i]);
}

DPWM_RT void DitherPwm::update_slice_(uint idx, uint32_t consumed) {
  DitherStats& st = st_[idx];
  st.ticks++;
  st.consumed_total += consumed;
  st.last_cc = pwm_hw->slice[slice_[idx]].cc;

  // DMA の DREQ 残クレジット(0 が正常。>=1 は「wrap が来たのにまだ転送できていない」)
  {
    const uint32_t bl = dma_debug_hw->ch[ch_[idx]].dbg_ctdreq & 0x3Fu;
    if (bl > st.dreq_backlog_max) st.dreq_backlog_max = bl;
    st.last_backlog = bl;
  }
  st.last_consumed = consumed;

  const uint32_t lead      = cfg_.lead_samples;
  const uint32_t M         = cfg_.samples_per_tick;
  const uint32_t guard     = cfg_.guard_samples;
  const uint32_t start_min = read_count_ + lead;

  if (resync_[idx]) {
    // 起動直後 / force_static 直後: リング内容は固定値(frac=0)なので変調器状態は
    // そのまま有効。統計を汚さずに commit 境界だけ合わせる。
    // (start() 直後の最初の tick は 1 周期も経っていないことがあるので stall 判定もしない)
    commit_end_[idx] = start_min;
    guard_cmd_[idx]  = cmd_[idx];
    resync_[idx]     = false;
  } else if (consumed == 0) {
    st.stall_ticks++;  // 前回 tick から DMA read index が動いていない = PWM か DMA が止まっている
  }

  Modulator ma = snap_a_[idx];
  Modulator mb = snap_b_[idx];
  uint32_t  start = commit_end_[idx];

  if (start_min > start) {
    // CPU が遅れた: DMA は前 tick の guard 領域(同じ指令)を delta サンプル食っている。
    const uint32_t delta = start_min - start;
    st.late_samples += delta;
    if (delta > st.late_max) st.late_max = delta;
    uint32_t ff = delta;
    if (delta > guard) {
      // guard も使い切り、1 周前の古いデータが再生された(fail-static の限界)。
      st.underrun_events++;
      st.underrun_samples += delta - guard;
      ff = guard;  // 連続性はここで失われる(誤差は 1 count 未満)
    }
    // guard で出した分だけ変調器を進めて連続性を保つ
    const uint32_t ga = guard_cmd_[idx].a, gb = guard_cmd_[idx].b;
    for (uint32_t k = 0; k < ff; ++k) { ma.next(ga); mb.next(gb); }
    start = start_min;
  } else if (start > start_min) {
    st.early_samples += start - start_min;
  }

  uint32_t end = read_count_ + lead + M;
  if (end < start) end = start;
  st.last_lead = start - read_count_;

  uint32_t* ring = ring_[idx];
  const uint32_t ca = cmd_[idx].a, cb = cmd_[idx].b;

  // commit 領域: この tick の指令。DMA はここを次の M 周期で順に latch していく。
  for (uint32_t s = start; s != end; ++s)
    ring[s & kRingMask] = pack_cc(ma.next(ca), mb.next(cb));

  commit_end_[idx] = end;
  snap_a_[idx]     = ma;   // 次 tick はここから再開する
  snap_b_[idx]     = mb;
  guard_cmd_[idx]  = cmd_[idx];

  // guard 領域: 同じ指令の続き。次 tick が間に合えば上書きされ、遅れればそのまま出る。
  for (uint32_t s = end; s != end + guard; ++s)
    ring[s & kRingMask] = pack_cc(ma.next(ca), mb.next(cb));
}

// ---------------------------------------------------------------------------
// デバッグ / 変換
// ---------------------------------------------------------------------------
uint32_t DitherPwm::dma_read_index(uint idx) const {
  const uint32_t ra = dma_hw->ch[ch_[idx]].read_addr;
  return ((ra - (uint32_t)(uintptr_t)ring_[idx]) >> 2) & kRingMask;
}

uint32_t DitherPwm::probe_write(uint idx, uint32_t offset, uint32_t word) {
  const uint32_t r = dma_read_index(idx);
  ring_[idx][(r + offset) & kRingMask] = word;
  return r + offset;
}
uint32_t DitherPwm::cc_reg(uint idx) const { return pwm_hw->slice[slice_[idx]].cc; }
uint32_t DitherPwm::ctr_reg(uint idx) const { return pwm_hw->slice[slice_[idx]].ctr; }

uint32_t DitherPwm::ring_word(uint idx, uint32_t sample_no) const {
  return ring_[idx][sample_no & kRingMask];
}

uint32_t DitherPwm::q16_from_counts(float counts) {
  if (counts < 0.0f) counts = 0.0f;
  float q = counts * 65536.0f + 0.5f;
  if (q > 4294967040.0f) q = 4294967040.0f;
  return (uint32_t)q;
}

uint32_t DitherPwm::q16_from_percent(float duty_percent, uint16_t top) {
  return q16_from_counts(duty_percent * 0.01f * (float)((uint32_t)top + 1u));
}

}  // namespace dpwm
