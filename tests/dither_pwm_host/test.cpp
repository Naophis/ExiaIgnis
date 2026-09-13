// DitherPwm のリング更新ロジックをホスト上でシミュレーションする。
// 「DMA」= 1 PWM 周期ごとに read_addr のリング語を CC へ転送して read_addr++(1024B ラップ)。
// 「PWM」= 周期 k に CC へ書かれた値は周期 k+1 の出力(double-buffer)。
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cmath>
#include "driver/dither_pwm.hpp"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/structs/dma_debug.h"

static pwm_hw_t g_pwm_regs; pwm_hw_t* pwm_hw = &g_pwm_regs;
static dma_hw_t g_dma_regs; dma_hw_t* dma_hw = &g_dma_regs;
static dma_debug_hw_t g_dbg; dma_debug_hw_t* dma_debug_hw = &g_dbg;

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

struct Sim {
  dpwm::DitherPwm pwm;
  uint n = 1;
  uint slices[2] = {2, 3};
  std::vector<uint16_t> out[2];   // 周期ごとの出力(A レベル)
  uint32_t pending_cc[2] = {0, 0};
  uintptr_t ring_base[2];
  uint32_t period = 0;
  Sim(uint nslices, dpwm::DitherPwm::Config cfg) : n(nslices) {
    g_next_ch = 0;
    bool ok = pwm.init(cfg, slices, n);
    if (!ok) { printf("init failed\n"); exit(1); }
    for (uint i = 0; i < n; ++i) ring_base[i] = dma_hw->ch[pwm.dma_channel(i)].read_addr;
  }
  // 1 PWM 周期進める: 出力 = 前周期に CC に書かれた値、その後 DMA が 1 転送
  void step_period() {
    for (uint i = 0; i < n; ++i) {
      out[i].push_back((uint16_t)(pwm_hw->slice[slices[i]].cc & 0xFFFF));  // latch at wrap
      dma_channel_hw_t& ch = dma_hw->ch[pwm.dma_channel(i)];
      if (ch.started) {
        uint32_t off = (ch.read_addr - ring_base[i]) & 1023u;
        uint32_t word = *(uint32_t*)(uintptr_t)(ring_base[i] + off);
        pwm_hw->slice[slices[i]].cc = word;                                  // DMA write (double-buffered)
        ch.read_addr = ring_base[i] + ((off + 4) & 1023u);                   // RING wrap
      }
    }
    period++;
  }
};

static double mean_of(const std::vector<uint16_t>& v, size_t a, size_t b) {
  double s = 0; for (size_t i = a; i < b; ++i) s += v[i]; return s / (double)(b - a);
}

int main() {
  const uint M = 25, LEAD = 4;
  dpwm::DitherPwm::Config cfg; cfg.top = 1499; cfg.samples_per_tick = M; cfg.lead_samples = LEAD; cfg.guard_samples = 0;

  // ---------------- Test 1: 定常指令の厳密平均 + 値域 + 遅延 ----------------
  printf("[T1] steady-state mean / value set / latency\n");
  {
    Sim s(1, cfg);
    s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(60.0f), 0);
    s.pwm.start();
    const float cmds[] = {60.0f, 60.25f, 60.36f, 500.36f, 1499.5f, 0.02f};
    size_t seg_start[6]; int k = 0;
    for (float c : cmds) {
      seg_start[k++] = s.out[0].size();
      for (int t = 0; t < 40; ++t) {                  // 40 tick = 1000 周期
        s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(c), 0);
        s.pwm.update();
        for (uint p = 0; p < M; ++p) s.step_period();
      }
    }
    for (int i = 0; i < 6; ++i) {
      size_t a = seg_start[i] + LEAD + 1 + M + 2;    // 切替後の過渡を除く
      size_t b = (i < 5 ? seg_start[i + 1] : s.out[0].size()) + LEAD;  // 次の切替が効く直前まで
      if (b > s.out[0].size()) b = s.out[0].size();
      double m = mean_of(s.out[0], a, b);
      uint32_t q = dpwm::DitherPwm::q16_from_counts(cmds[i]);
      double want = q / 65536.0;
      uint16_t lo = (uint16_t)(q >> 16), hi = (uint16_t)((q >> 16) + ((q & 0xffff) ? 1 : 0));
      bool set_ok = true; for (size_t j = a; j < b; ++j) if (s.out[0][j] != lo && s.out[0][j] != hi) set_ok = false;
      printf("  cmd=%9.4f mean=%10.5f err=%+.2e  values in {%u,%u}: %s\n", cmds[i], m, m - want, lo, hi, set_ok ? "yes" : "NO");
      CHECK(fabs(m - want) < 2.0 / (double)(b - a), "mean error too large for cmd %f", cmds[i]);
      CHECK(set_ok, "value outside {N,N+1} for cmd %f", cmds[i]);
    }
    // 遅延: 60.0 → 60.25 の切替は seg_start[1] の tick で指令。新しい値(61)が最初に出る周期
    size_t first61 = 0; for (size_t j = seg_start[1]; j < seg_start[2]; ++j) if (s.out[0][j] == 61) { first61 = j; break; }
    printf("  latency: cmd tick at period %zu, first '61' at period %zu (= lead %u + latch 1 + diffusion)\n", seg_start[1], first61, LEAD);
    CHECK(first61 >= seg_start[1] + LEAD + 1 && first61 <= seg_start[1] + LEAD + 1 + 4, "latency out of range");
    const dpwm::DitherStats& st = s.pwm.stats(0);
    CHECK(st.late_samples == 0 && st.underrun_events == 0 && st.stall_ticks == 0, "unexpected stats late=%u underrun=%u stall=%u", st.late_samples, st.underrun_events, st.stall_ticks);
    printf("  stats: ticks=%u consumed=%u late=%u early=%u underrun=%u stall=%u\n", st.ticks, st.consumed_total, st.late_samples, st.early_samples, st.underrun_events, st.stall_ticks);
  }

  // ---------------- Test 2: fail-static (CPU 遅延) ----------------
  printf("[T2] fail-static: skip 1 tick (guard covers), skip 2 ticks (underrun)\n");
  {
    Sim s(1, cfg);
    const uint32_t q = dpwm::DitherPwm::q16_from_counts(60.36f);
    s.pwm.set_levels_q16(0, q, 0); s.pwm.start();
    auto run_ticks = [&](int n, bool call_update) { for (int t = 0; t < n; ++t) { if (call_update) { s.pwm.set_levels_q16(0, q, 0); s.pwm.update(); } for (uint p = 0; p < M; ++p) s.step_period(); } };
    run_ticks(20, true);
    size_t a = s.out[0].size();
    run_ticks(1, false);            // 1 tick スキップ: guard(=M) が出るはず
    run_ticks(20, true);
    const dpwm::DitherStats& st = s.pwm.stats(0);
    printf("  after skip1: late=%u late_max=%u underrun=%u\n", st.late_samples, st.late_max, st.underrun_events);
    CHECK(st.late_samples == M && st.late_max == M && st.underrun_events == 0, "skip1 stats wrong");
    // 出力は途切れず {60,61} のまま、平均も維持
    bool ok = true; for (size_t j = a; j < s.out[0].size(); ++j) if (s.out[0][j] != 60 && s.out[0][j] != 61) ok = false;
    double m = mean_of(s.out[0], a, s.out[0].size());
    printf("  output during/after skip stays in {60,61}: %s, mean=%.4f (cmd 60.36)\n", ok ? "yes" : "NO", m);
    CHECK(ok, "output left {N,N+1} during skip"); CHECK(fabs(m - 60.36) < 0.01, "mean drift after skip");
    // 連続性: 全区間の累積誤差が 1 count 未満(誤差拡散が途切れていない)
    double cum = 0, worst = 0; for (size_t j = a; j < s.out[0].size(); ++j) { cum += (double)s.out[0][j] - q / 65536.0; if (fabs(cum) > worst) worst = fabs(cum); }
    printf("  max |cumulative error| = %.3f count (must be < 1)\n", worst); CHECK(worst < 1.0, "continuity broken");
    run_ticks(2, false);            // 2 tick スキップ: guard を超えて 1 周前のデータ → underrun 1 回、M サンプル
    run_ticks(5, true);
    printf("  after skip2: underrun_events=%u underrun_samples=%u (expect 1 / %u)\n", st.underrun_events, st.underrun_samples, M);
    CHECK(st.underrun_events == 1 && st.underrun_samples == M, "skip2 underrun accounting wrong");
    ok = true; for (size_t j = a; j < s.out[0].size(); ++j) if (s.out[0][j] != 60 && s.out[0][j] != 61) ok = false;
    CHECK(ok, "output left {N,N+1} even on underrun (ring-old data should still be a recent duty)");
  }

  // ---------------- Test 3: 2 slice 同一 wrap 反映 ----------------
  printf("[T3] two slices: identical commit boundaries and skew\n");
  {
    Sim s(2, cfg);
    s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(60.0f), 0);
    s.pwm.set_levels_q16(1, dpwm::DitherPwm::q16_from_counts(60.0f), 0);
    s.pwm.start();
    for (int t = 0; t < 30; ++t) {
      float c = (t < 10) ? 60.0f : (t < 20 ? 60.5f : 100.25f);
      s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(c), 0);
      s.pwm.set_levels_q16(1, dpwm::DitherPwm::q16_from_counts(c), 0);
      s.pwm.update();
      for (uint p = 0; p < M; ++p) s.step_period();
    }
    bool same = s.out[0] == s.out[1];
    printf("  slice outputs identical: %s, skew_max=%u\n", same ? "yes" : "NO", s.pwm.stats(0).skew_max);
    CHECK(same, "slice outputs differ"); CHECK(s.pwm.stats(0).skew_max == 0, "skew");
  }

  // ---------------- Test 4: 1 kHz 制御 (M=100) + force_static / stop ----------------
  printf("[T4] M=100 (1 kHz), force_static, stop\n");
  {
    dpwm::DitherPwm::Config c2 = cfg; c2.samples_per_tick = 100; c2.guard_samples = 0;
    Sim s(1, c2);
    s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(700.7f), 0); s.pwm.start();
    for (int t = 0; t < 30; ++t) { s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(700.7f), 0); s.pwm.update(); for (int p = 0; p < 100; ++p) s.step_period(); }
    double m = mean_of(s.out[0], 200, s.out[0].size()); printf("  mean=%.4f (cmd 700.7)\n", m); CHECK(fabs(m - 700.7) < 2e-3, "M=100 mean");
    s.pwm.force_static(0, 5, 0);
    for (int p = 0; p < 3; ++p) s.step_period();
    CHECK(s.out[0].back() == 5, "force_static not applied within 2 periods (got %u)", s.out[0].back());
    for (int t = 0; t < 3; ++t) { s.pwm.update(); for (int p = 0; p < 100; ++p) s.step_period(); }
    bool all5 = true; for (size_t j = s.out[0].size() - 250; j < s.out[0].size(); ++j) if (s.out[0][j] != 5) all5 = false;
    CHECK(all5, "force_static value not held");
    s.pwm.stop(false);
    for (int p = 0; p < 3; ++p) s.step_period();
    CHECK(s.out[0].back() == 0, "stop did not drive CC=0 (got %u)", s.out[0].back());
    // 再開
    s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(60.25f), 0); s.pwm.start();
    for (int t = 0; t < 10; ++t) { s.pwm.set_levels_q16(0, dpwm::DitherPwm::q16_from_counts(60.25f), 0); s.pwm.update(); for (int p = 0; p < 100; ++p) s.step_period(); }
    m = mean_of(s.out[0], s.out[0].size() - 500, s.out[0].size()); printf("  restart mean=%.4f (cmd 60.25)\n", m); CHECK(fabs(m - 60.25) < 5e-3, "restart mean");
    const dpwm::DitherStats& st = s.pwm.stats(0);
    CHECK(st.late_samples == 0 && st.underrun_events == 0, "restart stats late=%u underrun=%u", st.late_samples, st.underrun_events);
  }

  // ---------------- Test 5: 制御 tick のジッタ(±3 周期) ----------------
  printf("[T5] control tick jitter +-3 periods\n");
  {
    Sim s(1, cfg);
    const uint32_t q = dpwm::DitherPwm::q16_from_counts(333.333f);
    s.pwm.set_levels_q16(0, q, 0); s.pwm.start();
    srand(1);
    for (int t = 0; t < 400; ++t) {
      s.pwm.set_levels_q16(0, q, 0); s.pwm.update();
      int periods = (int)M + (rand() % 7) - 3;
      for (int p = 0; p < periods; ++p) s.step_period();
    }
    const dpwm::DitherStats& st = s.pwm.stats(0);
    double m = mean_of(s.out[0], 100, s.out[0].size());
    double cum = 0, worst = 0; for (size_t j = 100; j < s.out[0].size(); ++j) { cum += (double)s.out[0][j] - q / 65536.0; if (fabs(cum) > worst) worst = fabs(cum); }
    printf("  mean=%.5f (cmd %.5f) late=%u(max %u) early=%u underrun=%u max|cum err|=%.3f\n", m, q / 65536.0, st.late_samples, st.late_max, st.early_samples, st.underrun_events, worst);
    CHECK(st.underrun_events == 0, "jitter caused underrun"); CHECK(worst < 1.0, "jitter broke continuity");
  }

  printf("\n%s (%d failures)\n", fails ? "RESULT: FAIL" : "RESULT: PASS", fails);
  return fails ? 1 : 0;
}
