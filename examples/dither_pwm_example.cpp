// ============================================================================
// DitherPwm 検証用スタンドアロンサンプル(RP2350 / Pico SDK)
//
// ビルド:  cmake --build build --target dither_pwm_example
// 出力:    build/dither_pwm_example.uf2 (.bin)
// 書き込み: picotool load -f -u -x build/dither_pwm_example.bin -t bin -o 0x10000000
//          (または BOOTSEL で uf2 をコピー)
// モニタ:  ./serial_monitor.sh  (USB CDC, 115200)
//
// 何を検証するか
//   [S] ソフトウェア自己検査: 変調器の長期平均が指令に一致するか、N/N+1 が均等に
//       散っているか(起動時に USB に出力。ハード不要)。
//   [H1] オシロ: PWM_GPIO_A の H 幅が N count と N+1 count の 2 値に分かれ、その出現比が
//       指令の小数部に一致する(infinite persistence で 2 本の立下りが見える)。
//       100 kHz では 1 count = 6.67 ns なので見づらい。SLOW_SCOPE_MODE=1 にすると
//       clkdiv=16 (PWM 6.25 kHz, 1 count = 107 ns) になり普通のオシロで見える。
//   [H2] 平均電圧: PWM_GPIO_A に RC(10 kΩ + 1 µF, τ=10 ms)を付けて DC を DMM/オシロで見る。
//       小数部を 0.25 刻みで変えると 3.3 V/1500 × 0.25 = 0.55 mV 刻みで DC が動く。
//       ランプ区間ではステップではなく滑らかに変化する。
//   [H3] ロジアナ(≥150 MS/s): 1 ms 分の H 幅ヒストグラム → N と N+1 の比率 = 小数部。
//   [F] fail-static: 10 s ごとに制御 tick を故意に STRESS_SKIP_TICKS 回スキップする。
//       出力は「最後の duty」を保持したまま途切れず、stats の late/underrun が増える。
//
// ★注意: 既定ピンは ExiaIgnis の左モータ (GPIO4=M_PWM_L1, GPIO5=M_PWM_L2, slice 2)。
//   モータが繋がっていれば低 duty(4 %前後 = 12 V で約 0.5 V)で微弱に通電する。
//   ドライバの nSLEEP を落とすかモータを外して測ること。REF_ON_B=1 は B にも出力する。
// ============================================================================
#include <stdio.h>
#include <string.h>

#include "driver/dither_pwm.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// ---- 設定 ------------------------------------------------------------------
#define PWM_GPIO_A        4        // slice 2 ch A (M_PWM_L1)
#define PWM_GPIO_B        5        // slice 2 ch B (M_PWM_L2)
#define USE_SECOND_SLICE  1        // GPIO6/7 (slice 3) も同時に駆動して slice 間同期を見る
#define PWM2_GPIO_A       6
#define PWM2_GPIO_B       7
#define REF_ON_B          0        // 1: B に「ディザ無しの整数部」を出す(A と幅を直接比較できる)
#define SLOW_SCOPE_MODE   0        // 1: clkdiv=16 → PWM 6.25 kHz、1 count = 107 ns
#define CONTROL_HZ        4000     // 制御周期(将来想定の 4 kHz)。1000 でも可
#define PWM_TOP           1499     // 150 MHz / 1500 = 100 kHz
#define STRESS_SKIP_TICKS 12       // 10 s ごとに制御 tick をこれだけ飛ばす(3 ms @4kHz)。0 で無効

static dpwm::DitherPwm g_pwm;

// 制御 tick から見た指令(A ch)。B は REF_ON_B に応じて整数部 or 0。
static volatile uint32_t g_cmd_a_q16 = 0;
static volatile uint32_t g_skip      = 0;
static volatile uint32_t g_tick      = 0;

// ---- 制御周期 tick(RT 側の担当: 指令を渡して update() を呼ぶだけ) ----------
static bool control_tick_cb(repeating_timer_t*) {
  if (g_skip) { g_skip = g_skip - 1; return true; }   // fail-static 試験: CPU 遅延を模擬 (volatile の -- は C++20 で非推奨)
  const uint32_t a = g_cmd_a_q16;
  const uint32_t b = REF_ON_B ? (a & 0xFFFF0000u) : 0u;
  g_pwm.set_levels_q16(0, a, b);
#if USE_SECOND_SLICE
  g_pwm.set_levels_q16(1, a, b);
#endif
  g_pwm.update();
  g_tick = g_tick + 1;
  return true;
}

// ---- [S] ソフトウェア自己検査 ------------------------------------------------
static void self_test() {
  printf("\n[S] modulator self test (ErrorDiffusion1, Q16.16)\n");
  const float cases[] = {500.25f, 500.36f, 500.5f, 60.75f, 1499.999f, 0.02f};
  for (float c : cases) {
    dpwm::ErrorDiffusion1 m;
    const uint32_t q = dpwm::DitherPwm::q16_from_counts(c);
    const uint32_t N = 65536;
    uint64_t sum = 0;
    uint32_t n_hi = 0;
    for (uint32_t i = 0; i < N; ++i) {
      const uint16_t o = m.next(q);
      sum += o;
      if (o > (q >> 16)) n_hi++;
    }
    // 期待値: q * N / 65536 = q ちょうど(N=65536 なので)
    const double mean = (double)sum / N;
    const double want = (double)q / 65536.0;
    printf("  cmd=%9.4f  mean=%10.5f  err=%+.2e count  N+1 ratio=%.4f  %s\n",
           c, mean, mean - want, (double)n_hi / N,
           (mean - want < 1e-4 && want - mean < 1e-4) ? "OK" : "NG");
  }
  // 分配の様子(25 周期 = 4 kHz 制御 1 tick 分)
  {
    dpwm::ErrorDiffusion1 m;
    const uint32_t q = dpwm::DitherPwm::q16_from_counts(500.36f);
    printf("  pattern for 500.36 over 25 periods (expect 9 x '501' spread evenly):\n   ");
    for (int i = 0; i < 25; ++i) printf(" %u", m.next(q));
    printf("\n");
  }
}

static void print_stats(const char* tag) {
  for (uint i = 0; i < (USE_SECOND_SLICE ? 2u : 1u); ++i) {
    const dpwm::DitherStats& s = g_pwm.stats(i);
    printf("[%s] slice%u ch%u ridx=%3lu ticks=%lu consumed=%lu late=%lu(max %lu) early=%lu "
           "underrun=%lu/%lu stall=%lu backlog_max=%lu skew_max=%lu\n",
           tag, g_pwm.slice(i), g_pwm.dma_channel(i), (unsigned long)g_pwm.dma_read_index(i),
           (unsigned long)s.ticks, (unsigned long)s.consumed_total,
           (unsigned long)s.late_samples, (unsigned long)s.late_max,
           (unsigned long)s.early_samples, (unsigned long)s.underrun_events,
           (unsigned long)s.underrun_samples, (unsigned long)s.stall_ticks,
           (unsigned long)s.dreq_backlog_max, (unsigned long)s.skew_max);
  }
}

// 直近 commit 領域のリング内容を表示(DMA が読む実データ)
static void print_ring_pattern(uint idx, uint32_t n) {
  const uint32_t end = g_pwm.commit_end(idx);
  printf("  ring[commit-%lu..commit) A-level:", (unsigned long)n);
  for (uint32_t s = end - n; s != end; ++s) printf(" %u", (unsigned)(g_pwm.ring_word(idx, s) & 0xFFFFu));
  printf("\n");
}

int main() {
  stdio_init_all();
  sleep_ms(2500);  // USB CDC を開く時間
  printf("\n==== DitherPwm example (RP2350) ====\n");
  printf("clk_sys=%lu Hz  PWM_TOP=%d  CONTROL_HZ=%d  SLOW_SCOPE_MODE=%d\n",
         (unsigned long)clock_get_hz(clk_sys), PWM_TOP, CONTROL_HZ, SLOW_SCOPE_MODE);

  self_test();

  // ---- ハードウェア設定 ---------------------------------------------------
  gpio_set_function(PWM_GPIO_A, GPIO_FUNC_PWM);
  gpio_set_function(PWM_GPIO_B, GPIO_FUNC_PWM);
  uint slices[2] = {pwm_gpio_to_slice_num(PWM_GPIO_A), 0};
  uint n = 1;
#if USE_SECOND_SLICE
  gpio_set_function(PWM2_GPIO_A, GPIO_FUNC_PWM);
  gpio_set_function(PWM2_GPIO_B, GPIO_FUNC_PWM);
  slices[1] = pwm_gpio_to_slice_num(PWM2_GPIO_A);
  n = 2;
#endif

  dpwm::DitherPwm::Config cfg;
  cfg.top          = PWM_TOP;
  cfg.clkdiv_int   = SLOW_SCOPE_MODE ? 16 : 1;
  cfg.clkdiv_frac4 = 0;
  // 制御周期あたりの PWM 周期数(切り上げ)
  {
    const uint32_t pwm_hz = clock_get_hz(clk_sys) / ((uint32_t)(PWM_TOP + 1) * cfg.clkdiv_int);
    cfg.samples_per_tick  = (uint16_t)((pwm_hz + CONTROL_HZ - 1) / CONTROL_HZ);
  }
  cfg.lead_samples  = 4;   // 指令レイテンシ ≈ 4 周期 + 1 周期(latch) = 50 µs @100 kHz
  cfg.guard_samples = 0;   // = samples_per_tick(1 制御周期ぶん「最後の duty」を保持)
  cfg.high_priority = true;

  if (!g_pwm.init(cfg, slices, n)) {
    printf("init FAILED (ring alignment / span > ring?)\n");
    while (true) tight_loop_contents();
  }
  printf("init OK: samples_per_tick=%u lead=%u guard=%u period=%lu us  ring=%u samples\n",
         cfg.samples_per_tick, cfg.lead_samples, cfg.samples_per_tick,
         (unsigned long)g_pwm.period_us(), dpwm::DitherPwm::kRingSamples);

  // 初期指令(60.0 count = 4.0 %)。start() 前に set しておく。
  g_cmd_a_q16 = dpwm::DitherPwm::q16_from_counts(60.0f);
  g_pwm.set_levels_q16(0, g_cmd_a_q16, 0);
  if (n > 1) g_pwm.set_levels_q16(1, g_cmd_a_q16, 0);
  g_pwm.start();

  // 制御 tick(実機では Core1 の TIMER IRQ に相当)
  repeating_timer_t timer;
  add_repeating_timer_us(-(int32_t)(1000000 / CONTROL_HZ), control_tick_cb, nullptr, &timer);
  printf("started. PWM on GPIO%d/%d%s\n", PWM_GPIO_A, PWM_GPIO_B,
         USE_SECOND_SLICE ? " and GPIO6/7" : "");

  // ---- 試験シーケンス ----------------------------------------------------
  //  0-2 s : 60.00  (ディザ無し基準)
  //  2-4 s : 60.25
  //  4-6 s : 60.50
  //  6-8 s : 60.75
  //  8-10 s: 61.00
  //  10-15 s: 60.00 → 61.00 ランプ(1/256 count 刻み, 20 tick ごと)
  //  以後繰り返し。10 s ごとに STRESS_SKIP_TICKS の tick スキップ(fail-static 試験)。
  const float steps[] = {60.0f, 60.25f, 60.5f, 60.75f, 61.0f};
  absolute_time_t t_next_print = make_timeout_time_ms(1000);
  uint32_t phase = 0;
  absolute_time_t t_phase = get_absolute_time();
  uint32_t ramp_q16 = 0;
  uint32_t ramp_tick_last = 0;
  bool pattern_shown = false;
  uint32_t cycle = 0;

  while (true) {
    const int64_t el_ms = absolute_time_diff_us(t_phase, get_absolute_time()) / 1000;
    if (phase < 5) {
      g_cmd_a_q16 = dpwm::DitherPwm::q16_from_counts(steps[phase]);
      if (phase == 1 && !pattern_shown && el_ms > 20) {
        printf("  cmd=60.25 →");
        print_ring_pattern(0, cfg.samples_per_tick);
        pattern_shown = true;
      }
      if (el_ms >= 2000) { phase++; t_phase = get_absolute_time(); pattern_shown = false; }
    } else {
      // ランプ
      if (el_ms == 0 && ramp_q16 == 0) ramp_q16 = dpwm::DitherPwm::q16_from_counts(60.0f);
      if (g_tick - ramp_tick_last >= 20) {
        ramp_tick_last = g_tick;
        ramp_q16 += 256;  // 1/256 count
        if (ramp_q16 > dpwm::DitherPwm::q16_from_counts(61.0f)) ramp_q16 = dpwm::DitherPwm::q16_from_counts(60.0f);
        g_cmd_a_q16 = ramp_q16;
      }
      if (el_ms >= 5000) {
        phase = 0; t_phase = get_absolute_time(); ramp_q16 = 0; cycle++;
        if (STRESS_SKIP_TICKS > 0) {
          printf("[F] skipping %d control ticks (fail-static test)\n", STRESS_SKIP_TICKS);
          g_skip = STRESS_SKIP_TICKS;
        }
      }
    }

    if (absolute_time_diff_us(get_absolute_time(), t_next_print) <= 0) {
      t_next_print = make_timeout_time_ms(1000);
      printf("t=%lus phase=%lu cmd=%.4f count (%.4f %%)  ",
             (unsigned long)(to_ms_since_boot(get_absolute_time()) / 1000), (unsigned long)phase,
             (double)g_cmd_a_q16 / 65536.0, (double)g_cmd_a_q16 / 65536.0 * 100.0 / (PWM_TOP + 1));
      print_stats("stat");
    }
    sleep_us(200);
  }
}
