#include "planning/bldc_actuator.hpp"
#include "define.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <initializer_list>
#include <stdio.h>

// V/Hz パラメータ (sample/bldc_pio での実機検証値。ALIGN/低速域はAMP_BASE、
// それ以上はhzに比例させ、MAX_AMPでクランプする点は旧sine版と同じ考え方)
static constexpr float AMP_BASE        = 0.06f;
static constexpr float AMP_BASE_HZ     = 120.0f;
static constexpr float MAX_AMP         = 0.35f;
// [重要] ペーシングPWM slice(整数clkdiv 1-255 x wrap 16bit)で表現できる
// 最小sector周波数は概ね clk_sys/(255*65536)≈9Hz。elec_hz=sector_hz/6 なので、
// START_ELEC_HZ はこれより十分高い値にする必要がある(旧CPUタイマー方式の
// 1Hzはこの構成では使えない)。
static constexpr float START_ELEC_HZ   = 3.0f;
static constexpr float RAMP_END_HZ     = 16000.0f;

static constexpr uint32_t BLDC_PWM_FREQ_HZ = 80000u;
static constexpr uint32_t ALIGN_TICKS =
    (uint32_t)(BldcActuator::PLANNING_HZ) * 600 / 1000;  // 600ms分の1kHz tick数

// U(slice4)/V,W(slice5)と重ならない、GPIOに紐付けないペーシング専用slice。
static constexpr uint PACE_SLICE = 6;

// [実機知見] 1〜2相をGNDに固定する疑似6-step(旧方式)は実機で全く回転せず、
// 全相を常に50%中心(HIGH=0.5+amp/LOW=0.5-amp/FLOAT=0.5)でチョップし続ける
// 教科書通りの台形波駆動でのみ回転できることが sample/bldc_pio で確認できた。
// GND固定を一切作らないことが肝(sample/bldc.cppのsine駆動と同じ電気的性質)。
enum class PhaseRole : uint8_t { ROLE_LOW = 0, ROLE_FLOAT = 1, ROLE_HIGH = 2 };

// [sector][U,V,W] のロール表。standard 6-step commutation table
// (常にHIGH1相・LOW1相・FLOAT1相)。
static constexpr PhaseRole SECTOR_ROLE[6][3] = {
    {PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_LOW,   PhaseRole::ROLE_FLOAT},  // sector0
    {PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_LOW},    // sector1
    {PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_LOW},    // sector2
    {PhaseRole::ROLE_LOW,   PhaseRole::ROLE_HIGH,  PhaseRole::ROLE_FLOAT},  // sector3
    {PhaseRole::ROLE_LOW,   PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_HIGH},   // sector4
    {PhaseRole::ROLE_FLOAT, PhaseRole::ROLE_LOW,   PhaseRole::ROLE_HIGH},   // sector5
};

static inline float amp_from_hz(float hz, float gain) {
  if (hz <= AMP_BASE_HZ) return AMP_BASE;
  const float a = AMP_BASE * (hz / AMP_BASE_HZ) * gain;
  return a < AMP_BASE ? AMP_BASE : (a > MAX_AMP ? MAX_AMP : a);
}

static inline float role_duty(PhaseRole role, float amp) {
  switch (role) {
  case PhaseRole::ROLE_HIGH: return 0.5f + amp;
  case PhaseRole::ROLE_LOW:  return 0.5f - amp;
  default:                   return 0.5f;
  }
}

// ============================================================
// init
// ============================================================
void BldcActuator::init() {
  gpio_set_function(SUCTION_PWM1, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM2, GPIO_FUNC_PWM);
  gpio_set_function(SUCTION_PWM3, GPIO_FUNC_PWM);
  gpio_init(SUCTION_EN);
  gpio_set_dir(SUCTION_EN, GPIO_OUT);
  gpio_put(SUCTION_EN, 0);

  slice_s1_   = pwm_gpio_to_slice_num(SUCTION_PWM1);
  slice_s2_   = pwm_gpio_to_slice_num(SUCTION_PWM2);
  slice_pace_ = PACE_SLICE;
  sys_clk_hz_ = clock_get_hz(clk_sys);
  wrap_ = (uint32_t)(sys_clk_hz_ / BLDC_PWM_FREQ_HZ) - 1u;  // = 1874

  for (uint s : {slice_s1_, slice_s2_}) {
    pwm_set_clkdiv_int_frac4(s, 1, 0);
    pwm_set_wrap(s, wrap_);
    pwm_set_enabled(s, false);
  }

  // ペーシング専用slice。どのGPIOにも紐付けない(DREQ生成専用)。
  pwm_set_clkdiv_int_frac4(slice_pace_, 1, 0);
  pwm_set_wrap(slice_pace_, 0xFFFF);
  pwm_set_enabled(slice_pace_, false);

  setup_dma();
}

// ============================================================
// DMA配線 (chained control block方式)
//
// data channel: 6word(sectorテーブル)をペーシングDREQに合わせて cc レジスタへ
// 書き込み続ける。完了(6word転送完了)ごとに reload_a -> reload_b を自動発火し、
// 「transfer_countを6へ再設定 -> read_addrをtable先頭へ再設定+再トリガ」を
// 行うことで、CPU一切無しの無限ループを実現する
// (RP2350データシート記載の "chained control block" 方式。
//  詳細は sample/bldc_pio/bldc.md 参照)。
// ============================================================
void BldcActuator::configure_phase_dma(int data_ch, int reload_a_ch, int reload_b_ch,
                                        volatile uint32_t *cc_reg, uint32_t *table,
                                        uint32_t &table_addr_holder, uint pace_dreq) {
  table_addr_holder = (uint32_t)(uintptr_t)table;

  dma_channel_config dc = dma_channel_get_default_config(data_ch);
  channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
  channel_config_set_read_increment(&dc, true);
  channel_config_set_write_increment(&dc, false);
  channel_config_set_dreq(&dc, pace_dreq);
  channel_config_set_chain_to(&dc, reload_a_ch);
  dma_channel_configure(data_ch, &dc, cc_reg, table, 6, false);

  // reload_a: 定数6を data_ch の transfer_count(非トリガ)へ書く
  static const uint32_t k_six = 6;
  dma_channel_config ra = dma_channel_get_default_config(reload_a_ch);
  channel_config_set_transfer_data_size(&ra, DMA_SIZE_32);
  channel_config_set_read_increment(&ra, false);
  channel_config_set_write_increment(&ra, false);
  channel_config_set_chain_to(&ra, reload_b_ch);
  dma_channel_configure(reload_a_ch, &ra,
                        &dma_hw->ch[data_ch].al2_transfer_count,
                        &k_six, 1, false);

  // reload_b: tableの先頭アドレスを data_ch の read_addr(トリガ)へ書く
  // → read_addr設定と同時に data_ch を再トリガ、次の6word転送が始まる
  dma_channel_config rb = dma_channel_get_default_config(reload_b_ch);
  channel_config_set_transfer_data_size(&rb, DMA_SIZE_32);
  channel_config_set_read_increment(&rb, false);
  channel_config_set_write_increment(&rb, false);
  channel_config_set_chain_to(&rb, reload_b_ch);  // 自己参照 = これ以上の連鎖なし
  dma_channel_configure(reload_b_ch, &rb,
                        &dma_hw->ch[data_ch].al3_read_addr_trig,
                        &table_addr_holder, 1, false);
}

void BldcActuator::setup_dma() {
  ch_u_data_      = dma_claim_unused_channel(true);
  ch_u_reload_a_  = dma_claim_unused_channel(true);
  ch_u_reload_b_  = dma_claim_unused_channel(true);
  ch_vw_data_     = dma_claim_unused_channel(true);
  ch_vw_reload_a_ = dma_claim_unused_channel(true);
  ch_vw_reload_b_ = dma_claim_unused_channel(true);

  const uint pace_dreq = pwm_get_dreq(slice_pace_);

  configure_phase_dma(ch_u_data_, ch_u_reload_a_, ch_u_reload_b_,
                      &pwm_hw->slice[slice_s1_].cc, u_table_, u_table_addr_, pace_dreq);
  configure_phase_dma(ch_vw_data_, ch_vw_reload_a_, ch_vw_reload_b_,
                      &pwm_hw->slice[slice_s2_].cc, vw_table_, vw_table_addr_, pace_dreq);
}

void BldcActuator::stop_dma() {
  dma_channel_abort(ch_u_data_);
  dma_channel_abort(ch_u_reload_a_);
  dma_channel_abort(ch_u_reload_b_);
  dma_channel_abort(ch_vw_data_);
  dma_channel_abort(ch_vw_reload_a_);
  dma_channel_abort(ch_vw_reload_b_);
  pwm_set_enabled(slice_pace_, false);
}

// ALIGN終了時、初めてDMAループを開始する。
void BldcActuator::start_dma_ramp() {
  recompute_tables(amp_from_hz(elec_hz_, amp_gain_));
  set_pacing_freq(6.0f * elec_hz_);

  dma_channel_set_read_addr(ch_u_data_, u_table_, false);
  dma_channel_set_trans_count(ch_u_data_, 6, false);
  dma_channel_set_read_addr(ch_vw_data_, vw_table_, false);
  dma_channel_set_trans_count(ch_vw_data_, 6, false);

  pwm_set_enabled(slice_pace_, true);
  dma_channel_start(ch_u_data_);
  dma_channel_start(ch_vw_data_);
}

// RAMP/RUN中、1kHzごとにテーブルとペーシング周波数だけを更新する
// (DMAループそのものは止めない・触らない)。
void BldcActuator::update_running(float hz) {
  recompute_tables(amp_from_hz(hz, amp_gain_));
  set_pacing_freq(6.0f * hz);
}

void BldcActuator::recompute_tables(float amp) {
  const float wu1 = (float)(wrap_ + 1u);
  for (int s = 0; s < 6; s++) {
    const float du = role_duty(SECTOR_ROLE[s][0], amp);
    const float dv = role_duty(SECTOR_ROLE[s][1], amp);
    const float dw = role_duty(SECTOR_ROLE[s][2], amp);
    const uint16_t lu = (uint16_t)(wu1 * du);
    const uint16_t lv = (uint16_t)(wu1 * dv);
    const uint16_t lw = (uint16_t)(wu1 * dw);
    u_table_[s]  = (uint32_t)lu << 16;
    vw_table_[s] = reverse_ ? (((uint32_t)lv << 16) | lw)
                             : (((uint32_t)lw << 16) | lv);
  }
}

// ペーシングslice の wrap 周波数(=sector切替周波数)を設定する。
// clkdiv(整数1-255) x wrap(16bit) で表現できる範囲に収める。
void BldcActuator::set_pacing_freq(float sector_hz) {
  if (sector_hz < 1.0f) sector_hz = 1.0f;
  const float period_ticks = (float)sys_clk_hz_ / sector_hz;
  uint32_t clkdiv = 1;
  while ((period_ticks / (float)clkdiv) > 65536.0f && clkdiv < 255u) clkdiv++;
  uint32_t wrap = (uint32_t)(period_ticks / (float)clkdiv);
  if (wrap < 2u) wrap = 2u;
  if (wrap > 65536u) wrap = 65536u;
  pwm_set_clkdiv_int_frac4(slice_pace_, (uint8_t)clkdiv, 0);
  pwm_set_wrap(slice_pace_, wrap - 1u);
}

// ALIGN用: DMAを介さず直接1回だけCCレジスタに書く(静的保持)。
void BldcActuator::write_direct(uint8_t sector, float amp) {
  const float du = role_duty(SECTOR_ROLE[sector][0], amp);
  const float dv = role_duty(SECTOR_ROLE[sector][1], amp);
  const float dw = role_duty(SECTOR_ROLE[sector][2], amp);
  write_direct_raw(du, dv, dw);
}

void BldcActuator::write_direct_raw(float du, float dv, float dw) {
  const float wu1 = (float)(wrap_ + 1u);
  const uint16_t lu = (uint16_t)(wu1 * du);
  const uint16_t lv = (uint16_t)(wu1 * dv);
  const uint16_t lw = (uint16_t)(wu1 * dw);
  pwm_hw->slice[slice_s1_].cc = (uint32_t)lu << 16;
  pwm_hw->slice[slice_s2_].cc = reverse_ ? (((uint32_t)lv << 16) | lw)
                                          : (((uint32_t)lw << 16) | lv);
}

// ============================================================
// add_repeating_timer_us コールバック (1kHz planning tick)
// ============================================================
bool BldcActuator::timer_cb(repeating_timer_t *rt) {
  auto *self = static_cast<BldcActuator *>(rt->user_data);
  self->tick();
  return true;  // 継続
}

void BldcActuator::tick() {
  constexpr float dt = 1.0f / (float)PLANNING_HZ;

  switch (state_) {
  case State::STOP:
    break;

  case State::ALIGN:
    write_direct(0, AMP_BASE);
    {
      uint32_t cnt = state_cnt_ + 1u;
      state_cnt_ = cnt;
      if (cnt >= ALIGN_TICKS) {
        state_     = State::RAMP;
        state_cnt_ = 0u;
        elec_hz_   = START_ELEC_HZ;
        start_dma_ramp();
      }
    }
    break;

  case State::RAMP:
    elec_hz_ += ramp_hz_per_sec_ * dt;
    if (elec_hz_ > RAMP_END_HZ) elec_hz_ = RAMP_END_HZ;
    update_running(elec_hz_);
    if (elec_hz_ >= RAMP_END_HZ) state_ = State::RUN;
    break;

  case State::RUN:
    if (elec_hz_ < target_elec_hz_) {
      elec_hz_ += ramp_hz_per_sec_ * dt;
      if (elec_hz_ > target_elec_hz_) elec_hz_ = target_elec_hz_;
    } else if (elec_hz_ > target_elec_hz_) {
      elec_hz_ -= ramp_hz_per_sec_ * dt;
      if (elec_hz_ < target_elec_hz_) elec_hz_ = target_elec_hz_;
    }
    update_running(elec_hz_);
    break;
  }
}

void BldcActuator::set_duty(float /*duty_pct*/) {}

// ============================================================
// enable
// ============================================================
void BldcActuator::enable() {
  if (enabled_) return;
  stop_dma();

  pwm_set_mask_enabled((1u << slice_s1_) | (1u << slice_s2_));
  write_direct(0, AMP_BASE);
  gpio_put(SUCTION_EN, 1);
  sleep_ms(20);
  state_     = State::ALIGN;
  elec_hz_   = 0.0f;
  state_cnt_ = 0u;
  amp_gain_  = 0.12f;
  enabled_   = true;
  // 1kHz planning tick。sector切替(最大数万Hz)はPWM+DMAが自律的に行うため、
  // CPU割り込みはこの1kHzだけで済む。
  add_repeating_timer_us(-(int32_t)(1000000 / PLANNING_HZ), timer_cb, this, &timer_);
}

// ============================================================
// disable
// ============================================================
void BldcActuator::disable() {
  enabled_ = false;
  state_   = State::STOP;
  cancel_repeating_timer(&timer_);
  stop_dma();
  gpio_put(SUCTION_EN, 0);
  pwm_set_enabled(slice_s1_, false);
  pwm_set_enabled(slice_s2_, false);
  pwm_set_chan_level(slice_s1_, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_s2_, PWM_CHAN_B, 0);
}

// ============================================================
// test_direct: 診断用。enable()→5秒運転→disable()
// ============================================================
void BldcActuator::test_direct(float amplitude_pct) {
  (void)amplitude_pct;
  if (enabled_) disable();
  enable();
  printf("bldc: ALIGN start (600ms)...\n");
  sleep_ms(700);
  printf("bldc: RAMP start, hz=%.0f\n", (double)elec_hz_);
  sleep_ms(2000);
  printf("bldc: RUN, hz=%.0f target=%.0f\n", (double)elec_hz_, (double)target_elec_hz_);
  sleep_ms(2300);
  printf("bldc: done, hz=%.0f\n", (double)elec_hz_);
  disable();
}
