#pragma once
#include "hardware/pwm.h"
#include "pico/time.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ (PWM+DMA自己ループ, RP2350専用)
//
// 波形は当初サイン波(SPWM)、その後CPU 80kHzタイマー駆動の台形波を経て、
// 現在は sample/bldc_pio (詳細は sample/bldc_pio/bldc.md 参照) で検証した
// 「PWMペリフェラル + DMAの自己ループ(chained control block)」方式に
// 置き換えた。理由: CPU 80kHzタイマー割り込みはcore0のUI/ボタン/printf
// 処理を圧迫するため不採用となり、CPU割り込みを1kHzのplanning tickのみに
// 抑え、sector切替(6-step commutation, 最大数万Hz)は完全にPWM+DMAへ
// 自律的に任せる構成にした。
//
// 波形自体(HIGH=0.5+amp/LOW=0.5-amp/FLOAT=0.5、60°ごとに役割を入れ替える
// 教科書通りの台形波)は変わっていない。1〜2相をGNDに固定する疑似6-stepは
// 実機で全く回転しなかったため、全相を常に50%中心でチョップし続ける
// (=どの相も一度もGND固定にならない)ことが必須。
//
// 起動シーケンス: ALIGN(600ms, 静的保持) → RAMP(3→1200Hz) → RUN(target_elec_hz_ を追従)
// 振幅: V/Hz制御 (AMP_BASE=0.06, AMP_BASE_HZ=120, MAX_AMP=0.35)
//
// GPIO8=SUCTION_EN / GPIO9(PWM4B)=U相 / GPIO10(PWM5A)=V相 / GPIO11(PWM5B)=W相
class BldcActuator {
public:
  static constexpr int CONTROL_HZ  = 80000;  // U/V/W出力PWMの物理キャリア周波数(固定)
  static constexpr int PLANNING_HZ = 1000;   // CPUが触る唯一のレート(1kHz)

  void init();
  void set_duty(float duty_pct);
  void enable();
  void disable();
  void set_direction(bool reverse) { reverse_ = reverse; }
  void set_min_amplitude(float v)  { (void)v; }
  void set_ramp_rate(float hz_per_sec)  { ramp_hz_per_sec_ = hz_per_sec; }
  void set_target_hz(float hz)          { target_elec_hz_  = hz; }
  // V/Hzカーブのゲイン(sample/bldc_pioのg_amp_gainに相当)。amp_from_hzの
  // 傾きを決める。enable()では上書きしない(電源投入後は外部設定を保持)。
  void set_amp_gain(float gain)         { amp_gain_ = gain; }
  // 振幅上限(sample/bldc_pioのg_max_ampに相当)。高hzでの飽和先を決める。
  void set_max_amp(float v)             { max_amp_  = v; }
  void test_direct(float amplitude_pct);

  // [診断用] RAMP/RUN中でも安全なトレース機能。printfは1kHz tickを遅延させ
  // 脱調の原因になり得るため使えない。代わりに10ms間隔でelec_hz/stateを
  // 軽量バッファへ記録しておき、is_ramping()==falseになってから(=安全な
  // タイミングで) dump_trace() でまとめて出力する。
  void dump_trace() const;

  bool  is_enabled()   const { return enabled_; }
  bool  is_ramping()   const {
    return state_ == State::ALIGN || state_ == State::RAMP ||
           (state_ == State::RUN && elec_hz_ < target_elec_hz_ - 1.0f);
  }
  bool  is_dma_busy()  const { return state_ == State::RAMP || state_ == State::RUN; }
  float get_elec_hz()   const { return elec_hz_; }
  float get_target_hz() const { return target_elec_hz_; }
  float get_amp_gain()  const { return amp_gain_; }
  float get_max_amp()   const { return max_amp_; }
  float get_ramp_rate() const { return ramp_hz_per_sec_; }

private:
  static bool timer_cb(repeating_timer_t *rt);
  void tick();  // 1kHz planning tick

  void setup_dma();
  void configure_phase_dma(int data_ch, int reload_a_ch, int reload_b_ch,
                            volatile uint32_t *cc_reg, uint32_t *table,
                            uint32_t &table_addr_holder, uint pace_dreq);
  void stop_dma();
  void start_dma_ramp();
  void update_running(float hz);
  void recompute_tables(float amp);
  void set_pacing_freq(float sector_hz);
  void write_direct(uint8_t sector, float amp);
  void write_direct_raw(float du, float dv, float dw);

  enum class State : uint8_t { STOP, ALIGN, RAMP, RUN };

  uint     slice_s1_   = 0;
  uint     slice_s2_   = 0;
  uint     slice_pace_ = 0;   // GPIO非接続、sector切替ペーシング専用
  uint32_t wrap_        = 0;
  uint32_t sys_clk_hz_  = 0;

  // DMAチャンネル (phase毎に data + reload x2 = 3ch x 2相 = 計6ch)
  int ch_u_data_ = -1, ch_u_reload_a_ = -1, ch_u_reload_b_ = -1;
  int ch_vw_data_ = -1, ch_vw_reload_a_ = -1, ch_vw_reload_b_ = -1;

  uint32_t u_table_[6]    = {};
  uint32_t vw_table_[6]   = {};
  uint32_t u_table_addr_  = 0;
  uint32_t vw_table_addr_ = 0;

  // [診断用トレース] 10ms間隔 x 600件 = 6秒分。printfを使わず記録するだけ
  // なので1kHz tickへの影響は無視できる。
  static constexpr int TRACE_LEN = 600;
  float   trace_hz_[TRACE_LEN]    = {};
  uint8_t trace_state_[TRACE_LEN] = {};
  int     trace_idx_    = 0;
  bool    trace_full_   = false;
  uint32_t trace_div_cnt_ = 0;

  volatile State    state_           = State::STOP;
  volatile float    elec_hz_         = 0.0f;
  volatile float    amp_gain_        = 0.10f;
  volatile float    max_amp_         = 0.35f;
  volatile uint32_t state_cnt_       = 0u;
  volatile float    ramp_hz_per_sec_ = 2000.0f;

  float    target_elec_hz_ = 6000.0f;
  bool     enabled_        = false;
  bool     reverse_        = true;

  repeating_timer_t timer_ = {};
};
