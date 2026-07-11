#pragma once
#include "hardware/pwm.h"
#include <stdint.h>
#include <vector>

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
// [重要] tick() は PlanningTask::timer_irq_handler() (core1, TIMER0 alarm
// 直叩き) から毎tick呼び出すこと。以前は enable() 内で
// add_repeating_timer_us() を使い独自の1kHzタイマーを持たせていたが、これは
// SDKのデフォルトalarm_pool(crt0を実行したcore=core0に固定)経由で動くため、
// core0のUI/ボタン/USB/printf処理と割り込みを奪い合いジッタの原因になって
// いた(sample/bldc_pioは他に何も動いていない単一コアだったため問題化しな
// かった)。alarm_pool経由をやめてPlanningTaskに統合したことで、
// sensing_task.cpp と同じ「alarm_poolを介さない直接ハードウェアアラーム」
// 方式になり、ジッタ要因が排除される。
//
// 波形自体(HIGH=0.5+amp/LOW=0.5-amp/FLOAT=0.5、60°ごとに役割を入れ替える
// 教科書通りの台形波)は変わっていない。1〜2相をGNDに固定する疑似6-stepは
// 実機で全く回転しなかったため、全相を常に50%中心でチョップし続ける
// (=どの相も一度もGND固定にならない)ことが必須。
//
// 起動シーケンス: ALIGN(600ms, 静的保持) → RAMP(3→1200Hz) → RUN(target_elec_hz_ を追従)
//
// [重要] ampはPWMデューティの変調幅(0〜0.5)であり電圧そのものではない。
// 実際に巻線へ掛かる電圧は概ね amp×バッテリー電圧 になる。このモーターは
// ホール/エンコーダの無い完全オープンループ駆動(センサレス6-step)であり、
// 目標hzに対して必要以上の電圧(=電流)を掛けると同期を保つトルク角の
// マージンを消費し、脱調しやすくなる(ステッピングモーターの過電流脱調と
// 同じ理屈)。
//
// 振幅は "amp = AMP_BASE*(hz/AMP_BASE_HZ)*gain" のV/Hz比例式(hz<=AMP_BASE_HZ
// はAMP_BASE固定)を、その時点のmax_ampでクランプして決める。gain・max_amp
// はどちらも固定値ではなく、battery_v→{gain, max_amp} LUT(可変長、X軸
// battery_v共通)を区分線形補間して都度求める。満充電(12.6V)と使用後
// (11V台)とでは実電圧が10%以上変わり、同じgain/max_ampのままだと同期を
// 保つトルク角のマージンが変動して脱調しやすくなるため、実測でバッテリー
// 電圧ごとに個別調整できるようにしている。X軸(電圧)・Y軸(gain/max_amp)は
// sys_.test.suction_batt_v_table / suction_batt_gain_table /
// suction_batt_max_amp_table (system.yaml) をそのまま保持する。電圧側は
// 昇順、3配列は同じ長さで指定すること(config_mapping.hpp参照。長さが
// 合わない場合の後始末はしない)。
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
  // battery_v→{gain, max_amp} LUTを丸ごと差し替える(区分線形補間、電圧は
  // 昇順で渡すこと)。v_bpが空なら何もしない。max_ampは0.5=100%duty(片側が
  // 完全に0または1に張り付く)に対する安全マージンとして[0.10, 0.48]に
  // 各要素クランプする(configの誤入力対策)。
  void set_batt_tables(std::vector<float> v_bp, std::vector<float> gain,
                        std::vector<float> max_amp) {
    if (v_bp.empty()) return;
    for (float &v : max_amp) v = v < 0.10f ? 0.10f : (v > 0.48f ? 0.48f : v);
    batt_v_bp_ = std::move(v_bp);
    batt_gain_table_ = std::move(gain);
    batt_max_amp_table_ = std::move(max_amp);
  }
  int   get_batt_table_len()      const { return (int)batt_v_bp_.size(); }
  float get_batt_v_bp(int idx)    const { return batt_v_bp_[idx]; }
  float get_batt_gain_point(int idx) const { return batt_gain_table_[idx]; }
  float get_batt_max_amp_point(int idx) const { return batt_max_amp_table_[idx]; }
  void test_direct(float amplitude_pct);

  // PlanningTask::timer_irq_handler() (core1, TIMER0 alarm, 1kHz) から
  // 毎tick呼ぶこと。BldcActuator自身はもうタイマーを持たない
  // (上記クラスコメント参照)。battery_v は se->ego.battery_lp を渡す
  // (電圧フィードフォワード補正。下記クラスコメント参照)。
  void tick(float battery_v);

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
  float get_ramp_rate() const { return ramp_hz_per_sec_; }

private:
  void setup_dma();
  void configure_phase_dma(int data_ch, int reload_a_ch, int reload_b_ch,
                            volatile uint32_t *cc_reg, uint32_t *table,
                            uint32_t &table_addr_holder, uint pace_dreq);
  void stop_dma();
  void start_dma_ramp();
  void update_running(float hz);
  float compensated_amp(float hz) const;
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
  volatile uint32_t state_cnt_       = 0u;
  volatile float    ramp_hz_per_sec_ = 2000.0f;

  float    target_elec_hz_ = 6000.0f;
  bool     enabled_        = false;
  bool     reverse_        = true;

  // バッテリー電圧→{gain, max_amp} LUT(区分線形補間、可変長)。X/Yとも
  // set_batt_tables() で丸ごと差し替え可能(上記クラスコメント参照)。
  // デフォルトは11.1V基準gain=0.1195を(11.1/battery_v)で比例配分した3点、
  // max_ampは旧固定値0.35のまま(電圧に依らずフラット)。
  std::vector<float> batt_v_bp_          = {9.9f, 11.1f, 12.6f};
  std::vector<float> batt_gain_table_    = {0.1340f, 0.1195f, 0.1053f};
  std::vector<float> batt_max_amp_table_ = {0.35f, 0.35f, 0.35f};

  // battery_v_ はtick()毎にse->ego.battery_lpで更新される。起動直後で
  // まだセンシング値が来ていない場合は11.1V相当(=無補正)にフォールバック。
  volatile float battery_v_ = 11.1f;
};
