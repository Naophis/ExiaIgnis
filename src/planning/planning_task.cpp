#include "planning/planning_task.hpp"
#include "define.hpp" // M_PWM_L1/L2/R1/R2, SUCTION_PWM, MOTOR_PWM_FREQ_HZ
#include "hardware/irq.h"
#include "hardware/timer.h" // timer_hw, TIMER0_IRQ_0
#include "logging/logging_task.hpp"
#include "pico/stdlib.h"
#include "sensing_task.hpp"
#include "utils/irq_log.hpp"
#include <algorithm>
#include <cmath>

// ============================================================
// static メンバの定義
// ============================================================
std::shared_ptr<PlanningTask> PlanningTask::s_instance;

std::shared_ptr<PlanningTask> PlanningTask::create() {
  s_instance = std::shared_ptr<PlanningTask>(new PlanningTask());
  return s_instance;
}

void PlanningTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
  if (sensing_result && param) {
    ego.reset_kf_state(true, sensing_result, param);
  }
}

void PlanningTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
  // sensing_result と param が揃った段階で KF を初回初期化する
  if (sensing_result && param) {
    ego.reset_kf_state(true, sensing_result, param);
  }
}

// ============================================================
// 初期化 (Core0 の main から呼ぶ)
// ============================================================
void PlanningTask::init(std::shared_ptr<SensingTask> sensing) {
  sensing_ = sensing;
  sem_init(&tick_sem_, 0, 1);
  motor_.init();
  bldc_.init();
  sensor_.init(sensing_result, param, tgt_val);
  ctl_.init(&motor_, &bldc_, &sensor_, &trj_, &ego, tgt_val, sensing_result, param);
  ego.init(sensing_result, param, tgt_val);
  trj_.init(tgt_val, param, sensing_result);
}

// ============================================================
// TIMER0 alarm 0 IRQ 登録 (Core1 エントリから呼ぶ)
// ============================================================
void PlanningTask::start_irq() {
  irq_set_exclusive_handler(TIMER0_IRQ_0, timer_irq_handler);
  irq_set_priority(TIMER0_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(TIMER0_IRQ_0, true);
  hw_set_bits(&timer_hw->inte, 1u << 0);

  // sensing (alarm 1) の直後に start するため位相が重なる。
  // sensing 自身の実行時間 (~350us) より後に発火するよう 600us 遅らせる。
  next_alarm_ = timer_hw->timerawl + interval_us_ + 600;
  timer_hw->alarm[0] = next_alarm_;
}

__attribute__((noinline, section(".time_critical.planning_cmd")))
void PlanningTask::send_command(const motion_tgt_val_t &tgt) {
  // double buffer: Core0 は write_cmd_idx_ 側に書き込み、store(release) で公開する。
  // IRQ 内で shared_ptr や new/delete が一切発生しない。
  // printf("send-");
  PlanningCmd &buf = cmd_buf_[write_cmd_idx_];
  buf.nmr    = tgt.nmr;
  buf.pl_req = tgt.pl_req;
  pending_cmd_idx_.store(write_cmd_idx_, std::memory_order_release);
  write_cmd_idx_ ^= 1;
  // printf("-end\n");
}

// ============================================================
// TIMER0 alarm 0 IRQ ハンドラ (Core1)
// ============================================================
__attribute__((noinline, section(".time_critical.planning_tick")))
void PlanningTask::timer_irq_handler() {
  // ハンドラ入口で即計測 — 以後の処理による可変遅延を period 計測に含めない
  const uint64_t now = time_us_64();
  timer_hw->intr = 1u << 0;

  auto *self = s_instance.get();

  // 次回アラームを絶対時刻で設定 (ドリフトなし)
  self->next_alarm_ += self->interval_us_;
  {
    // now を使い回すことで余分な timerawl 読み出しを省く
    const uint32_t now32 = (uint32_t)now;
    if ((int32_t)(now32 - self->next_alarm_) > (int32_t)self->interval_us_)
      self->next_alarm_ = now32 + self->interval_us_;
  }
  timer_hw->alarm[0] = self->next_alarm_;

  const uint32_t dt_us = self->prev_ts_ ? (uint32_t)(now - self->prev_ts_) : 0;
  self->prev_ts_ = now;

  // BLDC吸引モーターのsectorテーブル/ペーシング更新。以前はBldcActuator
  // 自身がadd_repeating_timer_us()で独自の1kHzタイマー(SDKのデフォルト
  // alarm_pool = core0固定)を持っていたが、core0のUI/ボタン/USB/printf
  // 処理と割り込みを奪い合いジッタの原因になっていたため、他のセンサー/
  // 制御処理と同じくこのalarm_poolを介さない直接ハードウェアアラームIRQへ
  // 統合した。battery_lpを渡すことでamp_gain/max_ampをバッテリー電圧に
  // 依らず一定の実電圧になるよう補正する(bldc_actuator.hpp参照)。
  self->bldc_.tick(self->sensing_result->ego.battery_lp);

  self->tick(dt_us);
  sem_release(&self->tick_sem_);
}

// ============================================================
// 1tick 処理 (IRQ ハンドラから呼ばれる)
// ============================================================
__attribute__((noinline, section(".time_critical.planning_tick")))
void PlanningTask::tick(uint32_t dt_us) {
  if (dt_us == 0) [[unlikely]]
    return;

  // 最初の 30 tick + コマンド受信後 6 tick だけログ出力（クラッシュ箇所の特定用）
  static uint32_t tick_num = 0;
  static uint32_t cmd_log_remain = 0;
  ++tick_num;
  if (cmd_log_remain > 0) --cmd_log_remain;
  if (g_ctl_debug_ticks > 0) g_ctl_debug_ticks = g_ctl_debug_ticks - 1;

  // T0 はコマンド処理前に評価（コマンドによる cmd_log_remain 変化は T1 以降に影響）
  const bool do_log0 = false; // (tick_num <= 30) || (cmd_log_remain > 0);
  if (do_log0) g_irq_log.push("T0");

  tgt_val->calc_time_diff = dt_us;
  uint64_t t_prev = time_us_64();
  {
    int idx = pending_cmd_idx_.load(std::memory_order_acquire);
    if (idx >= 0) {
      cmd_log_remain = 0; // このコマンドを受けた tick + 次 N tick を強制ログ (0=無効)
      g_ctl_debug_ticks = 0; // ctl_.calc() サブチェックポイント (0=無効)
      // g_irq_log.push("TR");
      active_cmd_ = cmd_buf_[idx];
      receive_req = &active_cmd_;
      int expected = idx;
      pending_cmd_idx_.compare_exchange_strong(expected, -1,
                                               std::memory_order_relaxed);
      first_req = true;
      cp_request();
      // g_irq_log.push("TQ");
    }
  }

  const float dt = param->dt;

  // T1 以降: コマンド受信で cmd_log_remain が更新された後に評価
  const bool do_log = false; // (tick_num <= 30) || (cmd_log_remain > 0);

  if (do_log) g_irq_log.push("T1"); // pre ego.update
  ego.update(motor_en);
  {
    uint64_t t_now = time_us_64();
    tgt_val->pln_t_ego = (int16_t)(t_now - t_prev);
    t_prev = t_now;
  }

  if (do_log) g_irq_log.push("T2"); // pre sensor.calc_dist
  sensor_.calc_dist();
  {
    uint64_t t_now = time_us_64();
    tgt_val->pln_t_sensor = (int16_t)(t_now - t_prev);
    t_prev = t_now;
  }

  if (do_log) g_irq_log.push("T3"); // pre generate
  if (first_req) {
    if (search_mode && tgt_val->motion_type == MotionType::SLALOM) {
      tgt_val->tgt_in.enable_slip_decel = 1;
    } else {
      tgt_val->tgt_in.enable_slip_decel = 0;
    }
    trj_.generate(last_tgt_angle);
    uint64_t t_now = time_us_64();
    tgt_val->pln_t_trj = (int16_t)(t_now - t_prev);
    t_prev = t_now;
  }

  if (tgt_val->motion_type == MotionType::STRAIGHT ||
      tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
      tgt_val->motion_type == MotionType::SLA_BACK_STR) {
    if (tgt_val->ego_in.img_dist >= tgt_val->tgt_in.tgt_dist) {
      trj_.mpc_next_ego.v = tgt_val->tgt_in.end_v;
    }
    if (tgt_val->ego_in.dist >= tgt_val->tgt_in.tgt_dist) {
      trj_.mpc_next_ego.v = tgt_val->tgt_in.end_v;
    }
  }
  if (do_log) g_irq_log.push("T4"); // pre calc_kanayama
  trj_.calc_kanayama(ego, sensor_, last_tgt_angle);
  {
    uint64_t t_now = time_us_64();
    tgt_val->pln_t_kanayama = (int16_t)(t_now - t_prev);
    t_prev = t_now;
  }

  if (tgt_val->motion_type == MotionType::PIVOT ||
      tgt_val->motion_type == MotionType::FRONT_CTRL) {
    trj_.v_cmd = tgt_val->ego_in.v;
    trj_.w_cmd = tgt_val->ego_in.w;
  }

  if (do_log) g_irq_log.push("T5"); // pre copy_tgt
  trj_.copy_tgt(dt);
  {
    uint64_t t_now = time_us_64();
    tgt_val->pln_t_copy = (int16_t)(t_now - t_prev);
    t_prev = t_now;
  }

  if (do_log) g_irq_log.push("T6"); // pre ctl.calc
  ctl_.calc(motor_en, suction_en, search_mode, last_tgt_angle, dt);
  {
    uint64_t t_now = time_us_64();
    tgt_val->pln_t_ctl = (int16_t)(t_now - t_prev);
  }

  if (do_log) g_irq_log.push("T7"); // tick complete
  tgt_val->calc_time = tgt_val->pln_t_ctl;
}

std::shared_ptr<sensing_result_entity_t> PlanningTask::get_sensing_entity() {
  return sensing_result;
}

float PlanningTask::adjust_b_to_target90(float data, float a) {
  int idx = static_cast<int>(data);

  // インデックス範囲チェック（※ここは index としての妥当性を見るのが自然）
  if (idx < 0 || idx >= static_cast<int>(sensor_.log_table.size())) {
    return NAN; // 計算不能
  }

  float L = sensor_.log_table.at(idx);
  if (!isfinite(L) || L == 0.0f) {
    return NAN;
  }

  // 目標 90 に合う b'
  float b_new = a / L - 90.0f;
  return b_new;
}

float PlanningTask::adjust_b_to_target45(float data, float a) {
  int idx = static_cast<int>(data);

  // インデックス範囲チェック（※ここは index としての妥当性を見るのが自然）
  if (idx < 0 || idx >= static_cast<int>(sensor_.log_table.size())) {
    return NAN; // 計算不能
  }

  float L = sensor_.log_table.at(idx);
  if (!isfinite(L) || L == 0.0f) {
    return NAN;
  }

  // 目標 45 に合う b'
  float b_new = a / L - 45.0f;
  return b_new;
}

void PlanningTask::pl_req_activate() {
  if (receive_req->pl_req.time_stamp != pid_req_timestamp) {
    ctl_.pl_req_activate(receive_req->pl_req);
    pid_req_timestamp = receive_req->pl_req.time_stamp;
  }
}

void PlanningTask::cp_request() {
  const auto se = get_sensing_entity();
  pl_req_activate();
  if (motion_req_timestamp == receive_req->nmr.timstamp) {
    return;
  }
  const float dt = param->dt;
  slip_param.K = param->slip_param_K;
  slip_param.k = param->slip_param_k2;
  motion_req_timestamp = receive_req->nmr.timstamp;

  tgt_val->tgt_in.v_max = receive_req->nmr.v_max;
  // printf("v_max: %f\n", tgt_val->tgt_in.v_max);

  tgt_val->td = TurnDirection::None;
  tgt_val->tt = TurnType::None;
  if (receive_req->nmr.motion_type == MotionType::STRAIGHT ||
      receive_req->nmr.motion_type == MotionType::SLA_FRONT_STR) {
    if (tgt_val->ego_in.v > receive_req->nmr.v_max) {
      tgt_val->tgt_in.v_max = tgt_val->ego_in.v;
    }
  } else if (receive_req->nmr.motion_type == MotionType::SLALOM) {
    tgt_val->td = receive_req->nmr.td;
    tgt_val->tt = receive_req->nmr.tt;
  }
  if (receive_req->nmr.motion_type == MotionType::SLA_BACK_STR) {
    left_keep.star_dist = right_keep.star_dist = tgt_val->global_pos.dist;
  }
  tgt_val->tgt_in.end_v = receive_req->nmr.v_end;
  tgt_val->tgt_in.accl = receive_req->nmr.accl;
  tgt_val->tgt_in.decel = receive_req->nmr.decel;
  tgt_val->tgt_in.w_max = receive_req->nmr.w_max;
  tgt_val->tgt_in.end_w = receive_req->nmr.w_end;
  tgt_val->tgt_in.alpha = receive_req->nmr.alpha;

  tgt_val->tgt_in.tgt_dist = receive_req->nmr.dist;
  last_tgt_angle = tgt_val->tgt_in.tgt_angle;
  tgt_val->tgt_in.tgt_angle = receive_req->nmr.ang;

  tgt_val->motion_mode = (int)(receive_req->nmr.motion_mode);
  tgt_val->motion_type = receive_req->nmr.motion_type;

  tgt_val->ego_in.sla_param.base_alpha = receive_req->nmr.sla_alpha;
  tgt_val->ego_in.sla_param.base_time = receive_req->nmr.sla_time;
  tgt_val->ego_in.sla_param.limit_time_count =
      receive_req->nmr.sla_time * 2 / dt;
  tgt_val->ego_in.sla_param.pow_n = receive_req->nmr.sla_pow_n;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.pivot_state = 0;

  tgt_val->dia_state.left_save = false;
  tgt_val->dia_state.right_save = false;
  tgt_val->dia_state.left_old = tgt_val->dia_state.right_old = 0;

  // if (!(tgt_val->motion_type == MotionType::NONE ||
  //       tgt_val->motion_type == MotionType::STRAIGHT ||
  //       tgt_val->motion_type == MotionType::PIVOT_PRE ||
  //       tgt_val->motion_type == MotionType::PIVOT_AFTER ||
  //       tgt_val->motion_type == MotionType::READY ||
  //       tgt_val->motion_type == MotionType::SENSING_DUMP ||
  //       tgt_val->motion_type == MotionType::WALL_OFF ||
  //       tgt_val->motion_type == MotionType::WALL_OFF_DIA ||
  //       tgt_val->motion_type == MotionType::BACK_STRAIGHT)) {
  //   tgt_val->ego_in.ang -= last_tgt_angle;
  //   tgt_val->ego_in.img_ang = 0;
  //   kf_ang.offset(-last_tgt_angle);
  // } else if (tgt_val->motion_type == MotionType::NONE) {
  //   tgt_val->ego_in.ang = tgt_val->ego_in.img_ang = last_tgt_angle = 0;
  //   kf_ang.reset(0);
  //   tgt_val->ego_in.img_dist = tgt_val->ego_in.dist = 0;
  //   kf_dist.reset(0);
  // }

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::PIVOT ||
      tgt_val->motion_type == MotionType::FRONT_CTRL ||
      tgt_val->motion_type == MotionType::BACK_STRAIGHT ||
      tgt_val->motion_type == MotionType::PIVOT_AFTER) {
    tgt_val->ego_in.ang = tgt_val->ego_in.img_ang = last_tgt_angle = 0;
    ego.kf_ang.reset(0);
    tgt_val->ego_in.img_dist = tgt_val->ego_in.dist = 0;
    ego.kf_dist.reset(0);
  } else {
    tgt_val->ego_in.ang -= last_tgt_angle;
    ego.kim.theta -= last_tgt_angle;
    tgt_val->ego_in.img_ang = 0;
    ego.kf_ang.offset(-last_tgt_angle);
  }

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::READY) {
    last_tgt_angle = 0;
    tgt_val->global_pos.ang -= tgt_val->global_pos.img_ang;
    tgt_val->global_pos.img_ang = 0;

    tgt_val->global_pos.dist -= tgt_val->global_pos.img_dist;
    tgt_val->global_pos.img_dist = 0;
    tgt_val->tgt_in.end_v = 0;
    tgt_val->tgt_in.v_max = 0;
  }

  // right_keep.star_dist = tgt_val->global_pos.dist;
  // left_keep.star_dist = tgt_val->global_pos.dist;

  if (tgt_val->tgt_in.tgt_angle != 0) {
    const auto tmp_ang = tgt_val->ego_in.ang;
    tgt_val->ego_in.img_ang -= last_tgt_angle;
    ego.kf_ang.offset(-last_tgt_angle);
    tgt_val->ego_in.ang -= last_tgt_angle;
    // } else {
    //   ego.kf_ang.reset(0);
  }
  if (tgt_val->tgt_in.tgt_dist != 0) {
    const auto tmp_dist = tgt_val->ego_in.dist;
    tgt_val->ego_in.img_dist -= tmp_dist;
    ego.kf_dist.offset(-tmp_dist);
    tgt_val->ego_in.dist = 0;
    // } else {
    //   ego.kf_dist.reset(0);
  }

  sensing_result->ego.dist_kf = ego.kf_dist.get_state();
  // sensing_result->ego.ang_kf = ego.kf_ang.get_state();

  if (param->enable_kalman_gyro == 1) {
    se->ego.ang_kf = ego.kf_ang.get_state();
  } else if (param->enable_kalman_gyro == 2) {
    se->ego.ang_kf = tgt_val->ego_in.ang;
  } else {
    se->ego.ang_kf = tgt_val->ego_in.ang;
  }

  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.state = 0;

  tgt_val->motion_dir = receive_req->nmr.motion_dir;
  tgt_val->dia_mode = receive_req->nmr.dia_mode;

  tgt_val->tgt_in.accl_param.limit = 5500;
  tgt_val->tgt_in.accl_param.n = 4;

  tgt_val->tgt_in.slip_gain_K1 = param->slip_param_K;
  tgt_val->tgt_in.slip_gain_K2 = param->slip_param_k2;
  if (receive_req->nmr.motion_type == MotionType::SLALOM) {
    tgt_val->ego_in.v = receive_req->nmr.v_max;
  }

  if (receive_req->nmr.motion_type == MotionType::STRAIGHT) {
    // if (se->sen.r45.sensor_dist == 0 || se->sen.r45.sensor_dist == 180) {
    //   se->sen.r45.sensor_dist = param->sen_ref_p.normal.ref.right45;
    //   se->sen.r45.global_run_dist =
    //       tgt_val->global_pos.dist - param->wall_off_hold_dist;
    // }
    // if (se->sen.l45.sensor_dist == 0 || se->sen.l45.sensor_dist == 180) {
    //   se->sen.l45.sensor_dist = param->sen_ref_p.normal.ref.left45;
    //   se->sen.l45.global_run_dist =
    //       tgt_val->global_pos.dist - param->wall_off_hold_dist;
    // }
  }

  if (receive_req->nmr.motion_type == MotionType::STRAIGHT ||
      receive_req->nmr.motion_type == MotionType::SLA_FRONT_STR) {
    ego.kim.x = ego.kim.y = 0;
    tgt_val->ego_in.ideal_px = tgt_val->ego_in.ideal_py =
        tgt_val->ego_in.img_ang = 0;
  } else if (receive_req->nmr.motion_type == MotionType::WALL_OFF ||
             receive_req->nmr.motion_type == MotionType::WALL_OFF_DIA) {
  }
  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    se->sen.r45.sensor_dist = se->ego.right45_dist;
    se->sen.l45.sensor_dist = se->ego.left45_dist;
  }
  if (search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
    se->sen.r45.sensor_dist = se->ego.right45_dist;
    se->sen.l45.sensor_dist = se->ego.left45_dist;
  }
}

void PlanningTask::motor_enable() {
  // 2回目呼び出し（motor_en=true 中）は generate() と競合するためスキップ
  if (!motor_en) {
    trj_.setup();
  }

  ego.kf_dist.reset(0);
  ego.kf_ang.reset(0);
  ego.kf_v.reset(0);
  ego.kf_w.reset(0);
  ego.kf_w2.reset(0);
  ego.kf_v_l.reset(0);
  ego.kf_v_r.reset(0);
  motor_en = true;
  motor_.motor_enable();
}
void PlanningTask::motor_disable() { // IDLE コマンドでモーター停止
  motor_en = false;
  motor_.motor_disable();
}
void PlanningTask::suction_enable(float duty, float duty_low) {
  suction_en = true;
  ctl_.set_suction_target(duty, duty_low);
  bldc_.enable();
}
void PlanningTask::suction_disable() {
  suction_en = false;
  bldc_.disable();
}

void PlanningTask::wait_tick() {
  sem_acquire_blocking(&tick_sem_);
}

bool PlanningTask::try_wait_tick_ms(uint32_t ms) {
  return sem_acquire_timeout_ms(&tick_sem_, ms);
}
