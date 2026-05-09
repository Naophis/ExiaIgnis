#include "planning/planning_task.hpp"
#include "define.hpp" // M_PWM_L1/L2/R1/R2, SUCTION_PWM, MOTOR_PWM_FREQ_HZ
#include "hardware/irq.h"
#include "hardware/sync.h"  // __dmb
#include "hardware/timer.h" // timer1_hw, TIMER1_IRQ_0
#include "logging/logging_task.hpp"
#include "pico/stdlib.h"
#include "sensing_task.hpp"
#include <algorithm>
#include <cmath>

// ============================================================
// 機体パラメータ (要チューニング)
// ============================================================
static constexpr float TIRE_DIAM_MM = 12.0f;      // 車輪直径 [mm]
static constexpr float GEAR_RATIO = 37.0f / 8.0f; // モーター:車輪 ギア比
static constexpr float TREAD_MM = 38.0f;          // 左右車輪間距離 [mm]
static constexpr float ENC_STEPS = 16384.0f; // 14-bit エンコーダ分解能
static constexpr float MM_PER_STEP =
    (float)(M_PI)*TIRE_DIAM_MM / (ENC_STEPS * GEAR_RATIO);

// ============================================================
// PID ゲイン (初期値: 実機で要チューニング)
// ============================================================
static constexpr float VEL_KP = 0.5f;
static constexpr float VEL_KI = 0.1f;
static constexpr float GYRO_KP = 0.3f;
static constexpr float GYRO_KI = 0.05f;
static constexpr float DUTY_MAX = 90.0f; // デューティ上限 [%]

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
  motor_.init();
  sensor_.init(sensing_result, param);
  ctl_.init(&motor_, &sensor_, &trj_, &ego);
  ego.init(sensing_result, param);
}

// ============================================================
// TIMER1 IRQ 登録 (Core1 エントリから呼ぶ)
// ============================================================
void PlanningTask::start_irq() {
  irq_set_exclusive_handler(TIMER1_IRQ_0, timer_irq_handler);
  irq_set_priority(TIMER1_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(TIMER1_IRQ_0, true);
  hw_set_bits(&timer1_hw->inte, 1u << 0);

  next_alarm_ = timer1_hw->timelr + interval_us_;
  timer1_hw->alarm[0] = next_alarm_;
}

void PlanningTask::send_command(std::shared_ptr<motion_tgt_val_t> tgt) {
  pending_tgt_ = tgt;
  __dmb();
  tgt_cmd_pending_ = true;
}

// ============================================================
// TIMER1 alarm 0 IRQ ハンドラ (Core0)
// ============================================================
void PlanningTask::timer_irq_handler() {
  timer1_hw->intr = 1u << 0;

  auto *self = s_instance.get();

  // 次回アラームを絶対時刻で設定 (ドリフトなし)
  // flash write 等で Core1 が長時間停止した場合にリセット (sensing_task と同様)
  self->next_alarm_ += self->interval_us_;
  {
    uint32_t now32 = timer1_hw->timelr;
    if ((int32_t)(now32 - self->next_alarm_) > (int32_t)self->interval_us_)
      self->next_alarm_ = now32 + self->interval_us_;
  }
  timer1_hw->alarm[0] = self->next_alarm_;

  uint64_t now = time_us_64();
  uint32_t dt_us = self->prev_ts_ ? (uint32_t)(now - self->prev_ts_) : 0;
  self->prev_ts_ = now;

  self->tick(dt_us);
}

// ============================================================
// 1tick 処理 (IRQ ハンドラから呼ばれる)
// ============================================================
void PlanningTask::tick(uint32_t dt_us) {
  if (dt_us == 0)
    return;

  // --- コマンド受け取り (Core1 からの cross-core 書き込みを安全に読む) ---
  if (cmd_pending_) {
    __dmb(); // load barrier: cmd_pending_ を読んだ後、pending_cmd_
             // フィールドを確実に読む
    active_cmd_ = Command(pending_cmd_);
    cmd_pending_ = false;
    // 新コマンド受信: 積分項・軌道をリセット
    vel_err_i_ = 0.0f;
    gyro_err_i_ = 0.0f;
    img_v_ = 0.0f;
    img_w_ = 0.0f;
    img_dist_ = 0.0f;
    img_ang_ = 0.0f;
  }

  // --- motion_tgt_val_t コマンド受け取り (send_command 経由) ---
  if (tgt_cmd_pending_) {
    __dmb(); // pending_tgt_ の書き込みが見えることを保証
    active_tgt_ = pending_tgt_;
    receive_req = active_tgt_.get();
    tgt_cmd_pending_ = false;
    cp_request();
  }

  // --- 初回 tick: エンコーダ初期値を記録してスキップ ---
  // if (first_tick_) {
  //   enc_r_prev_ = d.enc_r;
  //   enc_l_prev_ = d.enc_l;
  //   first_tick_ = false;
  //   return;
  // }

  float dt = dt_us * 1e-6f;

  {
    ego.update(tgt_val, motor_en); // 30 usec
    sensor_.calc_dist(tgt_val);    // 15 ~ 20 usec

    if (trj_.first_req) {
      if (search_mode && tgt_val->motion_type == MotionType::SLALOM) {
        tgt_val->tgt_in.enable_slip_decel = 1;
      } else {
        tgt_val->tgt_in.enable_slip_decel = 0;
      }
      trj_.generate(tgt_val, param, last_tgt_angle);
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
    trj_.calc_kanayama(tgt_val, sensing_result, param, ego, sensor_,
                       last_tgt_angle);

    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::FRONT_CTRL) {
      trj_.v_cmd = tgt_val->ego_in.v;
      trj_.w_cmd = tgt_val->ego_in.w;
    }

    // 算出結果をコピー
    trj_.copy_tgt(tgt_val, sensing_result, param, dt); // 1~2usec

    // PID制御・デューティ計算・モーター出力
    ctl_.calc(tgt_val, sensing_result, param, motor_en, suction_en, search_mode,
              w_reset, last_tgt_angle, dt);
  }

  // --- ログ記録 (LoggingTask が active な場合のみ実行) ---
  {
    PlanningTask::State snap(state); // volatile → non-volatile コピー
    // LoggingTask::append_from_irq(d, snap);
  }
}

// ============================================================
// 軌道生成 (台形速度プロファイル)
// ============================================================
void PlanningTask::update_trajectory(float dt) {
  const Command &cmd = active_cmd_;

  switch (cmd.mode) {
  case MotionMode::IDLE:
    img_v_ = 0.0f;
    img_w_ = 0.0f;
    break;

  case MotionMode::STOP: {
    float dv = (cmd.decel > 0.0f ? cmd.decel : 3000.0f) * dt;
    if (img_v_ > 0.0f) {
      img_v_ = std::max(0.0f, img_v_ - dv);
    } else if (img_v_ < 0.0f) {
      img_v_ = std::min(0.0f, img_v_ + dv);
    }
    img_w_ = 0.0f;
    if (img_v_ == 0.0f) {
      active_cmd_.mode = MotionMode::IDLE;
    }
    break;
  }

  case MotionMode::STRAIGHT: {
    float remaining = cmd.dist - img_dist_;
    if (remaining <= 0.0f) {
      img_v_ = cmd.v_end;
      active_cmd_.mode = MotionMode::STOP;
      break;
    }
    float decel = cmd.decel > 0.0f ? cmd.decel : cmd.accl;
    float decel_d = std::max(0.0f, (img_v_ * img_v_ - cmd.v_end * cmd.v_end) /
                                       (2.0f * decel));

    if (remaining <= decel_d) {
      img_v_ -= decel * dt;
      if (img_v_ < cmd.v_end)
        img_v_ = cmd.v_end;
    } else {
      if (img_v_ < cmd.v_max) {
        img_v_ = std::min(cmd.v_max, img_v_ + cmd.accl * dt);
      }
    }
    img_dist_ += img_v_ * dt;
    img_w_ = 0.0f;
    break;
  }

  case MotionMode::PIVOT: {
    float sign = cmd.ang >= 0.0f ? 1.0f : -1.0f;
    float remaining = std::fabs(cmd.ang) - std::fabs(img_ang_);
    if (remaining <= 0.0f) {
      img_w_ = 0.0f;
      active_cmd_.mode = MotionMode::IDLE;
      break;
    }
    float alpha = cmd.alpha > 0.0f ? cmd.alpha : 30.0f;
    float decel_a = (img_w_ * img_w_) / (2.0f * alpha);
    if (remaining <= decel_a) {
      img_w_ -= sign * alpha * dt;
      if ((sign > 0.0f && img_w_ < 0.0f) || (sign < 0.0f && img_w_ > 0.0f)) {
        img_w_ = 0.0f;
      }
    } else {
      if (std::fabs(img_w_) < cmd.w_max) {
        img_w_ += sign * alpha * dt;
        if (std::fabs(img_w_) > cmd.w_max)
          img_w_ = sign * cmd.w_max;
      }
    }
    img_ang_ += img_w_ * dt;
    img_v_ = 0.0f;
    break;
  }
  }
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
    ctl_.pl_req_activate(*receive_req);
    pid_req_timestamp = receive_req->pl_req.time_stamp;
  }
}

void PlanningTask::cp_request() {
  // tgt_val->tgt_in.mass = param->Mass;

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

  tgt_val->dia_state.left_save = receive_req->dia_state.right_save = false;
  tgt_val->dia_state.left_old = receive_req->dia_state.right_old = 0;

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
    kf_ang.reset(0);
    tgt_val->ego_in.img_dist = tgt_val->ego_in.dist = 0;
    kf_dist.reset(0);
  } else {
    tgt_val->ego_in.ang -= last_tgt_angle;
    kim.theta -= last_tgt_angle;
    tgt_val->ego_in.img_ang = 0;
    kf_ang.offset(-last_tgt_angle);
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
    kf_ang.offset(-last_tgt_angle);
    tgt_val->ego_in.ang -= last_tgt_angle;
    // } else {
    //   kf_ang.reset(0);
  }
  if (tgt_val->tgt_in.tgt_dist != 0) {
    const auto tmp_dist = tgt_val->ego_in.dist;
    tgt_val->ego_in.img_dist -= tmp_dist;
    kf_dist.offset(-tmp_dist);
    tgt_val->ego_in.dist = 0;
    // } else {
    //   kf_dist.reset(0);
  }

  sensing_result->ego.dist_kf = kf_dist.get_state();
  // sensing_result->ego.ang_kf = kf_ang.get_state();

  if (param->enable_kalman_gyro == 1) {
    se->ego.ang_kf = kf_ang.get_state();
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
    kim.x = kim.y = 0;
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