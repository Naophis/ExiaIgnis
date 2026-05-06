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
  ctl_.init(&motor_, &sensor_, &trj_, &ego);
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

// ============================================================
// コマンド投入 (Core0 main ループから呼ぶ, IRQ-safe)
// ============================================================
// Core1 (MainTask) から Core0 (Planning IRQ) へ cross-core 安全に投入する。
// __dmb() で全フィールドの書き込みが cmd_pending_
// より前に完了することを保証する。 void PlanningTask::send_command(const
// Command &cmd) {
//     pending_cmd_.mode          = cmd.mode;
//     pending_cmd_.v_max         = cmd.v_max;
//     pending_cmd_.v_end         = cmd.v_end;
//     pending_cmd_.accl          = cmd.accl;
//     pending_cmd_.decel         = cmd.decel;
//     pending_cmd_.dist          = cmd.dist;
//     pending_cmd_.w_max         = cmd.w_max;
//     pending_cmd_.alpha         = cmd.alpha;
//     pending_cmd_.ang           = cmd.ang;
//     pending_cmd_.duty_suction  = cmd.duty_suction;
//     pending_cmd_.timestamp     = cmd.timestamp;
//     __dmb();
//     cmd_pending_ = true;
// }

// Astraea 互換: motion_tgt_val_t ベースのコマンド。
// Astraea の xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite)
// に相当。 IRQ が次の tick で cp_request() 相当の処理を実行する。
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
  self->next_alarm_ += self->interval_us_;
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

  // --- センシングデータをスナップショット ---
  SensingTask::Data d = sensing_->data;

  // --- 初回 tick: エンコーダ初期値を記録してスキップ ---
  if (first_tick_) {
    enc_r_prev_ = d.enc_r;
    enc_l_prev_ = d.enc_l;
    first_tick_ = false;
    return;
  }

  float dt = dt_us * 1e-6f;

  {
    ego.update(tgt_val, sensing_result, param, motor_en); // 30 usec
    sensor_.calc_dist(tgt_val, sensing_result, param);    // 15 ~ 20 usec

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
    LoggingTask::append_from_irq(d, snap);
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
