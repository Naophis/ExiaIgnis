#include "planning/planning_task.hpp"
#include "define.hpp" // M_PWM_L1/L2/R1/R2, SUCTION_PWM, MOTOR_PWM_FREQ_HZ
#include "hardware/clocks.h" // clock_get_hz
#include "hardware/irq.h"
#include "hardware/pwm.h"
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
}

// ============================================================
// 初期化 (Core0 の main から呼ぶ)
// ============================================================
void PlanningTask::init(std::shared_ptr<SensingTask> sensing) {
  sensing_ = sensing;

  // --- モーター / 吸引 PWM GPIO 設定 ---
  const uint motor_pins[] = {M_PWM_L1, M_PWM_L2, M_PWM_R1, M_PWM_R2,
                             SUCTION_PWM};
  for (uint pin : motor_pins) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
  }

  slice_L_ = pwm_gpio_to_slice_num(M_PWM_L1);
  slice_R_ = pwm_gpio_to_slice_num(M_PWM_R1);
  slice_S_ = pwm_gpio_to_slice_num(SUCTION_PWM);

  // wrap = sys_clk / pwm_freq - 1  (クロック変更後に呼ぶこと)
  motor_wrap_ = (uint32_t)(clock_get_hz(clk_sys) / MOTOR_PWM_FREQ_HZ) - 1u;

  for (uint slice : {slice_L_, slice_R_, slice_S_}) {
    pwm_set_clkdiv_int_frac4(slice, 1, 0); // 分周なし
    pwm_set_wrap(slice, motor_wrap_);
    pwm_set_enabled(slice, true);
  }

  // 初期デューティ: 全チャンネル 0
  apply_motor();
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

    update_ego_motion();    // 30 usec
    calc_sensor_dist_all(); // 15 ~ 20 usec
    // recv_notify();
    // 物理量ベース計算
    float axel_degenerate_gain = 1;
    diff_old = diff;
    if (!search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
      // if (tgt_val->motion_type == MotionType::STRAIGHT) {
      if (axel_degenerate_x.size() > 0 &&
          tgt_val->nmr.sct == SensorCtrlType::Straight) {
        // if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
        SensingControlType type = SensingControlType::None;
        diff = ABS(check_sen_error(type));
        // } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
        //   diff = ABS(check_sen_error_dia());
        // }
        if (diff == 0) {
          diff = diff_old;
        }
        axel_degenerate_gain =
            interp1d(axel_degenerate_x, axel_degenerate_y, diff, false);
        tgt_val->tgt_in.axel_degenerate_gain =
            (1 - param->sensor_gain.front2.b) *
                tgt_val->tgt_in.axel_degenerate_gain +
            param->sensor_gain.front2.b * axel_degenerate_gain;
      } else if (axel_degenerate_dia_x.size() > 0 &&
                 tgt_val->nmr.sct == SensorCtrlType::Dia) {
        SensingControlType type = SensingControlType::None;
        diff = ABS(check_sen_error_dia(type));
        if (diff == 0) {
          diff = diff_old;
        }
        axel_degenerate_gain =
            interp1d(axel_degenerate_dia_x, axel_degenerate_dia_y, diff, false);
        if (axel_degenerate_gain < 0 &&
            tgt_val->tgt_in.end_v > tgt_val->ego_in.v) {
          tgt_val->tgt_in.axel_degenerate_gain = 0.01f;
        }
        tgt_val->tgt_in.axel_degenerate_gain =
            (1 - param->sensor_gain.front2.b) *
                tgt_val->tgt_in.axel_degenerate_gain +
            param->sensor_gain.front2.b * axel_degenerate_gain;
      }
    } else {
      diff = diff_old = 0;
      tgt_val->tgt_in.axel_degenerate_gain = axel_degenerate_gain;
    }

    // start_calc_mpc = esp_timer_get_time();
    if (first_req) {

      if (search_mode && tgt_val->motion_type == MotionType::SLALOM) {
        tgt_val->tgt_in.enable_slip_decel = 1;
      } else {
        tgt_val->tgt_in.enable_slip_decel = 0;
      }
      // auto time = esp_timer_get_time();
      // Generate trajectory points
      generate_trajectory();
      // auto time2 = esp_timer_get_time();

      // printf("mpc calc time: %lld usec\n", time2 - time);
    }
    // end_calc_mpc = esp_timer_get_time();
    // check 3

    if (tgt_val->motion_type == MotionType::STRAIGHT ||
        tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val->motion_type == MotionType::SLA_BACK_STR) {
      if (tgt_val->ego_in.img_dist >= tgt_val->tgt_in.tgt_dist) {
        mpc_next_ego.v = tgt_val->tgt_in.end_v;
      }
      if (tgt_val->ego_in.dist >= tgt_val->tgt_in.tgt_dist) {
        mpc_next_ego.v = tgt_val->tgt_in.end_v;
      }
    }
    calc_kanamaya_ctrl();

    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::FRONT_CTRL) {
      v_cmd = tgt_val->ego_in.v;
      w_cmd = tgt_val->ego_in.w;
    }

    // 算出結果をコピー
    cp_tgt_val(); // 1~2usec

    // Duty計算
    calc_tgt_duty(); // 15 ~ 20 usec

    check_fail_safe(); // 7 ~ 9 usec

    // chekc 4
    // 22 ~ 25 usec
    set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);

    // buzzer(buzzer_ch, buzzer_timer);
    // global_msec_timer++;

    // end = esp_timer_get_time();
    // tgt_val->calc_time = (int16_t)(end - start);
    // vTaskDelay(xDelay);
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

// ============================================================
// PID 制御量計算
// ============================================================
void PlanningTask::update_control(float dt) {
  if (active_cmd_.mode == MotionMode::IDLE) {
    duty_l_ = 0.0f;
    duty_r_ = 0.0f;
    vel_err_i_ = 0.0f;
    gyro_err_i_ = 0.0f;
    return;
  }

  float vel_err = img_v_ - v_est_;
  vel_err_i_ += vel_err * dt;
  float duty_c = VEL_KP * vel_err + VEL_KI * vel_err_i_;

  float gyro_err = img_w_ - w_est_;
  gyro_err_i_ += gyro_err * dt;
  float duty_roll = GYRO_KP * gyro_err + GYRO_KI * gyro_err_i_;

  duty_r_ = std::clamp(duty_c + duty_roll, -DUTY_MAX, DUTY_MAX);
  duty_l_ = std::clamp(duty_c - duty_roll, -DUTY_MAX, DUTY_MAX);
}

// ============================================================
// モーター / 吸引 PWM 出力
// ============================================================
void PlanningTask::apply_motor() {
  // 左右モーター: duty > 0 = 正転 (CHAN_A), duty < 0 = 逆転 (CHAN_B)
  auto set_drive = [&](uint slice, float duty) {
    uint16_t level = (uint16_t)((float)motor_wrap_ * std::fabs(duty) / 100.0f);
    if (duty >= 0.0f) {
      pwm_set_chan_level(slice, PWM_CHAN_A, level);
      pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else {
      pwm_set_chan_level(slice, PWM_CHAN_A, 0);
      pwm_set_chan_level(slice, PWM_CHAN_B, level);
    }
  };

  set_drive(slice_L_, duty_l_);
  set_drive(slice_R_, duty_r_);

  // 吸引モーター: 単方向 (CHAN_A のみ)
  uint16_t suction_level =
      (uint16_t)((float)motor_wrap_ * std::clamp(duty_suction_, 0.0f, 100.0f) /
                 100.0f);
  pwm_set_chan_level(slice_S_, PWM_CHAN_A, suction_level);
  pwm_set_chan_level(slice_S_, PWM_CHAN_B, 0);
}

std::shared_ptr<sensing_result_entity_t> PlanningTask::get_sensing_entity() {
  return sensing_result;
}

void PlanningTask::update_ego_motion() {
  const auto se = get_sensing_entity();
  const float dt = param->dt;
  const float tire = param->tire;
  tgt_val->ego_in.ff_duty_low_th = param->ff_front_dury;
  tgt_val->ego_in.ff_duty_low_v_th = param->ff_v_th;
  if (!motor_en) {
    tgt_val->ego_in.v = 0;
    tgt_val->ego_in.w = 0;
  }

  se->ego.w_lp = se->ego.w_lp * (1 - param->gyro_param.lp_delay) +
                 se->ego.w_raw * param->gyro_param.lp_delay;
  se->ego.w_lp2 = se->ego.w_lp2 * (1 - param->gyro_param.lp_delay) +
                  se->ego.w_raw2 * param->gyro_param.lp_delay;

  if (std::isfinite(tgt_val->ego_in.accl) && std::isfinite(se->ego.v_c)) {
    auto tmp_v_l = kf_v_l.get_state();
    auto tmp_v_r = kf_v_r.get_state();
    kf_v.predict(tgt_val->ego_in.accl);
    kf_v.update((tmp_v_l + tmp_v_r) / 2);
    se->ego.v_kf = kf_v.get_state();

    if ((tgt_val->motion_type == MotionType::NONE ||
         tgt_val->motion_type == MotionType::READY)) {
      se->ego.v_kf = 0;
    }
  }

  if (std::isfinite(tgt_val->ego_in.v)) {
    kf_dist.predict(tgt_val->ego_in.v);
    kf_dist.update(tgt_val->ego_in.dist);
    se->ego.dist_kf = kf_dist.get_state();
  }
  if (std::isfinite(tgt_val->ego_in.w)) {
    kf_ang.predict(tgt_val->ego_in.w);
    kf_ang.update(tgt_val->ego_in.ang);
    kf_ang2.predict(tgt_val->ego_in.w);
    kf_ang2.update(tgt_val->ego_in.ang);

    if (param->enable_kalman_gyro == 1) {
      se->ego.ang_kf = kf_ang.get_state();
      se->ego.ang_kf2 = kf_ang2.get_state();
    } else if (param->enable_kalman_gyro == 2) {
      se->ego.ang_kf = tgt_val->ego_in.ang;
      se->ego.ang_kf2 = tgt_val->ego_in.ang;
    } else {
      se->ego.ang_kf = tgt_val->ego_in.ang;
      se->ego.ang_kf2 = tgt_val->ego_in.ang;
    }
  }

  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::FRONT_CTRL ||
        tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::PIVOT_PRE ||
        tgt_val->motion_type == MotionType::PIVOT_PRE2 ||
        tgt_val->motion_type == MotionType::PIVOT_AFTER ||
        tgt_val->motion_type == MotionType::PIVOT_OFFSET ||
        tgt_val->motion_type == MotionType::BACK_STRAIGHT ||
        tgt_val->motion_type == MotionType::READY)) {
    if (std::isfinite(se->ego.v_kf) && std::isfinite(se->ego.ang_kf)) {

      const auto pos_state_z = pos.get_state();
      const auto pos_x_z = pos_state_z[0];
      const auto pos_y_z = pos_state_z[1];
      const auto pos_theta_z = pos_state_z[2];

      pos.ang += se->ego.w_kf * dt;
      // pos.ang = fmod(pos.ang + M_PI, 2 * M_PI) - M_PI;
      const auto tmp_dist = se->ego.v_kf * dt;
      tgt_val->ego_in.pos_x += tmp_dist * cosf(pos.ang);
      tgt_val->ego_in.pos_y += tmp_dist * sinf(pos.ang);
      pos.predict(tgt_val->ego_in.v, tgt_val->ego_in.w, dt);
      const std::array<float, 3> z = {tgt_val->ego_in.pos_x, //
                                      tgt_val->ego_in.pos_y, //
                                      pos.ang};
      pos.update(z);
      const auto pos_state = pos.get_state();
      se->ego.pos_x = pos_state[0];
      se->ego.pos_y = pos_state[1];
      se->ego.pos_ang = pos_state[2];

      const auto d_x = se->ego.pos_x - pos_x_z;
      const auto d_y = se->ego.pos_y - pos_y_z;
      const auto d_ang = se->ego.pos_ang - pos_theta_z;

      // calc local_pos;
      kim.x += d_x;
      kim.y += d_y;
      kim.theta += d_ang;

      sensing_result->ang_kf_sum += d_ang;
      sensing_result->img_ang_sum +=
          tgt_val->ego_in.img_ang - sensing_result->img_ang_z;
    }
  }

  se->ego.battery_raw = se->battery.data;

  se->ego.battery_lp =
      se->ego.battery_lp * (1 - param->battery_param.lp_delay) +
      se->ego.battery_raw * param->battery_param.lp_delay;

  kf_batt.predict(0);
  kf_batt.update(se->ego.battery_raw);
  se->ego.batt_kf = kf_batt.get_state();

  se->ego.left45_lp_old = se->ego.left45_lp;
  se->ego.left90_lp_old = se->ego.left90_lp;
  se->ego.front_lp_old = se->ego.front_lp;
  se->ego.right45_lp_old = se->ego.right45_lp;
  se->ego.right90_lp_old = se->ego.right90_lp;

  // sensing_result->ego.right45_2_lp_old = sensing_result->ego.right45_2_lp;
  // sensing_result->ego.left45_2_lp_old = sensing_result->ego.left45_2_lp;

  se->ego.right90_raw = se->led_sen.right90.raw;
  se->ego.right90_lp = se->ego.right90_lp * (1 - param->led_param.lp_delay) +
                       se->ego.right90_raw * param->led_param.lp_delay;

  se->ego.right45_raw = se->led_sen.right45.raw;
  se->ego.right45_2_raw = se->led_sen.right45_2.raw;
  se->ego.right45_3_raw = se->led_sen.right45_3.raw;

  se->ego.right45_lp = se->ego.right45_lp * (1 - param->led_param.lp_delay) +
                       se->ego.right45_raw * param->led_param.lp_delay;
  se->ego.right45_2_lp =
      se->ego.right45_2_lp * (1 - param->led_param.lp_delay) +
      se->ego.right45_2_raw * param->led_param.lp_delay;
  se->ego.right45_3_lp =
      se->ego.right45_3_lp * (1 - param->led_param.lp_delay) +
      se->ego.right45_3_raw * param->led_param.lp_delay;

  se->ego.front_raw = se->led_sen.front.raw;
  se->ego.front_lp = se->ego.front_lp * (1 - param->led_param.lp_delay) +
                     se->ego.front_raw * param->led_param.lp_delay;

  se->ego.left45_raw = se->led_sen.left45.raw;
  se->ego.left45_2_raw = se->led_sen.left45_2.raw;
  se->ego.left45_3_raw = se->led_sen.left45_3.raw;

  se->ego.left45_lp = se->ego.left45_lp * (1 - param->led_param.lp_delay) +
                      se->ego.left45_raw * param->led_param.lp_delay;
  se->ego.left45_2_lp = se->ego.left45_2_lp * (1 - param->led_param.lp_delay) +
                        se->ego.left45_2_raw * param->led_param.lp_delay;
  se->ego.left45_3_lp = se->ego.left45_3_lp * (1 - param->led_param.lp_delay) +
                        se->ego.left45_3_raw * param->led_param.lp_delay;

  se->ego.left90_raw = se->led_sen.left90.raw;
  se->ego.left90_lp = se->ego.left90_lp * (1 - param->led_param.lp_delay) +
                      se->ego.left90_raw * param->led_param.lp_delay;

  // sensing_result->ego.right45_2_raw = sensing_result->led_sen.right45_2.raw;
  // sensing_result->ego.right45_2_lp =
  //     sensing_result->ego.right45_2_lp * (1 - param->led_param.lp_delay) +
  //     sensing_result->ego.right45_2_raw * param->led_param.lp_delay;

  // sensing_result->ego.left45_2_raw = sensing_result->led_sen.left45_2.raw;
  // sensing_result->ego.left45_2_lp =
  //     sensing_result->ego.left45_2_lp * (1 - param->led_param.lp_delay) +
  //     sensing_result->ego.left45_2_raw * param->led_param.lp_delay;
  // コピー
  tgt_val->ego_in.slip_point.w = se->ego.w_lp;
}

void PlanningTask::calc_sensor_dist_all() {
  const auto se = get_sensing_entity();
  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::PIVOT)) {
    se->ego.left90_dist_old = se->ego.left90_dist;
    se->ego.left45_dist_old = se->ego.left45_dist;
    se->ego.left45_2_dist_old = se->ego.left45_2_dist;
    se->ego.left45_3_dist_old = se->ego.left45_3_dist;
    se->ego.front_dist_old = se->ego.front_dist;
    se->ego.right45_dist_old = se->ego.right45_dist;
    se->ego.right45_2_dist_old = se->ego.right45_2_dist;
    se->ego.right45_3_dist_old = se->ego.right45_3_dist;
    se->ego.right90_dist_old = se->ego.right90_dist;

    se->ego.left90_dist = calc_sensor(
        se->ego.left90_lp, param->sensor_gain.l90.a, param->sensor_gain.l90.b);
    se->ego.left45_dist = calc_sensor(
        se->ego.left45_lp, param->sensor_gain.l45.a, param->sensor_gain.l45.b);
    se->ego.left45_2_dist =
        calc_sensor(se->ego.left45_2_lp, param->sensor_gain.l45_2.a,
                    param->sensor_gain.l45_2.b);
    se->ego.left45_3_dist =
        calc_sensor(se->ego.left45_3_lp, param->sensor_gain.l45_3.a,
                    param->sensor_gain.l45_3.b);
    se->ego.right45_dist = calc_sensor(
        se->ego.right45_lp, param->sensor_gain.r45.a, param->sensor_gain.r45.b);
    se->ego.right45_2_dist =
        calc_sensor(se->ego.right45_2_lp, param->sensor_gain.r45_2.a,
                    param->sensor_gain.r45_2.b);
    se->ego.right45_3_dist =
        calc_sensor(se->ego.right45_3_lp, param->sensor_gain.r45_3.a,
                    param->sensor_gain.r45_3.b);
    se->ego.right90_dist = calc_sensor(
        se->ego.right90_lp, param->sensor_gain.r90.a, param->sensor_gain.r90.b);

    se->ego.left90_far_dist =
        calc_sensor(se->ego.left90_lp, param->sensor_gain.l90_far.a,
                    param->sensor_gain.l90_far.b);
    se->ego.right90_far_dist =
        calc_sensor(se->ego.right90_lp, param->sensor_gain.r90_far.a,
                    param->sensor_gain.r90_far.b);

    se->ego.left90_mid_dist =
        calc_sensor(se->ego.left90_lp, param->sensor_gain.l90_mid.a,
                    param->sensor_gain.l90_mid.b);
    se->ego.right90_mid_dist =
        calc_sensor(se->ego.right90_lp, param->sensor_gain.r90_mid.a,
                    param->sensor_gain.r90_mid.b);

    if (se->ego.left90_dist < param->sensor_range_far_max &&
        se->ego.right90_dist < param->sensor_range_far_max) {
      se->ego.front_dist = (se->ego.left90_dist + se->ego.right90_dist) / 2;
    } else if (se->ego.left90_dist > param->sensor_range_far_max &&
               se->ego.right90_dist < param->sensor_range_far_max) {
      se->ego.front_dist = se->ego.right90_dist;
    } else if (se->ego.left90_dist < param->sensor_range_far_max &&
               se->ego.right90_dist > param->sensor_range_far_max) {
      se->ego.front_dist = se->ego.left90_dist;
    } else {
      se->ego.front_dist = param->sensor_range_max;
    }

    if (se->ego.left90_far_dist < param->sensor_range_far_max &&
        se->ego.right90_far_dist < param->sensor_range_far_max) {
      se->ego.front_far_dist =
          (se->ego.left90_far_dist + se->ego.right90_far_dist) / 2;
    } else if (se->ego.left90_far_dist > param->sensor_range_far_max &&
               se->ego.right90_far_dist < param->sensor_range_far_max) {
      se->ego.front_far_dist = se->ego.right90_far_dist;
    } else if (se->ego.left90_far_dist < param->sensor_range_far_max &&
               se->ego.right90_far_dist > param->sensor_range_far_max) {
      se->ego.front_far_dist = se->ego.left90_far_dist;
    } else {
      se->ego.front_far_dist = param->sensor_range_max;
    }
    if (se->ego.left90_mid_dist < param->sensor_range_far_max &&
        se->ego.right90_mid_dist < param->sensor_range_far_max) {
      se->ego.front_mid_dist =
          (se->ego.left90_mid_dist + se->ego.right90_mid_dist) / 2;
    } else if (se->ego.left90_mid_dist > param->sensor_range_far_max &&
               se->ego.right90_mid_dist < param->sensor_range_far_max) {
      se->ego.front_mid_dist = se->ego.right90_mid_dist;
    } else if (se->ego.left90_mid_dist < param->sensor_range_far_max &&
               se->ego.right90_mid_dist > param->sensor_range_far_max) {
      se->ego.front_mid_dist = se->ego.left90_mid_dist;
    } else {
      se->ego.front_mid_dist = param->sensor_range_max;
    }
  } else {
    se->ego.left90_dist          //
        = se->ego.left45_dist    //
        = se->ego.left45_2_dist  //
        = se->ego.left45_3_dist  //
        = se->ego.front_dist     //
        = se->ego.right45_dist   //
        = se->ego.right45_2_dist //
        = se->ego.right45_3_dist //
        = se->ego.right90_dist = param->sensor_range_max;

    se->ego.left45_dist_diff          //
        = se->ego.right45_dist_diff   //
        = se->ego.right45_2_dist_diff //
        = se->ego.right45_3_dist_diff //
        = se->ego.left45_2_dist_diff  //
        = se->ego.left45_3_dist_diff  //
        = se->ego.right90_dist_diff   //
        = se->ego.left90_dist_diff = 0;
  }

  se->ego.left45_dist_diff = se->ego.left45_dist - se->ego.left45_dist_old;
  se->ego.left45_2_dist_diff =
      se->ego.left45_2_dist - se->ego.left45_2_dist_old;
  se->ego.left45_3_dist_diff =
      se->ego.left45_3_dist - se->ego.left45_3_dist_old;

  se->ego.right45_dist_diff = se->ego.right45_dist - se->ego.right45_dist_old;
  se->ego.right45_2_dist_diff =
      se->ego.right45_2_dist - se->ego.right45_2_dist_old;
  se->ego.right45_3_dist_diff =
      se->ego.right45_3_dist - se->ego.right45_3_dist_old;

  se->ego.left90_dist_diff = se->ego.left90_dist - se->ego.left90_dist_old;
  se->ego.right90_dist_diff = se->ego.right90_dist - se->ego.right90_dist_old;

  // 壁からの距離に変換。あとで斜め用に変更
  calc_sensor_dist_diff();
}

void PlanningTask::calc_sensor_dist_diff() {
  const auto se = get_sensing_entity();
  // l45
  if (se->sen.l45.sensor_dist > se->ego.left45_dist ||
      se->sen.l45.sensor_dist == 0) {
    se->sen.l45.sensor_dist = se->ego.left45_dist;
    se->sen.l45.global_run_dist = se->sen.l45_2.global_run_dist =
        se->sen.l45_3.global_run_dist = tgt_val->global_pos.dist;
    se->sen.l45.angle = tgt_val->ego_in.ang;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.l45.global_run_dist) >
         param->wall_off_hold_dist) &&
        se->ego.left45_dist < param->sen_ref_p.normal2.exist.left90) {
      se->sen.l45.sensor_dist = se->ego.left45_dist;
      se->sen.l45.angle = tgt_val->ego_in.ang;
    }
  }

  // r45
  if (se->sen.r45.sensor_dist > se->ego.right45_dist ||
      se->sen.r45.sensor_dist == 0) {
    se->sen.r45.sensor_dist = se->ego.right45_dist;
    se->sen.r45.global_run_dist = se->sen.r45_2.global_run_dist =
        se->sen.r45_3.global_run_dist = tgt_val->global_pos.dist;
    se->sen.r45.angle = tgt_val->ego_in.ang;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.r45.global_run_dist) >
         param->wall_off_hold_dist) &&
        se->ego.right45_dist < param->sen_ref_p.normal2.exist.right90) {
      se->sen.r45.sensor_dist = se->ego.right45_dist;
      se->sen.r45.angle = tgt_val->ego_in.ang;
    }
  }

  // l45_2
  if (se->sen.l45_2.sensor_dist > se->ego.left45_2_dist ||
      se->sen.l45_2.sensor_dist == 0) {
    se->sen.l45_2.sensor_dist = se->ego.left45_2_dist;
    se->sen.l45.global_run_dist = se->sen.l45_2.global_run_dist =
        se->sen.l45_3.global_run_dist = tgt_val->global_pos.dist;
    se->sen.l45_2.angle = tgt_val->ego_in.ang;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.l45_2.global_run_dist) >
         param->wall_off_hold_dist) &&
        se->ego.left45_dist < param->sen_ref_p.normal2.exist.left90) {
      se->sen.l45_2.sensor_dist = se->ego.left45_2_dist;
      se->sen.l45_2.angle = tgt_val->ego_in.ang;
    }
  }

  // r45_2
  if (se->sen.r45_2.sensor_dist > se->ego.right45_2_dist ||
      se->sen.r45_2.sensor_dist == 0) {
    se->sen.r45_2.sensor_dist = se->ego.right45_2_dist;
    se->sen.r45.global_run_dist = se->sen.r45_2.global_run_dist =
        se->sen.r45_3.global_run_dist = tgt_val->global_pos.dist;
    se->sen.r45_2.angle = tgt_val->ego_in.ang;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.r45_2.global_run_dist) >
         param->wall_off_hold_dist) &&
        se->ego.right45_dist < param->sen_ref_p.normal2.exist.right90) {
      se->sen.r45_2.sensor_dist = se->ego.right45_2_dist;
      se->sen.r45_2.angle = tgt_val->ego_in.ang;
    }
  }

  // l45_3
  if (se->sen.l45_3.sensor_dist > se->ego.left45_3_dist ||
      se->sen.l45_3.sensor_dist == 0) {
    se->sen.l45_3.sensor_dist = se->ego.left45_3_dist;
    se->sen.l45.global_run_dist = se->sen.l45_2.global_run_dist =
        se->sen.l45_3.global_run_dist = tgt_val->global_pos.dist;
    se->sen.l45_3.angle = tgt_val->ego_in.ang;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.l45_3.global_run_dist) >
         param->wall_off_hold_dist) &&
        se->ego.left45_dist < param->sen_ref_p.normal2.exist.left90) {
      se->sen.l45_3.sensor_dist = se->ego.left45_3_dist;
      se->sen.l45_3.angle = tgt_val->ego_in.ang;
    }
  }

  // r45_3
  if (se->sen.r45_3.sensor_dist > se->ego.right45_3_dist ||
      se->sen.r45_3.sensor_dist == 0) {
    se->sen.r45_3.sensor_dist = se->ego.right45_3_dist;
    se->sen.r45.global_run_dist = se->sen.r45_2.global_run_dist =
        se->sen.r45_3.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.r45_3.global_run_dist) >
         param->wall_off_hold_dist) &&
        se->ego.right45_dist < param->sen_ref_p.normal2.exist.right90) {
      se->sen.r45_3.sensor_dist = se->ego.right45_3_dist;
    }
  }
}
float PlanningTask::calc_sensor(float data, float a, float b) {
  int idx = (int)data;
  if (idx <= param->sensor_range_min || idx >= log_table.size()) {
    return param->sensor_range_max;
  }
  auto res = a / log_table.at(idx) - b;
  if (res < param->sensor_range_min || res > param->sensor_range_max) {
    return param->sensor_range_max;
  }
  if (!isfinite(res)) {
    return param->sensor_range_max;
  }
  return res;
}
float PlanningTask::interp1d(vector<float> &vx, vector<float> &vy, float x,
                             bool extrapolate) {
  int size = vx.size();
  int i = 0;
  if (x >= vx[size - 2]) {
    i = size - 2;
  } else {
    while (x > vx[i + 1])
      i++;
  }
  float xL = vx[i], yL = vy[i], xR = vx[i + 1], yR = vy[i + 1];
  if (!extrapolate) {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  float dydx = (yR - yL) / (xR - xL);

  return yL + dydx * (x - xL);
}

int PlanningTask::interp1d(vector<int> &vx, vector<int> &vy, float x,
                           bool extrapolate) {
  int size = vx.size();
  int i = 0;

  if (size == 0)
    return 0;

  if (x >= vx[size - 2]) {
    i = size - 2;
  } else {
    while (x > vx[i + 1])
      i++;
  }
  float xL = vx[i], yL = vy[i], xR = vx[i + 1], yR = vy[i + 1];
  if (!extrapolate) {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  float dydx = (yR - yL) / (xR - xL);

  return (int)(yL + dydx * (x - xL));
}

void PlanningTask::generate_trajectory() {
  // mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in,
  // tgt_val->motion_mode,
  //                   mpc_step, &mpc_next_ego, &dynamics);
  // tgt_val->ego_in.ideal_px = tgt_val->ego_in.ideal_py = 0;
  auto tmp = tgt_val->ego_in.img_ang;
  tgt_val->ego_in.img_ang += last_tgt_angle;

  // if (tgt_val->motion_type == MotionType::NONE && motion_req_timestamp > 10)
  // {
  //   return;
  // }

  if (param->trj_length <= 0) {
    return;
  }

  for (int i = 0; i < param->trj_length; i++) {
    int32_T index = i + 1;
    if (i == 0) {
      mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in,
                        tgt_val->motion_mode, mpc_step, &mpc_next_ego,
                        &dynamics, &index);
      trajectory_points[i] = mpc_next_ego;
      mpc_next_ego_prev = mpc_next_ego;
    } else {
      mpc_tgt_calc.step(&tgt_val->tgt_in, &mpc_next_ego_prev,
                        tgt_val->motion_mode, mpc_step, &mpc_next_ego2,
                        &dynamics, &index);
      trajectory_points[i] = mpc_next_ego2;
      mpc_next_ego_prev = mpc_next_ego2;
    }
  }
  mpc_next_ego.img_ang -= last_tgt_angle;
}

void PlanningTask::calc_kanamaya_ctrl() {

  if (trj_idx_v.size() == 0 || trj_idx_val.size() == 0)
    return;

  const auto idx_val =
      interp1d(trj_idx_v, trj_idx_val, tgt_val->ego_in.v, false);

  const int idx = std::min(param->trj_length - 1, idx_val);
  const auto se = get_sensing_entity();

  // ideal
  odm.x = trajectory_points[idx].ideal_px;
  odm.y = trajectory_points[idx].ideal_py;
  odm.theta = trajectory_points[idx].img_ang;

  float vd = odm.v = trajectory_points[idx].v;
  float wd = odm.w = trajectory_points[idx].w;

  float dx = odm.x - kim.x;
  float dy = odm.y - kim.y;
  if (tgt_val->ego_in.v < 10) {
    dx = dy = 0;
  }

  dx = std::clamp(dx, 0.0f, ABS(dx));

  float d_theta = odm.theta - (se->ego.ang_kf + last_tgt_angle);
  float e_theta = d_theta;
  // std::clamp(d_theta, (float)(-M_PI), (float)(M_PI));
  const float cos_theta = std::cos(se->ego.ang_kf);
  const float sin_theta = std::sin(se->ego.ang_kf);
  if (tgt_val->ego_in.v < 10) {
    e_theta = 0;
  }
  const float ex = cos_theta * dx + sin_theta * dy;
  const float ey = -sin_theta * dx + cos_theta * dy;

  const float cos_e_theta = std::cos(e_theta);
  const float sin_e_theta = std::sin(e_theta);

  const float kx = param->kanayama.kx;
  const float ky = param->kanayama.ky;
  const float k_theta = param->kanayama.k_theta;

  sensing_result->ego.knym_v = vd * cos_e_theta + kx * ex;
  sensing_result->ego.knym_w = wd + vd * (ky * ey + k_theta * sin_e_theta);
  sensing_result->ego.odm_x = odm.x;
  sensing_result->ego.odm_y = odm.y;
  sensing_result->ego.odm_theta = odm.theta;
  sensing_result->ego.kim_x = kim.x;
  sensing_result->ego.kim_y = kim.y;
  sensing_result->ego.kim_theta = kim.theta;

  if (param->kanayama.enable > 0 &&
      (tgt_val->motion_type == MotionType::SLALOM ||
       tgt_val->motion_type == MotionType::SLA_BACK_STR)) {
    v_cmd = sensing_result->ego.knym_v;
    w_cmd = sensing_result->ego.knym_w;
  } else {
    v_cmd = tgt_val->ego_in.v;
    w_cmd = tgt_val->ego_in.w;
  }
  if (tgt_val->motion_type != MotionType::SLALOM ||
      tgt_val->motion_type == MotionType::SLA_BACK_STR) {
    sensing_result->ego.knym_v = tgt_val->ego_in.v;
    sensing_result->ego.knym_w = tgt_val->ego_in.w;
  }
}

void PlanningTask::cp_tgt_val() {
  const auto se = get_sensing_entity();
  tgt_val->ego_in.accl = mpc_next_ego.accl;
  tgt_val->ego_in.alpha = mpc_next_ego.alpha;
  tgt_val->ego_in.pivot_state = mpc_next_ego.pivot_state;
  tgt_val->ego_in.sla_param = mpc_next_ego.sla_param;
  tgt_val->ego_in.state = mpc_next_ego.state;
  tgt_val->ego_in.decel_delay_cnt = mpc_next_ego.decel_delay_cnt;

  const auto tmp_v = tgt_val->ego_in.v;
  tgt_val->ego_in.v = mpc_next_ego.v;
  tgt_val->ego_in.v_l = mpc_next_ego.v - mpc_next_ego.w * param->tire_tread / 2;
  tgt_val->ego_in.v_r = mpc_next_ego.v + mpc_next_ego.w * param->tire_tread / 2;
  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.v < 10) {
      tgt_val->ego_in.v = tmp_v;
    }
    se->sen.r45.sensor_dist = 0;
    se->sen.l45.sensor_dist = 0;
    se->sen.r45_2.sensor_dist = 0;
    se->sen.l45_2.sensor_dist = 0;
    se->sen.r45_3.sensor_dist = 0;
    se->sen.l45_3.sensor_dist = 0;
  }
  tgt_val->ego_in.w = mpc_next_ego.w;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;
  tgt_val->ego_in.sla_param.counter = mpc_next_ego.sla_param.counter;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;
  sensing_result->img_ang_z = tgt_val->ego_in.img_ang;
  tgt_val->ego_in.img_ang = mpc_next_ego.img_ang;
  tgt_val->ego_in.img_dist = mpc_next_ego.img_dist;

  tgt_val->global_pos.img_ang += mpc_next_ego.w * dt;

  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.pivot_state == 3) {
      tgt_val->global_pos.img_ang = //
          tgt_val->ego_in.img_ang = tgt_val->tgt_in.tgt_angle;
    }
  }

  tgt_val->global_pos.img_dist += mpc_next_ego.v * dt;

  tgt_val->ego_in.slip_point.slip_angle = mpc_next_ego.slip_point.slip_angle;

  tgt_val->ego_in.cnt_delay_accl_ratio = mpc_next_ego.cnt_delay_accl_ratio;
  tgt_val->ego_in.cnt_delay_decel_ratio = mpc_next_ego.cnt_delay_decel_ratio;

  // const auto theta = tgt_val->ego_in.img_ang + slip_param.beta;
  // const auto x = tgt_val->ego_in.v * std::cos(theta);
  // const auto y = tgt_val->ego_in.v * std::sin(theta);
  // tgt_val->p.x += x;
  // tgt_val->p.y += y;

  tgt_val->ego_in.slip.beta = mpc_next_ego.slip.beta;
  tgt_val->ego_in.slip.accl = mpc_next_ego.slip.accl;
  tgt_val->ego_in.slip.v = mpc_next_ego.slip.v;
  tgt_val->ego_in.slip.vx = mpc_next_ego.slip.vx;
  tgt_val->ego_in.slip.vy = mpc_next_ego.slip.vy;

  ideal_v_r = tgt_val->ego_in.v - tgt_val->ego_in.w * param->tire_tread / 2;
  ideal_v_l = tgt_val->ego_in.v + tgt_val->ego_in.w * param->tire_tread / 2;

  tgt_val->ego_in.ideal_px = mpc_next_ego.ideal_px;
  tgt_val->ego_in.ideal_py = mpc_next_ego.ideal_py;
}

void PlanningTask::calc_tgt_duty() {

  float duty_ff_front = 0;
  float duty_ff_roll = 0;
  const unsigned char reset_req = motor_en ? 1 : 0;
  const unsigned char enable = 1;
  duty_sen = 0;
  sen_ang = 0;

  ee->s_val.p = ee->s_val.i = ee->s_val.d = 0;
  ee->s_val.p_val = ee->s_val.i_val = ee->s_val.d_val = 0;
  ee->s_val.z = ee->s_val.zz = 0;

  if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
    duty_sen = calc_sensor_pid();
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
    duty_sen = calc_sensor_pid_dia();
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::NONE) {
    duty_sen = sen_ang = 0;
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  }
  sensing_result->ego.duty.sen = duty_sen;
  sensing_result->ego.duty.sen_ang = sen_ang;

  // 重心速度PID偏差計算
  calc_pid_val();
  // 角度PID偏差計算
  calc_pid_val_ang();
  // 角速度PID偏差計算
  calc_pid_val_ang_vel();
  // 前壁制御PID偏差計算
  calc_pid_val_front_ctrl();

  duty_c = 0;
  duty_c2 = 0;
  duty_roll = 0;
  duty_front_ctrl_roll_keep = 0;
  duty_roll_ang = 0;
  duty_front_ctrl_trans = 0;
  duty_front_ctrl_roll = 0;
  reset_pid_val();

  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    calc_front_ctrl_duty();
  } else {
    // 重心速度制御
    calc_translational_ctrl();
    // 角速度制御
    calc_angle_velocity_ctrl();
  }
  sensing_result->ego.duty.sen = duty_sen;

  summation_duty();

  // Duty limitter
  apply_duty_limitter();

  if (tgt_val->motion_type == MotionType::NONE) {
    tgt_duty.duty_l = tgt_duty.duty_r = 0;
  }
  if (!motor_en) {
    clear_ctrl_val();
  }

  // set duty for log
  sensing_result->ego.duty.duty_r = tgt_duty.duty_r;
  sensing_result->ego.duty.duty_l = tgt_duty.duty_l;

  sensing_result->ego.duty.ff_duty_front = mpc_next_ego.ff_duty_front;
  sensing_result->ego.duty.ff_duty_roll = mpc_next_ego.ff_duty_roll;
  sensing_result->ego.duty.ff_duty_rpm_r = mpc_next_ego.ff_duty_rpm_r;
  sensing_result->ego.duty.ff_duty_rpm_l = mpc_next_ego.ff_duty_rpm_l;

  // sensing_result->ego.ff_duty.front = duty_ff_front;
  // sensing_result->ego.ff_duty.roll = duty_ff_roll;
  // copy_error_entity(error_entity);
}

float PlanningTask::calc_sensor_pid() {
  float duty = 0;

  SensingControlType type = SensingControlType::None;
  ee->sen.error_i += ee->sen.error_p;
  ee->sen.error_d = ee->sen.error_p;
  ee->sen.error_p = check_sen_error(type);
  if (search_mode) {
    if (ee->sen.error_p > param->search_sen_ctrl_limitter) {
      ee->sen.error_p = param->search_sen_ctrl_limitter;
    } else if (ee->sen.error_p < -param->search_sen_ctrl_limitter) {
      ee->sen.error_p = -param->search_sen_ctrl_limitter;
    }
  }
  ee->sen.error_d = ee->sen.error_p - ee->sen.error_d;

  if (search_mode) {
    if (ee->sen.error_p != 0) {
      duty = param->str_ang_pid.p * ee->sen.error_p -
             param->str_ang_pid.i * ee->sen.error_d;

      //  (ee->sen_log.gain_z - ee->sen_log.gain_zz) * dt;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param->str_ang_pid.p * ee->sen.error_p, 0, 0,
                   -param->str_ang_pid.d * ee->sen.error_d, ee->sen_log.gain_zz,
                   ee->sen_log.gain_z);
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  } else {

    if (ee->sen.error_p != 0) {
      duty = param->str_ang_pid.b * ee->sen.error_p -
             param->str_ang_pid.d * ee->sen.error_d;

      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param->str_ang_pid.b * ee->sen.error_p, 0, 0,
                   -param->str_ang_pid.d * ee->sen.error_d, ee->sen_log.gain_zz,
                   ee->sen_log.gain_z);
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, ee->sen_log.gain_zz,
                   ee->sen_log.gain_z);
    }
  }
  // if (search_mode) {
  //   if (duty > param->sensor_gain.front.a) {
  //     duty = param->sensor_gain.front.a;
  //   } else if (duty < -param->sensor_gain.front.a) {
  //     duty = -param->sensor_gain.front.a;
  //   }
  // } else {
  //   if (duty > param->sensor_gain.front2.a) {
  //     duty = param->sensor_gain.front2.a;
  //   } else if (duty < -param->sensor_gain.front2.a) {
  //     duty = -param->sensor_gain.front2.a;
  //   }
  // }

  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::Wall) {
    limit = interp1d(sensor_deg_limitter_v, sensor_deg_limitter_str,
                     tgt_val->ego_in.v, false);
  } else if (type == SensingControlType::Piller) {
    limit = interp1d(sensor_deg_limitter_v, sensor_deg_limitter_piller,
                     tgt_val->ego_in.v, false);
  }
  duty = std::clamp(duty, -limit, limit);
  if (!search_mode) {
    if (tgt_val->motion_type == MotionType::WALL_OFF ||
        tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val->motion_type == MotionType::SLA_BACK_STR) {
      duty = std::clamp(duty, -param->angle_pid.b, param->angle_pid.b);
    }
  }
  return duty;
}
float PlanningTask::calc_sensor_pid_dia() {
  float duty = 0;
  SensingControlType type = SensingControlType::None;
  ee->sen_dia.error_i += ee->sen_dia.error_p;
  ee->sen_dia.error_d = ee->sen_dia.error_p;
  ee->sen_dia.error_p = check_sen_error_dia(type);
  ee->sen_dia.error_d = ee->sen_dia.error_p - ee->sen_dia.error_d;

  if (type == SensingControlType::DiaPiller && ee->sen_dia.error_p != 0) {
    duty = param->sensor_pid_dia.p * ee->sen_dia.error_p -
           param->sensor_pid_dia.d * sensing_result->ego.w_kf;

    const float gain = 0.1;
    set_ctrl_val(ee->s_val, ee->sen_dia.error_p * gain, 0, 0,
                 ee->sen_dia.error_d * gain,
                 param->sensor_pid_dia.p * ee->sen_dia.error_p * gain,
                 param->sensor_pid_dia.i * ee->sen_dia.error_i * gain, 0,
                 param->sensor_pid_dia.d * ee->sen_dia.error_d * gain, 0, 0);
  } else {
    duty = 0;
    set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::DiaPiller) {
    limit = interp1d(sensor_deg_limitter_v, sensor_deg_limitter_dia,
                     tgt_val->ego_in.v, false);
  }
  duty = std::clamp(duty, -limit, limit);
  return duty;
}

float PlanningTask::check_sen_error(SensingControlType &type) {
  const auto se = get_sensing_entity();
  const auto prm = get_param();
  float error = 0;
  int check = 0;
  float dist_mod = (int)(tgt_val->ego_in.dist / param->dist_mod_num);
  float tmp_dist = tgt_val->ego_in.dist - param->dist_mod_num * dist_mod;

  bool expand_right = false;
  bool expand_left = false;
  bool expand_right_2 = false;
  bool expand_left_2 = false;

  auto exist_right45 = prm->sen_ref_p.normal.exist.right45;
  auto exist_left45 = prm->sen_ref_p.normal.exist.left45;

  auto wall_th =
      search_mode ? interp1d(param->clear_dist_ragne_dist_list,
                             param->clear_dist_ragne_th_list, tmp_dist, false)
                  : std::min(exist_left45, exist_right45);
  // printf("wall_th %f %f\n", wall_th, tmp_dist);

  // auto exist_right45_expand = prm->sen_ref_p.normal.expand.right45;
  // auto exist_left45_expand = prm->sen_ref_p.normal.expand.left45;
  auto exist_right45_expand = wall_th;
  auto exist_left45_expand = wall_th;

  // auto exist_right45_expand_2 = prm->sen_ref_p.normal.expand.right45_2;
  // auto exist_left45_expand_2 = prm->sen_ref_p.normal.expand.left45_2;
  float val_left = 1000;
  float val_right = 1000;

  // 範囲チェック
  bool range_check_right =
      (1 < se->ego.right45_dist) && (se->ego.right45_dist < exist_right45);
  bool range_check_left =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45);

  // 柱１個文連続で検出しているか
  bool dist_check_right = ABS(tgt_val->global_pos.dist - right_keep.star_dist) >
                          prm->right_keep_dist_th;
  bool dist_check_left = ABS(tgt_val->global_pos.dist - left_keep.star_dist) >
                         prm->left_keep_dist_th;
  // 切れ目チェック
  bool check_diff_right =
      ABS(se->ego.right45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_r;
  bool check_diff_left =
      ABS(se->ego.left45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_l;

  // 壁切れ前時の切れ目妥協
  if (!search_mode) {
    if (tgt_val->motion_type == MotionType::WALL_OFF ||
        tgt_val->motion_type == MotionType::SLA_FRONT_STR) {
      if (se->ego.right45_dist_diff < 0) {
        // 接近時は緩め
        check_diff_right = ABS(se->ego.right45_dist_diff) <
                           prm->sen_ref_p.normal.ref.kireme_r_wall_off2;
      } else {
        check_diff_right = ABS(se->ego.right45_dist_diff) <
                           prm->sen_ref_p.normal.ref.kireme_r_wall_off;
      }
      if (se->ego.left45_dist_diff < 0) {
        // 接近時は緩め
        check_diff_left = ABS(se->ego.left45_dist_diff) <
                          prm->sen_ref_p.normal.ref.kireme_l_wall_off2;
      } else {
        check_diff_left = ABS(se->ego.left45_dist_diff) <
                          prm->sen_ref_p.normal.ref.kireme_l_wall_off;
      }
    } else {
      check_diff_right = ABS(se->ego.right45_dist_diff) <
                         prm->sen_ref_p.normal.ref.kireme_r_fast;
      check_diff_left = ABS(se->ego.left45_dist_diff) <
                        prm->sen_ref_p.normal.ref.kireme_l_fast;
    }
  }
  // 前壁チェック
  bool check_front_left =
      (10 < se->ego.left90_mid_dist) &&
      (se->ego.left90_mid_dist < prm->sen_ref_p.normal.exist.front);
  bool check_front_right =
      (10 < se->ego.right90_mid_dist) &&
      (se->ego.right90_mid_dist < prm->sen_ref_p.normal.exist.front);

  // 切れ目に反応したら拡張をやめる
  if (!check_diff_right) {
    enable_expand_right = false;
  }
  if (!check_diff_left) {
    enable_expand_left = false;
  }
  if (search_mode && tgt_val->tgt_in.tgt_dist > 80 &&
      tgt_val->tgt_in.tgt_dist < 100 &&
      tgt_val->motion_type == MotionType::STRAIGHT) {
    // 特殊条件(直進中)で拡張許可
    expand_right = (10 < se->ego.right45_dist) &&
                   (se->ego.right45_dist < prm->sen_ref_p.search_exist.right45);
    expand_left = (10 < se->ego.left45_dist) &&
                  (se->ego.left45_dist < prm->sen_ref_p.search_exist.left45);
  } else {
    // 拡張許可時に許容幅を広げる
    if (enable_expand_right) {
      exist_right45_expand = wall_th + 1.5f;
      expand_right = (10 < se->ego.right45_dist) &&
                     (se->ego.right45_dist < exist_right45_expand);
    } else {
      exist_right45_expand = 0;
    }
    if (enable_expand_left) {
      exist_left45_expand = wall_th + 1.5f;
      expand_left = (10 < se->ego.left45_dist) &&
                    (se->ego.left45_dist < exist_left45_expand);
    } else {
      exist_left45_expand = 0;
    }
  }
  bool range_check_right_expand = (1 < se->ego.right45_dist) &&
                                  (se->ego.right45_dist < exist_right45_expand);
  bool range_check_left_expand =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45_expand);

  if (!(check_front_left && check_front_right)) { // 前壁チェック
    // 即時関数で条件別にセンサーエラーチェックの順序を切り替え

    const bool is_wall_off_mode =
        (tgt_val->motion_type == MotionType::WALL_OFF);
    if (!is_wall_off_mode) {
      check_left_sensor_error(error, check, range_check_left, dist_check_left,
                              check_diff_left, expand_left,
                              range_check_left_expand);
      check_right_sensor_error(error, check, range_check_right,
                               dist_check_right, check_diff_right, expand_right,
                               range_check_right_expand);
    } else {
      // 壁切れモードの場合、反対側が見えているなら優先度を下げる
      if (tgt_val->motion_dir == MotionDirection::LEFT) {
        check_right_sensor_error(error, check, range_check_right,
                                 dist_check_right, check_diff_right,
                                 expand_right, range_check_right_expand);
        check_left_sensor_error(error, check, range_check_left, dist_check_left,
                                check_diff_left, expand_left,
                                range_check_left_expand);
      } else {
        check_left_sensor_error(error, check, range_check_left, dist_check_left,
                                check_diff_left, expand_left,
                                range_check_left_expand);
        check_right_sensor_error(error, check, range_check_right,
                                 dist_check_right, check_diff_right,
                                 expand_right, range_check_right_expand);
      }
    }
    if (check != 0) {
      type = SensingControlType::Wall;
    }
  }

  // 片壁制御から両壁に移る際に、壁の切れ目でなければ拡張許可を出す
  if (enable_expand_left && !enable_expand_right && check_diff_right &&
      ((1 < se->ego.right45_dist) && //
       (se->ego.right45_dist < wall_th + 0.75))) {
    enable_expand_right = true;
  } else if (!enable_expand_left && enable_expand_right && check_diff_left &&
             ((1 < se->ego.left45_dist) &&
              (se->ego.left45_dist < wall_th + 0.75))) {
    enable_expand_left = true;
  }

  if (check == 0) {
    // 柱制御
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
    bool right_check = false;
    bool left_check = false;
    float right_error = 0;
    float left_error = 0;

    // 切れる前の値が規定値以上

    const bool range_check_passed_right =
        (prm->sen_ref_p.normal2.ref.kireme_r < se->sen.r45.sensor_dist) &&
        (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.exist.right45) &&
        (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;

    const bool range_check_passed_left =
        (prm->sen_ref_p.normal2.ref.kireme_l < se->sen.l45.sensor_dist) &&
        (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.exist.left45) &&
        (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;

    const bool exist_right45 =
        se->ego.right45_dist < prm->sen_ref_p.search_exist.right45;
    const bool exist_left45 =
        se->ego.left45_dist < prm->sen_ref_p.search_exist.left45;

    if (!(check_front_left && check_front_right)) {
      if (range_check_passed_right && !exist_left45) {
        error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
        right_error +=
            prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
        check++;
        right_check = true;
      }
      if (range_check_passed_left && !exist_right45) {
        error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
        left_error -=
            (prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist);
        check++;
        left_check = true;
      }

      if (check == 0) {
        const bool range_check_passed_right_near =
            (prm->sen_ref_p.normal2.ref.right90 < se->sen.r45.sensor_dist) &&
            (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.ref.kireme_r) &&
            (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;
        const bool range_check_passed_left_near =
            (prm->sen_ref_p.normal2.ref.left90 < se->sen.l45.sensor_dist) &&
            (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.ref.kireme_l) &&
            (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;
        if (!range_check_passed_left_near && range_check_passed_right_near &&
            !exist_left45) {
          error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
          check++;
        } else if (range_check_passed_left_near &&
                   !range_check_passed_right_near && !exist_right45) {
          error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
          check++;
        }
      }
      if (check != 0 && //
          !(tgt_val->motion_type == MotionType::SLA_FRONT_STR)) {
        type = SensingControlType::Piller;
      }
      // error *= prm->sen_ref_p.normal2.exist.front;
    }
  } else {
    // TODO Uターン字は別ロジックに修正
    if (tgt_val->tgt_in.tgt_dist >= prm->clear_dist_order) {
      if (!(prm->clear_dist_ragne_from <= tmp_dist &&
            tmp_dist <= prm->clear_dist_ragne_to)) {
        if (std::abs(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) <
            prm->clear_angle) {

          // 目標距離まで遠いならリセット
          if ((tgt_val->tgt_in.tgt_dist - tgt_val->ego_in.dist) >
              (prm->cell / 2)) {
            tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
            ee->w.error_i = 0;
            ee->w.error_d = 0;
            ee->w.error_dd = 0;
            ee->w_kf.error_i = 0;
            ee->w_kf.error_d = 0;
            ee->w_kf.error_dd = 0;
            ee->ang.error_i = 0;
            ee->ang.error_d = 0;
            ee->ang.error_dd = 0;
            ee->ang.i_slow = 0;
            ee->ang.i_bias = 0;
            // se->ego.ang_kf = tgt_val->global_pos.ang = tgt_val->ego_in.ang =
            //     tgt_val->global_pos.img_ang;
            // kim.theta = 0;
            // kf_ang.reset(0);
            // gyro_pid_histerisis_i = 0;
            // gyro_pid_windup_histerisis = false;
            w_reset = 0;
          }
        }
      } else {
        // ee->sen.error_i = 0;
        // ee->sen_log.gain_zz = 0;
        // ee->sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  ee->sen.error_i = 0;
  ee->sen_log.gain_zz = 0;
  ee->sen_log.gain_z = 0;
  return 0;
}
float PlanningTask::check_sen_error_dia(SensingControlType &type) {
  float error = 0;
  int check = 0;
  const auto se = get_sensing_entity();

  // 斜め移動が一定距離以上するとき、かつ、終わりまでの距離が一定距離未満
  if (tgt_val->tgt_in.tgt_dist > param->sen_ctrl_front_th &&
      (tgt_val->tgt_in.tgt_dist - tgt_val->ego_in.dist) >
          param->sen_ctrl_front_diff_th) {
    // 右前センサーが一定距離以内のとき
    const bool valid_right90 =
        1 < se->ego.right90_mid_dist &&
        se->ego.right90_mid_dist < param->sen_ref_p.dia.exist.right90;
    // 左前センサーが一定距離以内のとき
    const bool valid_left90 =
        1 < se->ego.left90_mid_dist &&
        se->ego.left90_mid_dist < param->sen_ref_p.dia.exist.left90;

    // 右センサーが一定距離以内のとき
    const bool valid_right45 =
        1 < se->sen.r45.sensor_dist &&
        se->sen.r45.sensor_dist < param->sen_ref_p.dia.exist.right45 &&
        se->sen.r45.sensor_dist < se->ego.right45_dist;
    const bool valid_left45 =
        1 < se->sen.l45.sensor_dist &&
        se->sen.l45.sensor_dist < param->sen_ref_p.dia.exist.left45 &&
        se->sen.l45.sensor_dist < se->ego.left45_dist;

    if (valid_right90) {
      error += param->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val->dia_state.right_old =
          param->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val->dia_state.right_save = true;
      tgt_val->dia_state.left_save = false;
      check++;
    }
    if (valid_left90) {
      error -= param->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val->dia_state.left_old =
          param->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val->dia_state.left_save = true;
      tgt_val->dia_state.right_save = false;
      check++;
    }
    if (!valid_left90 && !valid_right90 && param->sensor_gain.front4.a != 0) {

      if (valid_right45) {
        error += param->sen_ref_p.dia.ref.right45 - se->sen.r45.sensor_dist;
        check++;
      }
      if (valid_left45) {
        error -= param->sen_ref_p.dia.ref.left45 - se->sen.l45.sensor_dist;
        check++;
      }
      if (!(valid_left45 && valid_right45)) {
        if (tgt_val->dia_state.right_save) {
          error += tgt_val->dia_state.right_old;
          check++;
        }
        if (tgt_val->dia_state.left_save) {
          error -= tgt_val->dia_state.left_old;
          check++;
        }
      }
    }
  }
  // }
  if (check == 0) {
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else {
    type = SensingControlType::DiaPiller;
    // TODO Uターン字は別ロジックに修正
    // ee->sen_dia.error_i = 0;
    // ee->sen_log_dia.gain_zz = 0;
    // ee->sen_log_dia.gain_z = 0;
    if (tgt_val->tgt_in.tgt_dist >= param->clear_dist_order) {
      if (std::abs(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) <
          param->clear_angle) {
        // tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
        // ee->w.error_i = 0;
        // ee->w.error_d = 0;
        // ee->ang.error_i = 0;
        // ee->ang.error_d = 0;
      } else {
        // ee->sen.error_i = 0;
        // ee->sen_log.gain_zz = 0;
        // ee->sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  return 0;
}

void PlanningTask::check_fail_safe() {
  bool no_problem = true;

  if (!motor_en) {
    tgt_val->fss.error = 0;
    return;
  }
  if (ABS(ee->ang.error_p) > param->fail_check_ang_th) {
    fail_check_ang++;
  } else {
    fail_check_ang = 0;
  }
  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    keep_wall_off_cnt++;
  } else {
    keep_wall_off_cnt = 0;
  }

  if (ABS(ee->v.error_i) > param->fail_check.v) {
    tgt_val->fss.error = 1;
  }
  if (ABS(ee->w.error_i) > param->fail_check.w) {
    tgt_val->fss.error = 1;
  }
  if (ABS(ee->ang.error_i) > param->fail_check.ang) {
    tgt_val->fss.error = 1;
  }
  if (keep_wall_off_cnt > param->fail_check.wall_off) {
    tgt_val->fss.error = 1;
  }

  // if (!gpio_get_level(SW1)) {
  //   tgt_val->fss.error = 1;
  // }

  // if (!std::isfinite(tgt_duty.duty_l) || !std::isfinite(tgt_duty.duty_r))
  // {
  //   tgt_val->fss.error = 1;
  // }
}

void PlanningTask::calc_pid_val() {
  ee->v.error_dd = ee->v.error_d;
  ee->v.error_dd = ee->v.error_d;
  ee->v_kf.error_dd = ee->v_kf.error_d;
  ee->dist.error_dd = ee->dist.error_d;

  ee->v.error_d = ee->v.error_p;
  ee->v_r.error_d = ee->v_r.error_p;
  ee->v_l.error_d = ee->v_l.error_p;
  ee->v_kf.error_d = ee->v_kf.error_p;
  ee->dist.error_d = ee->dist.error_p;

  ee->v.error_p = v_cmd - sensing_result->ego.v_c;

  ee->v_r.error_p = ideal_v_r - sensing_result->ego.v_r;
  ee->v_l.error_p = ideal_v_l - sensing_result->ego.v_l;
  ee->v_kf.error_p = v_cmd - sensing_result->ego.v_kf;
  ee->w_kf.error_p = w_cmd - sensing_result->ego.w_kf;

  ee->dist.error_p = tgt_val->global_pos.img_dist - tgt_val->global_pos.dist;

  if (ee->dist.error_p > param->front_ctrl_error_th) {
    ee->dist.error_p = param->front_ctrl_error_th;
  } else if (ee->dist.error_p < -param->front_ctrl_error_th) {
    ee->dist.error_p = -param->front_ctrl_error_th;
  }

  ee->v_kf.error_d = ee->v_kf.error_p - ee->v_kf.error_d;
  ee->v.error_d = ee->v.error_p - ee->v.error_d;
  ee->dist.error_d = ee->dist.error_p - ee->dist.error_d;

  ee->v_l.error_d = ee->v_l.error_p - ee->v_l.error_d;
  ee->v_r.error_d = ee->v_r.error_p - ee->v_r.error_d;

  ee->v_kf.error_dd = ee->v_kf.error_d - ee->v_kf.error_dd;
  ee->v.error_dd = ee->v.error_d - ee->v.error_dd;
  ee->dist.error_dd = ee->dist.error_d - ee->dist.error_dd;

  ee->v.error_i += ee->v.error_p;
  if (tgt_val->motion_type != MotionType::FRONT_CTRL) {
    ee->dist.error_i += ee->dist.error_p;
  }

  ee->v_l.error_i += ee->v_l.error_p;
  ee->v_r.error_i += ee->v_r.error_p;

  tgt_val->v_error = ee->v.error_i;
}
void PlanningTask::calc_pid_val_ang() {

  const auto tgt = get_tgt_entity();
  const auto se = get_sensing_entity();

  ee->ang.error_dd = ee->ang.error_d;
  ee->ang.error_d = ee->ang.error_p;

  // TODO カスケード制御条件分岐

  float offset = 0;

  if (tgt->motion_type != MotionType::FRONT_CTRL) {
    offset += duty_sen;
  }
  // 壁制御できないときは角度を固定
  if ((tgt->motion_type == MotionType::STRAIGHT) ||
      tgt->motion_type == MotionType::SLA_FRONT_STR ||
      tgt->motion_type == MotionType::SLA_BACK_STR ||
      tgt->motion_type == MotionType::WALL_OFF ||
      tgt->motion_type == MotionType::WALL_OFF_DIA) {
  }
  ee->ang.error_p = (tgt->ego_in.img_ang + offset) - se->ego.ang_kf;
  ee->ang.error_d = ee->ang.error_p - ee->ang.error_d;
  ee->ang.error_dd = ee->ang.error_d - ee->ang.error_dd;
  ee->ang.error_i += (ee->ang.error_p);
  // 壁制御できないときは角度を固定
  if (!(tgt->motion_type == MotionType::STRAIGHT) ||
      tgt->motion_type == MotionType::SLA_FRONT_STR ||
      tgt->motion_type == MotionType::SLA_BACK_STR ||
      tgt->motion_type == MotionType::WALL_OFF ||
      tgt->motion_type == MotionType::WALL_OFF_DIA) {
    ee->ang.error_d = ee->ang.error_dd = ee->ang.error_i = 0;
  }

  if (w_reset == 0) { // 角度リセット直後は誤差を0にする
    // ee->ang.error_d = ee->ang.error_dd = ee->ang.error_i = 0;
  }

  float ang_error_i = ee->ang.error_i;
  if (param->angle_pid.antiwindup) {
    if (ang_error_i * ee->ang.error_p < 0 &&
        ABS(ee->ang.error_p) > param->angle_pid.windup_dead_bind) {
      ang_error_i *= param->angle_pid.windup_gain;
    }
  }

  duty_roll_ang = param->angle_pid.p * ee->ang.error_p +
                  param->angle_pid.i * ang_error_i +
                  param->angle_pid.d * ee->ang.error_d;

  calc_angle_i_bias();

  set_ctrl_val(ee->ang_val,
               ee->ang.error_p,                      // p
               ee->ang.error_i,                      // i
               duty_sen,                             // i2
               ee->ang.error_d,                      // d
               param->angle_pid.p * ee->ang.error_p, // kp*p
               param->angle_pid.i * ang_error_i,     // ki*i
               param->gyro_pid.c * ee->ang.i_bias,   // kb*i2
               param->angle_pid.d * ee->ang.error_d, // kd*d
               0, 0);
}

void PlanningTask::calc_pid_val_ang_vel() {
  const auto tgt = get_tgt_entity();
  const auto se = get_sensing_entity();

  ee->w.error_dd = ee->w.error_d;
  ee->w_kf.error_dd = ee->w_kf.error_d;
  ee->w.error_d = ee->w.error_p;
  ee->w_kf.error_d = ee->w_kf.error_p;

  float offset = 0;

  if (param->torque_mode == 2) {
    if (!(tgt->motion_type == MotionType::PIVOT ||
          tgt->motion_type == MotionType::FRONT_CTRL)) {
      offset += duty_roll_ang;
    }
  }
  ee->aw_log.duty_roll_before = (tgt->ego_in.w + offset);

  ee->w.error_p = (tgt->ego_in.w + offset) - se->ego.w_lp;
  ee->w_kf.error_p = (tgt_val->ego_in.w + offset) - se->ego.w_kf;

  ee->w.error_d = ee->w.error_p - ee->w.error_d;
  ee->w_kf.error_d = ee->w_kf.error_p - ee->w_kf.error_d;

  ee->w.error_dd = ee->w.error_d - ee->w.error_dd;
  ee->w_kf.error_dd = ee->w_kf.error_d - ee->w_kf.error_dd;

  ee->w.error_i += ee->w.error_p;
  ee->w_kf.error_i += ee->w_kf.error_p;

  if (w_reset == 0) { // 角度リセット直後は誤差を0にする
    // ee->w.error_d = ee->w.error_dd = ee->w.error_i = ee->w_kf.error_d =
    //     ee->w_kf.error_dd = ee->w_kf.error_i = 0;
  }

  tgt_val->w_error = ee->w.error_i;
}

void PlanningTask::calc_pid_val_front_ctrl() {
  const auto se = get_sensing_entity();
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->w.error_i = ee->w.error_d = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    if (se->ego.front_dist < param->cell) {
      ee->dist.error_p =
          se->ego.front_dist - param->sen_ref_p.search_exist.front_ctrl;
      ee->ang.error_p = (se->ego.right90_dist - se->ego.left90_dist) / 2 -
                        param->sen_ref_p.search_exist.kireme_r;
      ee->dist.error_i += ee->dist.error_p;
    } else {
      ee->dist.error_p = ee->dist.error_i = ee->dist.error_d = 0;
      ee->ang.error_p = ee->ang.error_i = ee->ang.error_d = 0;
    }
  }
}

void PlanningTask::reset_pid_val() {
  //
  if (tgt_val->motion_type == MotionType::FRONT_CTRL || !motor_en ||
      tgt_val->motion_type == MotionType::NONE) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->v_kf.error_i = ee->v_kf.error_d = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = 0;
    ee->v_log.gain_z = ee->v_log.gain_zz = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l_log.gain_z = ee->v_l_log.gain_zz = 0;
    ee->v_r_log.gain_z = ee->v_r_log.gain_zz = 0;
    ee->sen.error_i = ee->sen.error_d = 0;
    ee->sen_log.gain_zz = ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = ee->sen_dia.error_d = 0;
    ee->sen_log_dia.gain_zz = ee->sen_log_dia.gain_z = 0;
    ee->aw_log.duty_roll_before = ee->aw_log.duty_roll = 0;
    mpc_u_prev = mpc_d_estimated = 0;
  }

  // reset
  ee->v_val.p = ee->v_val.i = ee->v_val.d = 0;
  ee->w_val.p = ee->w_val.i = ee->w_val.d = 0;
  ee->v_val.p_val = ee->v_val.i_val = ee->v_val.d_val = 0;
  ee->w_val.p_val = ee->w_val.i_val = ee->w_val.d_val = 0;
  ee->v_val.z = ee->v_val.zz = 0;
  ee->w_val.z = ee->w_val.zz = 0;
}

void PlanningTask::calc_angle_i_bias() {
  // --- 角度I（ee->ang.error_i）から“低帯域だけ”を取り出す ---
  const float dt = param->dt; // 既存の制御周期

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::PIVOT ||
      tgt_val->motion_type == MotionType::PIVOT_PRE ||
      tgt_val->motion_type == MotionType::PIVOT_PRE2 ||
      tgt_val->motion_type == MotionType::PIVOT_AFTER ||
      tgt_val->motion_type == MotionType::PIVOT_OFFSET ||
      tgt_val->motion_type == MotionType::BACK_STRAIGHT ||
      tgt_val->motion_type == MotionType::READY ||
      tgt_val->motion_type == MotionType::FRONT_CTRL) {
    ee->ang.i_bias = 0;
  } else {
    ee->ang.i_bias = tgt_val->ego_in.img_ang -
                     kim.theta; // 指示角度 - カルマンフィルタを使った角度
  }
  if (search_mode) {
    ee->ang.i_bias = 0;
  }
}

void PlanningTask::check_left_sensor_error(
    float &error, int &check, bool range_check_left, bool dist_check_left,
    bool check_diff_left, bool expand_left, bool range_check_left_expand) {
  const auto se = get_sensing_entity();
  const auto prm = get_param();
  const bool is_wall_off_mode = (tgt_val->motion_type == MotionType::WALL_OFF);

  if (is_wall_off_mode && error != 0) {
    expand_left = range_check_left_expand = false;
  }

  if (range_check_left) {
    if (dist_check_left && check_diff_left) {
      enable_expand_left = true;
      error -= prm->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    } else if (expand_left && range_check_left_expand && dist_check_left &&
               check_diff_left) {
      enable_expand_left = true;
      error -= param->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    }
    check++;
  } else if (expand_left && range_check_left_expand) {
    if (dist_check_left && check_diff_left) {
      enable_expand_left = true;
      error -= param->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    }
    check++;
  } else {
    left_keep.star_dist = tgt_val->global_pos.dist;
  }
}
void PlanningTask::check_right_sensor_error(
    float &error, int &check, bool range_check_right, bool dist_check_right,
    bool check_diff_right, bool expand_right, bool range_check_right_expand) {
  const auto se = get_sensing_entity();
  const auto prm = get_param();
  const bool is_wall_off_mode = (tgt_val->motion_type == MotionType::WALL_OFF);

  if (is_wall_off_mode && error != 0) {
    expand_right = range_check_right_expand = false;
  }

  if (range_check_right) {
    if (dist_check_right && check_diff_right) {
      enable_expand_right = true;
      error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
    } else if (expand_right && range_check_right_expand && dist_check_right &&
               check_diff_right) {
      enable_expand_right = true;
      error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
    }
    check++;
  } else if (expand_right && range_check_right_expand) {
    if (dist_check_right && check_diff_right) {
      error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      enable_expand_right = true;
    }
    check++;
  } else {
    right_keep.star_dist = tgt_val->global_pos.dist;
    enable_expand_right = false;
  }
}

void PlanningTask::set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                                float error_i2, float error_d, float val_p,
                                float val_i, float val_i2, float val_d,
                                float zz, float z) {
  val.p = error_p;
  val.i = error_i;
  val.i2 = error_i2;
  val.d = error_d;
  val.p_val = val_p;
  val.i_val = val_i;
  val.i2_val = val_i2;
  val.d_val = val_d;
  val.zz = zz;
  val.z = z;
}

void PlanningTask::calc_front_ctrl_duty() {
  const unsigned char reset = 0;
  param->motor_pid.i = param->motor_pid.d = 0;
  vel_pid.step(&ee->v.error_p, &param->motor_pid.p, &param->motor_pid.i,
               &param->motor_pid.d, &reset, &dt, &duty_c);
  set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
               param->motor_pid.p * ee->v.error_p,
               vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0, 0);
  // reset
  ee->w.error_i = ee->w.error_d = 0;
  ee->w_kf.error_i = ee->w_kf.error_d = 0;
  ee->w_log.gain_z = ee->w_log.gain_zz = 0;

  auto diff_ang = 0.0f;
  auto kp_gain = param->front_ctrl_roll_pid.p * ee->w.error_p;
  auto ki_gain = 0.0f;
  auto kb_gain = 0.0f;
  auto kc_gain = 0.0f;
  auto kd_gain = param->front_ctrl_roll_pid.d * ee->w_kf.error_d;

  limitter(kp_gain, ki_gain, kb_gain, kd_gain, param->gyro_pid_gain_limitter);
  duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
              (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt;

  ee->ang_log.gain_zz = ee->ang_log.gain_z;
  ee->ang_log.gain_z = duty_roll;

  set_ctrl_val(ee->w_val,
               ee->w.error_p,                        // p
               diff_ang,                             // i
               ee->w.error_i,                        // i2
               ee->w_kf.error_d,                     // d
               param->gyro_pid.p * ee->w.error_p,    // kp*p
               param->gyro_pid.i * diff_ang,         // ki*i
               param->gyro_pid.b * ee->w.error_i,    // kb*i2
               param->gyro_pid.d * ee->w_kf.error_d, // kd*d
               ee->ang_log.gain_zz, ee->ang_log.gain_z);

  sensing_result->ego.duty.sen = duty_roll;
  sensing_result->ego.duty.sen = 0;
  // calc front dist ctrl
  duty_front_ctrl_trans =
      param->front_ctrl_dist_pid.p * ee->dist.error_p +
      param->front_ctrl_dist_pid.i * ee->dist.error_i +
      param->front_ctrl_dist_pid.d * sensing_result->ego.v_c;
  // calc front roll ctrl
  duty_front_ctrl_roll = param->front_ctrl_angle_pid.p * ee->ang.error_p +
                         param->front_ctrl_angle_pid.i * ee->ang.error_i +
                         param->front_ctrl_angle_pid.d * ee->w_kf.error_p;
  duty_front_ctrl_roll_keep =
      param->front_ctrl_keep_angle_pid.p * ee->ang.error_p +
      param->front_ctrl_keep_angle_pid.i * ee->ang.error_i +
      param->front_ctrl_keep_angle_pid.d * ee->w_kf.error_p;
  gyro_pid_windup_histerisis = false;
  gyro_pid_histerisis_i = 0.0f;
}

void PlanningTask::calc_angle_velocity_ctrl() {
  const auto se = get_sensing_entity();
  if (tgt_val->motion_type != ee->ang_log.prev_motion_type) {
    ee->ang_log.omega_ref_prev = tgt_val->ego_in.w; // もしくは omega_meas
    ee->ang_log.prev_motion_type = tgt_val->motion_type;
  }
  if (tgt_val->motion_type == MotionType::NONE) {
    duty_roll = param->gyro_pid.p * ee->w.error_p +
                param->gyro_pid.b * ee->w.error_i +
                param->gyro_pid.c * ee->w.error_d;
    // (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt;
    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;

    set_ctrl_val(ee->w_val, ee->w.error_p, ee->w.error_i, 0, ee->w.error_d,
                 param->gyro_pid.p * ee->w.error_p,
                 param->gyro_pid.b * ee->w.error_i, param->gyro_pid.b * 0,
                 param->gyro_pid.c * ee->w.error_d, ee->ang_log.gain_zz,
                 ee->ang_log.gain_z);
    gyro_pid_windup_histerisis = false;
    gyro_pid_histerisis_i = 0;
  } else {
    // mode3 main

    auto diff_ang = (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf);
    auto ang_sum = ee->ang.error_i;
    if (tgt_val->motion_type == MotionType::SLALOM) {
      diff_ang = 0;
      ang_sum = 0;
    }
    auto w_error_i = ee->w.error_i;
    auto w_error_d = ee->w_kf.error_d;

    // Logging variables
    ee->aw_log.was_aw = (float)gyro_pid_windup_histerisis;
    ee->aw_log.w_i_base = w_error_i;

    if (param->gyro_pid.antiwindup) {
      float db = param->gyro_pid.windup_dead_bind;
      // if (duty_sen != 0 && tgt_val->nmr.sct == SensorCtrlType::Straight) {
      if (duty_sen != 0) {
        db *= param->gyro_pid.windup_gain;
      }
      if (tgt_val->motion_type == MotionType::SLALOM &&
          tgt_val->tgt_in.v_max < 500) {
        db *= param->gyro_pid.windup_gain;
      }
      if ((w_error_i * ee->w.error_p < 0) &&
          ((ABS(ee->w.error_p) > db) ||
           (gyro_pid_windup_histerisis && ABS(ee->w.error_p) > db * 0.75))) {
        gyro_pid_histerisis_i += ee->w.error_p;
        w_error_i = gyro_pid_histerisis_i;
        gyro_pid_windup_histerisis = true;
      } else {
        if (gyro_pid_windup_histerisis) { // true -> false
          // w_error_i = ee->w.error_i = ee->ang.error_p / dt;
          w_error_i = ee->w.error_i = ee->ang.i_bias / dt;
        }
        gyro_pid_windup_histerisis = false;
        gyro_pid_histerisis_i = 0;
      }

      ee->aw_log.w_error_i_raw = w_error_i;
      ee->aw_log.gyro_pid_histerisis_i = gyro_pid_histerisis_i;

      // Apply Clamp (Angle Limiter)
      if (tgt_val->motion_type == MotionType::SLALOM) {
        w_error_i = std::clamp(w_error_i * dt, -ABS(tgt_val->tgt_in.tgt_angle),
                               ABS(tgt_val->tgt_in.tgt_angle)) /
                    dt;
      } else if (tgt_val->motion_type == MotionType::SLA_BACK_STR) {
        w_error_i = std::clamp(w_error_i * dt, -ABS(last_tgt_angle),
                               ABS(last_tgt_angle)) /
                    dt;
      }
      ee->aw_log.w_error_i_clamped = w_error_i;
    }

    if (!(tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
          tgt_val->motion_type == MotionType::SLA_BACK_STR ||
          tgt_val->motion_type == MotionType::PIVOT)) {
      diff_ang = 0;
      ang_sum = 0;
    }

    auto kp_gain = param->gyro_pid.p * ee->w.error_p;
    auto ki_gain = param->gyro_pid.i * diff_ang;
    auto kb_gain = param->gyro_pid.b * w_error_i;
    auto kc_gain = param->gyro_pid.c * ee->ang.i_bias;
    auto kd_gain = param->gyro_pid.d * w_error_d;
    limitter(kp_gain, ki_gain, kb_gain, kd_gain, param->gyro_pid_gain_limitter);
    duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
                (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt;

    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;

    // --- Disturbance Observer (DOB) ---
    float dt = param->dt;
    float b = param->gyro_pid.mpc_b;
    float w_meas = -ee->w.error_p + tgt_val->ego_in.w;

    // Prediction: w_next = w + (u + d) * b * dt
    float w_pred = mpc_w_prev + (mpc_u_prev + mpc_d_estimated) * b * dt;
    float observer_k = param->gyro_pid.mpc_observer_k;

    mpc_d_estimated += observer_k * (w_meas - w_pred);
    mpc_w_prev = w_meas;

    // MPC Override Logic
    if (!(tgt_val->motion_type == MotionType::SLALOM)) {
      // ee->aw_log.duty_roll_before = ee->aw_log.duty_roll = duty_roll;
    } else if (param->enable_mpc > 0) {
      // float mpc_u =
      //     mpc_solver.solve({-w_error_i * dt, -ee->w.error_p,
      //     mpc_d_estimated},
      //                      -param->max_duty, param->max_duty);

      // // ee->aw_log.duty_roll_before = duty_roll;
      // ee->aw_log.duty_roll = duty_roll = mpc_u;
      // ee->aw_log.sat_flag = 2.0f;
    }
    // ee->aw_log.duty_roll_before = ee->aw_log.duty_roll = duty_roll;
    mpc_u_prev = duty_roll;
    set_ctrl_val(ee->w_val,
                 ee->w.error_p, // p
                 diff_ang,      // i
                 w_error_i,     // i2
                 w_error_d,     // d
                 kp_gain,       // kp*p
                 ki_gain,       // ki*i
                 kb_gain,       // kb*i2
                 kd_gain,       // kd*d
                 ee->ang_log.gain_zz, ee->ang_log.gain_z);
  }
}

void PlanningTask::limitter(float &kp, float &ki, float &kb, float &kd,
                            pid_param_t &limitter) {
  if (limitter.mode == 0) {
    return;
  }
  if (kp > limitter.p) {
    kp = limitter.p;
  } else if (kp < -limitter.p) {
    kp = -limitter.p;
  }
  if (ki > limitter.i) {
    ki = limitter.i;
  } else if (ki < -limitter.i) {
    ki = -limitter.i;
  }
  if (kb > limitter.b) {
    kb = limitter.b;
  } else if (kb < -limitter.b) {
    kb = -limitter.b;
  }
  if (kd > limitter.d) {
    kd = limitter.d;
  } else if (kd < -limitter.d) {
    kd = -limitter.d;
  }
}

void PlanningTask::summation_duty() {

  auto ff_front = mpc_next_ego.ff_duty_front;
  auto ff_roll = mpc_next_ego.ff_duty_roll;
  const auto se = get_sensing_entity();

  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    ff_front = param->ff_roll_gain_before * ff_front;
    mpc_next_ego.ff_duty_front = ff_front;
  }

  if (tgt_val->motion_type == MotionType::SLA_BACK_STR) {
    ff_front = param->ff_front_gain_14 * ff_front;
    mpc_next_ego.ff_duty_front = ff_front;
  }

  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.sla_param.base_alpha > 0) {
      ff_roll = (tgt_val->ego_in.alpha < 0)
                    ? param->ff_roll_gain_after * ff_roll
                    : ff_roll;
    } else if (tgt_val->ego_in.sla_param.base_alpha < 0) {
      ff_roll = (tgt_val->ego_in.alpha > 0)
                    ? param->ff_roll_gain_after * ff_roll
                    : ff_roll;
    }
  }
  se->ego.duty.ff_duty_roll = mpc_next_ego.ff_duty_roll = ff_roll;
  auto ff_duty_r = ff_front + ff_roll + mpc_next_ego.ff_duty_rpm_r;
  auto ff_duty_l = ff_front - ff_roll + mpc_next_ego.ff_duty_rpm_l;

  if (param->FF_keV == 0) {
    ff_duty_l = ff_duty_r = 0;
  }
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    tgt_duty.duty_r =
        (duty_c + duty_front_ctrl_trans + duty_roll + duty_front_ctrl_roll +
         duty_front_ctrl_roll_keep + ff_duty_r) /
        se->ego.battery_lp * 100;

    tgt_duty.duty_l =
        (duty_c + duty_front_ctrl_trans - duty_roll - duty_front_ctrl_roll -
         duty_front_ctrl_roll_keep + ff_duty_l) /
        se->ego.battery_lp * 100;

  } else if (param->torque_mode == 1) {
  } else if (param->torque_mode == 2) {
    auto ff_front = mpc_next_ego.ff_front_torque;
    auto ff_roll = mpc_next_ego.ff_roll_torque;
    auto ff_duty_r = mpc_next_ego.ff_duty_rpm_r;
    auto ff_duty_l = mpc_next_ego.ff_duty_rpm_l;
    auto ff_friction_r = mpc_next_ego.ff_friction_torque_r;
    auto ff_friction_l = mpc_next_ego.ff_friction_torque_l;

    // auto v = mpc_next_ego.v;
    // auto w = mpc_next_ego.w;
    // auto tread = param->tire_tread;
    // auto v_l = v - w * tread / 2;
    // auto v_r = v + w * tread / 2;

    // auto cf = param->coulomb_friction;
    // auto vf = param->viscous_friction;

    // auto ff_friction_l = (v_l > 0) ? (vf * v_l + cf) : (vf * v_l - cf);
    // auto ff_friction_r = (v_r > 0) ? (vf * v_r + cf) : (vf * v_r - cf);

    if (param->FF_keV == 0) {
      ff_front = ff_roll = ff_duty_r = ff_duty_l = ff_friction_r =
          ff_friction_l = 0;
    }
    float torque_r = (ff_front + ff_roll + duty_c + duty_roll + ff_friction_r);
    float torque_l = (ff_front - ff_roll + duty_c - duty_roll + ff_friction_l);

    const float km_gear = param->Km * (param->gear_a / param->gear_b);
    float req_v_r = torque_r * param->Resist / km_gear + ff_duty_r;
    float req_v_l = torque_l * param->Resist / km_gear + ff_duty_l;

    tgt_duty.duty_r = req_v_r / se->ego.battery_lp * 100;
    tgt_duty.duty_l = req_v_l / se->ego.battery_lp * 100;
  }
}

void PlanningTask::apply_duty_limitter() {
  if (tgt_val->motion_type == MotionType::STRAIGHT ||
      tgt_val->motion_type == MotionType::SLALOM ||
      tgt_val->motion_type == MotionType::SLA_BACK_STR ||
      tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
      tgt_val->motion_type == MotionType::PIVOT) {
    const auto min_duty = param->min_duty;

    if (0 <= tgt_duty.duty_r && tgt_duty.duty_r < min_duty) {
      tgt_duty.duty_r = min_duty;
    } else if (-min_duty < tgt_duty.duty_r && tgt_duty.duty_r <= 0) {
      tgt_duty.duty_r = -min_duty;
    }
    if (0 <= tgt_duty.duty_l && tgt_duty.duty_l < min_duty) {
      tgt_duty.duty_l = min_duty;
    } else if (-min_duty < tgt_duty.duty_l && tgt_duty.duty_l <= 0) {
      tgt_duty.duty_l = -min_duty;
    }
  } else if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    const auto max_duty = param->sen_ref_p.search_exist.offset_l;
    tgt_duty.duty_r = std::clamp(tgt_duty.duty_r, -max_duty, max_duty);
    tgt_duty.duty_l = std::clamp(tgt_duty.duty_l, -max_duty, max_duty);
  }
  const auto max_duty = param->max_duty;
  // is Numerical
  if (!isfinite(tgt_duty.duty_r)) {
    tgt_duty.duty_r = 0;
  }
  if (!isfinite(tgt_duty.duty_l)) {
    tgt_duty.duty_l = 0;
  }

  const float prev_r = tgt_duty.duty_r;
  const float prev_l = tgt_duty.duty_l;
  tgt_duty.duty_r = std::clamp(tgt_duty.duty_r, -max_duty, max_duty);
  tgt_duty.duty_l = std::clamp(tgt_duty.duty_l, -max_duty, max_duty);

  if (prev_r != tgt_duty.duty_r || prev_l != tgt_duty.duty_l) {
    ee->aw_log.sat_flag = 1.0f;
  } else {
    ee->aw_log.sat_flag = 0.0f;
  }
}

void PlanningTask::clear_ctrl_val() {
  duty_c = duty_c2 = duty_roll = duty_front_ctrl_roll_keep = duty_roll_ang = 0;
  ee->v.error_i = 0;
  ee->v.error_d = 0;
  ee->v.error_dd = 0;
  ee->dist.error_i = 0;
  ee->dist.error_d = 0;
  ee->dist.error_dd = 0;
  ee->w.error_i = 0;
  ee->w.error_d = 0;
  ee->w.error_dd = 0;
  ee->ang.error_i = 0;
  ee->ang.error_d = 0;
  ee->ang.error_dd = 0;
  ee->ang.i_slow = 0;
  ee->ang.i_bias = 0;
  ee->sen.error_i = 0;
  ee->sen.error_d = 0;
  ee->sen.error_dd = 0;
  ee->sen_dia.error_i = 0;
  ee->sen_dia.error_d = 0;
  ee->sen_dia.error_dd = 0;
  tgt_duty.duty_r = tgt_duty.duty_l = 0;
  ee->v_log.gain_zz = 0;
  ee->v_log.gain_z = 0;
  ee->dist_log.gain_zz = 0;
  ee->dist_log.gain_z = 0;
  ee->w_log.gain_zz = 0;
  ee->w_log.gain_z = 0;
  ee->ang_log.gain_zz = 0;
  ee->ang_log.gain_z = 0;
  ee->sen_log.gain_z = 0;
  ee->sen_log.gain_zz = 0;

  ee->v_l_log.gain_zz = 0;
  ee->v_l_log.gain_z = 0;
  ee->v_r_log.gain_zz = 0;
  ee->v_r_log.gain_z = 0;

  tgt_val->global_pos.ang = 0;
  tgt_val->global_pos.img_ang = 0;
  tgt_val->global_pos.dist = 0;
  tgt_val->global_pos.img_dist = 0;
  ee->v_val.p_val = 0;
}

void PlanningTask::set_next_duty(float duty_l, float duty_r,
                                 float duty_suction) {
  if (motor_en) {
    // change_pwm_freq(duty_l, duty_r);
  }
  if (suction_en) {
    float duty_suction_in = 0;

    if (tgt_val->tgt_in.tgt_dist > 60 &&
        (tgt_val->ego_in.state == 0 || tgt_val->ego_in.state == 1) &&
        tgt_val->motion_type == MotionType::STRAIGHT) {
      duty_suction_in =
          100.0 * tgt_duty.duty_suction_low / sensing_result->ego.batt_kf;
      // 100.0f * tgt_duty.duty_suction_low / sensing_result->ego.battery_raw;
    } else {
      duty_suction_in =
          100.0 * tgt_duty.duty_suction / sensing_result->ego.batt_kf;
      // 100.0f * tgt_duty.duty_suction / sensing_result->ego.battery_raw;
    }
    if (duty_suction_in > 100) {
      duty_suction_in = 100.0f;
    }
    gain_cnt += 1.0f;
    if (gain_cnt > suction_gain) {
      gain_cnt = suction_gain;
    }
    duty_suction_in = duty_suction_in * gain_cnt / suction_gain;
    if (duty_suction_in > 100) {
      duty_suction_in = 100.0f;
    }
    // is Numerical?
    if (!isfinite(duty_suction_in)) {
      duty_suction_in = 0;
    }

    tgt_val->duty_suction = duty_suction_in;
    // printf("duty_suction_in: %f\n", duty_suction_in);
    // mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    // mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_suction_in);
    // mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
    //                     MCPWM_DUTY_MODE_0);
  } else {
    tgt_val->duty_suction = 0;
  }
}
