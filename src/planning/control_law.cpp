#include "planning/control_law.hpp"
#include "define.hpp"
#include <algorithm>
#include <cmath>

void ControlLaw::init(MotorActuator *motor, SensorProcessor *sensor,
                      TrajectoryGenerator *trj, EgoEstimator *ego) {
  motor_ = motor;
  sensor_ = sensor;
  trj_ = trj;
  ego_ = ego;
  ee = std::make_shared<pid_error_entity_t>();
}

void ControlLaw::calc(std::shared_ptr<motion_tgt_val_t> tgt_val,
                      std::shared_ptr<sensing_result_entity_t> sensing_result,
                      std::shared_ptr<input_param_t> param, bool motor_en,
                      bool suction_en, bool search_mode, unsigned char w_reset,
                      float last_tgt_angle, float dt) {
  tgt_val_ = tgt_val;
  sensing_result_ = sensing_result;
  param_ = param;
  motor_en_ = motor_en;
  suction_en_ = suction_en;
  search_mode_ = search_mode;
  w_reset_ = w_reset;
  last_tgt_angle_ = last_tgt_angle;
  dt_ = dt;

  // axel degenerate gain (pre-calc before sensor PID, mirrors tick() logic)
  float axel_degenerate_gain = 1.0f;
  diff_old = diff;
  if (!search_mode_ && tgt_val_->motion_type == MotionType::STRAIGHT) {
    if (sensor_->axel_degenerate_x.size() > 0 &&
        tgt_val_->nmr.sct == SensorCtrlType::Straight) {
      SensingControlType type = SensingControlType::None;
      diff = ABS(check_sen_error(type));
      if (diff == 0)
        diff = diff_old;
      axel_degenerate_gain = sensor_->interp1d(
          sensor_->axel_degenerate_x, sensor_->axel_degenerate_y, diff, false);
      tgt_val_->tgt_in.axel_degenerate_gain =
          (1 - param_->sensor_gain.front2.b) *
              tgt_val_->tgt_in.axel_degenerate_gain +
          param_->sensor_gain.front2.b * axel_degenerate_gain;
    } else if (sensor_->axel_degenerate_dia_x.size() > 0 &&
               tgt_val_->nmr.sct == SensorCtrlType::Dia) {
      SensingControlType type = SensingControlType::None;
      diff = ABS(check_sen_error_dia(type));
      if (diff == 0)
        diff = diff_old;
      axel_degenerate_gain =
          sensor_->interp1d(sensor_->axel_degenerate_dia_x,
                            sensor_->axel_degenerate_dia_y, diff, false);
      if (axel_degenerate_gain < 0 &&
          tgt_val_->tgt_in.end_v > tgt_val_->ego_in.v) {
        tgt_val_->tgt_in.axel_degenerate_gain = 0.01f;
      }
      tgt_val_->tgt_in.axel_degenerate_gain =
          (1 - param_->sensor_gain.front2.b) *
              tgt_val_->tgt_in.axel_degenerate_gain +
          param_->sensor_gain.front2.b * axel_degenerate_gain;
    }
  } else {
    diff = diff_old = 0;
    tgt_val_->tgt_in.axel_degenerate_gain = axel_degenerate_gain;
  }

  calc_tgt_duty();
  check_fail_safe();
  set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);
}

// ============================================================

void ControlLaw::calc_tgt_duty() {
  const unsigned char reset_req = motor_en_ ? 1 : 0;
  const unsigned char enable = 1;
  duty_sen = 0;
  sen_ang = 0;

  ee->s_val.p = ee->s_val.i = ee->s_val.d = 0;
  ee->s_val.p_val = ee->s_val.i_val = ee->s_val.d_val = 0;
  ee->s_val.z = ee->s_val.zz = 0;

  if (tgt_val_->nmr.sct == SensorCtrlType::Straight) {
    duty_sen = calc_sensor_pid();
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else if (tgt_val_->nmr.sct == SensorCtrlType::Dia) {
    duty_sen = calc_sensor_pid_dia();
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
  } else if (tgt_val_->nmr.sct == SensorCtrlType::NONE) {
    duty_sen = sen_ang = 0;
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  }
  sensing_result_->ego.duty.sen = duty_sen;
  sensing_result_->ego.duty.sen_ang = sen_ang;

  calc_pid_val();
  calc_pid_val_ang();
  calc_pid_val_ang_vel();
  calc_pid_val_front_ctrl();

  duty_c = 0;
  duty_c2 = 0;
  duty_roll = 0;
  duty_front_ctrl_roll_keep = 0;
  duty_roll_ang = 0;
  duty_front_ctrl_trans = 0;
  duty_front_ctrl_roll = 0;
  reset_pid_val();

  if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    calc_front_ctrl_duty();
  } else {
    calc_translational_ctrl();
    calc_angle_velocity_ctrl();
  }
  sensing_result_->ego.duty.sen = duty_sen;

  summation_duty();
  apply_duty_limitter();

  if (tgt_val_->motion_type == MotionType::NONE) {
    tgt_duty.duty_l = tgt_duty.duty_r = 0;
  }
  if (!motor_en_) {
    clear_ctrl_val();
  }

  sensing_result_->ego.duty.duty_r = tgt_duty.duty_r;
  sensing_result_->ego.duty.duty_l = tgt_duty.duty_l;

  sensing_result_->ego.duty.ff_duty_front = trj_->mpc_next_ego.ff_duty_front;
  sensing_result_->ego.duty.ff_duty_roll = trj_->mpc_next_ego.ff_duty_roll;
  sensing_result_->ego.duty.ff_duty_rpm_r = trj_->mpc_next_ego.ff_duty_rpm_r;
  sensing_result_->ego.duty.ff_duty_rpm_l = trj_->mpc_next_ego.ff_duty_rpm_l;
}

void ControlLaw::calc_translational_ctrl() {
  const float dt = param_->dt;
  if (!motor_en_) {
    const unsigned char reset = 0;
    vel_pid.step(&ee->v.error_p, &param_->motor_pid.p, &param_->motor_pid.i,
                 &param_->motor_pid.d, &reset, &dt, &duty_c);
    set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
                 param_->motor_pid.p * ee->v.error_p,
                 vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0,
                 0);
  } else {
    if (tgt_val_->motion_type == MotionType::STRAIGHT ||
        tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
      if (last_accl > 0 && tgt_val_->ego_in.accl < 0) {
        ee->v.error_i *= param_->ff_front_gain_decel;
      }
    }

    auto v_error_i = ee->v.error_i;
    if (param_->motor_pid2.antiwindup) {
      if ((v_error_i * ee->v.error_p) < 0 &&
          ABS(ee->v.error_p) > param_->motor_pid2.windup_dead_bind) {
        v_error_i *= param_->motor_pid2.windup_gain;
      }
    }

    const auto diff_dist =
        tgt_val_->ego_in.img_dist - sensing_result_->ego.dist_kf;
    auto kp_gain = param_->motor_pid2.p * ee->v.error_p;
    auto ki_gain = param_->motor_pid2.i * v_error_i;
    auto kb_gain = param_->motor_pid2.b * diff_dist;
    auto kd_gain = param_->motor_pid2.d * ee->v_kf.error_d;
    duty_c = kp_gain + ki_gain + kb_gain + kd_gain;

    set_ctrl_val(ee->v_val, ee->v.error_p, v_error_i, diff_dist, ee->v.error_d,
                 kp_gain, ki_gain, kb_gain, kd_gain, ee->v_log.gain_zz,
                 ee->v_log.gain_z);
  }
  if (w_reset_ == 0 || !motor_en_) {
    ee->w.error_i = ee->w.error_d = 0;
    ee->w_log.gain_z = ee->w_log.gain_zz = 0;
  }
  last_accl = tgt_val_->ego_in.accl;
}

float ControlLaw::calc_sensor_pid() {
  float duty = 0;
  SensingControlType type = SensingControlType::None;
  ee->sen.error_i += ee->sen.error_p;
  ee->sen.error_d = ee->sen.error_p;
  ee->sen.error_p = check_sen_error(type);
  if (search_mode_) {
    if (ee->sen.error_p > param_->search_sen_ctrl_limitter) {
      ee->sen.error_p = param_->search_sen_ctrl_limitter;
    } else if (ee->sen.error_p < -param_->search_sen_ctrl_limitter) {
      ee->sen.error_p = -param_->search_sen_ctrl_limitter;
    }
  }
  ee->sen.error_d = ee->sen.error_p - ee->sen.error_d;

  if (search_mode_) {
    if (ee->sen.error_p != 0) {
      duty = param_->str_ang_pid.p * ee->sen.error_p -
             param_->str_ang_pid.i * ee->sen.error_d;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param_->str_ang_pid.p * ee->sen.error_p, 0, 0,
                   -param_->str_ang_pid.d * ee->sen.error_d,
                   ee->sen_log.gain_zz, ee->sen_log.gain_z);
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
      duty = param_->str_ang_pid.b * ee->sen.error_p -
             param_->str_ang_pid.d * ee->sen.error_d;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param_->str_ang_pid.b * ee->sen.error_p, 0, 0,
                   -param_->str_ang_pid.d * ee->sen.error_d,
                   ee->sen_log.gain_zz, ee->sen_log.gain_z);
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, ee->sen_log.gain_zz,
                   ee->sen_log.gain_z);
    }
  }

  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::Wall) {
    limit = sensor_->interp1d(sensor_->sensor_deg_limitter_v,
                              sensor_->sensor_deg_limitter_str,
                              tgt_val_->ego_in.v, false);
  } else if (type == SensingControlType::Piller) {
    limit = sensor_->interp1d(sensor_->sensor_deg_limitter_v,
                              sensor_->sensor_deg_limitter_piller,
                              tgt_val_->ego_in.v, false);
  }
  duty = std::clamp(duty, -limit, limit);
  if (!search_mode_) {
    if (tgt_val_->motion_type == MotionType::WALL_OFF ||
        tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
      duty = std::clamp(duty, -param_->angle_pid.b, param_->angle_pid.b);
    }
  }
  return duty;
}

float ControlLaw::calc_sensor_pid_dia() {
  float duty = 0;
  SensingControlType type = SensingControlType::None;
  ee->sen_dia.error_i += ee->sen_dia.error_p;
  ee->sen_dia.error_d = ee->sen_dia.error_p;
  ee->sen_dia.error_p = check_sen_error_dia(type);
  ee->sen_dia.error_d = ee->sen_dia.error_p - ee->sen_dia.error_d;

  if (type == SensingControlType::DiaPiller && ee->sen_dia.error_p != 0) {
    duty = param_->sensor_pid_dia.p * ee->sen_dia.error_p -
           param_->sensor_pid_dia.d * sensing_result_->ego.w_kf;
    const float gain = 0.1f;
    set_ctrl_val(ee->s_val, ee->sen_dia.error_p * gain, 0, 0,
                 ee->sen_dia.error_d * gain,
                 param_->sensor_pid_dia.p * ee->sen_dia.error_p * gain,
                 param_->sensor_pid_dia.i * ee->sen_dia.error_i * gain, 0,
                 param_->sensor_pid_dia.d * ee->sen_dia.error_d * gain, 0, 0);
  } else {
    duty = 0;
    set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::DiaPiller) {
    limit = sensor_->interp1d(sensor_->sensor_deg_limitter_v,
                              sensor_->sensor_deg_limitter_dia,
                              tgt_val_->ego_in.v, false);
  }
  duty = std::clamp(duty, -limit, limit);
  return duty;
}

float ControlLaw::check_sen_error(SensingControlType &type) {
  const auto se = sensing_result_;
  const auto prm = param_;
  float error = 0;
  int check = 0;
  float dist_mod = (int)(tgt_val_->ego_in.dist / param_->dist_mod_num);
  float tmp_dist = tgt_val_->ego_in.dist - param_->dist_mod_num * dist_mod;

  bool expand_right = false;
  bool expand_left = false;

  auto exist_right45 = prm->sen_ref_p.normal.exist.right45;
  auto exist_left45 = prm->sen_ref_p.normal.exist.left45;

  auto wall_th =
      search_mode_
          ? sensor_->interp1d(param_->clear_dist_ragne_dist_list,
                              param_->clear_dist_ragne_th_list, tmp_dist, false)
          : std::min(exist_left45, exist_right45);

  auto exist_right45_expand = wall_th;
  auto exist_left45_expand = wall_th;

  float val_left = 1000;
  float val_right = 1000;

  bool range_check_right =
      (1 < se->ego.right45_dist) && (se->ego.right45_dist < exist_right45);
  bool range_check_left =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45);

  bool dist_check_right = ABS(tgt_val_->global_pos.dist -
                              right_keep.star_dist) > prm->right_keep_dist_th;
  bool dist_check_left = ABS(tgt_val_->global_pos.dist - left_keep.star_dist) >
                         prm->left_keep_dist_th;

  bool check_diff_right =
      ABS(se->ego.right45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_r;
  bool check_diff_left =
      ABS(se->ego.left45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_l;

  if (!search_mode_) {
    if (tgt_val_->motion_type == MotionType::WALL_OFF ||
        tgt_val_->motion_type == MotionType::SLA_FRONT_STR) {
      check_diff_right = (se->ego.right45_dist_diff < 0)
                             ? ABS(se->ego.right45_dist_diff) <
                                   prm->sen_ref_p.normal.ref.kireme_r_wall_off2
                             : ABS(se->ego.right45_dist_diff) <
                                   prm->sen_ref_p.normal.ref.kireme_r_wall_off;
      check_diff_left = (se->ego.left45_dist_diff < 0)
                            ? ABS(se->ego.left45_dist_diff) <
                                  prm->sen_ref_p.normal.ref.kireme_l_wall_off2
                            : ABS(se->ego.left45_dist_diff) <
                                  prm->sen_ref_p.normal.ref.kireme_l_wall_off;
    } else {
      check_diff_right = ABS(se->ego.right45_dist_diff) <
                         prm->sen_ref_p.normal.ref.kireme_r_fast;
      check_diff_left = ABS(se->ego.left45_dist_diff) <
                        prm->sen_ref_p.normal.ref.kireme_l_fast;
    }
  }

  bool check_front_left =
      (10 < se->ego.left90_mid_dist) &&
      (se->ego.left90_mid_dist < prm->sen_ref_p.normal.exist.front);
  bool check_front_right =
      (10 < se->ego.right90_mid_dist) &&
      (se->ego.right90_mid_dist < prm->sen_ref_p.normal.exist.front);

  if (!check_diff_right)
    enable_expand_right = false;
  if (!check_diff_left)
    enable_expand_left = false;

  if (search_mode_ && tgt_val_->tgt_in.tgt_dist > 80 &&
      tgt_val_->tgt_in.tgt_dist < 100 &&
      tgt_val_->motion_type == MotionType::STRAIGHT) {
    expand_right = (10 < se->ego.right45_dist) &&
                   (se->ego.right45_dist < prm->sen_ref_p.search_exist.right45);
    expand_left = (10 < se->ego.left45_dist) &&
                  (se->ego.left45_dist < prm->sen_ref_p.search_exist.left45);
  } else {
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

  if (!(check_front_left && check_front_right)) {
    const bool is_wall_off_mode =
        (tgt_val_->motion_type == MotionType::WALL_OFF);
    if (!is_wall_off_mode) {
      check_left_sensor_error(error, check, range_check_left, dist_check_left,
                              check_diff_left, expand_left,
                              range_check_left_expand);
      check_right_sensor_error(error, check, range_check_right,
                               dist_check_right, check_diff_right, expand_right,
                               range_check_right_expand);
    } else {
      if (tgt_val_->motion_dir == MotionDirection::LEFT) {
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

  if (enable_expand_left && !enable_expand_right && check_diff_right &&
      ((1 < se->ego.right45_dist) &&
       (se->ego.right45_dist < wall_th + 0.75f))) {
    enable_expand_right = true;
  } else if (!enable_expand_left && enable_expand_right && check_diff_left &&
             ((1 < se->ego.left45_dist) &&
              (se->ego.left45_dist < wall_th + 0.75f))) {
    enable_expand_left = true;
  }

  if (check == 0) {
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;

    bool right_check = false;
    bool left_check = false;

    const bool range_check_passed_right =
        (prm->sen_ref_p.normal2.ref.kireme_r < se->sen.r45.sensor_dist) &&
        (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.exist.right45) &&
        (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;
    const bool range_check_passed_left =
        (prm->sen_ref_p.normal2.ref.kireme_l < se->sen.l45.sensor_dist) &&
        (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.exist.left45) &&
        (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;

    const bool exist_right45_b =
        se->ego.right45_dist < prm->sen_ref_p.search_exist.right45;
    const bool exist_left45_b =
        se->ego.left45_dist < prm->sen_ref_p.search_exist.left45;

    if (!(check_front_left && check_front_right)) {
      if (range_check_passed_right && !exist_left45_b) {
        error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
        check++;
        right_check = true;
      }
      if (range_check_passed_left && !exist_right45_b) {
        error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
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
            !exist_left45_b) {
          error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
          check++;
        } else if (range_check_passed_left_near &&
                   !range_check_passed_right_near && !exist_right45_b) {
          error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
          check++;
        }
      }
      if (check != 0 && !(tgt_val_->motion_type == MotionType::SLA_FRONT_STR)) {
        type = SensingControlType::Piller;
      }
    }
  } else {
    if (tgt_val_->tgt_in.tgt_dist >= prm->clear_dist_order) {
      if (!(prm->clear_dist_ragne_from <= tmp_dist &&
            tmp_dist <= prm->clear_dist_ragne_to)) {
        if (std::abs(tgt_val_->ego_in.ang - tgt_val_->ego_in.img_ang) <
            prm->clear_angle) {
          if ((tgt_val_->tgt_in.tgt_dist - tgt_val_->ego_in.dist) >
              (prm->cell / 2)) {
            tgt_val_->global_pos.ang = tgt_val_->global_pos.img_ang;
            ee->w.error_i = ee->w.error_d = ee->w.error_dd = 0;
            ee->w_kf.error_i = ee->w_kf.error_d = ee->w_kf.error_dd = 0;
            ee->ang.error_i = ee->ang.error_d = ee->ang.error_dd = 0;
            ee->ang.i_slow = ee->ang.i_bias = 0;
            w_reset_ = 0;
          }
        }
      }
    }
  }

  if (check == 2)
    return error;
  if (check == 1)
    return error * 2;
  ee->sen.error_i = 0;
  ee->sen_log.gain_zz = 0;
  ee->sen_log.gain_z = 0;
  return 0;
}

float ControlLaw::check_sen_error_dia(SensingControlType &type) {
  float error = 0;
  int check = 0;
  const auto se = sensing_result_;

  if (tgt_val_->tgt_in.tgt_dist > param_->sen_ctrl_front_th &&
      (tgt_val_->tgt_in.tgt_dist - tgt_val_->ego_in.dist) >
          param_->sen_ctrl_front_diff_th) {
    const bool valid_right90 =
        1 < se->ego.right90_mid_dist &&
        se->ego.right90_mid_dist < param_->sen_ref_p.dia.exist.right90;
    const bool valid_left90 =
        1 < se->ego.left90_mid_dist &&
        se->ego.left90_mid_dist < param_->sen_ref_p.dia.exist.left90;
    const bool valid_right45 =
        1 < se->sen.r45.sensor_dist &&
        se->sen.r45.sensor_dist < param_->sen_ref_p.dia.exist.right45 &&
        se->sen.r45.sensor_dist < se->ego.right45_dist;
    const bool valid_left45 =
        1 < se->sen.l45.sensor_dist &&
        se->sen.l45.sensor_dist < param_->sen_ref_p.dia.exist.left45 &&
        se->sen.l45.sensor_dist < se->ego.left45_dist;

    if (valid_right90) {
      error += param_->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val_->dia_state.right_old =
          param_->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val_->dia_state.right_save = true;
      tgt_val_->dia_state.left_save = false;
      check++;
    }
    if (valid_left90) {
      error -= param_->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val_->dia_state.left_old =
          param_->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val_->dia_state.left_save = true;
      tgt_val_->dia_state.right_save = false;
      check++;
    }
    if (!valid_left90 && !valid_right90 && param_->sensor_gain.front4.a != 0) {
      if (valid_right45) {
        error += param_->sen_ref_p.dia.ref.right45 - se->sen.r45.sensor_dist;
        check++;
      }
      if (valid_left45) {
        error -= param_->sen_ref_p.dia.ref.left45 - se->sen.l45.sensor_dist;
        check++;
      }
      if (!(valid_left45 && valid_right45)) {
        if (tgt_val_->dia_state.right_save) {
          error += tgt_val_->dia_state.right_old;
          check++;
        }
        if (tgt_val_->dia_state.left_save) {
          error -= tgt_val_->dia_state.left_old;
          check++;
        }
      }
    }
  }

  if (check == 0) {
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else {
    type = SensingControlType::DiaPiller;
    if (tgt_val_->tgt_in.tgt_dist >= param_->clear_dist_order) {
      if (std::abs(tgt_val_->ego_in.ang - tgt_val_->ego_in.img_ang) <
          param_->clear_angle) {
        // reserved
      }
    }
  }

  if (check == 2)
    return error;
  if (check == 1)
    return error * 2;
  return 0;
}

void ControlLaw::check_fail_safe() {
  if (!motor_en_) {
    tgt_val_->fss.error = 0;
    return;
  }
  if (ABS(ee->ang.error_p) > param_->fail_check_ang_th) {
    fail_check_ang++;
  } else {
    fail_check_ang = 0;
  }
  if (tgt_val_->motion_type == MotionType::WALL_OFF ||
      tgt_val_->motion_type == MotionType::WALL_OFF_DIA) {
    keep_wall_off_cnt++;
  } else {
    keep_wall_off_cnt = 0;
  }
  if (ABS(ee->v.error_i) > param_->fail_check.v)
    tgt_val_->fss.error = 1;
  if (ABS(ee->w.error_i) > param_->fail_check.w)
    tgt_val_->fss.error = 1;
  if (ABS(ee->ang.error_i) > param_->fail_check.ang)
    tgt_val_->fss.error = 1;
  if (keep_wall_off_cnt > param_->fail_check.wall_off)
    tgt_val_->fss.error = 1;
}

void ControlLaw::calc_pid_val() {
  ee->v.error_dd = ee->v.error_d;
  ee->v_kf.error_dd = ee->v_kf.error_d;
  ee->dist.error_dd = ee->dist.error_d;

  ee->v.error_d = ee->v.error_p;
  ee->v_r.error_d = ee->v_r.error_p;
  ee->v_l.error_d = ee->v_l.error_p;
  ee->v_kf.error_d = ee->v_kf.error_p;
  ee->dist.error_d = ee->dist.error_p;

  ee->v.error_p = trj_->v_cmd - sensing_result_->ego.v_c;
  ee->v_r.error_p = trj_->ideal_v_r - sensing_result_->ego.v_r;
  ee->v_l.error_p = trj_->ideal_v_l - sensing_result_->ego.v_l;
  ee->v_kf.error_p = trj_->v_cmd - sensing_result_->ego.v_kf;
  ee->w_kf.error_p = trj_->w_cmd - sensing_result_->ego.w_kf;

  ee->dist.error_p = tgt_val_->global_pos.img_dist - tgt_val_->global_pos.dist;
  if (ee->dist.error_p > param_->front_ctrl_error_th) {
    ee->dist.error_p = param_->front_ctrl_error_th;
  } else if (ee->dist.error_p < -param_->front_ctrl_error_th) {
    ee->dist.error_p = -param_->front_ctrl_error_th;
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
  if (tgt_val_->motion_type != MotionType::FRONT_CTRL) {
    ee->dist.error_i += ee->dist.error_p;
  }
  ee->v_l.error_i += ee->v_l.error_p;
  ee->v_r.error_i += ee->v_r.error_p;

  tgt_val_->v_error = ee->v.error_i;
}

void ControlLaw::calc_pid_val_ang() {
  const auto tgt = tgt_val_;
  const auto se = sensing_result_;

  ee->ang.error_dd = ee->ang.error_d;
  ee->ang.error_d = ee->ang.error_p;

  float offset = 0;
  if (tgt->motion_type != MotionType::FRONT_CTRL) {
    offset += duty_sen;
  }

  ee->ang.error_p = (tgt->ego_in.img_ang + offset) - se->ego.ang_kf;
  ee->ang.error_d = ee->ang.error_p - ee->ang.error_d;
  ee->ang.error_dd = ee->ang.error_d - ee->ang.error_dd;
  ee->ang.error_i += ee->ang.error_p;

  if (!(tgt->motion_type == MotionType::STRAIGHT) ||
      tgt->motion_type == MotionType::SLA_FRONT_STR ||
      tgt->motion_type == MotionType::SLA_BACK_STR ||
      tgt->motion_type == MotionType::WALL_OFF ||
      tgt->motion_type == MotionType::WALL_OFF_DIA) {
    ee->ang.error_d = ee->ang.error_dd = ee->ang.error_i = 0;
  }

  float ang_error_i = ee->ang.error_i;
  if (param_->angle_pid.antiwindup) {
    if (ang_error_i * ee->ang.error_p < 0 &&
        ABS(ee->ang.error_p) > param_->angle_pid.windup_dead_bind) {
      ang_error_i *= param_->angle_pid.windup_gain;
    }
  }

  duty_roll_ang = param_->angle_pid.p * ee->ang.error_p +
                  param_->angle_pid.i * ang_error_i +
                  param_->angle_pid.d * ee->ang.error_d;

  calc_angle_i_bias();

  set_ctrl_val(ee->ang_val, ee->ang.error_p, ee->ang.error_i, duty_sen,
               ee->ang.error_d, param_->angle_pid.p * ee->ang.error_p,
               param_->angle_pid.i * ang_error_i,
               param_->gyro_pid.c * ee->ang.i_bias,
               param_->angle_pid.d * ee->ang.error_d, 0, 0);
}

void ControlLaw::calc_pid_val_ang_vel() {
  const auto tgt = tgt_val_;
  const auto se = sensing_result_;

  ee->w.error_dd = ee->w.error_d;
  ee->w_kf.error_dd = ee->w_kf.error_d;
  ee->w.error_d = ee->w.error_p;
  ee->w_kf.error_d = ee->w_kf.error_p;

  float offset = 0;
  if (param_->torque_mode == 2) {
    if (!(tgt->motion_type == MotionType::PIVOT ||
          tgt->motion_type == MotionType::FRONT_CTRL)) {
      offset += duty_roll_ang;
    }
  }
  ee->aw_log.duty_roll_before = (tgt->ego_in.w + offset);

  ee->w.error_p = (tgt->ego_in.w + offset) - se->ego.w_lp;
  ee->w_kf.error_p = (tgt_val_->ego_in.w + offset) - se->ego.w_kf;

  ee->w.error_d = ee->w.error_p - ee->w.error_d;
  ee->w_kf.error_d = ee->w_kf.error_p - ee->w_kf.error_d;

  ee->w.error_dd = ee->w.error_d - ee->w.error_dd;
  ee->w_kf.error_dd = ee->w_kf.error_d - ee->w_kf.error_dd;

  ee->w.error_i += ee->w.error_p;
  ee->w_kf.error_i += ee->w_kf.error_p;

  tgt_val_->w_error = ee->w.error_i;
}

void ControlLaw::calc_pid_val_front_ctrl() {
  const auto se = sensing_result_;
  if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->w.error_i = ee->w.error_d = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    if (se->ego.front_dist < param_->cell) {
      ee->dist.error_p =
          se->ego.front_dist - param_->sen_ref_p.search_exist.front_ctrl;
      ee->ang.error_p = (se->ego.right90_dist - se->ego.left90_dist) / 2 -
                        param_->sen_ref_p.search_exist.kireme_r;
      ee->dist.error_i += ee->dist.error_p;
    } else {
      ee->dist.error_p = ee->dist.error_i = ee->dist.error_d = 0;
      ee->ang.error_p = ee->ang.error_i = ee->ang.error_d = 0;
    }
  }
}

void ControlLaw::reset_pid_val() {
  if (tgt_val_->motion_type == MotionType::FRONT_CTRL || !motor_en_ ||
      tgt_val_->motion_type == MotionType::NONE) {
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
  ee->v_val.p = ee->v_val.i = ee->v_val.d = 0;
  ee->w_val.p = ee->w_val.i = ee->w_val.d = 0;
  ee->v_val.p_val = ee->v_val.i_val = ee->v_val.d_val = 0;
  ee->w_val.p_val = ee->w_val.i_val = ee->w_val.d_val = 0;
  ee->v_val.z = ee->v_val.zz = 0;
  ee->w_val.z = ee->w_val.zz = 0;
}

void ControlLaw::calc_angle_i_bias() {
  if (tgt_val_->motion_type == MotionType::NONE ||
      tgt_val_->motion_type == MotionType::PIVOT ||
      tgt_val_->motion_type == MotionType::PIVOT_PRE ||
      tgt_val_->motion_type == MotionType::PIVOT_PRE2 ||
      tgt_val_->motion_type == MotionType::PIVOT_AFTER ||
      tgt_val_->motion_type == MotionType::PIVOT_OFFSET ||
      tgt_val_->motion_type == MotionType::BACK_STRAIGHT ||
      tgt_val_->motion_type == MotionType::READY ||
      tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    ee->ang.i_bias = 0;
  } else {
    ee->ang.i_bias = tgt_val_->ego_in.img_ang - ego_->kim.theta;
  }
  if (search_mode_) {
    ee->ang.i_bias = 0;
  }
}

void ControlLaw::check_left_sensor_error(float &error, int &check,
                                         bool range_check_left,
                                         bool dist_check_left,
                                         bool check_diff_left, bool expand_left,
                                         bool range_check_left_expand) {
  const auto se = sensing_result_;
  const auto prm = param_;
  const bool is_wall_off_mode = (tgt_val_->motion_type == MotionType::WALL_OFF);

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
      error -= param_->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    }
    check++;
  } else if (expand_left && range_check_left_expand) {
    if (dist_check_left && check_diff_left) {
      enable_expand_left = true;
      error -= param_->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    }
    check++;
  } else {
    left_keep.star_dist = tgt_val_->global_pos.dist;
  }
}

void ControlLaw::check_right_sensor_error(
    float &error, int &check, bool range_check_right, bool dist_check_right,
    bool check_diff_right, bool expand_right, bool range_check_right_expand) {
  const auto se = sensing_result_;
  const auto prm = param_;
  const bool is_wall_off_mode = (tgt_val_->motion_type == MotionType::WALL_OFF);

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
      error += param_->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      enable_expand_right = true;
    }
    check++;
  } else {
    right_keep.star_dist = tgt_val_->global_pos.dist;
    enable_expand_right = false;
  }
}

void ControlLaw::set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                              float error_i2, float error_d, float val_p,
                              float val_i, float val_i2, float val_d, float zz,
                              float z) {
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

void ControlLaw::calc_front_ctrl_duty() {
  const unsigned char reset = 0;
  param_->motor_pid.i = param_->motor_pid.d = 0;
  vel_pid.step(&ee->v.error_p, &param_->motor_pid.p, &param_->motor_pid.i,
               &param_->motor_pid.d, &reset, &dt_, &duty_c);
  set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
               param_->motor_pid.p * ee->v.error_p,
               vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0, 0);

  ee->w.error_i = ee->w.error_d = 0;
  ee->w_kf.error_i = ee->w_kf.error_d = 0;
  ee->w_log.gain_z = ee->w_log.gain_zz = 0;

  auto diff_ang = 0.0f;
  auto kp_gain = param_->front_ctrl_roll_pid.p * ee->w.error_p;
  auto ki_gain = 0.0f;
  auto kb_gain = 0.0f;
  auto kc_gain = 0.0f;
  auto kd_gain = param_->front_ctrl_roll_pid.d * ee->w_kf.error_d;

  limitter(kp_gain, ki_gain, kb_gain, kd_gain, param_->gyro_pid_gain_limitter);
  duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
              (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt_;

  ee->ang_log.gain_zz = ee->ang_log.gain_z;
  ee->ang_log.gain_z = duty_roll;

  set_ctrl_val(
      ee->w_val, ee->w.error_p, diff_ang, ee->w.error_i, ee->w_kf.error_d,
      param_->gyro_pid.p * ee->w.error_p, param_->gyro_pid.i * diff_ang,
      param_->gyro_pid.b * ee->w.error_i, param_->gyro_pid.d * ee->w_kf.error_d,
      ee->ang_log.gain_zz, ee->ang_log.gain_z);

  sensing_result_->ego.duty.sen = 0;

  duty_front_ctrl_trans =
      param_->front_ctrl_dist_pid.p * ee->dist.error_p +
      param_->front_ctrl_dist_pid.i * ee->dist.error_i +
      param_->front_ctrl_dist_pid.d * sensing_result_->ego.v_c;
  duty_front_ctrl_roll = param_->front_ctrl_angle_pid.p * ee->ang.error_p +
                         param_->front_ctrl_angle_pid.i * ee->ang.error_i +
                         param_->front_ctrl_angle_pid.d * ee->w_kf.error_p;
  duty_front_ctrl_roll_keep =
      param_->front_ctrl_keep_angle_pid.p * ee->ang.error_p +
      param_->front_ctrl_keep_angle_pid.i * ee->ang.error_i +
      param_->front_ctrl_keep_angle_pid.d * ee->w_kf.error_p;
  gyro_pid_windup_histerisis = false;
  gyro_pid_histerisis_i = 0.0f;
}

void ControlLaw::calc_angle_velocity_ctrl() {
  const auto se = sensing_result_;
  if (tgt_val_->motion_type != ee->ang_log.prev_motion_type) {
    ee->ang_log.omega_ref_prev = tgt_val_->ego_in.w;
    ee->ang_log.prev_motion_type = tgt_val_->motion_type;
  }

  if (tgt_val_->motion_type == MotionType::NONE) {
    duty_roll = param_->gyro_pid.p * ee->w.error_p +
                param_->gyro_pid.b * ee->w.error_i +
                param_->gyro_pid.c * ee->w.error_d;
    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;
    set_ctrl_val(ee->w_val, ee->w.error_p, ee->w.error_i, 0, ee->w.error_d,
                 param_->gyro_pid.p * ee->w.error_p,
                 param_->gyro_pid.b * ee->w.error_i, param_->gyro_pid.b * 0,
                 param_->gyro_pid.c * ee->w.error_d, ee->ang_log.gain_zz,
                 ee->ang_log.gain_z);
    gyro_pid_windup_histerisis = false;
    gyro_pid_histerisis_i = 0;
  } else {
    auto diff_ang = (tgt_val_->ego_in.img_ang - sensing_result_->ego.ang_kf);
    auto ang_sum = ee->ang.error_i;
    if (tgt_val_->motion_type == MotionType::SLALOM) {
      diff_ang = 0;
      ang_sum = 0;
    }
    auto w_error_i = ee->w.error_i;
    auto w_error_d = ee->w_kf.error_d;

    ee->aw_log.was_aw = (float)gyro_pid_windup_histerisis;
    ee->aw_log.w_i_base = w_error_i;

    if (param_->gyro_pid.antiwindup) {
      float db = param_->gyro_pid.windup_dead_bind;
      if (duty_sen != 0) {
        db *= param_->gyro_pid.windup_gain;
      }
      if (tgt_val_->motion_type == MotionType::SLALOM &&
          tgt_val_->tgt_in.v_max < 500) {
        db *= param_->gyro_pid.windup_gain;
      }
      if ((w_error_i * ee->w.error_p < 0) &&
          ((ABS(ee->w.error_p) > db) ||
           (gyro_pid_windup_histerisis && ABS(ee->w.error_p) > db * 0.75f))) {
        gyro_pid_histerisis_i += ee->w.error_p;
        w_error_i = gyro_pid_histerisis_i;
        gyro_pid_windup_histerisis = true;
      } else {
        if (gyro_pid_windup_histerisis) {
          w_error_i = ee->w.error_i = ee->ang.i_bias / dt_;
        }
        gyro_pid_windup_histerisis = false;
        gyro_pid_histerisis_i = 0;
      }

      ee->aw_log.w_error_i_raw = w_error_i;
      ee->aw_log.gyro_pid_histerisis_i = gyro_pid_histerisis_i;

      if (tgt_val_->motion_type == MotionType::SLALOM) {
        w_error_i =
            std::clamp(w_error_i * dt_, -ABS(tgt_val_->tgt_in.tgt_angle),
                       ABS(tgt_val_->tgt_in.tgt_angle)) /
            dt_;
      } else if (tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
        w_error_i = std::clamp(w_error_i * dt_, -ABS(last_tgt_angle_),
                               ABS(last_tgt_angle_)) /
                    dt_;
      }
      ee->aw_log.w_error_i_clamped = w_error_i;
    }

    if (!(tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
          tgt_val_->motion_type == MotionType::SLA_BACK_STR ||
          tgt_val_->motion_type == MotionType::PIVOT)) {
      diff_ang = 0;
      ang_sum = 0;
    }

    auto kp_gain = param_->gyro_pid.p * ee->w.error_p;
    auto ki_gain = param_->gyro_pid.i * diff_ang;
    auto kb_gain = param_->gyro_pid.b * w_error_i;
    auto kc_gain = param_->gyro_pid.c * ee->ang.i_bias;
    auto kd_gain = param_->gyro_pid.d * w_error_d;
    limitter(kp_gain, ki_gain, kb_gain, kd_gain,
             param_->gyro_pid_gain_limitter);
    duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
                (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt_;

    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;

    float dt = param_->dt;
    float b = param_->gyro_pid.mpc_b;
    float w_meas = -ee->w.error_p + tgt_val_->ego_in.w;

    float w_pred = mpc_w_prev + (mpc_u_prev + mpc_d_estimated) * b * dt;
    float observer_k = param_->gyro_pid.mpc_observer_k;
    mpc_d_estimated += observer_k * (w_meas - w_pred);
    mpc_w_prev = w_meas;

    if (param_->enable_mpc > 0 && tgt_val_->motion_type == MotionType::SLALOM) {
      // MPC override reserved
    }
    mpc_u_prev = duty_roll;

    set_ctrl_val(ee->w_val, ee->w.error_p, diff_ang, w_error_i, w_error_d,
                 kp_gain, ki_gain, kb_gain, kd_gain, ee->ang_log.gain_zz,
                 ee->ang_log.gain_z);
  }
}

void ControlLaw::summation_duty() {
  auto ff_front = trj_->mpc_next_ego.ff_duty_front;
  auto ff_roll = trj_->mpc_next_ego.ff_duty_roll;
  const auto se = sensing_result_;

  if (tgt_val_->motion_type == MotionType::WALL_OFF ||
      tgt_val_->motion_type == MotionType::WALL_OFF_DIA) {
    ff_front = param_->ff_roll_gain_before * ff_front;
    trj_->mpc_next_ego.ff_duty_front = ff_front;
  }
  if (tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
    ff_front = param_->ff_front_gain_14 * ff_front;
    trj_->mpc_next_ego.ff_duty_front = ff_front;
  }
  if (tgt_val_->motion_type == MotionType::SLALOM) {
    if (tgt_val_->ego_in.sla_param.base_alpha > 0) {
      ff_roll = (tgt_val_->ego_in.alpha < 0)
                    ? param_->ff_roll_gain_after * ff_roll
                    : ff_roll;
    } else if (tgt_val_->ego_in.sla_param.base_alpha < 0) {
      ff_roll = (tgt_val_->ego_in.alpha > 0)
                    ? param_->ff_roll_gain_after * ff_roll
                    : ff_roll;
    }
  }
  se->ego.duty.ff_duty_roll = trj_->mpc_next_ego.ff_duty_roll = ff_roll;
  auto ff_duty_r = ff_front + ff_roll + trj_->mpc_next_ego.ff_duty_rpm_r;
  auto ff_duty_l = ff_front - ff_roll + trj_->mpc_next_ego.ff_duty_rpm_l;

  if (param_->FF_keV == 0) {
    ff_duty_l = ff_duty_r = 0;
  }

  if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    tgt_duty.duty_r =
        (duty_c + duty_front_ctrl_trans + duty_roll + duty_front_ctrl_roll +
         duty_front_ctrl_roll_keep + ff_duty_r) /
        se->ego.battery_lp * 100;
    tgt_duty.duty_l =
        (duty_c + duty_front_ctrl_trans - duty_roll - duty_front_ctrl_roll -
         duty_front_ctrl_roll_keep + ff_duty_l) /
        se->ego.battery_lp * 100;
  } else if (param_->torque_mode == 2) {
    auto ff_front2 = trj_->mpc_next_ego.ff_front_torque;
    auto ff_roll2 = trj_->mpc_next_ego.ff_roll_torque;
    auto ff_duty_r2 = trj_->mpc_next_ego.ff_duty_rpm_r;
    auto ff_duty_l2 = trj_->mpc_next_ego.ff_duty_rpm_l;
    auto ff_friction_r = trj_->mpc_next_ego.ff_friction_torque_r;
    auto ff_friction_l = trj_->mpc_next_ego.ff_friction_torque_l;

    if (param_->FF_keV == 0) {
      ff_front2 = ff_roll2 = ff_duty_r2 = ff_duty_l2 = ff_friction_r =
          ff_friction_l = 0;
    }
    float torque_r = ff_front2 + ff_roll2 + duty_c + duty_roll + ff_friction_r;
    float torque_l = ff_front2 - ff_roll2 + duty_c - duty_roll + ff_friction_l;

    const float km_gear = param_->Km * (param_->gear_a / param_->gear_b);
    float req_v_r = torque_r * param_->Resist / km_gear + ff_duty_r2;
    float req_v_l = torque_l * param_->Resist / km_gear + ff_duty_l2;

    tgt_duty.duty_r = req_v_r / se->ego.battery_lp * 100;
    tgt_duty.duty_l = req_v_l / se->ego.battery_lp * 100;
  }
}

void ControlLaw::apply_duty_limitter() {
  if (tgt_val_->motion_type == MotionType::STRAIGHT ||
      tgt_val_->motion_type == MotionType::SLALOM ||
      tgt_val_->motion_type == MotionType::SLA_BACK_STR ||
      tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
      tgt_val_->motion_type == MotionType::PIVOT) {
    const auto min_duty = param_->min_duty;
    if (0 <= tgt_duty.duty_r && tgt_duty.duty_r < min_duty)
      tgt_duty.duty_r = min_duty;
    else if (-min_duty < tgt_duty.duty_r && tgt_duty.duty_r <= 0)
      tgt_duty.duty_r = -min_duty;
    if (0 <= tgt_duty.duty_l && tgt_duty.duty_l < min_duty)
      tgt_duty.duty_l = min_duty;
    else if (-min_duty < tgt_duty.duty_l && tgt_duty.duty_l <= 0)
      tgt_duty.duty_l = -min_duty;
  } else if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    const auto max_duty = param_->sen_ref_p.search_exist.offset_l;
    tgt_duty.duty_r = std::clamp(tgt_duty.duty_r, -max_duty, max_duty);
    tgt_duty.duty_l = std::clamp(tgt_duty.duty_l, -max_duty, max_duty);
  }

  const auto max_duty = param_->max_duty;
  if (!isfinite(tgt_duty.duty_r))
    tgt_duty.duty_r = 0;
  if (!isfinite(tgt_duty.duty_l))
    tgt_duty.duty_l = 0;

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

void ControlLaw::clear_ctrl_val() {
  duty_c = duty_c2 = duty_roll = duty_front_ctrl_roll_keep = duty_roll_ang = 0;
  ee->v.error_i = ee->v.error_d = ee->v.error_dd = 0;
  ee->dist.error_i = ee->dist.error_d = ee->dist.error_dd = 0;
  ee->w.error_i = ee->w.error_d = ee->w.error_dd = 0;
  ee->ang.error_i = ee->ang.error_d = ee->ang.error_dd = 0;
  ee->ang.i_slow = ee->ang.i_bias = 0;
  ee->sen.error_i = ee->sen.error_d = ee->sen.error_dd = 0;
  ee->sen_dia.error_i = ee->sen_dia.error_d = ee->sen_dia.error_dd = 0;
  tgt_duty.duty_r = tgt_duty.duty_l = 0;
  ee->v_log.gain_zz = ee->v_log.gain_z = 0;
  ee->dist_log.gain_zz = ee->dist_log.gain_z = 0;
  ee->w_log.gain_zz = ee->w_log.gain_z = 0;
  ee->ang_log.gain_zz = ee->ang_log.gain_z = 0;
  ee->sen_log.gain_z = ee->sen_log.gain_zz = 0;
  ee->v_l_log.gain_zz = ee->v_l_log.gain_z = 0;
  ee->v_r_log.gain_zz = ee->v_r_log.gain_z = 0;
  tgt_val_->global_pos.ang = 0;
  tgt_val_->global_pos.img_ang = 0;
  tgt_val_->global_pos.dist = 0;
  tgt_val_->global_pos.img_dist = 0;
  ee->v_val.p_val = 0;
}

void ControlLaw::limitter(float &kp, float &ki, float &kb, float &kd,
                          pid_param_t &lim) {
  if (lim.mode == 0)
    return;
  if (kp > lim.p)
    kp = lim.p;
  else if (kp < -lim.p)
    kp = -lim.p;
  if (ki > lim.i)
    ki = lim.i;
  else if (ki < -lim.i)
    ki = -lim.i;
  if (kb > lim.b)
    kb = lim.b;
  else if (kb < -lim.b)
    kb = -lim.b;
  if (kd > lim.d)
    kd = lim.d;
  else if (kd < -lim.d)
    kd = -lim.d;
}

void ControlLaw::set_next_duty(float duty_l, float duty_r, float duty_suction) {
  if (motor_en_) {
    // change_pwm_freq(duty_l, duty_r);  // TODO: motor apply via motor_
  }
  if (suction_en_) {
    float duty_suction_in = 0;
    if (tgt_val_->tgt_in.tgt_dist > 60 &&
        (tgt_val_->ego_in.state == 0 || tgt_val_->ego_in.state == 1) &&
        tgt_val_->motion_type == MotionType::STRAIGHT) {
      duty_suction_in =
          100.0f * tgt_duty.duty_suction_low / sensing_result_->ego.batt_kf;
    } else {
      duty_suction_in =
          100.0f * tgt_duty.duty_suction / sensing_result_->ego.batt_kf;
    }
    if (duty_suction_in > 100.0f)
      duty_suction_in = 100.0f;

    gain_cnt += 1.0f;
    if (gain_cnt > suction_gain)
      gain_cnt = suction_gain;
    duty_suction_in = duty_suction_in * gain_cnt / suction_gain;
    if (duty_suction_in > 100.0f)
      duty_suction_in = 100.0f;
    if (!isfinite(duty_suction_in))
      duty_suction_in = 0;

    tgt_val_->duty_suction = duty_suction_in;
  } else {
    tgt_val_->duty_suction = 0;
  }
}

void ControlLaw::pl_req_activate(motion_tgt_val_t &receive_req) {
  if (receive_req.pl_req.error_gyro_reset == 1) {
    ee->v.error_i = 0;
  }
  if (receive_req.pl_req.error_vel_reset == 1) {
    ee->dist.error_i = 0;
  }
  if (receive_req.pl_req.error_dist_reset == 1) {
    ee->w.error_i = 0;
    ee->w_kf.error_i = 0;
  }
  if (receive_req.pl_req.error_ang_reset == 1) {
    ee->ang.error_i = 0;
    ee->ang.i_slow = 0;
    ee->ang.i_bias = 0;
  }
  if (receive_req.pl_req.error_led_reset == 1) {
    // ee->led.error_i = 0;
  }
  // if (tgt_val->pl_req.log_start == 1) {
  //   log_active = true;
  // }
  // if (tgt_val->pl_req.log_end == 1) {
  //   log_active = false;
  // }
}