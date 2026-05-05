#include "planning/trajectory_generator.hpp"
#include <algorithm>
#include <cmath>

void TrajectoryGenerator::generate(std::shared_ptr<motion_tgt_val_t> tgt_val,
                                    std::shared_ptr<input_param_t>    param,
                                    float last_tgt_angle) {
  auto tmp = tgt_val->ego_in.img_ang;
  tgt_val->ego_in.img_ang += last_tgt_angle;

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
  (void)tmp;
}

void TrajectoryGenerator::calc_kanayama(
    std::shared_ptr<motion_tgt_val_t>        tgt_val,
    std::shared_ptr<sensing_result_entity_t> sensing_result,
    std::shared_ptr<input_param_t>           param,
    EgoEstimator    &ego,
    SensorProcessor &sensor,
    float last_tgt_angle) {

  if (trj_idx_v.size() == 0 || trj_idx_val.size() == 0)
    return;

  const auto idx_val =
      sensor.interp1d(trj_idx_v, trj_idx_val, tgt_val->ego_in.v, false);

  const int idx = std::min(param->trj_length - 1, idx_val);
  const auto se = sensing_result;

  ego.odm.x     = trajectory_points[idx].ideal_px;
  ego.odm.y     = trajectory_points[idx].ideal_py;
  ego.odm.theta = trajectory_points[idx].img_ang;

  float vd = ego.odm.v = trajectory_points[idx].v;
  float wd = ego.odm.w = trajectory_points[idx].w;

  float dx = ego.odm.x - ego.kim.x;
  float dy = ego.odm.y - ego.kim.y;
  if (tgt_val->ego_in.v < 10) {
    dx = dy = 0;
  }

  dx = std::clamp(dx, 0.0f, ABS(dx));

  float d_theta = ego.odm.theta - (se->ego.ang_kf + last_tgt_angle);
  float e_theta = d_theta;
  const float cos_theta = std::cos(se->ego.ang_kf);
  const float sin_theta = std::sin(se->ego.ang_kf);
  if (tgt_val->ego_in.v < 10) {
    e_theta = 0;
  }
  const float ex = cos_theta * dx + sin_theta * dy;
  const float ey = -sin_theta * dx + cos_theta * dy;

  const float cos_e_theta = std::cos(e_theta);
  const float sin_e_theta = std::sin(e_theta);

  const float kx      = param->kanayama.kx;
  const float ky      = param->kanayama.ky;
  const float k_theta = param->kanayama.k_theta;

  sensing_result->ego.knym_v = vd * cos_e_theta + kx * ex;
  sensing_result->ego.knym_w = wd + vd * (ky * ey + k_theta * sin_e_theta);
  sensing_result->ego.odm_x     = ego.odm.x;
  sensing_result->ego.odm_y     = ego.odm.y;
  sensing_result->ego.odm_theta = ego.odm.theta;
  sensing_result->ego.kim_x     = ego.kim.x;
  sensing_result->ego.kim_y     = ego.kim.y;
  sensing_result->ego.kim_theta = ego.kim.theta;

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

void TrajectoryGenerator::copy_tgt(
    std::shared_ptr<motion_tgt_val_t>        tgt_val,
    std::shared_ptr<sensing_result_entity_t> sensing_result,
    std::shared_ptr<input_param_t>           param,
    float dt) {

  const auto se = sensing_result;
  tgt_val->ego_in.accl            = mpc_next_ego.accl;
  tgt_val->ego_in.alpha           = mpc_next_ego.alpha;
  tgt_val->ego_in.pivot_state     = mpc_next_ego.pivot_state;
  tgt_val->ego_in.sla_param       = mpc_next_ego.sla_param;
  tgt_val->ego_in.state           = mpc_next_ego.state;
  tgt_val->ego_in.decel_delay_cnt = mpc_next_ego.decel_delay_cnt;

  const auto tmp_v = tgt_val->ego_in.v;
  tgt_val->ego_in.v   = mpc_next_ego.v;
  tgt_val->ego_in.v_l = mpc_next_ego.v - mpc_next_ego.w * param->tire_tread / 2;
  tgt_val->ego_in.v_r = mpc_next_ego.v + mpc_next_ego.w * param->tire_tread / 2;
  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.v < 10) {
      tgt_val->ego_in.v = tmp_v;
    }
    se->sen.r45.sensor_dist   = 0;
    se->sen.l45.sensor_dist   = 0;
    se->sen.r45_2.sensor_dist = 0;
    se->sen.l45_2.sensor_dist = 0;
    se->sen.r45_3.sensor_dist = 0;
    se->sen.l45_3.sensor_dist = 0;
  }
  tgt_val->ego_in.w                    = mpc_next_ego.w;
  tgt_val->ego_in.sla_param.state      = mpc_next_ego.sla_param.state;
  tgt_val->ego_in.sla_param.counter    = mpc_next_ego.sla_param.counter;
  tgt_val->ego_in.sla_param.state      = mpc_next_ego.sla_param.state;
  sensing_result->img_ang_z            = tgt_val->ego_in.img_ang;
  tgt_val->ego_in.img_ang              = mpc_next_ego.img_ang;
  tgt_val->ego_in.img_dist             = mpc_next_ego.img_dist;

  tgt_val->global_pos.img_ang  += mpc_next_ego.w * dt;

  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.pivot_state == 3) {
      tgt_val->global_pos.img_ang = //
          tgt_val->ego_in.img_ang = tgt_val->tgt_in.tgt_angle;
    }
  }

  tgt_val->global_pos.img_dist += mpc_next_ego.v * dt;

  tgt_val->ego_in.slip_point.slip_angle = mpc_next_ego.slip_point.slip_angle;
  tgt_val->ego_in.cnt_delay_accl_ratio  = mpc_next_ego.cnt_delay_accl_ratio;
  tgt_val->ego_in.cnt_delay_decel_ratio = mpc_next_ego.cnt_delay_decel_ratio;

  tgt_val->ego_in.slip.beta = mpc_next_ego.slip.beta;
  tgt_val->ego_in.slip.accl = mpc_next_ego.slip.accl;
  tgt_val->ego_in.slip.v    = mpc_next_ego.slip.v;
  tgt_val->ego_in.slip.vx   = mpc_next_ego.slip.vx;
  tgt_val->ego_in.slip.vy   = mpc_next_ego.slip.vy;

  ideal_v_r = tgt_val->ego_in.v - tgt_val->ego_in.w * param->tire_tread / 2;
  ideal_v_l = tgt_val->ego_in.v + tgt_val->ego_in.w * param->tire_tread / 2;

  tgt_val->ego_in.ideal_px = mpc_next_ego.ideal_px;
  tgt_val->ego_in.ideal_py = mpc_next_ego.ideal_py;
}
