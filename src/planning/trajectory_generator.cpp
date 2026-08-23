#include "planning/trajectory_generator.hpp"
#include "define.hpp" // SUCTION_ESC_PULSE_MIN_US
#include <algorithm>
#include <cmath>

void TrajectoryGenerator::init(std::shared_ptr<motion_tgt_val_t> tgt_val,
          std::shared_ptr<input_param_t> param,
          std::shared_ptr<sensing_result_entity_t> sensing_result) {
  this->tgt_val = tgt_val;
  this->param = param;
  this->se = sensing_result;
}

void TrajectoryGenerator::setup() {
  if (param->trj_length > 0 &&
      (int)trajectory_points.size() < param->trj_length) {
    trajectory_points.resize(param->trj_length);
  }
}

__attribute__((noinline, section(".time_critical.trajectory")))
void TrajectoryGenerator::generate(float last_tgt_angle) {
  auto tmp = tgt_val->ego_in.img_ang;
  tgt_val->ego_in.img_ang += last_tgt_angle;

  if (param->trj_length <= 0) {
    return;
  }

  if ((int)trajectory_points.size() < param->trj_length) {
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

__attribute__((noinline, section(".time_critical.trajectory")))
void TrajectoryGenerator::calc_kanayama(
    EgoEstimator    &ego,
    SensorProcessor &sensor,
    float last_tgt_angle) {

  if (param->trj_idx_v.size() == 0 || param->trj_idx_val.size() == 0)
    return;

  const auto idx_val =
      sensor.interp1d(param->trj_idx_v, param->trj_idx_val, tgt_val->ego_in.v, false);

  const int idx = std::min(param->trj_length - 1, idx_val);

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

  se->ego.knym_v = vd * cos_e_theta + kx * ex;
  se->ego.knym_w = wd + vd * (ky * ey + k_theta * sin_e_theta);
  se->ego.odm_x     = ego.odm.x;
  se->ego.odm_y     = ego.odm.y;
  se->ego.odm_theta = ego.odm.theta;
  se->ego.kim_x     = ego.kim.x;
  se->ego.kim_y     = ego.kim.y;
  se->ego.kim_theta = ego.kim.theta;

  // 2026-08-23: 当初SLALOM/SLA_BACK_STR限定だったが、v_cmd/w_cmdは
  // control_law.cpp calc_pid_val_ang_vel()のee->w.error_p計算にも配線した
  // (=ヨーの主力ゲインkp/kb/kcにもKanayama補正が及ぶ)ため、対象範囲を
  // ControlLaw::angle_i_bias_active()と同じ「実質全モーション」に拡張する。
  // PIVOT系/BACK_STRAIGHT/READY/FRONT_CTRLはimg_ang自体の意味が異なる
  // (またはこの区間で軌道追従補正が不要)ため除外(control_law.cpp参照、
  // 両者は将来ズレないよう同じ除外リストを保つこと)。
  const bool kanayama_active =
      param->kanayama.enable > 0 &&
      !(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::PIVOT_PRE ||
        tgt_val->motion_type == MotionType::PIVOT_PRE2 ||
        tgt_val->motion_type == MotionType::PIVOT_AFTER ||
        tgt_val->motion_type == MotionType::PIVOT_OFFSET ||
        tgt_val->motion_type == MotionType::BACK_STRAIGHT ||
        tgt_val->motion_type == MotionType::READY ||
        tgt_val->motion_type == MotionType::FRONT_CTRL);
  if (kanayama_active) {
    v_cmd = se->ego.knym_v;
    w_cmd = se->ego.knym_w;
  } else {
    v_cmd = tgt_val->ego_in.v;
    w_cmd = tgt_val->ego_in.w;
  }
  if (tgt_val->motion_type != MotionType::SLALOM ||
      tgt_val->motion_type == MotionType::SLA_BACK_STR) {
    se->ego.knym_v = tgt_val->ego_in.v;
    se->ego.knym_w = tgt_val->ego_in.w;
  }
}

__attribute__((noinline, section(".time_critical.trajectory")))
void TrajectoryGenerator::copy_tgt(float dt) {

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
  se->img_ang_z            = tgt_val->ego_in.img_ang;
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

  dynamics.mass = param->Mass;
  dynamics.lm = param->Lm;
  dynamics.km = param->Km;
  dynamics.resist = param->Resist;
  dynamics.tread = param->tread;
  dynamics.ke = param->Ke;
  dynamics.tire = param->tire;
  dynamics.gear_ratio = param->gear_a / param->gear_b;
  // 吸引ON/OFFで摩擦FFを切り替える(2026-08-23追加、要実機チューニング)。
  // coulomb_friction/viscous_frictionは吸引OFF基準でチューニングされた値。
  // 吸引ONだと荷重(押し付け力)が約250g増えて摩擦も増えるため、OFF基準の
  // ままだと加速時にFFが摩擦分を過小評価しFB側が過大反応する(latest.csv
  // 解析、structs.hpp coulomb_friction_suction参照)。tgt_val->duty_suctionは
  // ControlLaw::set_next_duty()が書き込む実際の吸引パルス幅(1000=OFF,
  // 2000=フル)で、1tick遅れ(TrajectoryGenerator→ControlLawの実行順)だが
  // 吸引は0.5秒スケールでランプするため無視できる。中間値の補間はせず、
  // まずは閾値越えでの二値切替とする。
  constexpr float kSuctionActiveThresholdUs =
      (float)SUCTION_ESC_PULSE_MIN_US + 100.0f;
  const bool suction_active =
      tgt_val->duty_suction > kSuctionActiveThresholdUs;
  dynamics.coulomb_friction = suction_active ? param->coulomb_friction_suction
                                              : param->coulomb_friction;
  dynamics.viscous_friction = suction_active ? param->viscous_friction_suction
                                              : param->viscous_friction;
}
