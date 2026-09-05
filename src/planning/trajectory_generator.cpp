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
  // mpcへ渡す間だけego_in.img_angを前セグメント基準(+last_tgt_angle)へ
  // 回し、抜けるときにtmpへ戻す(下のコメント参照)。
  const auto tmp = tgt_val->ego_in.img_ang;
  tgt_val->ego_in.img_ang += last_tgt_angle;

  if (param->trj_length <= 0) {
    tgt_val->ego_in.img_ang = tmp;
    return;
  }

  if ((int)trajectory_points.size() < param->trj_length) {
    tgt_val->ego_in.img_ang = tmp;
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
  // 2026-09-06: mpcへ渡すために足したlast_tgt_angleを元に戻す。従来はtmpを
  // 取っておきながら戻しておらず((void)tmp)、copy_tgt()でmpc_next_ego.img_ang
  // が代入されるまでの間ego_in.img_angが前セグメント基準のまま残っていた。
  // Core1側はその間img_angを読まないが、Core0の1kHzログタイマは位相次第で
  // この窓を踏むため、SLALOM直後のSTRAIGHT/SLA_BACK_STR(last_tgt_angle=
  // ±45/90/135°)でideal_angが45°等のまま記録され、ang/kim_theta(セグメント
  // ローカル)と乖離して見えていた(20260906_025821.csv idx1124-1288:
  // ideal_ang=45.0に対しang≈0)。copy_tgt()のimg_ang_z(=前tickのimg_ang)
  // 経由でimg_ang_sumにも毎tick -last_tgt_angle が混入していた。
  tgt_val->ego_in.img_ang = tmp;
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

  // 参照(odm)と実測(kim)はセグメントローカル座標系で比較する(2026-09-06)。
  // trajectory_pointsはgenerate()が`ego_in.img_ang += last_tgt_angle`を掛けて
  // mpcへ渡すため前セグメント基準(旋回前)のグローバル座標系で積分される
  // 一方、kimはcp_request()で`kim.theta -= last_tgt_angle`とリベースされた
  // セグメントローカル座標系にある(last_tgt_angle!=0になるのはSLA_BACK_STRと
  // SLALOM直後のSTRAIGHT)。
  //  - 2026-09-05まで: kim側をグローバルへ回して(kim_x_g/kim_y_g/kim_theta_g)
  //    差を取っていた(x/yを回していなかった頃はSLA_BACK_STRでdx/dyが毎tick
  //    v*dtずつ開きknym_wが-6→+39rad/sまでランプする事故があった、
  //    20260905_042530.csv)。計算は正しかったがログにはodm(グローバル)と
  //    kim(ローカル)を別々の座標系のまま出していたため、SLALOM直後の
  //    STRAIGHT/SLA_BACK_STRで odm_theta=45°/-135° に対し kim_theta≈0、
  //    odm_x==odm_y(45°方向へ積分)のように見え、Kanayamaの姿勢誤差が45°
  //    あるかのように読めていた(20260906_025821.csv idx1124-1288 / 795-831。
  //    実際は同csvのSLA_BACK_STR中 knym_w=0.001〜0.012rad/s で実害なし)。
  //  - 2026-09-06: odm側をR(-last_tgt_angle)でローカルへ回す方式に変更。
  //    回転は一次変換なので ex/ey/e_theta は従来と同一
  //    (R(-θl)(odm_l-kim_l) = R(-θl-lta)·R(lta)(odm_l-kim_l))。ログの
  //    odm_x/odm_y/odm_theta はkim_x/kim_y/kim_thetaと同じ座標系になり、
  //    差がそのままdx/dy/d_thetaとして読める。last_tgt_angle==0なら恒等変換。
  const auto &tp = trajectory_points[idx];
  const float cos_lta = std::cos(last_tgt_angle);
  const float sin_lta = std::sin(last_tgt_angle);
  ego.odm.x     =  cos_lta * tp.ideal_px + sin_lta * tp.ideal_py;
  ego.odm.y     = -sin_lta * tp.ideal_px + cos_lta * tp.ideal_py;
  ego.odm.theta = tp.img_ang - last_tgt_angle;

  float vd = ego.odm.v = tp.v;
  float wd = ego.odm.w = tp.w;

  float dx = ego.odm.x - ego.kim.x;
  float dy = ego.odm.y - ego.kim.y;
  if (tgt_val->ego_in.v < 10) {
    dx = dy = 0;
  }

  // 実測姿勢にはang_kfではなくkim.thetaを使う(kanayama_straight/
  // control_law.cppと同様)。enable_kalman_gyro=0のときang_kfはego_in.angの
  // コピーだが、kim.thetaはw_kf積分でcp_request()のリベースも受けており
  // odm側(上でローカルへ回した)と同じセグメントローカル座標系にある。
  const float kim_theta = ego.kim.theta;

  float d_theta = ego.odm.theta - kim_theta;
  float e_theta = d_theta;
  const float cos_theta = std::cos(kim_theta);
  const float sin_theta = std::sin(kim_theta);
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

  // 2026-08-23: STRAIGHT等へ「実質全モーション」に拡張していた版から
  // SLALOM/SLA_BACK_STR限定に戻した。STRAIGHTのe_theta(kim.theta基準)は
  // 壁センサーのような絶対基準を持たないデッドレコニングのため、セグメント0で
  // 一度乱れた残留角度誤差を自力で戻せず、次セグメントへそのまま持ち越されて
  // 張り付く実機不具合を確認したため([[project-kanayama-2d-bugfix-2026-08-23]])。
  // STRAIGHTの向き補正は絶対基準(壁センサー)を持つkanayama_straight/
  // str_ang_pid_fastに委ねる。
  const bool kanayama_active =
      param->kanayama.enable > 0 &&
      (tgt_val->motion_type == MotionType::SLALOM ||
       tgt_val->motion_type == MotionType::SLA_BACK_STR);
  if (kanayama_active) {
    v_cmd = se->ego.knym_v;
    w_cmd = se->ego.knym_w;
  } else {
    v_cmd = tgt_val->ego_in.v;
    w_cmd = tgt_val->ego_in.w;
  }
  se->ego.knym_v = v_cmd;
  se->ego.knym_w = w_cmd;
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
