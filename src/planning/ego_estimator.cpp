#include "planning/ego_estimator.hpp"
#include <array>
#include <cmath>

void EgoEstimator::init(std::shared_ptr<sensing_result_entity_t> sensing_result,
                        std::shared_ptr<input_param_t> prm) {
  se = sensing_result;
  param = prm;
}

void EgoEstimator::update(std::shared_ptr<motion_tgt_val_t> tgt_val,
                          bool motor_en) {
  const float dt = param->dt;
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

    if (tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::READY) {
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
      const auto tmp_dist = se->ego.v_kf * dt;
      tgt_val->ego_in.pos_x += tmp_dist * cosf(pos.ang);
      tgt_val->ego_in.pos_y += tmp_dist * sinf(pos.ang);
      pos.predict(tgt_val->ego_in.v, tgt_val->ego_in.w, dt);
      const std::array<float, 3> z = {tgt_val->ego_in.pos_x,
                                      tgt_val->ego_in.pos_y, pos.ang};
      pos.update(z);
      const auto pos_state = pos.get_state();
      se->ego.pos_x = pos_state[0];
      se->ego.pos_y = pos_state[1];
      se->ego.pos_ang = pos_state[2];

      const auto d_x = se->ego.pos_x - pos_x_z;
      const auto d_y = se->ego.pos_y - pos_y_z;
      const auto d_ang = se->ego.pos_ang - pos_theta_z;

      kim.x += d_x;
      kim.y += d_y;
      kim.theta += d_ang;

      se->ang_kf_sum += d_ang;
      se->img_ang_sum += tgt_val->ego_in.img_ang - se->img_ang_z;
    }
  }

  se->ego.battery_raw = se->battery.data;
  se->ego.battery_lp =
      se->ego.battery_lp * (1 - param->battery_param.lp_delay) +
      se->ego.battery_raw * param->battery_param.lp_delay;
  kf_batt.predict(0);
  kf_batt.update(se->ego.battery_raw);
  // battery_raw が未確定(0)の間は lp 値にフォールバックして除算 Inf/NaN を防ぐ
  const float batt_kf_raw = kf_batt.get_state();
  se->ego.batt_kf = (batt_kf_raw > 0.01f) ? batt_kf_raw : se->ego.battery_lp;

  se->ego.left45_lp_old = se->ego.left45_lp;
  se->ego.left90_lp_old = se->ego.left90_lp;
  se->ego.front_lp_old = se->ego.front_lp;
  se->ego.right45_lp_old = se->ego.right45_lp;
  se->ego.right90_lp_old = se->ego.right90_lp;

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

  tgt_val->ego_in.slip_point.w = se->ego.w_lp;
}

void EgoEstimator::reset_kf_state(
    bool reset_battery, std::shared_ptr<sensing_result_entity_t> sensing_result,
    std::shared_ptr<input_param_t> param) {
  const float initial_state = 0.0f;

  kf_w.dt = 0.001f;
  kf_w2.dt = 0.001f;
  kf_v_r.dt = 0.001f;
  kf_v_l.dt = 0.001f;

  if (reset_battery) {
    kf_batt.init(sensing_result->ego.battery_raw, param->battery_init_cov,
                 param->battery_p_noise, param->battery_m_noise);
    pos.init(-param->offset_start_dist, 0.0f, 0.0f, param->pos_init_cov,
             param->pos_p_noise, param->pos_m_noise);
  } else {
    pos.reset_cov(param->pos_init_cov, param->pos_p_noise, param->pos_m_noise);
  }

  kf_w.init(initial_state, param->w_init_cov, param->w_p_noise,
            param->w_m_noise);
  kf_w2.init(initial_state, param->w_init_cov, param->w_p_noise,
             param->w_m_noise);
  kf_v.init(initial_state, param->v_init_cov, param->v_p_noise,
            param->v_m_noise);
  kf_v_r.init(initial_state, param->encoder_init_cov, param->encoder_p_noise,
              param->encoder_m_noise);
  kf_v_l.init(initial_state, param->encoder_init_cov, param->encoder_p_noise,
              param->encoder_m_noise);
  kf_dist.init(initial_state, param->dist_init_cov, param->dist_p_noise,
               param->dist_m_noise);
  kf_ang.init(initial_state, param->ang_init_cov, param->ang_p_noise,
              param->ang_m_noise);
  kf_ang2.init(initial_state, param->ang_init_cov, param->ang_p_noise,
               param->ang_m_noise);

  sensing_result->ang_kf_sum = 0;
  sensing_result->img_ang_sum = 0;
  sensing_result->img_ang_z = 0;
}