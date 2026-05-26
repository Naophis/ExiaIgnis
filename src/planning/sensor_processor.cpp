#include "planning/sensor_processor.hpp"
#include <algorithm>
#include <cmath>

void SensorProcessor::init(
    std::shared_ptr<sensing_result_entity_t> sensing_result,
    std::shared_ptr<input_param_t> prm,
    std::shared_ptr<motion_tgt_val_t> tgt_val) {
  se = sensing_result;
  param = prm;
  this->tgt_val = tgt_val;
  log_table.clear();
  for (int i = 0; i < 4097; i++) {
    log_table.emplace_back(std::log(i));
  }
}

__attribute__((noinline, section(".time_critical.sensor_processor")))
void SensorProcessor::calc_dist() {
  // if (!(tgt_val->motion_type == MotionType::NONE ||
  //       tgt_val->motion_type == MotionType::PIVOT)) {
  if (true) {
    se->ego.left90_dist_old = se->ego.left90_dist;
    se->ego.left45_dist_old = se->ego.left45_dist;
    se->ego.left45_2_dist_old = se->ego.left45_2_dist;
    se->ego.left45_3_dist_old = se->ego.left45_3_dist;
    se->ego.front_dist_old = se->ego.front_dist;
    se->ego.right45_dist_old = se->ego.right45_dist;
    se->ego.right45_2_dist_old = se->ego.right45_2_dist;
    se->ego.right45_3_dist_old = se->ego.right45_3_dist;
    se->ego.right90_dist_old = se->ego.right90_dist;

    se->ego.left90_dist = calc_sensor_val(
        se->ego.left90_lp, param->sensor_gain.l90.a, param->sensor_gain.l90.b);
    se->ego.left45_dist = calc_sensor_val(
        se->ego.left45_lp, param->sensor_gain.l45.a, param->sensor_gain.l45.b);
    se->ego.left45_2_dist =
        calc_sensor_val(se->ego.left45_2_lp, param->sensor_gain.l45_2.a,
                        param->sensor_gain.l45_2.b);
    se->ego.left45_3_dist =
        calc_sensor_val(se->ego.left45_3_lp, param->sensor_gain.l45_3.a,
                        param->sensor_gain.l45_3.b);
    se->ego.right45_dist = calc_sensor_val(
        se->ego.right45_lp, param->sensor_gain.r45.a, param->sensor_gain.r45.b);
    se->ego.right45_2_dist =
        calc_sensor_val(se->ego.right45_2_lp, param->sensor_gain.r45_2.a,
                        param->sensor_gain.r45_2.b);
    se->ego.right45_3_dist =
        calc_sensor_val(se->ego.right45_3_lp, param->sensor_gain.r45_3.a,
                        param->sensor_gain.r45_3.b);
    se->ego.right90_dist = calc_sensor_val(
        se->ego.right90_lp, param->sensor_gain.r90.a, param->sensor_gain.r90.b);

    se->ego.left90_far_dist =
        calc_sensor_val(se->ego.left90_lp, param->sensor_gain.l90_far.a,
                        param->sensor_gain.l90_far.b);
    se->ego.right90_far_dist =
        calc_sensor_val(se->ego.right90_lp, param->sensor_gain.r90_far.a,
                        param->sensor_gain.r90_far.b);

    se->ego.left90_mid_dist =
        calc_sensor_val(se->ego.left90_lp, param->sensor_gain.l90_mid.a,
                        param->sensor_gain.l90_mid.b);
    se->ego.right90_mid_dist =
        calc_sensor_val(se->ego.right90_lp, param->sensor_gain.r90_mid.a,
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

  calc_dist_diff();
}

void SensorProcessor::calc_dist_diff() {
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

float SensorProcessor::calc_sensor_val(float data, float a, float b) {
  int idx = (int)data;
  if (idx <= param->sensor_range_min || idx >= (int)log_table.size()) {
    return param->sensor_range_max;
  }
  auto res = a / log_table.at(idx) - b;
  if (res < param->sensor_range_min || res > param->sensor_range_max) {
    return param->sensor_range_max;
  }
  if (!std::isfinite(res)) {
    return param->sensor_range_max;
  }
  return res;
}

float SensorProcessor::interp1d(vector<float> &vx, vector<float> &vy, float x,
                                bool extrapolate) {
  int size = vx.size();
  if (size < 2) return size == 1 ? vy[0] : 0.0f;
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

int SensorProcessor::interp1d(vector<int> &vx, vector<int> &vy, float x,
                              bool extrapolate) {
  int size = vx.size();
  if (size < 2) return size == 1 ? vy[0] : 0;
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
  return (int)(yL + dydx * (x - xL));
}
