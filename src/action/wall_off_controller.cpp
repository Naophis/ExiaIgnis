#include "action/wall_off_controller.hpp"
#include "planning/planning_task.hpp"
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>

std::shared_ptr<sensing_result_entity_t>
WallOffController::get_sensing_entity() {
  return sensing_result;
}

std::shared_ptr<input_param_t> WallOffController::get_input_param_entity() {
  return param;
}

bool WallOffController::execute_wall_off(TurnDirection td,
                                         param_straight_t &ps_front) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();

  tgt_val->nmr.v_max = ps_front.v_max;
  tgt_val->nmr.v_end = ps_front.v_end;
  tgt_val->nmr.accl = ps_front.accl;
  tgt_val->nmr.decel = ps_front.decel;
  tgt_val->nmr.dist = param->wall_off_wait_dist;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->nmr.motion_type = MotionType::WALL_OFF;
  tgt_val->nmr.motion_dir = (td == TurnDirection::Left)
                                ? MotionDirection::LEFT
                                : MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = ps_front.dia_mode;
  tgt_val->nmr.sct = SensorCtrlType::Straight;
  tgt_val->nmr.timstamp++;

  if (pt) pt->send_command(*tgt_val);
  sleep_ms(1);

  return (td == TurnDirection::Right) ? process_right_wall_off(ps_front)
                                      : process_left_wall_off(ps_front);
}

bool WallOffController::process_right_wall_off(param_straight_t &ps_front) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();
  const auto strategy = get_right_strategy();

  bool exist = se->ego.right45_dist < p_wall_off.wall_off_exist_wall_th_r;
  auto tmp_dist_before = tgt_val->global_pos.dist;
  auto tmp_dist_after = tmp_dist_before;

  // 第一段階：壁を見つけるまで待機
  while (true) {
    tmp_dist_after = tgt_val->global_pos.dist;
    if (strategy.exist_wall()) {
      break;
    }
    if (!exist) {
      exist = se->ego.right45_dist < p_wall_off.wall_off_exist_wall_th_r;
      if (strategy.find_vertical_wall()) {
        break;
      }
      if (strategy.wall_missing()) {
        ps_front.dist += p_wall_off.right_str;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    } else {
      if (strategy.wall_missing()) {
        ps_front.dist += p_wall_off.right_str;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
      break;
    }

    if (strategy.detect_pass_through_case1(tmp_dist_before, tmp_dist_after)) {
      ps_front.dist -= param->wall_off_pass_through_offset_r;
      ps_front.dist = MAX(ps_front.dist, 0.1);
      return true;
    }

    if (strategy.detect_pass_through_case2(tmp_dist_before, tmp_dist_after)) {
      ps_front.dist -= param->wall_off_pass_through_offset_r;
      ps_front.dist = MAX(ps_front.dist, 0.1);
      return true;
    }

    if (apply_front_sensor_correction(ps_front, tmp_dist_before, tmp_dist_after,
                                      TurnDirection::Right)) {
      return true;
    }

    if (strategy.detect_distance(tmp_dist_before, tmp_dist_after)) {
      ps_front.dist -= param->wall_off_pass_through_offset_r;
      ps_front.dist = MAX(ps_front.dist, 0.1);
      return true;
    }

    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }

  // 第二段階：壁切れを待つ
  while (true) {
    tmp_dist_after = tgt_val->global_pos.dist;

    if (exist) {
      if (strategy.detect_wall_off(exist)) {
        ps_front.dist += p_wall_off.right_str_exist;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    } else {
      if (strategy.detect_wall_missing_by_deviation(exist)) {
        ps_front.dist += p_wall_off.right_str;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }

    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }
  return false;
}

bool WallOffController::process_left_wall_off(param_straight_t &ps_front) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();
  const auto strategy = get_left_strategy();

  bool exist = se->ego.left45_dist < p_wall_off.wall_off_exist_wall_th_l;
  auto tmp_dist_before = tgt_val->global_pos.dist;
  auto tmp_dist_after = tmp_dist_before;

  // 第一段階：壁を見つけるまで待機
  while (true) {
    tmp_dist_after = tgt_val->global_pos.dist;
    if (strategy.exist_wall()) {
      break;
    }
    if (!exist) {
      exist = se->ego.left45_dist < p_wall_off.wall_off_exist_wall_th_l;
      if (se->ego.left45_dist < p_wall_off.exist_dist_l2) {
        break;
      }
      if (strategy.wall_missing()) {
        ps_front.dist += p_wall_off.left_str;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    } else {
      if (strategy.wall_missing()) {
        ps_front.dist += p_wall_off.left_str;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
      break;
    }

    if (strategy.detect_pass_through_case1(tmp_dist_before, tmp_dist_after)) {
      ps_front.dist -= param->wall_off_pass_through_offset_l;
      ps_front.dist = MAX(ps_front.dist, 0.1);
      return true;
    }

    if (strategy.detect_pass_through_case2(tmp_dist_before, tmp_dist_after)) {
      ps_front.dist -= param->wall_off_pass_through_offset_l;
      ps_front.dist = MAX(ps_front.dist, 0.1);
      return true;
    }

    if (apply_front_sensor_correction(ps_front, tmp_dist_before, tmp_dist_after,
                                      TurnDirection::Left)) {
      return true;
    }

    if (strategy.detect_distance(tmp_dist_before, tmp_dist_after)) {
      ps_front.dist -= param->wall_off_pass_through_offset_l;
      ps_front.dist = MAX(ps_front.dist, 0.1);
      return true;
    }

    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }

  // 第二段階：壁切れを待つ
  while (true) {
    tmp_dist_after = tgt_val->global_pos.dist;

    if (exist) {
      if (strategy.detect_wall_off(exist)) {
        ps_front.dist += p_wall_off.left_str_exist;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    } else {
      if (strategy.detect_wall_missing_by_deviation(exist)) {
        ps_front.dist += p_wall_off.left_str;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }

    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }
  return false;
}

bool WallOffController::apply_front_sensor_correction(
    param_straight_t &ps_front, float tmp_dist_before, float tmp_dist_after,
    TurnDirection td) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();

  const auto diff_front =
      std::abs(se->ego.left90_far_dist - se->ego.right90_dist);
  auto valid_diff = diff_front < param->wall_off_diff_ref_front_th;

  auto diff_decrease =
      (td == TurnDirection::Left)
          ? (se->ego.left45_dist_diff < 0 &&
             (45 < se->ego.left45_dist && se->ego.left45_dist < 90)) ||
                !(se->ego.left45_dist > 90)
          : (se->ego.right45_dist_diff < 0 &&
             (45 < se->ego.right45_dist && se->ego.right45_dist < 90)) ||
                !(se->ego.right45_dist > 90);

  if (param->wall_off_dist.right_diff_th > 180) { // is circuit
    valid_diff = true;
    diff_decrease = false;
  }

  if (std::abs(tmp_dist_after - tmp_dist_before) >=
      std::abs(param->wall_off_front_move_dist_th)) {
    if (valid_diff && !diff_decrease &&
        param->wall_off_front_ctrl_min < se->ego.left90_far_dist &&
        se->ego.left90_far_dist < param->front_dist_offset4 &&
        param->wall_off_front_ctrl_min < se->ego.right90_far_dist &&
        se->ego.right90_far_dist < param->front_dist_offset4) {
      if (se->ego.front_far_dist < param->front_dist_offset3) {
        ps_front.dist -= (param->front_dist_offset2 - se->ego.front_dist);
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }
  }
  return false;
}

bool WallOffController::execute_wall_off_dia(TurnDirection td,
                                              param_straight_t &ps_front,
                                              bool &use_oppo_wall,
                                              bool &exist_wall) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();

  tgt_val->nmr.v_max = ps_front.v_max;
  tgt_val->nmr.v_end = ps_front.v_end;
  tgt_val->nmr.accl = ps_front.accl;
  tgt_val->nmr.decel = ps_front.decel;
  tgt_val->nmr.dist = param->wall_off_wait_dist_dia;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->nmr.motion_type = MotionType::WALL_OFF_DIA;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = ps_front.dia_mode;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.timstamp++;

  use_oppo_wall = false;
  exist_wall = false;

  if (pt) pt->send_command(*tgt_val);
  sleep_ms(1);

  return (td == TurnDirection::Right)
             ? process_right_wall_off_dia(ps_front, use_oppo_wall, exist_wall)
             : process_left_wall_off_dia(ps_front, use_oppo_wall, exist_wall);
}

MotionResult WallOffController::execute_search_wall_off(param_straight_t &p,
                                                         bool search_mode) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();
  static int wall_off_state = 0;

  bool exist_right = se->ego.right45_dist < p_wall_off.ctrl_exist_wall_th_r;
  bool exist_left = se->ego.left45_dist < p_wall_off.ctrl_exist_wall_th_l;

  if (!search_mode || !p_wall_off.search_wall_off_enable) {
    return MotionResult::NONE;
  }
  if (wall_off_state == 0) {
    if (se->ego.right45_dist < p_wall_off.exist_dist_r && exist_right) {
      wall_off_state |= 1;
    }
    if (se->ego.left45_dist < p_wall_off.exist_dist_l && exist_left) {
      wall_off_state |= 2;
    }
  } else if (wall_off_state == 3 && exist_right && exist_left) {
    if (se->ego.right45_dist > p_wall_off.noexist_th_r) {
      wall_off_state = 4;
      p.dist = p_wall_off.search_wall_off_r_dist_offset;
      return MotionResult::WALL_OFF_DETECTED;
    } else if (se->ego.left45_dist > p_wall_off.noexist_th_l) {
      wall_off_state = 4;
      p.dist = p_wall_off.search_wall_off_l_dist_offset;
      return MotionResult::WALL_OFF_DETECTED;
    }
  } else if (wall_off_state == 1 && exist_right) {
    if (se->ego.right45_dist > p_wall_off.noexist_th_r) {
      wall_off_state = 4;
      p.dist = p_wall_off.search_wall_off_r_dist_offset;
      return MotionResult::WALL_OFF_DETECTED;
    }
  } else if (wall_off_state == 2 && exist_left) {
    if (se->ego.left45_dist > p_wall_off.noexist_th_l) {
      wall_off_state = 4;
      p.dist = p_wall_off.search_wall_off_l_dist_offset;
      return MotionResult::WALL_OFF_DETECTED;
    }
  }

  return MotionResult::NONE;
}

bool WallOffController::is_wall_exist(TurnDirection td, float threshold_l,
                                      float threshold_r) {
  const auto se = get_sensing_entity();
  if (td == TurnDirection::Right) {
    return se->ego.right45_dist < threshold_r;
  } else {
    return se->ego.left45_dist < threshold_l;
  }
}

bool WallOffController::process_right_wall_off_dia(param_straight_t &ps_front,
                                                    bool &use_oppo_wall,
                                                    bool &exist_wall) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();
  const auto strategy = get_right_dia_strategy();
  const auto init_r45_dist = se->ego.right45_dist;

  float tmp_dist_before = tgt_val->global_pos.dist;
  float tmp_dist_after = tmp_dist_before;

  auto exist = exist_wall =
      se->ego.right45_dist < p_wall_off.wall_off_exist_dia_wall_th_r;

  // 第一段階：壁切れ開始を待つ
  while (true) {
    if (exist) {
      if (strategy.exist_dia_wall_start()) {
        break;
      }
      break;
    } else {
      exist = exist_wall =
          se->ego.right45_dist < p_wall_off.wall_off_exist_dia_wall_th_r;
      if (exist) {
        continue;
      }
      if (strategy.exist_dia_wall_start_alt()) {
        break;
      }
    }

    tmp_dist_after = tgt_val->global_pos.dist;

    if (continuous_turn_flag) {
      if (strategy.detect_pass_through_case1(tmp_dist_before, tmp_dist_after,
                                             init_r45_dist)) {
        ps_front.dist += p_wall_off.right_dia;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }

    if (strategy.detect_pass_through_case2(tmp_dist_before, tmp_dist_after,
                                           init_r45_dist)) {
      float diff = (se->ego.left45_dist - param->dia_turn_ref_l);
      diff = std::clamp(diff, -param->dia_turn_max_dist_l,
                        param->dia_turn_max_dist_l);
      ps_front.dist = ps_front.dist + diff;
      use_oppo_wall = true;
      return true;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }

  // 第二段階：壁切れ終了を待つ
  while (true) {
    if (exist) {
      if (strategy.detect_wall_off_exist()) {
        ps_front.dist += p_wall_off.right_dia;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    } else {
      if (strategy.detect_wall_off_by_deviation()) {
        ps_front.dist += p_wall_off.right_dia2;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }
  return false;
}

bool WallOffController::process_left_wall_off_dia(param_straight_t &ps_front,
                                                   bool &use_oppo_wall,
                                                   bool &exist_wall) {
  const auto se = get_sensing_entity();
  const auto p_wall_off = get_wall_off_param();
  const auto strategy = get_left_dia_strategy();

  const auto init_l45_dist = se->ego.left45_dist;
  float tmp_dist_before = tgt_val->global_pos.dist;
  float tmp_dist_after = tmp_dist_before;

  auto exist = exist_wall =
      se->ego.left45_dist < p_wall_off.wall_off_exist_dia_wall_th_l;

  // 第一段階：壁切れ開始を待つ
  while (true) {
    if (exist) {
      if (strategy.exist_dia_wall_start()) {
        break;
      }
      break;
    } else {
      exist = exist_wall =
          se->ego.left45_dist < p_wall_off.wall_off_exist_dia_wall_th_l;
      if (exist) {
        continue;
      }
      if (strategy.exist_dia_wall_start_alt()) {
        break;
      }
    }

    tmp_dist_after = tgt_val->global_pos.dist;

    if (continuous_turn_flag) {
      if (strategy.detect_pass_through_case1(tmp_dist_before, tmp_dist_after,
                                             init_l45_dist)) {
        ps_front.dist += p_wall_off.left_dia;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }

    if (strategy.detect_pass_through_case2(tmp_dist_before, tmp_dist_after,
                                           init_l45_dist)) {
      float diff = (se->ego.right45_dist - param->dia_turn_ref_r);
      diff = std::clamp(diff, -param->dia_turn_max_dist_r,
                        param->dia_turn_max_dist_r);
      ps_front.dist = ps_front.dist + diff;
      use_oppo_wall = true;
      return true;
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }

  // 第二段階：壁切れ終了を待つ
  while (true) {
    if (exist) {
      if (strategy.detect_wall_off_exist()) {
        ps_front.dist += p_wall_off.left_dia;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    } else {
      if (strategy.detect_wall_off_by_deviation()) {
        ps_front.dist += p_wall_off.left_dia2;
        ps_front.dist = MAX(ps_front.dist, 0.1);
        return true;
      }
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return false;
    }
    sleep_ms(1);
  }
  return false;
}

float WallOffController::calculate_dia_wall_off_distance(TurnDirection td,
                                                          TurnType turn_type,
                                                          bool &exist_wall) {
  const auto se = get_sensing_entity();
  auto ref = (td == TurnDirection::Right) ? param->dia_wall_off_ref_r_wall
                                          : param->dia_wall_off_ref_l_wall;
  if (continuous_turn_flag) {
    ref = (td == TurnDirection::Right) ? param->dia_wall_off_ref_r_wall2
                                       : param->dia_wall_off_ref_l_wall2;
  }
  if (!exist_wall) {
    ref = (td == TurnDirection::Right) ? param->dia_wall_off_ref_r_piller
                                       : param->dia_wall_off_ref_l_piller;
  }
  if (turn_type == TurnType::Dia135_2 || turn_type == TurnType::Dia90) {
    ref = (td == TurnDirection::Right) ? param->dia_wall_off_ref_r_wall
                                       : param->dia_wall_off_ref_l_wall;
    if (continuous_turn_flag) {
      ref = (td == TurnDirection::Right) ? param->dia_wall_off_ref_r_wall2
                                         : param->dia_wall_off_ref_l_wall2;
    }
  }

  float diff = 0;
  float offset = 0;

  const static float ang32 = 32.0f / 180.0f * M_PI;
  const static float cos32 = std::cos(ang32);
  const static float tan32 = std::tan(ang32);

  float gain = 1.0f;
  if (turn_type == TurnType::Dia45_2) {
    if (td == TurnDirection::Right) {
      const float ego_ang =
          std::clamp(se->sen.r45.angle, -param->lim_angle, param->lim_angle);
      gain = std::cos(ang32 + ego_ang) / cos32;
      diff = (gain * se->sen.r45.sensor_dist - ref);
    } else {
      const float ego_ang =
          std::clamp(se->sen.l45.angle, -param->lim_angle, param->lim_angle);
      gain = std::cos(ang32 - ego_ang) / cos32;
      diff = (gain * se->sen.l45.sensor_dist - ref);
    }
    offset = (1.0f - tan32) * diff;
    offset = std::clamp(offset, -param->dia45_2_offset_max_dist,
                        param->dia45_2_offset_max_dist);
  } else if (turn_type == TurnType::Dia135_2) {
    if (td == TurnDirection::Right) {
      const float ego_ang =
          std::clamp(se->sen.r45.angle, -param->lim_angle, param->lim_angle);
      gain = std::cos(ang32 + ego_ang) / cos32;
      diff = (gain * se->sen.r45.sensor_dist - ref);
    } else {
      const float ego_ang =
          std::clamp(se->sen.l45.angle, -param->lim_angle, param->lim_angle);
      gain = std::cos(ang32 - ego_ang) / cos32;
      diff = (gain * se->sen.l45.sensor_dist - ref);
    }
    offset = (1.0f + tan32) * diff;
    offset = std::clamp(offset, -param->dia135_2_offset_max_dist,
                        param->dia135_2_offset_max_dist);
  } else if (turn_type == TurnType::Dia90) {
    if (td == TurnDirection::Right) {
      const float ego_ang =
          std::clamp(se->sen.r45.angle, -param->lim_angle, param->lim_angle);
      gain = std::cos(ang32 + ego_ang) / cos32;
      diff = (gain * se->sen.r45.sensor_dist - ref);
    } else {
      const float ego_ang =
          std::clamp(se->sen.l45.angle, -param->lim_angle, param->lim_angle);
      gain = std::cos(ang32 - ego_ang) / cos32;
      diff = (gain * se->sen.l45.sensor_dist - ref);
    }
    offset = tan32 * diff;
    offset = std::clamp(diff, -param->dia90_offset_max_dist,
                        param->dia90_offset_max_dist);
  }
  return offset;
}

wall_off_hold_dist_t &WallOffController::get_wall_off_param() {
  return param->wall_off_dist;
}

WallSensorStrategy &WallOffController::get_right_strategy() {
  if (right_strategy.has_value()) {
    return *right_strategy;
  }

  right_strategy = WallSensorStrategy{
      // wall_missing
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist_diff > p_wall_off.div_th_r2 &&
               se->ego.right45_2_dist_diff > 0 &&
               se->ego.right45_dist < 70;
      },
      // exist_wall
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist < p_wall_off.exist_dist_r;
      },
      // find_vertical_wall
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist < p_wall_off.exist_dist_r2;
      },
      // detect_pass_through_case1
      [this](float before, float after) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (std::abs(after - before) >=
                std::abs(p_wall_off.diff_check_dist)) &&
               se->ego.right45_dist_diff > p_wall_off.diff_dist_th_r &&
               se->ego.right45_2_dist_diff > 0 &&
               se->ego.right45_dist_diff < 100;
      },
      // detect_pass_through_case2
      [this](float before, float after) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return std::abs(after - before) >=
                   std::abs(p_wall_off.diff_check_dist) &&
               se->ego.right45_dist > 170;
      },
      // detect_distance
      [this](float before, float after) -> bool {
        const auto param = get_input_param_entity();
        return std::abs(after - before) >=
                   std::abs(param->wall_off_front_move_dist_th) &&
               std::abs(tgt_val->ego_in.dist) >= std::abs(tgt_val->nmr.dist);
      },
      // detect_missing_by_deviation
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return std::abs(se->ego.right45_dist_diff) > p_wall_off.right_diff_th &&
               se->ego.right45_dist < 100;
      },
      // detect_wall_off
      [this](float exist) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (se->ego.right45_dist > p_wall_off.noexist_th_r &&
                se->ego.right45_dist_diff > 0) &&
               (se->ego.right45_dist_diff > p_wall_off.div_th_r3 &&
                se->ego.right45_2_dist_diff > 0 &&
                se->ego.right45_dist < 100);
      },
      // detect_wall_missing_by_deviation
      [this](float exist) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        const auto p = get_input_param_entity();
        return ((se->ego.right45_dist > p->wall_off_dist.noexist_th_r2) ||
                (se->sen.r45.sensor_dist + 1) < se->ego.right45_dist) &&
               se->ego.right45_dist_diff > p_wall_off.div_th_r3 &&
               se->ego.right45_2_dist_diff > 0 &&
               se->ego.right45_dist < 100;
      },
      // detect_wall_off_vertical
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist > p_wall_off.noexist_th_r2;
      },
  };
  return *right_strategy;
}

WallSensorStrategy &WallOffController::get_left_strategy() {
  if (left_strategy.has_value()) {
    return *left_strategy;
  }

  left_strategy = WallSensorStrategy{
      // wall_missing
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist_diff > p_wall_off.div_th_l3 &&
               se->ego.left45_2_dist_diff > 0 &&
               se->ego.left45_dist < 70;
      },
      // exist_wall
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist < p_wall_off.exist_dist_l;
      },
      // find_vertical_wall
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist < p_wall_off.exist_dist_l2;
      },
      // detect_pass_through_case1
      [this](float before, float after) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (std::abs(after - before) >=
                std::abs(p_wall_off.diff_check_dist)) &&
               se->ego.left45_dist_diff > p_wall_off.diff_dist_th_l &&
               se->ego.left45_2_dist_diff > 0 &&
               se->ego.left45_dist < 100;
      },
      // detect_pass_through_case2
      [this](float before, float after) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return std::abs(after - before) >=
                   std::abs(p_wall_off.diff_check_dist) &&
               se->ego.left45_dist > 170;
      },
      // detect_distance
      [this](float before, float after) -> bool {
        const auto param = get_input_param_entity();
        return std::abs(after - before) >=
                   std::abs(param->wall_off_front_move_dist_th) &&
               std::abs(tgt_val->ego_in.dist) >= std::abs(tgt_val->nmr.dist);
      },
      // detect_missing_by_deviation
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return std::abs(se->ego.left45_dist_diff) > p_wall_off.left_diff_th &&
               se->ego.left45_dist < 100;
      },
      // detect_wall_off
      [this](float exist) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (se->ego.left45_dist > p_wall_off.noexist_th_l &&
                se->ego.left45_dist_diff > 0) &&
               (se->ego.left45_dist_diff > p_wall_off.div_th_l3 &&
                se->ego.left45_2_dist_diff > 0 &&
                se->ego.left45_dist < 100);
      },
      // detect_wall_missing_by_deviation
      [this](float exist) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        const auto param = get_input_param_entity();
        return ((se->ego.left45_dist > param->wall_off_dist.noexist_th_l2) ||
                (se->sen.l45.sensor_dist + 1) < se->ego.left45_dist) &&
               se->ego.left45_dist_diff > p_wall_off.div_th_l3 &&
               se->ego.left45_2_dist_diff > 0 &&
               se->ego.left45_dist < 100;
      },
      // detect_wall_off_vertical
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist > p_wall_off.noexist_th_l2;
      },
  };
  return *left_strategy;
}

DiagonalWallOffStrategy &WallOffController::get_left_dia_strategy() {
  if (left_dia_strategy.has_value()) {
    return *left_dia_strategy;
  }

  left_dia_strategy = DiagonalWallOffStrategy{
      // exist_dia_wall_start
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist < p_wall_off.exist_dia_th_l;
      },
      // exist_dia_wall_start_alt
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist < p_wall_off.exist_dia_th_l2;
      },
      // detect_pass_through_case1
      [this](float before, float after, float init) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        const auto param = get_input_param_entity();
        return std::abs(after - before) >=
                   std::abs(p_wall_off.diff_check_dist_dia_2) &&
               (se->ego.left45_dist - init) > param->wall_off_pass_dist;
      },
      // detect_pass_through_case2
      [this](float before, float after, float init) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        const auto param = get_input_param_entity();
        bool valid_right90 =
            1 < se->ego.right90_mid_dist &&
            se->ego.right90_mid_dist < param->sen_ref_p.dia.exist.right90 &&
            se->ego.right90_dist_diff < 0;
        return std::abs(after - before) >=
                   std::abs(p_wall_off.diff_check_dist_dia) &&
               45 < se->ego.right45_dist &&
               se->ego.right45_dist < param->dia_turn_th_r && valid_right90;
      },
      // detect_wall_off_exist
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (se->ego.left45_dist > p_wall_off.noexist_dia_th_l &&
                se->ego.left45_dist_diff > 0) &&
               (se->ego.left45_dist_diff > p_wall_off.div_th_dia_l &&
                se->ego.left45_2_dist_diff > 0 &&
                se->ego.left45_dist < 100);
      },
      // detect_wall_off_by_deviation
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (se->ego.left45_dist > p_wall_off.noexist_dia_th_l2) &&
               (se->ego.left45_dist_diff > p_wall_off.div_th_dia_l &&
                se->ego.left45_2_dist_diff > 0 &&
                se->ego.left45_dist < 100);
      },
      // detect_wall_off_alt
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.left45_dist > p_wall_off.noexist_dia_th_l2;
      },
  };
  return *left_dia_strategy;
}

DiagonalWallOffStrategy &WallOffController::get_right_dia_strategy() {
  if (right_dia_strategy.has_value()) {
    return *right_dia_strategy;
  }

  right_dia_strategy = DiagonalWallOffStrategy{
      // exist_dia_wall_start
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist < p_wall_off.exist_dia_th_r;
      },
      // exist_dia_wall_start_alt
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist < p_wall_off.exist_dia_th_r2;
      },
      // detect_pass_through_case1
      [this](float before, float after, float init) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        const auto param = get_input_param_entity();
        return std::abs(after - before) >=
                   std::abs(p_wall_off.diff_check_dist_dia_2) &&
               (se->ego.right45_dist - init) > param->wall_off_pass_dist;
      },
      // detect_pass_through_case2
      [this](float before, float after, float init) -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        const auto param = get_input_param_entity();
        bool valid_left90 =
            1 < se->ego.left90_mid_dist &&
            se->ego.left90_mid_dist < param->sen_ref_p.dia.exist.left90 &&
            se->ego.left90_dist_diff < 0;
        return std::abs(after - before) >=
                   std::abs(p_wall_off.diff_check_dist_dia) &&
               45 < se->ego.left45_dist &&
               se->ego.left45_dist < param->dia_turn_th_l && valid_left90;
      },
      // detect_wall_off_exist
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (se->ego.right45_dist > p_wall_off.noexist_dia_th_r &&
                se->ego.right45_dist_diff > 0) &&
               (se->ego.right45_dist_diff > p_wall_off.div_th_dia_r &&
                se->ego.right45_2_dist_diff > 0 &&
                se->ego.right45_dist < 100);
      },
      // detect_wall_off_by_deviation
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return (se->ego.right45_dist > p_wall_off.noexist_dia_th_r2) &&
               (se->ego.right45_dist_diff > p_wall_off.div_th_dia_r &&
                se->ego.right45_2_dist_diff > 0 &&
                se->ego.right45_dist < 100);
      },
      // detect_wall_off_alt
      [this]() -> bool {
        const auto p_wall_off = get_wall_off_param();
        const auto se = get_sensing_entity();
        return se->ego.right45_dist > p_wall_off.noexist_dia_th_r2;
      },
  };
  return *right_dia_strategy;
}
