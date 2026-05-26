#pragma once

#include "include/structs.hpp"
#include "planning/astraea_types.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <optional>

class PlanningTask;

struct WallSensorStrategy {
  std::function<bool()> wall_missing;
  std::function<bool()> exist_wall;
  std::function<bool()> find_vertical_wall;
  std::function<bool(float, float)> detect_pass_through_case1;
  std::function<bool(float, float)> detect_pass_through_case2;
  std::function<bool(float, float)> detect_distance;
  std::function<bool()> detect_missing_by_deviation;
  std::function<bool(float)> detect_wall_off;
  std::function<bool(float)> detect_wall_missing_by_deviation;
  std::function<bool()> detect_wall_off_vertical;
};

struct DiagonalWallOffStrategy {
  std::function<bool()> exist_dia_wall_start;
  std::function<bool()> exist_dia_wall_start_alt;
  std::function<bool(float, float, float)> detect_pass_through_case1;
  std::function<bool(float, float, float)> detect_pass_through_case2;
  std::function<bool()> detect_wall_off_exist;
  std::function<bool()> detect_wall_off_by_deviation;
  std::function<bool()> detect_wall_off_alt;
};

class WallOffController {
public:
  WallSensorStrategy &get_right_strategy();
  WallSensorStrategy &get_left_strategy();
  DiagonalWallOffStrategy &get_left_dia_strategy();
  DiagonalWallOffStrategy &get_right_dia_strategy();
  WallOffController() = default;
  ~WallOffController() = default;

  bool execute_wall_off(TurnDirection td, param_straight_t &ps_front);
  bool execute_wall_off_dia(TurnDirection td, param_straight_t &ps_front,
                            bool &use_oppo_wall, bool &exist_wall);
  MotionResult execute_search_wall_off(param_straight_t &p, bool search_mode);
  float calculate_dia_wall_off_distance(TurnDirection td, TurnType turn_type,
                                        bool &exist_wall);

  void set_input_param_entity(std::shared_ptr<input_param_t> &_param) {
    param = _param;
  }
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
    tgt_val = _tgt_val;
  }
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity) {
    sensing_result = _entity;
  }
  void set_planning_task(std::shared_ptr<PlanningTask> _pt) { pt = _pt; }

  bool continuous_turn_flag = false;

private:
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<PlanningTask> pt;

  std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
  wall_off_hold_dist_t &get_wall_off_param();
  std::shared_ptr<input_param_t> get_input_param_entity();

  bool process_right_wall_off(param_straight_t &ps_front);
  bool process_left_wall_off(param_straight_t &ps_front);
  bool apply_front_sensor_correction(param_straight_t &ps_front,
                                     float tmp_dist_before,
                                     float tmp_dist_after, TurnDirection td);
  bool is_wall_exist(TurnDirection td, float threshold_l, float threshold_r);
  bool process_right_wall_off_dia(param_straight_t &ps_front,
                                  bool &use_oppo_wall, bool &exist_wall);
  bool process_left_wall_off_dia(param_straight_t &ps_front,
                                 bool &use_oppo_wall, bool &exist_wall);

  std::optional<WallSensorStrategy> right_strategy;
  std::optional<WallSensorStrategy> left_strategy;
  std::optional<DiagonalWallOffStrategy> left_dia_strategy;
  std::optional<DiagonalWallOffStrategy> right_dia_strategy;
};
