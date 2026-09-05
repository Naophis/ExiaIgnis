#ifndef MotionPlanning_HPP
#define MotionPlanning_HPP

#include "pico/stdlib.h"
#include "search/adachi.hpp"
#include "action/path_creator.hpp"
#include "action/trajectory_creator.hpp"
#include "action/wall_off_controller.hpp"
#include "logging/logging_task.hpp"
#include "planning/planning_task.hpp"
#include "planning/astraea_types.hpp"
#include "ui.hpp"
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class MotionPlanning {
public:
  MotionPlanning();
  virtual ~MotionPlanning() {}

  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
  void set_userinterface(std::shared_ptr<UserInterface> &_ui);
  void set_path_creator(std::shared_ptr<PathCreator> &_pc);
  void set_planning_task(std::shared_ptr<PlanningTask> &_pt);
  void set_logging_task(std::shared_ptr<LoggingTask> &_lt);

  std::shared_ptr<Adachi> fake_adachi;

  MotionResult go_straight(param_straight_t &p);
  MotionResult go_straight(param_straight_t &p, std::shared_ptr<Adachi> &adachi,
                           bool search_mode);
  // 現在の速度・角速度をゼロに能動的に保持する専用モーション(2026-09-05追加)。
  // 吸引ファンのランプアップ中のような「本走行はまだ始めたくないが、外乱
  // (ファン反動トルク等)で機体が動いてしまうのは防ぎたい」場面向け。
  // motion_type=NONEはduty出力を強制的に0にする(control_law.cpp参照)ため
  // 保持にならず、go_straight()を極小速度・長距離のダミー引数で騙して使うのも
  // 目的外利用で紛らわしいため、一度だけ v=0/w=0 のSTRAIGHT指令を送る専用
  // 関数として分離する。呼び出し側が次の本コマンドをpt->send_command()経由で
  // 送るまで、この保持状態はCore1側で持続する(ブロックしない、呼び出し側で
  // 好きな時間sleepしてから次のコマンドに進めばよい)。
  // hold()はgyro_pid.cを一時的にparam->hold_ang_gainへ差し替える(通常の
  // gyro_pid.cは長い直進向けの弱いゲインで、吸引ランプ中の短時間インパルス
  // 外乱を戻すには弱すぎるため)。呼び出し側はhold()と対でunhold()を呼び、
  // 実走行前に元のゲインへ戻す責任を持つこと。
  void hold();
  void unhold();
  // 吸引プラトー後、向き(kim_theta)が収束するまで待つ(上限付き、
  // structs.hpp hold_settle_t参照)。hold()〜unhold()の間で呼ぶ。
  void hold_settle_wait();
  MotionResult pivot_turn(param_roll_t &p);
  void normal_slalom(param_normal_slalom_t &p, param_straight_t &p_str);

  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motion_t &next_motion, bool dia);
  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motion_t &next_motion);
  MotionResult slalom(slalom_param2_t &sp, TurnDirection dir,
                      next_motion_t &next_motion, bool dia,
                      std::shared_ptr<Adachi> &adachi, bool search_mode);
  MotionResult search_front_ctrl(param_straight_t &p);
  MotionResult wall_off(param_straight_t &p, bool dia);

  void reset_tgt_data();
  void reset_ego_data();
  void reset_gyro_ref();
  void reset_gyro_ref_with_check();
  void coin();
  void keep();
  void exec_path_running(param_set_t &param_set);
  MotionResult front_ctrl(bool limit);

  bool wall_off(TurnDirection td, param_straight_t &ps_front);
  bool wall_off_dia(TurnDirection td, param_straight_t &ps_front,
                    bool &use_oppo_wall, bool &exist_wall);
  void req_error_reset();
  void system_identification(MotionType mt, float duty_l, float duty_r,
                             float time);

  std::shared_ptr<motion_tgt_val_t> tgt_val;

  void notify() {}  // no-op: ExiaIgnis では IRQ が 1kHz で tgt_val を参照

  std::shared_ptr<WallOffController> wall_off_controller;

  volatile bool skip_gyro_bias_check = false;

  float g_offset_y_l = 0;
  float g_offset_y_r = 0;
  float g_offset_x1  = 0;
  float g_offset_x2  = 0;
  float g_total_offset = 0;
  float g_sen_r_dist = 0;
  float g_sen_l_dist = 0;
  float g_sen_ang    = 0;

private:
  float hold_gyro_pid_c_prev_ = 0.0f;

  float calc_orval_offset(TurnDirection dir);
  void  calc_large_offset(param_straight_t &front, param_straight_t &back,
                          TurnDirection dir, bool exec_wall_off);
  void  calc_dia135_offset(param_straight_t &front, param_straight_t &back,
                           TurnDirection dir, bool exec_wall_off);
  void  calc_dia45_offset(param_straight_t &front, param_straight_t &back,
                          TurnDirection dir, bool exec_wall_off);

  std::shared_ptr<UserInterface> ui;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<sensing_result_entity_t> get_sensing_entity() {
    return sensing_result;
  }
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<PathCreator>   pc;
  TrajectoryCreator              tc;
  std::shared_ptr<PlanningTask>  pt;
  std::shared_ptr<LoggingTask>   lt;

  param_straight_t ps_front;
  param_straight_t ps_back;
  ego_odom_t       ego;
  bool             dia = false;
  param_straight_t ps;
  next_motion_t    nm;
  MotionResult     res_f;

  void wait_tick();
};
#endif
