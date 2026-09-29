#pragma once
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#include "gen_code_mpc/mpc_tgt_calc.h"
#pragma GCC diagnostic pop
#include "planning/astraea_types.hpp"
#include "planning/ego_estimator.hpp"
#include "planning/sensor_processor.hpp"
#include "planning/sla_start_align.hpp"
#include "structs.hpp"
#include <memory>
#include <vector>

// MPC 軌道生成・Kanayama 追従制御・tgt_val コピーをまとめる。
// 1kHz IRQ tick 内で generate() → calc_kanayama() → copy_tgt() の順に呼ぶ。
class TrajectoryGenerator {
public:
  void init(std::shared_ptr<motion_tgt_val_t> tgt_val,
            std::shared_ptr<input_param_t> param,
            std::shared_ptr<sensing_result_entity_t> sensing_result);
  // motor_enable() から呼ばれる: trj_length に合わせて trajectory_points を確保する
  void setup();
  void generate(float last_tgt_angle);

  void calc_kanayama(EgoEstimator &ego, SensorProcessor &sensor,
                     float last_tgt_angle);

  void copy_tgt(float dt);

  // 旋回の始まりを tick の途中へ合わせる(2026-09-30、sla_start_align.hpp)。
  // cp_request() が SLALOM の受信で arm(start_x = 旋回を始める global_pos.dist)、
  // それ以外の受信で disarm する。
  void arm_sla_align(float start_x) { sla_align_.arm(start_x); }
  void disarm_sla_align() { sla_align_.disarm(); }

  // tick() が直接書き込むため public
  t_ego mpc_next_ego{};

  // ControlLaw が読む出力値
  float v_cmd    = 0.0f;
  float w_cmd    = 0.0f;
  float ideal_v_r = 0.0f;
  float ideal_v_l = 0.0f;

  // trj_idx_v / trj_idx_val は input_param_t 経由で param->trj_idx_v/val を参照

private:
  int32_t    mpc_step = 1;
  t_dynamics dynamics{};
  t_ego      mpc_next_ego2{};
  t_ego      mpc_next_ego_prev{};
  mpc_tgt_calcModelClass mpc_tgt_calc;
  std::vector<t_ego> trajectory_points;
  SlaStartAlign      sla_align_;
  void generate_sla_aligned(float last_tgt_angle);

  std::shared_ptr<sensing_result_entity_t> se;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
  std::shared_ptr<input_param_t> param;
};
