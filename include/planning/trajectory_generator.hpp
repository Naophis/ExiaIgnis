#pragma once
#include "gen_code_mpc/mpc_tgt_calc.h"
#include "planning/astraea_types.hpp"
#include "planning/ego_estimator.hpp"
#include "planning/sensor_processor.hpp"
#include "structs.hpp"
#include <memory>
#include <vector>

// MPC 軌道生成・Kanayama 追従制御・tgt_val コピーをまとめる。
// 1kHz IRQ tick 内で generate() → calc_kanayama() → copy_tgt() の順に呼ぶ。
class TrajectoryGenerator {
public:
  void generate(std::shared_ptr<motion_tgt_val_t> tgt_val,
                std::shared_ptr<input_param_t>    param,
                float last_tgt_angle);

  void calc_kanayama(std::shared_ptr<motion_tgt_val_t>        tgt_val,
                     std::shared_ptr<sensing_result_entity_t> sensing_result,
                     std::shared_ptr<input_param_t>           param,
                     EgoEstimator   &ego,
                     SensorProcessor &sensor,
                     float last_tgt_angle);

  void copy_tgt(std::shared_ptr<motion_tgt_val_t>        tgt_val,
                std::shared_ptr<sensing_result_entity_t> sensing_result,
                std::shared_ptr<input_param_t>           param,
                float dt);

  // tick() が直接書き込むため public
  t_ego mpc_next_ego{};

  // ControlLaw が読む出力値
  float v_cmd    = 0.0f;
  float w_cmd    = 0.0f;
  float ideal_v_r = 0.0f;
  float ideal_v_l = 0.0f;

  // 外部から設定するルックアップテーブル
  std::vector<int> trj_idx_v;
  std::vector<int> trj_idx_val;

private:
  int32_t    mpc_step = 1;
  t_dynamics dynamics{};
  t_ego      mpc_next_ego2{};
  t_ego      mpc_next_ego_prev{};
  mpc_tgt_calcModelClass mpc_tgt_calc;
  std::vector<t_ego> trajectory_points;
};
