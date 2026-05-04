#pragma once

#include "planning/astraea_types.hpp"
#include "structs.hpp"
#include "utils/kalman_filter.hpp"
#include "utils/kalman_filter_matrix.hpp"
#include <memory>

class EgoEstimator {
public:
  // センサーデータから自己位置・速度を推定し sensing_result / tgt_val
  // を更新する。 PlanningTask::tick() から毎周期呼ぶ。
  void update(std::shared_ptr<motion_tgt_val_t> tgt_val,
              std::shared_ptr<sensing_result_entity_t> sensing_result,
              std::shared_ptr<input_param_t> param, bool motor_en);

  // 以下は公開状態 (PlanningTask の公開メンバーから移動)
  KalmanFilter kf_w, kf_w2;
  KalmanFilter kf_v, kf_v_r, kf_v_l;
  KalmanFilter kf_dist, kf_ang, kf_ang2, kf_batt;
  KalmanFilterMatrix pos;
  kinematics_t odm{};
  kinematics_t kim{};
};
