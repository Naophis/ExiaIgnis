#pragma once

#include "planning/astraea_types.hpp"
#include "planning/pillar_trough_detector.hpp"
#include "structs.hpp"
#include <memory>
#include <vector>

using std::vector;

// センサー値 → mm 距離変換および補間テーブルを管理するクラス。
// check_sen_error / calc_sensor_pid など ee に依存するものは Step5 (ControlLaw) で扱う。
class SensorProcessor {
public:
  // EgoEstimator::init() と同様に se/param を受け取りフィールドに保持する。
  void init(std::shared_ptr<sensing_result_entity_t> sensing_result,
            std::shared_ptr<input_param_t>           param,
            std::shared_ptr<motion_tgt_val_t>       tgt_val);

  // センサー LP 値 → mm 距離に変換し se を更新する。tick 毎に呼ぶ。
  void calc_dist();

  // 補間ユーティリティ (複数の呼び出し元から使用される)
  float interp1d(vector<float> &vx, vector<float> &vy, float x, bool extrapolate);
  int   interp1d(vector<int>   &vx, vector<int>   &vy, float x, bool extrapolate);

  // 設定テーブル (外部から直接代入)
  vector<float> log_table;

private:
  std::shared_ptr<sensing_result_entity_t> se;
  std::shared_ptr<input_param_t>           param;
  std::shared_ptr<motion_tgt_val_t>       tgt_val;

  void  calc_dist_diff();
  // 柱の谷(下に凸)検知(2026-09-23)。calc_dist() の末尾で毎 tick 呼び、
  // 結果を se->pillar_l/r に公開する。
  void  update_pillar_trough();
  PillarTroughDetector pillar_l_;
  PillarTroughDetector pillar_r_;
  float pillar_x_prev_ = 0.0f;
  bool  pillar_x_valid_ = false;
  float calc_sensor_val(float data, float a, float b);
};
