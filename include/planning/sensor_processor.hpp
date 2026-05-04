#pragma once

#include "planning/astraea_types.hpp"
#include "structs.hpp"
#include <memory>
#include <vector>

using std::vector;

// センサー値 → mm 距離変換および補間テーブルを管理するクラス。
// check_sen_error / calc_sensor_pid など ee に依存するものは Step5 (ControlLaw) で扱う。
class SensorProcessor {
public:
  // センサー LP 値 → mm 距離に変換し sensing_result を更新する。tick 毎に呼ぶ。
  void calc_dist(std::shared_ptr<motion_tgt_val_t>        tgt_val,
                 std::shared_ptr<sensing_result_entity_t> sensing_result,
                 std::shared_ptr<input_param_t>           param);

  // 補間ユーティリティ (複数の呼び出し元から使用される)
  float interp1d(vector<float> &vx, vector<float> &vy, float x, bool extrapolate);
  int   interp1d(vector<int>   &vx, vector<int>   &vy, float x, bool extrapolate);

  // 設定テーブル (外部から直接代入)
  vector<float> log_table;
  vector<float> axel_degenerate_x,     axel_degenerate_y;
  vector<float> axel_degenerate_dia_x, axel_degenerate_dia_y;
  vector<float> sensor_deg_limitter_v,   sensor_deg_limitter_str;
  vector<float> sensor_deg_limitter_dia, sensor_deg_limitter_piller;

private:
  void  calc_dist_diff(std::shared_ptr<motion_tgt_val_t>        tgt_val,
                       std::shared_ptr<sensing_result_entity_t> sensing_result,
                       std::shared_ptr<input_param_t>           param);
  float calc_sensor_val(float data, float a, float b,
                        std::shared_ptr<input_param_t> param);
};
