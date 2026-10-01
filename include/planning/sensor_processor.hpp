#pragma once

#include "planning/astraea_types.hpp"
#include "planning/dia_post_edge_detector.hpp"
#include <cmath>
#include "planning/pillar_trough_detector.hpp"
#include "planning/wall_edge_detector.hpp"
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
  // 壁の切れ目の形の検知(2026-09-30)。calc_dist() の末尾で毎 tick 呼び、
  // 1 tick に 4 回読む 45° LED1(se->wo)を距離にして WallEdgeDetector へ入れ、
  // 結果を se->edge_l/r に公開する。
  void  update_wall_edge();
  WallEdgeDetector edge_l_;
  WallEdgeDetector edge_r_;
  int   edge_wo_seq_ = -1;
  // 柱の谷底を細かいサンプルで求め直した発火の番号(PillarTroughDetector::seq)
  int   pillar_hf_seq_l_ = -1;
  int   pillar_hf_seq_r_ = -1;
  static constexpr float kPillarHfMaxShift = 3.0f; // [mm] 1 tick 1 点の谷底からのずれの上限
  // 斜めの柱の立ち下がりから横位置を出す(2026-10-01)。calc_dist() の末尾で毎 tick 呼び、
  // 45° LED1 の S0 の読み(se->wo.l/r[0])を入れて、結果を se->dia_post に公開する。
  void  update_dia_post_edge();
  DiaPostEdgeDetector dia_post_;
  int   dia_post_wo_seq_ = -1;
  bool  dia_post_hf_[2] = {false, false}; // 左右それぞれ S1〜S3 を使っているか
  float dia_c_ = 0.0f;         // ジャイロの向き(右向き +)の走行距離での積分 [mm·rad]
  float dia_c_x_ = 0.0f;
  bool  dia_c_valid_ = false;
  // 直進の壁なし区間の柱の立ち下がりから横位置を出す(2026-10-02、structs.hpp str_post_ctrl_t)。
  // 同じ検知器を pair_pitch 0 / same_pitch 90 / 相対しきい値で使い、結果を se->str_post に公開する。
  // 向きはジャイロの純積分 ang_kf_sum で数える(壁のスナップで ang が切られても折れない)。
  void  update_str_post_edge();
  DiaPostEdgeDetector str_post_;
  int   str_post_wo_seq_ = -1;
  bool  str_post_hf_[2] = {false, false};
  float str_c_ = 0.0f;
  float str_c_x_ = 0.0f;
  bool  str_c_valid_ = false;
  float str_ref_ = 0.0f;          // この直進の向きの基準 [rad](ang_kf_sum を 90° の倍数へ丸めたもの)
  // 壁からの引き継ぎ(str_post_ctrl_t::wall_seed): 両壁の区間の長さと、区間の終わりの値の指数平均
  bool  str_wall_active_ = false;
  bool  str_wall_hold_ = false;   // 読みが離れ始めて区間を閉じた後、両壁が一度消えるまで新しい区間を始めない
  float str_wall_x0_ = 0.0f;
  float str_wall_x_prev_ = 0.0f;
  float str_wall_ema_ang_ = 0.0f; // ang_kf_sum [rad]
  float str_wall_ema_l_ = 0.0f;   // L45 [mm]
  float str_wall_ema_r_ = 0.0f;   // R45 [mm]
  float calc_sensor_val(float data, float a, float b);
  // 距離 [mm] → 生値(calc_sensor_val の逆、raw = exp(a/(dist + b)))。縁の条件の換算用
  static float raw_of_dist(float dist, float a, float b) { return std::exp(a / (dist + b)); }
};
