#include "planning/sensor_processor.hpp"
#include "hardware/sync.h" // __dmb
#include "pico/stdlib.h"
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
  log_table.reserve(4097);
  for (int i = 0; i < 4097; i++) {
    log_table.emplace_back(std::log(i));
  }
}

__attribute__((noinline, section(".time_critical.sensor_processor")))
void SensorProcessor::calc_dist() {
  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::PIVOT)) {
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
    se->ego.left90_dist       = param->sensor_range_max;
    se->ego.left45_dist       = param->sensor_range_max;
    se->ego.left45_2_dist     = param->sensor_range_max;
    se->ego.left45_3_dist     = param->sensor_range_max;
    se->ego.front_dist        = param->sensor_range_max;
    se->ego.right45_dist      = param->sensor_range_max;
    se->ego.right45_2_dist    = param->sensor_range_max;
    se->ego.right45_3_dist    = param->sensor_range_max;
    se->ego.right90_dist      = param->sensor_range_max;

    se->ego.left45_dist_diff      = 0;
    se->ego.right45_dist_diff     = 0;
    se->ego.right45_2_dist_diff   = 0;
    se->ego.right45_3_dist_diff   = 0;
    se->ego.left45_2_dist_diff    = 0;
    se->ego.left45_3_dist_diff    = 0;
    se->ego.right90_dist_diff     = 0;
    se->ego.left90_dist_diff      = 0;
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

  // 切れ目(kireme)判定用の速度正規化差分 (2026-09-05、structs.hpp
  // input_param_t::kireme_diff_v_ref のコメント参照)。生の *_dist_diff は
  // WallOffController が別しきい値で使っているので触らず、別フィールドに出す。
  {
    float gain = 1.0f;
    const float v = std::fabs(tgt_val->ego_in.v);
    // 停止付近は 1tick の移動量がゼロに近く、正規化すると差分が発散する。
    // 走り出してから(v > kKiremeNormMinV)だけ有効にする。
    constexpr float kKiremeNormMinV = 50.0f;   // mm/s
    constexpr float kKiremeNormMaxGain = 10.0f;
    if (param->kireme_diff_v_ref > 0 && v > kKiremeNormMinV) {
      gain = std::clamp(param->kireme_diff_v_ref / v, 1.0f / kKiremeNormMaxGain,
                        kKiremeNormMaxGain);
    }
    se->ego.left45_dist_diff_norm = se->ego.left45_dist_diff * gain;
    se->ego.right45_dist_diff_norm = se->ego.right45_dist_diff * gain;
  }

  calc_dist_diff();
  update_pillar_trough();
  update_wall_edge();
}

__attribute__((noinline, section(".time_critical.sensor_processor")))
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
  // 2026-09-15: 切り捨て(int)から四捨五入へ。dataはego_estimator.cppの
  // LP値(led_param.lp_delay=0.995)で、生値が上昇中/上昇後は常に生値より
  // わずかに小さく、切り捨てると生値-1カウントのテーブルを引いていた。
  // ログ(logging_task.cpp calc_sensor)は整数生値で復元するため、firmwareの
  // ego.*_distとログの*_d列が1カウント分ずれ、遠距離側(前壁130〜150mm、
  // 生値20〜60)では約1〜1.6mmの差になる。この差でright90 nearが
  // sensor_range_far_max(150)を跨ぎ、wall_off_controller.cppの前壁補正が
  // 180フォールバック値で発火した(20260915_013256.csv idx1371、
  // log 149.87 vs firmware 150.80)。四捨五入なら実質的に生値と一致する。
  int idx = (int)(data + 0.5f);
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

__attribute__((noinline, section(".time_critical.interp")))
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

__attribute__((noinline, section(".time_critical.interp")))
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

__attribute__((noinline, section(".time_critical.sensor_processor")))
void SensorProcessor::update_pillar_trough() {
  const auto &pp = param->wall_off_dist;
  const float x = tgt_val->global_pos.dist;
  const auto mt = tgt_val->motion_type;

  // 形が意味を持たない区間(旋回・超信地・停止・前壁制御)は毎 tick 再アーム。
  // 既存の sen.*.sensor_dist の SLALOM リセット(trajectory_generator.cpp)と同じ考え。
  // STRAIGHT / SLA_BACK_STR / WALL_OFF 等では連続して追跡するので、谷底が
  // WALL_OFF 開始より前(直前の直線や SLA_BACK_STR 中)にあっても拾える。
  const bool rearm =
      (mt == MotionType::SLALOM || mt == MotionType::PIVOT ||
       mt == MotionType::PIVOT_PRE || mt == MotionType::PIVOT_PRE2 ||
       mt == MotionType::PIVOT_AFTER || mt == MotionType::PIVOT_OFFSET ||
       mt == MotionType::NONE || mt == MotionType::READY ||
       mt == MotionType::FRONT_CTRL);

  float travel = 0.0f;
  if (pillar_x_valid_) {
    travel = x - pillar_x_prev_;
    if (travel < 0.0f) travel = 0.0f; // global_pos.dist のリセット(走行開始)時
  }
  pillar_x_prev_ = x;
  pillar_x_valid_ = true;

  if (!pp.pillar_enable || rearm) {
    pillar_r_.arm(se->ego.right45_dist, x);
    pillar_l_.arm(se->ego.left45_dist, x);
  } else {
    PillarTroughParams p;
    p.depth_min = pp.pillar_depth_min;
    p.bottom_min = pp.pillar_bottom_min;
    p.bottom_max = pp.pillar_bottom_max;
    p.curv_th = pp.pillar_curv_th;
    p.curv_n = pp.pillar_curv_n;
    p.curv_sum = pp.pillar_curv_sum;
    p.slope_min = pp.pillar_slope_min;
    p.both_diff_min = pp.pillar_both_diff_min;
    p.rise_min = pp.pillar_rise_min;
    p.far_th = pp.pillar_far_th;
    p.max_lag = pp.pillar_max_lag;
    p.stale_dist = pp.pillar_stale_dist;
    p.vertex_interp = (pp.pillar_vertex_interp != 0);
    const bool fire_ok = std::fabs(se->ego.v_c) >= pp.pillar_min_v;
    pillar_r_.update(se->ego.right45_dist, se->ego.right45_2_dist_diff, x, travel,
                     fire_ok, p);
    pillar_l_.update(se->ego.left45_dist, se->ego.left45_2_dist_diff, x, travel,
                     fire_ok, p);
  }

  auto publish = [x](const PillarTroughDetector &d, pillar_trough_out_t &o) {
    o.bottom = d.bottom();
    o.bottom_x = d.bottom_x();
    o.peak = d.peak();
    o.fire_x = d.fire_x();
    o.lag = x - d.bottom_x();
    o.seq = d.seq();
    __dmb();
    o.state = d.state();
  };
  publish(pillar_r_, se->pillar_r);
  publish(pillar_l_, se->pillar_l);
}

__attribute__((noinline, section(".time_critical.sensor_processor")))
void SensorProcessor::update_wall_edge() {
  const auto &pp = param->wall_off_dist;
  const float x_now = tgt_val->global_pos.dist;
  const auto mt = tgt_val->motion_type;
  // 再アームする区間は柱の谷の検知(update_pillar_trough)と同じ。直線中も追跡して
  // おくのは、WALL_OFF の開始直後に壁の距離(過去 16〜4mm の中央値)が要るため。
  const bool rearm =
      (mt == MotionType::SLALOM || mt == MotionType::PIVOT ||
       mt == MotionType::PIVOT_PRE || mt == MotionType::PIVOT_PRE2 ||
       mt == MotionType::PIVOT_AFTER || mt == MotionType::PIVOT_OFFSET ||
       mt == MotionType::NONE || mt == MotionType::READY ||
       mt == MotionType::FRONT_CTRL);

  // se->wo は S3(read_enc_bat の終わり)で 1 tick 分まとめて写される。同じ Core1 の
  // 中なので途中の状態は見えない。同じ組を 2 回入れないよう seq で確かめる。
  const int wo_seq = se->wo.seq;
  if (rearm) {
    edge_l_.arm();
    edge_r_.arm();
  } else if (wo_seq != edge_wo_seq_) {
    const int n = std::clamp((int)se->wo.n, 0, 4);
    // サンプルの位置: global_pos.dist はエンコーダーを読んだ時刻(S3)の位置なので、
    // 読んだ時刻の差 × 速度で各サンプルの時刻へ戻す(速度は距離の積分と同じ
    // 先読みなしの値)。時刻はどれも tick(S0)の開始からの us。
    const float v = 0.5f * (se->ego.v_l_dist + se->ego.v_r_dist); // [mm/s]
    const float t_enc = 0.5f * ((float)se->t_encl + (float)se->t_encr);
    float xl[4], dl[4], xr[4], dr[4];
    for (int q = 0; q < n; q++) {
      xl[q] = x_now + v * ((float)se->wo.tl[q] - t_enc) * 1e-6f;
      xr[q] = x_now + v * ((float)se->wo.tr[q] - t_enc) * 1e-6f;
      dl[q] = calc_sensor_val((float)se->wo.l[q], param->sensor_gain.l45.a,
                              param->sensor_gain.l45.b);
      dr[q] = calc_sensor_val((float)se->wo.r[q], param->sensor_gain.r45.a,
                              param->sensor_gain.r45.b);
    }
    WallEdgeParams p;
    p.win_far = pp.edge_win_far;
    p.win_near = pp.edge_win_near;
    p.level_max = pp.edge_level_max;
    p.depth = pp.edge_depth;
    p.span = pp.edge_span;
    p.band = pp.edge_band;
    p.noise_back = pp.edge_noise_back;
    p.anchor_h = pp.edge_anchor_h;
    p.min_run = pp.edge_min_run;
    auto publish = [](const WallEdgeDetector &d, wall_edge_out_t &o) {
      o.edge_x = d.edge_x();
      o.fire_x = d.fire_x();
      o.level = d.level();
      __dmb();
      o.seq = d.seq();
    };
    if (edge_l_.update(xl, dl, n, p)) publish(edge_l_, se->edge_l);
    if (edge_r_.update(xr, dr, n, p)) publish(edge_r_, se->edge_r);
  }
  edge_wo_seq_ = wo_seq;
  se->edge_l.lag = x_now - edge_l_.edge_x();
  se->edge_r.lag = x_now - edge_r_.edge_x();
}
