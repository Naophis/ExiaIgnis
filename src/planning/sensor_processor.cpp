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
  update_dia_post_edge();
  update_str_post_edge();
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
  // サンプルの位置: global_pos.dist はエンコーダーを読んだ時刻(S3)の位置なので、
  // 読んだ時刻の差 × 速度で各サンプルの時刻へ戻す(速度は距離の積分と同じ
  // 先読みなしの値)。時刻はどれも tick(S0)の開始からの us。
  const float v = 0.5f * (se->ego.v_l_dist + se->ego.v_r_dist); // [mm/s]
  const float t_enc = 0.5f * ((float)se->t_encl + (float)se->t_encr);
  if (rearm) {
    edge_l_.arm();
    edge_r_.arm();
  } else if (wo_seq != edge_wo_seq_) {
    const int n = std::clamp((int)se->wo.n, 0, 4);
    // wall_off_hf_mode 1 で細かく読んでいる tick は、S1〜S3 の読みだけを入れる。
    // S0 は同じ位置でも S1〜S3 より低く出るので(右の柱で最大 4mm 遠く、
    // 20260930_030005)、混ぜると 1 tick ごとの段になる。
    const bool extras_only = (pp.hf_mode != 0) && n == 4;
    float xl[4], dl[4], xr[4], dr[4];
    int nl = 0, nr = 0;
    for (int q = extras_only ? 1 : 0; q < n; q++) {
      if (!extras_only || se->wo.tl[q] > 0) {
        xl[nl] = x_now + v * ((float)se->wo.tl[q] - t_enc) * 1e-6f;
        dl[nl] = calc_sensor_val((float)se->wo.l[q], param->sensor_gain.l45.a,
                                 param->sensor_gain.l45.b);
        nl++;
      }
      if (!extras_only || se->wo.tr[q] > 0) {
        xr[nr] = x_now + v * ((float)se->wo.tr[q] - t_enc) * 1e-6f;
        dr[nr] = calc_sensor_val((float)se->wo.r[q], param->sensor_gain.r45.a,
                                 param->sensor_gain.r45.b);
        nr++;
      }
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
    if (edge_l_.update(xl, dl, nl, p)) publish(edge_l_, se->edge_l);
    if (edge_r_.update(xr, dr, nr, p)) publish(edge_r_, se->edge_r);
  }
  edge_wo_seq_ = wo_seq;
  se->edge_l.lag = x_now - edge_l_.edge_x();
  se->edge_r.lag = x_now - edge_r_.edge_x();

  // 2026-09-30: 柱の谷底の位置を、貯めてある細かいサンプルで求め直す。柱かどうかの
  // 判定は PillarTroughDetector(update_pillar_trough、この関数の前に更新済み)のまま。
  // 発火した tick から、窓の先の端までサンプルが来るまで毎 tick 試す。
  auto refine = [&](const PillarTroughDetector &pd, const WallEdgeDetector &ed,
                    pillar_trough_out_t &o, int &done_seq, float t_s0) {
    if (!pp.pillar_hf || pp.hf_mode == 0 || !pd.fired()) {
      o.lag_hf = 0.0f;
      return;
    }
    if (done_seq != pd.seq()) {
      // 1 tick 1 点の谷底(planning の時刻の位置)を、S0 を読んだ時刻の位置へ戻して
      // 初期値にする
      const float x0 = pd.bottom_x() - v * (t_enc - t_s0) * 1e-6f;
      float xv = 0.0f, dv = 0.0f;
      const int r = ed.trough_vertex(x0, pp.pillar_hf_win, kPillarHfMaxShift, xv, dv);
      if (r == WallEdgeDetector::VERTEX_WAIT) {
        o.lag_hf = 0.0f;
        return;
      }
      done_seq = pd.seq();
      o.bottom_x_hf = xv;
      __dmb();
      o.hf_tag = pd.seq() * 4 + r;
    }
    o.lag_hf = ((o.hf_tag & 3) == WallEdgeDetector::VERTEX_OK)
                   ? x_now - o.bottom_x_hf
                   : -1.0f;
  };
  refine(pillar_l_, edge_l_, se->pillar_l, pillar_hf_seq_l_, (float)se->wo.tl[0]);
  refine(pillar_r_, edge_r_, se->pillar_r, pillar_hf_seq_r_, (float)se->wo.tr[0]);
}

__attribute__((noinline, section(".time_critical.sensor_processor")))
void SensorProcessor::update_dia_post_edge() {
  const float x_now = tgt_val->global_pos.dist;
  const auto mt = tgt_val->motion_type;
  // 再アームする区間は壁の切れ目の検知(update_wall_edge)と同じ。旋回を抜けたら
  // 斜めかどうかによらず追う(直交の直線では左右の縁の間隔が 0 か 90mm 前後になり、
  // 63.64 ± tol に入らないので組はできない。実走 11 本のホスト再生で斜めの外は 0 組)。
  const bool rearm =
      (mt == MotionType::SLALOM || mt == MotionType::PIVOT ||
       mt == MotionType::PIVOT_PRE || mt == MotionType::PIVOT_PRE2 ||
       mt == MotionType::PIVOT_AFTER || mt == MotionType::PIVOT_OFFSET ||
       mt == MotionType::NONE || mt == MotionType::READY ||
       mt == MotionType::FRONT_CTRL);
  const int wo_seq = se->wo.seq;
  // ジャイロで見た右向きの向き [rad]。斜めの直進(SLA_BACK_STR / STRAIGHT / WALL_OFF_DIA /
  // SLA_FRONT_STR)は目標の角度が 0 なので ang_kf をそのまま使う(img_ang を引くと、
  // 区間が切り替わる tick に基準の切り替えが 1 tick ずれて 45° 等が混じる)。
  const float psi_g = -se->ego.ang_kf;
  if (rearm) {
    dia_post_.arm();
    se->dia_post.n_pairs = 0;
    se->dia_post.eps = 0;
    se->dia_post.psi0 = 0;
    se->dia_post.n_psi = 0;
    dia_c_ = 0.0f;
    dia_c_valid_ = false;
  } else {
    if (dia_c_valid_) dia_c_ += psi_g * (x_now - dia_c_x_);
    dia_c_x_ = x_now;
    dia_c_valid_ = true;
  }
  if (!rearm && wo_seq != dia_post_wo_seq_ && se->wo.n >= 1) {
    // その側を 1 tick に 4 回読んでいれば(wall_off_hf_mode、斜めの直進は dia_post_ctrl で
    // 左右とも)S1〜S3 だけを、読んでいなければ S0 を使う。S0 は同じ位置でも S1〜S3 より
    // 低く出る(右で 10〜17 raw)ので混ぜない。読み方が変わったらその側の途中状態を捨てる。
    // 位置は読んだ時刻へ戻す(update_wall_edge と同じ)。左右で読んだ時刻が違う分も入る。
    const float v = 0.5f * (se->ego.v_l_dist + se->ego.v_r_dist); // [mm/s]
    const float t_enc = 0.5f * ((float)se->t_encl + (float)se->t_encr);
    DiaPostEdgeParams p;
    p.kappa = param->dia_post_ctrl.kappa;
    p.conf_accel = param->dia_post_ctrl.conf_accel;
    dia_post_.set_accel(tgt_val->ego_in.accl);
    const int n = std::clamp((int)se->wo.n, 0, 4);
    bool paired = false;
    for (int k = 0; k < 2; k++) {
      const auto side = (k == 0) ? DiaPostEdgeDetector::LEFT : DiaPostEdgeDetector::RIGHT;
      const auto &val = (k == 0) ? se->wo.l : se->wo.r;
      const auto &tim = (k == 0) ? se->wo.tl : se->wo.tr;
      const bool hf = (n == 4) && tim[1] > 0;
      if (hf != dia_post_hf_[k]) {
        dia_post_.reset_side(side);
        dia_post_hf_[k] = hf;
      }
      for (int q = hf ? 1 : 0; q < (hf ? 4 : 1); q++) {
        if (hf && tim[q] <= 0) continue;
        const float xs = x_now + v * ((float)tim[q] - t_enc) * 1e-6f;
        const float cs = dia_c_ + psi_g * (xs - x_now);
        paired |= dia_post_.update(side, xs, (float)val[q], cs, psi_g, p);
      }
    }
    if (paired) {
      se->dia_post.delta = dia_post_.delta();
      se->dia_post.pos = dia_post_.pos();
      se->dia_post.eps = dia_post_.eps_deg();
      se->dia_post.n_pairs = dia_post_.n_pairs();
      se->dia_post.psi0 = dia_post_.psi0();
      se->dia_post.n_psi = dia_post_.n_psi();
      __dmb();
      se->dia_post.seq = dia_post_.seq();
    }
  }
  dia_post_wo_seq_ = wo_seq;
  se->dia_post.lag = x_now - se->dia_post.pos;
  se->dia_post.dnow = (dia_post_.n_pairs() >= 1) ? dia_post_.now_delta(x_now, dia_c_) : 0.0f;
}

// 直進の壁なし区間の柱の立ち下がり(2026-10-02、structs.hpp str_post_ctrl_t)。update_dia_post_edge
// と同じ入れ方(再アームの区間・S1〜S3 か S0 か・位置の戻し)で、検知器を直進の柱の並び
// (pair_pitch 0 / same_pitch 90)と相対しきい値で動かす。向きはジャイロの純積分 ang_kf_sum から
// この直進の基準(90° の倍数)を引いたもの(右向き + に符号を変える)。斜めの直進では左右の縁が
// 63.64mm ずれるので組はできない。
__attribute__((noinline, section(".time_critical.sensor_processor")))
void SensorProcessor::update_str_post_edge() {
  const auto &pc = param->str_post_ctrl;
  const float x_now = tgt_val->global_pos.dist;
  const auto mt = tgt_val->motion_type;
  const bool rearm =
      (mt == MotionType::SLALOM || mt == MotionType::PIVOT ||
       mt == MotionType::PIVOT_PRE || mt == MotionType::PIVOT_PRE2 ||
       mt == MotionType::PIVOT_AFTER || mt == MotionType::PIVOT_OFFSET ||
       mt == MotionType::NONE || mt == MotionType::READY ||
       mt == MotionType::FRONT_CTRL);
  const int wo_seq = se->wo.seq;
  if (rearm) {
    str_post_.arm();
    if (pc.psi0_w > 0.0f) str_post_.set_psi0_prior(0.0f, pc.psi0_w);
    se->str_post.n_pairs = 0;
    se->str_post.eps = 0;
    se->str_post.psi0 = 0;
    se->str_post.n_psi = 0;
    str_c_ = 0.0f;
    str_c_valid_ = false;
    str_wall_active_ = false;
  } else if (!str_c_valid_) {
    // 直進の最初: 向きの基準 = ang_kf_sum を 90° の倍数へ丸めたもの(旋回の残りと壁のスナップの
    // 分は ψ0 が吸う)。ang_kf_sum をそのまま使うと 90° 旋回の後の直進で ψ_g が −90° になり、
    // κ·ψ の補正(160mm/rad)が −250mm になって機体が回った(20261002_024646)。
    str_ref_ = std::round(se->ang_kf_sum / (float)M_PI_2) * (float)M_PI_2;
    se->str_post.ref = str_ref_;
    str_c_x_ = x_now;
    str_c_valid_ = true;
  }
  const float psi_g = -(se->ang_kf_sum - str_ref_); // この直進の向きに対するジャイロの向き [rad](右 +)
  if (!rearm) {
    str_c_ += psi_g * (x_now - str_c_x_);
    str_c_x_ = x_now;
  }
  // 壁からの引き継ぎ(structs.hpp str_post_ctrl_t::wall_seed)。両壁(45° が 30〜60mm)が続く間、
  // ang_kf_sum と L45 / R45 を距離 seed_tau の指数平均で追い、区間が seed_min_len 以上続いて
  // 終わった tick に、横位置(読みの座標 (L − R)/2 + k0)を組 1 つ分、ang_kf_sum を ψ0 の事前値
  // として検知器へ入れる(壁に沿って走っていた = 迷路に対して向き 0 とみなす)。横位置は実際の
  // mm(検知器の δ も (読み − k0)/gain で実際の mm)。
  // 区間の終わりは「両壁の条件が外れた」ではなく「どちらかの読みが指数平均から seed_dev 以上
  // 離れた」で決める: 壁の終わりは 45° の読みが約 20mm かけて遠のく(004431: L 46 → 60 で x 88 → 102)
  // ので、60mm を越えるまで待つと区間の終わりの値がその上りを含んで (L − R)/2 が 1.1 → 1.9mm に
  // ずれる。閉じた後は両壁が一度消えるまで新しい区間を始めない(上りの途中で始め直さない)。
  if (!rearm && pc.wall_seed) {
    const float l = se->ego.left45_dist, r = se->ego.right45_dist;
    const bool both = l > 30.0f && l < 60.0f && r > 30.0f && r < 60.0f;
    if (!both) str_wall_hold_ = false;
    bool close = false;
    if (both && !str_wall_hold_) {
      if (!str_wall_active_) {
        str_wall_active_ = true;
        str_wall_x0_ = x_now;
        str_wall_ema_ang_ = se->ang_kf_sum - str_ref_;
        str_wall_ema_l_ = l;
        str_wall_ema_r_ = r;
        str_wall_x_prev_ = x_now;
      } else if (std::fabs(l - str_wall_ema_l_) > pc.seed_dev || std::fabs(r - str_wall_ema_r_) > pc.seed_dev) {
        close = true;
        str_wall_hold_ = true;
      } else {
        const float a = (pc.seed_tau > 0.0f) ? std::min(1.0f, (x_now - str_wall_x_prev_) / pc.seed_tau) : 1.0f;
        str_wall_ema_ang_ += ((se->ang_kf_sum - str_ref_) - str_wall_ema_ang_) * a;
        str_wall_ema_l_ += (l - str_wall_ema_l_) * a;
        str_wall_ema_r_ += (r - str_wall_ema_r_) * a;
        str_wall_x_prev_ = x_now;
      }
    } else if (str_wall_active_) {
      close = true;
    }
    if (close) {
      str_wall_active_ = false;
      if (str_wall_x_prev_ - str_wall_x0_ >= pc.seed_min_len) {
        // 迷路に対してまっすぐ(ψ_true = −ang_kf_sum + ψ0 = 0)→ ψ0 = ang_kf_sum
        const float dw = 0.5f * (str_wall_ema_l_ - str_wall_ema_r_); // 実際の横位置 [mm]
        const float gain = (pc.gain > 0.05f) ? pc.gain : 1.0f;
        str_post_.seed(str_wall_x_prev_, str_c_ + psi_g * (str_wall_x_prev_ - x_now),
                       -str_wall_ema_ang_, dw, str_wall_ema_ang_, pc.psi0_w, pc.kappa / gain);
        se->str_post.delta = str_post_.delta();
        se->str_post.pos = str_post_.pos();
        se->str_post.n_pairs = str_post_.n_pairs();
        se->str_post.psi0 = str_post_.psi0();
        se->str_post.n_psi = str_post_.n_psi();
        __dmb();
        se->str_post.seq = str_post_.seq();
      }
    }
  }
  if (!rearm && wo_seq != str_post_wo_seq_ && se->wo.n >= 1) {
    const float v = 0.5f * (se->ego.v_l_dist + se->ego.v_r_dist); // [mm/s]
    const float t_enc = 0.5f * ((float)se->t_encl + (float)se->t_encr);
    DiaPostEdgeParams p;
    p.pair_pitch = 0.0f;
    p.same_pitch = 90.0f;
    p.gate_kmax = 3;
    p.scale_fix = 0;
    p.conf_accel = 0.0f;
    p.tol = pc.tol;
    p.gain = (pc.gain > 0.05f) ? pc.gain : 1.0f;
    p.offset = pc.k0;
    p.kappa = pc.kappa / p.gain; // 読みの座標 → 実際の mm/rad
    p.rel_thr = (pc.rel_thr > 0.0f) ? pc.rel_thr : 0.5f;
    p.low_ratio = pc.low_ratio;
    p.contrast_min = pc.contrast_min;
    p.rise_max = pc.rise_max;
    p.fall_max = pc.fall_max;
    const int n = std::clamp((int)se->wo.n, 0, 4);
    bool paired = false;
    for (int k = 0; k < 2; k++) {
      const auto side = (k == 0) ? DiaPostEdgeDetector::LEFT : DiaPostEdgeDetector::RIGHT;
      const auto &val = (k == 0) ? se->wo.l : se->wo.r;
      const auto &tim = (k == 0) ? se->wo.tl : se->wo.tr;
      const auto &g = (k == 0) ? param->sensor_gain.l45 : param->sensor_gain.r45;
      // 山の下限は距離で指定し、側ごとのゲインで生値へ換算する
      p.peak_min = raw_of_dist(pc.post_dist_max, g.a, g.b);
      const bool hf = (n == 4) && tim[1] > 0;
      if (hf != str_post_hf_[k]) {
        str_post_.reset_side(side);
        str_post_hf_[k] = hf;
      }
      for (int q = hf ? 1 : 0; q < (hf ? 4 : 1); q++) {
        if (hf && tim[q] <= 0) continue;
        const float xs = x_now + v * ((float)tim[q] - t_enc) * 1e-6f;
        const float cs = str_c_ + psi_g * (xs - x_now);
        paired |= str_post_.update(side, xs, (float)val[q], cs, psi_g, p);
      }
    }
    if (paired) {
      se->str_post.delta = str_post_.delta();
      se->str_post.pos = str_post_.pos();
      se->str_post.eps = str_post_.eps_deg();
      se->str_post.n_pairs = str_post_.n_pairs();
      se->str_post.psi0 = str_post_.psi0();
      se->str_post.n_psi = str_post_.n_psi();
      __dmb();
      se->str_post.seq = str_post_.seq();
    }
  }
  str_post_wo_seq_ = wo_seq;
  se->str_post.lag = x_now - se->str_post.pos;
  // 制御へ渡す形(車軸 + head_gain·向き、実際の mm)。斜め(dia_post.dnow)は車軸の横位置なので別
  se->str_post.dnow =
      (str_post_.n_pairs() >= 1) ? str_post_.now_delta_ctrl(x_now, str_c_, psi_g, pc.head_gain) : 0.0f;
}
