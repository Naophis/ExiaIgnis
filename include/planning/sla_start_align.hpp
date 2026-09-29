#pragma once
// 旋回(SLALOM)の始まりを tick の途中の位置へ合わせる(2026-09-30、sla_start_align)
//
// 従来の切り替え: Core0 の go_straight(SLA_FRONT_STR)は global_pos.dist が
// 目標位置 X を越えた tick k で終わり、SLALOM の指令は tick k+1 の planning で
// 効く。距離が進むのは 1ms に 1 回(S3)なので、tick k で X をどれだけ越えて
// いたか(0〜1 tick、2200mm/s で 0〜2.2mm)がそのまま旋回の始まりのばらつきに
// なっていた(一様分布で ±0.63mm)。Core0 を速く起こしても、指令が効くのは
// planning の 1kHz の tick だけなので消えない。
//
// ここでは、旋回の角速度の出力列(generate() が tick ごとに出す O_0, O_1, ...)を
// tick の途中まで遅らせて出す。
//   - Core0 は X の 0.5 tick 手前で SLA_FRONT_STR を終え、X を SLALOM の指令に
//     付けて送る(MotionPlanning::go_straight / slalom)。
//   - planning は受け取った tick で tau = (global_pos.dist − X)/(v·dt) − 1.5 を
//     求める。tau は「従来の出力列の 0 番目を出すべき時刻」からのずれ [tick] で、
//     従来の切り替えの平均(X を 0.5 tick 越えた tick の次)と同じ位置が tau = 0。
//     tau < −1 の間は直進を 1 tick 出して待ち(最大 kMaxWait)、
//     tau ∈ [−1, 0) になったら f = tau + 1 として、以後ずっと
//     出力 = O_{j} と O_{j+1} を f で直線補間したもの(O_{−1} は旋回前の直進)。
//     tau ≥ 0(指令が遅れた)は f = 1 で、従来と同じ出力列になる。
//   - 生成器の中身(カウンタ・角速度の積分)は従来のまま影の状態(prev_)で進め、
//     補間は出力(ego_in へ写す値・trajectory_points)にだけ掛ける。生成器の入力は
//     「ego_in のうち copy_tgt() が生成器の出力から写す項目」を影の値に置き換えた
//     もの(sla_state_fields)で、f = 1 なら従来と同じ入力・同じ出力になる。
//
// 平均の旋回位置は従来と同じになるように tau の基準を取ってあるので、ターンの
// front/back の調整値はそのまま使える。変わるのはばらつきだけ。
//
// Pico SDK に依存しない。ホスト検証は tests/sla_align_host/。

#include "gen_code_mpc/bus.h"
#include <algorithm>
#include <cmath>

// b = a + (b − a)·f(連続量、b をその場で書き換える)。離散量(sla_param / state /
// pivot_state / decel_delay_cnt)と外から与える定数(ff_duty_low_*)は b のまま。
// f ≥ 1 は b をそのまま残す(従来と同じ出力列。a + (b − a) の丸めも避ける)。
inline void sla_lerp_ego(t_ego &b, const t_ego &a, float f) {
  if (f >= 1.0f) return;
#define SLA_LERP(x) b.x = a.x + (b.x - a.x) * f
  SLA_LERP(v); SLA_LERP(v_r); SLA_LERP(v_l);
  SLA_LERP(pos_x); SLA_LERP(pos_y);
  SLA_LERP(ideal_px); SLA_LERP(ideal_py);
  SLA_LERP(accl); SLA_LERP(w); SLA_LERP(alpha); SLA_LERP(alpha2);
  SLA_LERP(dist); SLA_LERP(ang); SLA_LERP(img_dist); SLA_LERP(img_ang);
  SLA_LERP(ideal_point.x); SLA_LERP(ideal_point.y); SLA_LERP(ideal_point.theta);
  SLA_LERP(ideal_point.v); SLA_LERP(ideal_point.w); SLA_LERP(ideal_point.slip_angle);
  SLA_LERP(slip_point.x); SLA_LERP(slip_point.y); SLA_LERP(slip_point.theta);
  SLA_LERP(slip_point.v); SLA_LERP(slip_point.w); SLA_LERP(slip_point.slip_angle);
  SLA_LERP(kanayama_point.x); SLA_LERP(kanayama_point.y); SLA_LERP(kanayama_point.theta);
  SLA_LERP(kanayama_point.v); SLA_LERP(kanayama_point.w);
  SLA_LERP(trj_diff.x); SLA_LERP(trj_diff.y); SLA_LERP(trj_diff.theta);
  SLA_LERP(delay_accl); SLA_LERP(delay_v);
  SLA_LERP(cnt_delay_accl_ratio); SLA_LERP(cnt_delay_decel_ratio);
  SLA_LERP(slip.beta); SLA_LERP(slip.vx); SLA_LERP(slip.vy); SLA_LERP(slip.v); SLA_LERP(slip.accl);
  SLA_LERP(ff_duty_l); SLA_LERP(ff_duty_r);
  SLA_LERP(ff_duty_front); SLA_LERP(ff_duty_roll);
  SLA_LERP(ff_duty_rpm_r); SLA_LERP(ff_duty_rpm_l);
  SLA_LERP(ff_front_torque); SLA_LERP(ff_roll_torque);
  SLA_LERP(ff_friction_torque_l); SLA_LERP(ff_friction_torque_r);
#undef SLA_LERP
}

// TrajectoryGenerator::copy_tgt() が生成器の出力(src)から ego_in(dst)へ写す
// 項目だけを写す。copy_tgt() を変えたらここも合わせること。
inline void sla_state_fields(t_ego &dst, const t_ego &src, float tire_tread) {
  dst.accl = src.accl;
  dst.alpha = src.alpha;
  dst.pivot_state = src.pivot_state;
  dst.sla_param = src.sla_param;
  dst.state = src.state;
  dst.decel_delay_cnt = src.decel_delay_cnt;
  dst.v = src.v;
  dst.v_l = src.v - src.w * tire_tread / 2;
  dst.v_r = src.v + src.w * tire_tread / 2;
  dst.w = src.w;
  dst.img_ang = src.img_ang;
  dst.img_dist = src.img_dist;
  dst.slip_point.slip_angle = src.slip_point.slip_angle;
  dst.cnt_delay_accl_ratio = src.cnt_delay_accl_ratio;
  dst.cnt_delay_decel_ratio = src.cnt_delay_decel_ratio;
  dst.slip.beta = src.slip.beta;
  dst.slip.accl = src.slip.accl;
  dst.slip.v = src.slip.v;
  dst.slip.vx = src.slip.vx;
  dst.slip.vy = src.slip.vy;
  dst.ideal_px = src.ideal_px;
  dst.ideal_py = src.ideal_py;
}

class SlaStartAlign {
public:
  static constexpr int kMaxWait = 3; // 直進で待つ tick の上限

  void arm(float start_x) {
    armed_ = true;
    started_ = false;
    start_x_ = start_x;
    frac_ = -1.0f;
    tau0_ = 0.0f;
    wait_ = 0;
  }
  void disarm() {
    armed_ = false;
    started_ = false;
  }
  bool armed() const { return armed_; }
  float frac() const { return frac_; } // 決めた f(未決定は −1)
  float tau0() const { return tau0_; } // 待っている間は今の tau、決めた後は決めたときの tau [tick]
  int wait() const { return wait_; }   // 直進で待った tick 数

  // tau: 従来の出力列の 0 番目を出すべき時刻からのずれ [tick]
  static float tau(float gx, float start_x, float v, float dt) {
    return (gx - start_x) / (std::max(std::fabs(v), 1.0f) * dt) - 1.5f;
  }

  // 1 tick 分。step(in, mode, pts, n) は in から n 点を pts に作る(pts[0] が次の
  // 状態、生成器の座標系 = img_ang に lta を足したもの)。last_raw は前の tick の
  // 生成器の出力そのもの(ego_in の座標系、ControlLaw が書き換える前)。out は次の
  // 状態(ego_in の座標系)、pts は補間済みの点列(lta を足した座標系)で返す。
  // raw() はこの tick の生成器の出力そのもの(次の tick の last_raw)。
  template <class Step>
  void tick(const t_ego &ego_in, const t_ego &last_raw, float gx, float dt,
            float lta, float tire_tread, int straight_mode, int slalom_mode,
            Step &&step, t_ego *pts, int n, t_ego &out) {
    if (!started_) {
      const float t = tau(gx, start_x_, ego_in.v, dt);
      tau0_ = t;
      if (t < -1.0f && wait_ < kMaxWait) {
        wait_++;
        t_ego in = ego_in;
        in.img_ang += lta;
        step(in, straight_mode, pts, n);
        out = pts[0];
        out.img_ang -= lta;
        prev_ = out;
        return;
      }
      started_ = true;
      frac_ = std::clamp(t + 1.0f, 0.0f, 1.0f);
      // O_{−1}: 旋回前の直進の出力。FF・alpha2 などは前の tick の生成器の出力、
      // copy_tgt() が写す項目(角度・距離・カウンタ等)は SLALOM の受信で付け直した
      // 後の ego_in の値(img_dist・img_ang は受信で基準が変わる)。
      prev_ = last_raw;
      sla_state_fields(prev_, ego_in, tire_tread);
    }
    // 生成器の入力: ego_in のうち copy_tgt() が写す項目を影の状態(O_j)に戻したもの
    t_ego in = ego_in;
    sla_state_fields(in, prev_, tire_tread);
    in.img_ang += lta; // 生成器の座標系
    step(in, slalom_mode, pts, n);
    // 出力は O_j(prev_、FF も含めた出力そのもの)と O_{j+1} の間。影を O_{j+1} へ進める
    out = pts[0];
    out.img_ang -= lta;
    if (frac_ < 1.0f) {
      const t_ego raw = out;
      sla_lerp_ego(out, prev_, frac_);
      // 点列も 1 つ前の点との間へ(pts[−1] = O_j、生成器の座標系)
      t_ego before = prev_;
      before.img_ang += lta;
      for (int i = 0; i < n; i++) {
        const t_ego cur = pts[i];
        sla_lerp_ego(pts[i], before, frac_);
        before = cur;
      }
      prev_ = raw;
    } else {
      prev_ = out;
    }
  }
  const t_ego &raw() const { return prev_; }

private:
  bool armed_ = false;
  bool started_ = false;
  float start_x_ = 0.0f;
  float frac_ = -1.0f;
  float tau0_ = 0.0f;
  int wait_ = 0;
  t_ego prev_{};
};
