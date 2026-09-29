// 旋回の始まりを tick の途中へ合わせる処理(include/planning/sla_start_align.hpp)の
// ホスト検証(2026-09-30)。生成コード mpc_tgt_calc をそのまま使う。
//
// 機体の代わりに「出力の v・w をそのまま 1 tick 保つ理想の追従」で経路を積分し、
// 旋回を始める位置 X が tick の格子のどこに来るか(u = 0〜1)を振って、
//   従来: SLA_FRONT_STR が X を越えた tick の次に SLALOM(counter=1)が効く
//   新  : X の 0.5 tick 手前で SLA_FRONT_STR を終え、SlaStartAlign で出力を遅らせる
// の旋回後の直線の横位置(出口の線の法線方向の位置)を比べる。
#include "gen_code_mpc/mpc_tgt_calc.h"
#include "planning/sla_start_align.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

static constexpr float kDt = 0.001f;
static constexpr float kTread = 38.0f;

struct Turn { const char *name; float v, ang_deg, rad, pow_n; };

struct Result { double exit_off; double end_ang; int ticks; };

// mode: 0 = 従来, 1 = 新
// shift: Core0 が SLALOM を送る tick のずれ(+1 = 1 tick 遅れ、−1 = 1 tick 早い)
static Result run(const Turn &t, float u, int mode, std::vector<float> *w_out = nullptr,
                  int shift = 0, int *wait = nullptr, float *frac = nullptr,
                  int force_recv = -1, int *recv_out = nullptr) {
  mpc_tgt_calcModelClass mpc;
  mpc.initialize();
  t_dynamics dyn{};
  dyn.mass = 0.0285f; dyn.lm = 6e-6f; dyn.km = 0.00589f; dyn.resist = 4.4f;
  dyn.tread = kTread; dyn.ke = 0.00589f; dyn.tire = 13.0f; dyn.gear_ratio = 37.0f / 8.0f;
  const float Et = 0.7632146181989743f;
  const float ang = t.ang_deg / 180.0f * (float)M_PI;
  const float time = t.rad * ang / (2.0f * t.v * Et);

  t_tgt tgt{};
  tgt.v_max = t.v; tgt.end_v = t.v; tgt.accl = 10000; tgt.decel = -10000;
  tgt.tgt_dist = 180 * 1000; tgt.tgt_angle = ang;
  tgt.accl_param.limit = 5500; tgt.accl_param.n = 4;

  t_ego ego{};
  ego.v = t.v;
  // 前の直進(SLA_FRONT_STR)
  const float X = 30.0f + u * t.v * kDt; // 旋回を始める位置(弧長)
  double x = 0, y = 0, th = 0, s = 0;    // 経路の積分(弧長 s = global_pos.dist)
  int recv = -1;                          // SLALOM を受け取る tick
  SlaStartAlign al;
  std::vector<t_ego> pts(1);
  auto step = [&](const t_ego &in, int m, t_ego *p, int n) {
    for (int i = 0; i < n; i++) {
      int32_T index = i + 1;
      mpc.step(&tgt, (i == 0) ? &in : &p[i - 1], m, 1, &p[i], &dyn, &index);
    }
  };
  bool turning = false, done = false;
  int n_turn = 0;
  const int limit = (int)(time * 2 / kDt);
  for (int k = 0; k < 400; k++) {
    float w = 0.0f;
    if (!turning && recv < 0) {
      // Core0 の go_straight の終了判定(s は tick k の位置)
      const bool end = (mode == 0) ? (s >= X) : (s + (0.5 - std::min(shift, 0)) * t.v * kDt >= X);
      if (end) recv = k + 1 + std::max(shift, 0);
      if (force_recv >= 0) recv = force_recv;
    } else if (!turning && k == recv) {
      turning = true;
      ego.sla_param.counter = 1; ego.sla_param.state = 0;
      ego.sla_param.base_alpha = t.v / t.rad; ego.sla_param.base_time = time;
      ego.sla_param.limit_time_count = time * 2 / kDt; ego.sla_param.pow_n = t.pow_n;
      ego.img_ang = 0; ego.w = 0; ego.alpha = 0;
      if (mode == 1) al.arm(X);
    }
    if (turning && !done) {
      t_ego out;
      if (mode == 0) {
        step(ego, 1, pts.data(), 1);
        out = pts[0];
      } else {
        al.tick(ego, (float)s, kDt, 0.0f, kTread, 3, 1, step, pts.data(), 1, out);
      }
      sla_state_fields(ego, out, kTread); // copy_tgt
      w = out.w;
      n_turn++;
      if (ego.sla_param.counter >= limit) { done = true; }
    }
    if (w_out) w_out->push_back(w);
    // 1 tick 進める(中点の向きで)
    const double th_mid = th + 0.5 * w * kDt;
    x += t.v * kDt * std::cos(th_mid);
    y += t.v * kDt * std::sin(th_mid);
    th += w * kDt;
    s += t.v * kDt;
  }
  // 出口の直線の法線方向の位置(直線の上なら一定)
  // 旋回を X だけ後ろで始めると出口の線は法線方向へ −X·sin(ang) ずれるので、
  // それを戻した値が、X に対して旋回をどこで始めたかのずれになる(理想は u によらず一定)。
  const double exit_off = -std::sin(ang) * x + std::cos(ang) * y + X * std::sin(ang);
  if (recv_out) *recv_out = recv;
  if (wait) *wait = al.wait();
  if (frac) *frac = al.frac();
  return {exit_off, th * 180.0 / M_PI, n_turn};
}

int main() {
  const Turn turns[] = {
      {"large90 v2200", 2200, 90, 52, 4},
      {"dia45   v2200", 2200, 45, 52, 4},
      {"dia135  v2200", 2200, 135, 37.5f, 4},
      {"large90 v1500", 1500, 90, 57, 4},
  };
  int fail = 0;
  for (const auto &t : turns) {
    double m[2] = {0, 0}, mn[2] = {1e9, 1e9}, mx[2] = {-1e9, -1e9}, sq[2] = {0, 0}, amax[2] = {0, 0};
    const int N = 100;
    for (int i = 0; i < N; i++) {
      const float u = (i + 0.5f) / N;
      for (int mode = 0; mode < 2; mode++) {
        const Result r = run(t, u, mode);
        m[mode] += r.exit_off / N; sq[mode] += r.exit_off * r.exit_off / N;
        mn[mode] = std::fmin(mn[mode], r.exit_off); mx[mode] = std::fmax(mx[mode], r.exit_off);
        amax[mode] = std::fmax(amax[mode], std::fabs(r.end_ang - t.ang_deg));
      }
    }
    std::printf("%s  出口の横位置: 従来 平均 %.3f 幅 %.3f std %.3f / 新 平均 %.3f 幅 %.3f std %.3f mm  角度の誤差 max 従来 %.3f 新 %.3f deg\n",
                t.name, m[0], mx[0] - mn[0], std::sqrt(sq[0] - m[0] * m[0]), m[1], mx[1] - mn[1],
                std::sqrt(sq[1] - m[1] * m[1]), amax[0], amax[1]);
    if (std::fabs(m[0] - m[1]) > 0.05 || (mx[1] - mn[1]) > 0.2) fail++;
  }
  // Core0 が 1 tick 遅れた(tau ≥ 0): f = 1 で、同じ tick に受け取った従来と出力列が一致するか
  {
    int same = 0, total = 0;
    for (int i = 0; i < 20; i++) {
      const float u = (i + 0.5f) / 20;
      std::vector<float> w1, w0;
      float f = -1;
      int R = -1;
      run(turns[0], u, 1, &w1, +1, nullptr, &f, -1, &R);
      if (f < 1.0f) continue;
      total++;
      run(turns[0], u, 0, &w0, 0, nullptr, nullptr, R);
      same += (w1 == w0);
    }
    std::printf("1 tick 遅れ: f = 1 が %d 件(全 20)、従来と出力列が完全一致 %d 件\n", total, same);
    if (total != 20 || same != total) fail++;
  }
  // Core0 が 1 tick 早い: 直進で 1 tick 待ってから合わせる
  {
    double mn = 1e9, mx = -1e9;
    int wmin = 99, wmax = -1;
    for (int i = 0; i < 50; i++) {
      const float u = (i + 0.5f) / 50;
      int w = 0;
      const Result r = run(turns[0], u, 1, nullptr, -1, &w);
      mn = std::fmin(mn, r.exit_off); mx = std::fmax(mx, r.exit_off);
      wmin = std::min(wmin, w); wmax = std::max(wmax, w);
    }
    std::printf("1 tick 早い: 出口の横位置 幅 %.3f mm(%.3f〜%.3f)、待ち %d〜%d tick\n", mx - mn, mn, mx, wmin, wmax);
    if (mx - mn > 0.2 || wmin < 1) fail++;
  }
  std::printf(fail ? "NG\n" : "OK\n");
  return fail ? 1 : 0;
}
