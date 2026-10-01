// 直進の柱の立ち下がり(DiaPostEdgeDetector の rel_thr > 0 の経路)のホスト検証(2026-10-02)。
// 入力(標準入力、tools/param_tuner/str_post_edge.py --dump の出力):
//   "PARAM rel_thr peak_min_l peak_min_r low_ratio contrast_min rise_max fall_max tol gate_kmax kappa_true psi0_w gain offset"
//   "seg ARM"                      区間の最初(再アーム。ファームと同じく ψ0 の事前値 0 を重み psi0_w で)
//   "seg SEED x c psi delta psi0"  壁からの引き継ぎ(DiaPostEdgeDetector::seed)
//   "seg side x y c psi"           1 サンプル(side 0 = 左 / 1 = 右、ファームと同じ順)
//   "PAIR seg pos delta psi0_deg n_psi"  Python 版が出した組(そのまま通す)
// 出力: C++ 版の組 "CPP seg pos delta psi0_deg n_psi" と、入力の PAIR 行。
#include "planning/dia_post_edge_detector.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
int main() {
  DiaPostEdgeDetector det;
  DiaPostEdgeParams p;
  p.pair_pitch = 0.0f;
  p.same_pitch = 90.0f;
  p.scale_fix = 0;
  p.conf_accel = 0.0f;
  float peak_min[2] = {0, 0};
  float psi0_w = 0.0f;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string a, b;
    is >> a >> b;
    if (a == "PARAM") {
      std::istringstream ps(line);
      std::string tag;
      ps >> tag >> p.rel_thr >> peak_min[0] >> peak_min[1] >> p.low_ratio >> p.contrast_min >> p.rise_max >> p.fall_max >>
          p.tol >> p.gate_kmax >> p.kappa >> psi0_w >> p.gain >> p.offset;
      continue;
    }
    if (a == "PAIR") {
      std::puts(line.c_str());
      continue;
    }
    if (b == "ARM") {
      det.arm();
      if (psi0_w > 0.0f) det.set_psi0_prior(0.0f, psi0_w);
      continue;
    }
    if (b == "SEED") {
      float x, c, psi, delta, psi0;
      is >> x >> c >> psi >> delta >> psi0;
      det.seed(x, c, psi, delta, psi0, psi0_w, p.kappa);
      continue;
    }
    const int side = std::stoi(b);
    float x, y, c, psi;
    is >> x >> y >> c >> psi;
    p.peak_min = peak_min[side];
    if (det.update(side == 0 ? DiaPostEdgeDetector::LEFT : DiaPostEdgeDetector::RIGHT, x, y, c, psi, p)) {
      std::printf("CPP %s %.4f %.5f %.5f %d\n", a.c_str(), det.pos(), det.delta(), det.psi0() * 57.29578f,
                  det.n_psi());
    }
  }
}
