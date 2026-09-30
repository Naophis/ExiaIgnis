// DiaPostEdgeDetector のホスト検証(2026-10-01)。
// 入力(標準入力): 1 行 1 tick "case_id rearm x y_left y_right c psi"。rearm=1 の行と
// case_id が変わった行で再アーム(その行のサンプルは入れない)。c はジャイロの向きの
// 積分 [mm·rad]、psi はジャイロの向き [rad](どちらも右向き +)。
// 引数 1 つ目: κ [mm/rad](既定 0)。
// 出力: 組ができるたびに "case_id seq delta pos eps n_pairs psi0_deg n_psi now_delta delta_axle"。
// 引数 2 つ目が pred のときは、組ができる直前の tick の now_delta も "PRED case_id x now_delta" で出す。
#include "planning/dia_post_edge_detector.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
int main(int argc, char **argv) {
  DiaPostEdgeDetector det;
  DiaPostEdgeParams p;
  if (argc > 1) p.kappa = std::stof(argv[1]);
  const bool pred = (argc > 2 && std::string(argv[2]) == "pred");
  std::string line, cur;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string id;
    int rearm;
    float x, yl, yr, c = 0.0f, psi = 0.0f;
    is >> id >> rearm >> x >> yl >> yr >> c >> psi;
    if (id != cur || rearm) {
      det.arm();
      cur = id;
      if (rearm) continue;
    }
    // ファームと同じく左 → 右の順に入れる
    const DiaPostEdgeDetector::Side sides[2] = {DiaPostEdgeDetector::LEFT, DiaPostEdgeDetector::RIGHT};
    const float ys[2] = {yl, yr};
    const float before = det.n_pairs() >= 1 ? det.now_delta(x, c) : 0.0f;
    const int n_before = det.n_pairs();
    for (int k = 0; k < 2; k++) {
      if (det.update(sides[k], x, ys[k], c, psi, p)) {
        if (pred && n_before >= 1) std::printf("PRED %s %.3f %.4f\n", id.c_str(), x, before);
        std::printf("%s %u %.4f %.3f %.4f %d %.4f %d %.4f %.4f\n", id.c_str(), (unsigned)det.seq(),
                    det.delta(), det.pos(), det.eps_deg(), det.n_pairs(),
                    det.psi0() * 57.29578f, det.n_psi(), det.now_delta(x, c), det.delta_axle());
      }
    }
  }
}
