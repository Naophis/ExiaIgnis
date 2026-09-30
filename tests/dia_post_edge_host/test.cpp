// DiaPostEdgeDetector のホスト検証(2026-10-01)。
// 入力(標準入力): 1 行 1 tick "case_id rearm x y_left y_right"。rearm=1 の行と
// case_id が変わった行で再アーム(その行のサンプルは入れない)。
// 出力: 組ができるたびに "case_id seq delta pos eps n_pairs"。
#include "planning/dia_post_edge_detector.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
int main() {
  DiaPostEdgeDetector det;
  DiaPostEdgeParams p;
  std::string line, cur;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string id;
    int rearm;
    float x, yl, yr;
    is >> id >> rearm >> x >> yl >> yr;
    if (id != cur || rearm) {
      det.arm();
      cur = id;
      if (rearm) continue;
    }
    // ファームと同じく左 → 右の順に入れる
    const DiaPostEdgeDetector::Side sides[2] = {DiaPostEdgeDetector::LEFT, DiaPostEdgeDetector::RIGHT};
    const float ys[2] = {yl, yr};
    for (int k = 0; k < 2; k++) {
      if (det.update(sides[k], x, ys[k], p)) {
        std::printf("%s %u %.4f %.3f %.4f %d\n", id.c_str(), (unsigned)det.seq(), det.delta(),
                    det.pos(), det.eps_deg(), det.n_pairs());
      }
    }
  }
}
