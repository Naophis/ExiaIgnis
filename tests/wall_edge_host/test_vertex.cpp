// WallEdgeDetector::trough_vertex のホスト検証(2026-09-30)。
// 入力(標準入力): 1 行 1 ケース "case_id x0 w n x_0 d_0 x_1 d_1 ..."(サンプルは古い順)。
// 出力: "case_id result xv dv"(result: 0 待ち / 1 OK / 2 失敗)。
#include "planning/wall_edge_detector.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
int main() {
  std::string line;
  WallEdgeParams p;
  p.level_max = -1.0f; // 壁の切れ目の判定は使わない(サンプルを貯めるだけ)
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string id;
    float x0, w;
    int n;
    is >> id >> x0 >> w >> n;
    WallEdgeDetector det;
    for (int i = 0; i < n; i++) {
      float x, d;
      is >> x >> d;
      det.update(&x, &d, 1, p);
    }
    float xv = 0, dv = 0;
    const int r = det.trough_vertex(x0, w, 3.0f, xv, dv);
    std::printf("%s %d %.4f %.4f\n", id.c_str(), r, xv, dv);
  }
}
