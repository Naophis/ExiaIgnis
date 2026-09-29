// WallEdgeDetector のホスト検証(2026-09-30)。
// 入力(標準入力): 1 行 1 tick "case_id n x0 d0 x1 d1 ..."。case_id が変わったら再アーム。
// 出力: case_id ごとに最初の発火 "case_id fire_x edge_x level"(発火しなければ nan)。
// 引数 all: 発火をすべて出す(ログ 1 本を丸ごと流して、同じ上昇で 2 回発火しないかを見る)。
#include "planning/wall_edge_detector.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <iostream>
#include <sstream>
int main(int argc, char **argv) {
  const bool all = (argc > 1 && std::string(argv[1]) == "all");
  WallEdgeDetector det;
  WallEdgeParams p;
  std::string line, cur;
  bool fired = false;
  auto flush = [&](const std::string &id) {
    if (!id.empty() && !fired) std::printf("%s nan nan nan\n", id.c_str());
  };
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string id; int n; is >> id >> n;
    if (id != cur) { flush(cur); cur = id; det.arm(); fired = false; }
    float x[8], d[8];
    for (int i = 0; i < n; i++) is >> x[i] >> d[i];
    if (fired && !all) continue;
    if (det.update(x, d, n, p)) {
      std::printf("%s %.3f %.3f %.3f\n", id.c_str(), det.fire_x(), det.edge_x(), det.level());
      fired = true;
    }
  }
  flush(cur);
}
