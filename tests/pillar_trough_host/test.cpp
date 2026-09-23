// PillarTroughDetector のホストテスト(run.sh 参照)。
#include "planning/pillar_trough_detector.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int g_fail = 0;
#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      g_fail++;                                                                \
      std::printf("  FAIL %s:%d: ", __FILE__, __LINE__);                       \
      std::printf(__VA_ARGS__);                                                \
      std::printf("\n");                                                       \
    }                                                                          \
  } while (0)

struct Row {
  float x;   // 走行距離(WALL_OFF 開始基準でも可、単調増加なら何でもよい)
  float d;   // 注視側 45° LED1 [mm]
  float d2;  // 両点灯 ch [mm]
};
static float m180(float v) { return v <= 0.0f ? 180.0f : v; }

// rows を順に流し、最初の発火 index(-1 なら不発)と発火時の detector を返す。
static int replay(PillarTroughDetector &det, const std::vector<Row> &rows,
                  const PillarTroughParams &p, bool fire_enable = true) {
  float prev_x = rows[0].x, prev_d2 = rows[0].d2;
  det.arm(rows[0].d, rows[0].x);
  for (size_t i = 1; i < rows.size(); i++) {
    const float travel = rows[i].x - prev_x;
    det.update(rows[i].d, rows[i].d2 - prev_d2, rows[i].x, travel, fire_enable, p);
    prev_x = rows[i].x;
    prev_d2 = rows[i].d2;
    if (det.fired()) return (int)i;
  }
  return -1;
}

static void test_fixed() {
  PillarTroughParams p; // 既定値 = offset.yaml の初期値
  std::printf("[fixed] 20260923_064918 idx2196-2218 (右, 壁なし開始, 柱谷底68.5)\n");
  {
    // index, dist(WALL_OFF 開始=0; 直前の直線は v_c から逆算), right45_d, right45_2_d
    std::vector<Row> r = {
        {-12.6f, 0.0f, 0.0f},    {-10.7f, 144.80f, 0.0f}, {-8.8f, 128.54f, 0.0f},
        {-6.9f, 113.88f, 0.0f},  {-5.1f, 100.61f, 0.0f},  {-3.3f, 88.97f, 0.0f},
        {-1.7f, 83.16f, 0.0f},   {0.00f, 79.86f, 0.0f},   {1.92f, 77.39f, 70.31f},
        {3.88f, 73.85f, 68.07f}, {5.85f, 70.59f, 66.46f}, {7.68f, 68.53f, 65.06f},
        {9.44f, 68.53f, 63.83f}, {11.22f, 72.00f, 63.55f}, {12.98f, 78.18f, 63.83f},
        {14.67f, 89.64f, 65.75f}, {16.33f, 113.88f, 70.21f}, {18.00f, 0.0f, 79.37f},
        {19.70f, 0.0f, 94.71f},  {21.41f, 0.0f, 116.47f}, {23.19f, 0.0f, 166.79f},
        {24.95f, 0.0f, 0.0f},    {26.70f, 0.0f, 0.0f}};
    for (auto &q : r) { q.d = m180(q.d); q.d2 = m180(q.d2); }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == 13, "fire index %d (expected 13 = idx2209)", k);
    CHECK(det.state() == PillarTroughDetector::FIRED_CURV, "state %d", det.state());
    CHECK(std::fabs(det.bottom() - 68.53f) < 0.01f, "bottom %.2f", det.bottom());
    // idx2207 と idx2208 が同値(68.53)の平坦な谷底。真の最小はその中間なので
    // 頂点補間が +0.5 tick(= +0.88mm)寄せる。
    CHECK(std::fabs(det.bottom_x() - 8.56f) < 0.01f, "bottom_x %.2f (expected 8.56 = idx2207 + 0.5tick)", det.bottom_x());
    std::printf("  fire@%d state=%d bottom=%.2f bottom_x=%.2f lag=%.2f\n", k, det.state(),
                det.bottom(), det.bottom_x(), det.fire_x() - det.bottom_x());
  }
  std::printf("[fixed] 20260923_151724 idx78-92 (右, v=1500 実機テスト: 谷底 idx86、idx87 で発火)\n");
  {
    std::vector<Row> r = {
        {-8.5f, 84.21f, 0.0f},  {-7.0f, 79.00f, 0.0f},  {-5.5f, 75.90f, 0.0f},  {-4.0f, 74.18f, 0.0f},
        {-2.5f, 72.90f, 0.0f},  {-1.0f, 71.14f, 0.0f},  {0.00f, 69.03f, 0.0f},  {1.50f, 67.36f, 64.40f},
        {3.02f, 66.69f, 63.48f}, {4.56f, 68.05f, 62.87f}, {6.09f, 70.86f, 62.61f}, {7.62f, 74.51f, 62.67f},
        {9.12f, 82.66f, 63.83f}, {10.64f, 97.62f, 0.0f}, {12.14f, 134.22f, 0.0f}};
    for (auto &q : r) { q.d = m180(q.d); q.d2 = m180(q.d2); }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == 9, "fire index %d (expected 9 = idx87, 谷底+1)", k);
    CHECK(std::fabs(det.bottom_x() - 2.76f) < 0.01f, "bottom_x %.2f (expected 2.76 = idx86 - 0.17tick)", det.bottom_x());
    std::printf("  fire@%d state=%d bottom=%.2f bottom_x=%.2f lag=%.2f\n", k, det.state(), det.bottom(), det.bottom_x(), det.fire_x() - det.bottom_x());
  }
  std::printf("[fixed] 20260923_013712 idx372-395 (右, 首振れのランプ73.9→83.7 では発火せず、本物の谷64.4 で発火)\n");
  {
    std::vector<Row> r = {
        {-6.5f, 102.81f, 66.71f}, {-4.3f, 75.90f, 65.75f}, {-2.1f, 75.19f, 65.44f},
        {0.00f, 73.85f, 65.06f},  {1.53f, 75.19f, 64.99f}, {3.47f, 75.54f, 64.91f},
        {5.55f, 76.26f, 65.14f},  {7.46f, 77.39f, 65.29f}, {9.31f, 79.00f, 65.67f},
        {11.24f, 79.00f, 65.91f}, {13.39f, 81.21f, 66.55f}, {15.45f, 82.17f, 66.71f},
        {17.36f, 83.68f, 66.71f}, {19.20f, 82.66f, 66.38f}, {21.11f, 81.68f, 65.44f},
        {23.14f, 77.01f, 63.76f}, {25.19f, 74.18f, 61.96f}, {27.23f, 69.79f, 60.05f},
        {29.33f, 66.47f, 58.43f}, {31.41f, 64.44f, 57.23f}, {33.44f, 65.02f, 57.13f},
        {35.41f, 68.53f, 58.07f}, {37.3f, 77.78f, 60.68f},  {39.1f, 93.31f, 0.0f}};
    for (auto &q : r) { q.d = m180(q.d); q.d2 = m180(q.d2); }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    // idx376-384 は ang が -1.2° から -3.3° へ振れている最中の緩い上昇。
    // 1階微分では +2.21mm/tick まで出て柱(+2.69)と紛らわしいが、2階微分は
    // 符号が交互に振れるだけなので curv_n=2 が成立しない。ここで発火しないこと
    // (= 最初の発火が本物の谷の側であること)がこのケースの主眼。
    CHECK(k == 20, "fire index %d (expected 20 = idx392、谷底+1。ランプでは発火しない)", k);
    CHECK(std::fabs(det.bottom_x() - 31.97f) < 0.01f, "bottom_x %.2f (expected 31.97 = idx391 + 0.28tick)", det.bottom_x());
    std::printf("  fire@%d state=%d bottom=%.2f bottom_x=%.2f lag=%.2f\n", k, det.state(),
                det.bottom(), det.bottom_x(), det.fire_x() - det.bottom_x());
  }
  std::printf("[fixed] 20260923_173424 idx78-91 (右, 谷底が丸く谷底の曲率が 0.206 しかない例)\n");
  {
    // 谷底(idx86)の曲率は 0.206 で、per-tick しきい値 0.3 では谷底+2 になっていた。
    // 連続区間の合計(0.073+0.153+0.206+0.791 = 1.22)で見ると谷底+1 で出る。
    std::vector<Row> r = {
        {-9.00f, 84.21f, 0.00f}, {-7.50f, 78.59f, 0.00f}, {-6.00f, 74.51f, 0.00f}, {-4.50f, 72.60f, 0.00f},
        {-3.00f, 70.59f, 0.00f}, {-1.50f, 68.78f, 0.00f}, {0.00f, 67.13f, 0.00f},  {1.44f, 65.84f, 63.76f},
        {2.94f, 65.02f, 62.87f}, {4.44f, 66.05f, 62.16f}, {5.94f, 68.78f, 62.22f}, {7.44f, 73.53f, 62.41f},
        {8.94f, 80.75f, 0.00f},  {10.50f, 94.11f, 0.00f}};
    for (auto &q : r) { q.d = m180(q.d); q.d2 = m180(q.d2); }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == 9, "fire index %d (expected 9 = idx87, 谷底+1)", k);
    CHECK(std::fabs(det.bottom() - 65.02f) < 0.01f, "bottom %.2f", det.bottom());
    std::printf("  fire@%d state=%d bottom=%.2f bottom_x=%.2f lag=%.2f\n", k, det.state(),
                det.bottom(), det.bottom_x(), det.fire_x() - det.bottom_x());
  }
  std::printf("[fixed] 20260923_064338 idx399-416 (右, 二重谷: 1つ目(柱)で発火、2つ目(首振れ)は無視)\n");
  {
    // idx397(SLALOM 末尾, 180)と idx398(85.88)を含める: 深さゲートのピーク(180)は
    // SLALOM 中の毎 tick 再アームで履歴に入る(実機・CSV 再生と同じ)。
    std::vector<Row> r = {
        {-7.9f, 0.0f, 0.0f},      {-5.9f, 85.88f, 62.87f},
        {-3.9f, 70.05f, 62.28f},  {-1.9f, 68.78f, 61.78f},  {0.00f, 69.53f, 61.78f},
        {1.39f, 70.05f, 62.22f},  {3.27f, 74.85f, 63.69f},  {5.30f, 77.01f, 64.69f},
        {7.15f, 77.01f, 64.91f},  {8.88f, 78.18f, 65.06f},  {10.80f, 78.18f, 64.62f},
        {12.80f, 78.18f, 64.11f}, {14.67f, 75.54f, 63.28f}, {16.42f, 73.53f, 62.28f},
        {18.12f, 73.53f, 61.84f}, {19.98f, 74.51f, 61.71f}, {21.83f, 79.00f, 62.87f},
        {23.62f, 88.32f, 65.06f}, {25.47f, 107.82f, 69.25f}, {27.3f, 144.80f, 76.91f}};
    for (auto &q : r) { q.d = m180(q.d); q.d2 = m180(q.d2); }
    // 1 つ目の谷 68.78(開始前 −1.9mm)が柱(同ターンの 20260923_064918 idx407 では谷が
    // WALL_OFF 開始位置にあり 1.4mm で発火)。曲率ルールで idx401 まで早まる。
    // 2 つ目の谷 73.5 は ang −1.3→−4.5° の首振れ中で深さ 4.7mm → 拾わない。
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == 4, "fire index %d (expected 4 = idx401、谷底+2)", k);
    CHECK(std::fabs(det.bottom() - 68.78f) < 0.01f, "bottom %.2f (expected 68.78 first trough)", det.bottom());
    std::printf("  fire@%d state=%d bottom=%.2f bottom_x=%.2f lag=%.2f\n", k, det.state(), det.bottom(), det.bottom_x(), det.fire_x() - det.bottom_x());
  }
  std::printf("[fixed] 放物線の頂点補間: 真の谷底がサンプルの中間にあっても位置を復元すること\n");
  {
    // 合成データ: y = 62 + 1.2*(x - x_true)^2。谷底サンプル(x=10.5)から真の頂点を
    // +0.3 tick ずらしてある。tick 単位のままだと 0.45mm ずれるので、頂点補間が
    // それを取り切れるかを見る。
    const float h = 1.5f;
    const float x_true = 10.5f + 0.3f * h;
    std::vector<Row> r;
    for (int i = 0; i < 14; i++) {
      const float x = i * h;
      float d = 62.0f + 1.2f * (x - x_true) * (x - x_true);
      if (d > 180.0f) d = 180.0f;
      r.push_back({x, d, d - 2.0f});
    }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k > 0, "should fire on a clean parabola");
    CHECK(std::fabs(det.bottom_x() - x_true) < 0.10f,
          "bottom_x %.3f (true vertex %.3f, 補間なしなら 10.500)", det.bottom_x(), x_true);
    std::printf("  fire@%d bottom_x=%.3f (真の頂点 %.3f, 補間なしの谷底 tick は 10.500)\n",
                k, det.bottom_x(), x_true);
  }
  std::printf("[fixed] 壁の平坦(47±0.3, 60mm)→壁切れ: 発火しないこと(壁は exist 経路の担当)\n");
  {
    std::vector<Row> r;
    float x = 0;
    for (int i = 0; i < 6; i++) { r.push_back({x, 180.0f - 25.0f * i, 180.0f}); x += 1.8f; }
    // 壁の平坦: ゆっくり下がる(-0.02mm/tick)+非周期ノイズ → 谷底(最小値)が末尾近くまで
    // 更新され続ける(stale 再アームが起きない)。ピークを窓で取らないとここで
    // 180 が残り、壁切れの急上昇で「深い谷」として誤発火する。
    for (int i = 0; i < 34; i++) {
      const float noise = 0.25f * std::sin(i * 1.7f) + 0.15f * std::sin(i * 0.37f);
      r.push_back({x, 47.0f - 0.02f * i + noise, 46.5f - 0.02f * i + 0.5f * noise});
      x += 1.8f;
    }
    const float tail[] = {49.0f, 53.0f, 60.0f, 75.0f, 110.0f, 180.0f};
    for (float v : tail) { r.push_back({x, v, v - 2.0f}); x += 1.8f; }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == -1, "unexpected fire at %d (bottom %.2f bottom_x %.2f)", k, det.bottom(), det.bottom_x());
    std::printf("  no fire (state=%d)\n", det.state());
    // 対照: ピーク窓を事実上無限(stale_dist=1000)にすると壁開始前の 180 を引きずって
    // 壁切れで誤発火する = 窓が壁端の誤アンカーを防いでいることの確認。
    PillarTroughParams pw = p;
    pw.stale_dist = 1000.0f;
    pw.bottom_min = 0.0f; // 壁の 46mm は bottom_min でも弾かれるので、窓の効果だけを見る
    PillarTroughDetector det2;
    int k2 = replay(det2, r, pw);
    CHECK(k2 != -1, "control case (no peak window) should fire at the wall end");
    std::printf("  control(no window): fire@%d bottom=%.2f lag=%.2f\n", k2, det2.bottom(), det2.fire_x() - det2.bottom_x());
  }
  std::printf("[fixed] 首振れ相当のゆっくりした谷(1.1mm/deg×1.5deg/tick, 深さ6mm)→急上昇無し: 発火しないこと\n");
  {
    std::vector<Row> r;
    float x = 0;
    const float seq[] = {70, 68.4f, 66.8f, 65.2f, 64.0f, 64.0f, 65.6f, 67.2f, 68.8f, 70.0f, 70.0f, 70.0f};
    for (float v : seq) { r.push_back({x, v, v - 3.0f}); x += 1.8f; }
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == -1, "unexpected fire at %d", k);
    std::printf("  no fire (state=%d bottom=%.2f peak=%.2f)\n", det.state(), det.bottom(), det.peak());
  }
  std::printf("[fixed] 旋回直後の近い柱(谷底33mm、急上昇): bottom_min=48 で発火しないこと\n");
  {
    std::vector<Row> r = {{0, 180, 180}, {1.8f, 120, 130}, {3.6f, 60, 70}, {5.4f, 38, 42},
                          {7.2f, 33.2f, 36}, {9.0f, 34.5f, 36.5f}, {10.8f, 40, 40}, {12.6f, 52, 48},
                          {14.4f, 75, 62}, {16.2f, 120, 90}, {18.0f, 180, 140}};
    PillarTroughDetector det;
    int k = replay(det, r, p);
    CHECK(k == -1, "unexpected fire at %d (bottom %.2f)", k, det.bottom());
    std::printf("  no fire (state=%d bottom=%.2f)\n", det.state(), det.bottom());
  }
  std::printf("[fixed] 低速(fire_enable=false)では追跡のみで発火しないこと\n");
  {
    std::vector<Row> r = {{0, 180, 180}, {1.8f, 120, 180}, {3.6f, 80, 90}, {5.4f, 62, 70},
                          {7.2f, 66, 66}, {9.0f, 72, 68}, {10.8f, 90, 75}, {12.6f, 180, 100}};
    PillarTroughDetector det;
    int k = replay(det, r, p, false);
    CHECK(k == -1, "unexpected fire at %d", k);
    CHECK(std::fabs(det.bottom() - 62.0f) < 0.01f, "bottom %.2f", det.bottom());
  }
}

// ---- CSV 再生 ----
static std::vector<std::string> split(const std::string &s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') { out.push_back(cur); cur.clear(); } else if (c != '\r') cur += c;
  }
  out.push_back(cur);
  return out;
}
static int col(const std::vector<std::string> &h, const char *name) {
  for (size_t i = 0; i < h.size(); i++) if (h[i] == name) return (int)i;
  return -1;
}
static bool rearm_state(int ms) {
  // SLALOM=4, PIVOT=2, PIVOT_PRE=8/9, PIVOT_AFTER=10, PIVOT_OFFSET=12, NONE=0, READY=7, FRONT_CTRL=11
  return ms == 4 || ms == 2 || ms == 8 || ms == 9 || ms == 10 || ms == 12 || ms == 0 || ms == 7 || ms == 11;
}
static void replay_csv(const char *path) {
  std::ifstream f(path);
  if (!f) { std::printf("cannot open %s\n", path); return; }
  std::string line;
  std::getline(f, line);
  auto h = split(line);
  const int ci = col(h, "index"), cs = col(h, "motion_state"), cv = col(h, "v_c");
  const int cr = col(h, "right45_d"), cr2 = col(h, "right45_2_d");
  const int cl = col(h, "left45_d"), cl2 = col(h, "left45_2_d");
  if (ci < 0 || cs < 0 || cv < 0 || cr < 0 || cr2 < 0 || cl < 0 || cl2 < 0) {
    std::printf("%s: missing columns\n", path); return;
  }
  PillarTroughParams p;
  PillarTroughDetector dr, dl;
  float x = 0, prev_r2 = 180, prev_l2 = 180;
  int seq_r = 0, seq_l = 0, seg_start_x_i = 0; float seg_x0 = 0; int prev_ms = -1;
  std::printf("== %s\n", path);
  std::printf("  index  side kind  ms  bottom  bottom_x-seg  fire_x-seg   lag\n");
  while (std::getline(f, line)) {
    auto c = split(line);
    if ((int)c.size() <= cl2) continue;
    const int idx = std::atoi(c[ci].c_str());
    const int ms = std::atoi(c[cs].c_str());
    const float v = std::strtof(c[cv].c_str(), nullptr);
    const float r = m180(std::strtof(c[cr].c_str(), nullptr));
    const float r2 = m180(std::strtof(c[cr2].c_str(), nullptr));
    const float l = m180(std::strtof(c[cl].c_str(), nullptr));
    const float l2 = m180(std::strtof(c[cl2].c_str(), nullptr));
    const float travel = std::fabs(v) * 0.001f;
    x += travel;
    if (ms != prev_ms) { seg_x0 = x; seg_start_x_i = idx; prev_ms = ms; }
    if (rearm_state(ms)) {
      dr.arm(r, x); dl.arm(l, x);
    } else {
      const bool fire_ok = std::fabs(v) >= 500.0f;
      dr.update(r, r2 - prev_r2, x, travel, fire_ok, p);
      dl.update(l, l2 - prev_l2, x, travel, fire_ok, p);
    }
    prev_r2 = r2; prev_l2 = l2;
    if (dr.seq() != seq_r) {
      seq_r = dr.seq();
      std::printf("  %5d   R   %s  %2d  %6.2f  %8.2f      %8.2f  %5.2f\n", idx,
                  dr.state() == 2 ? "early" : "confm", ms, dr.bottom(), dr.bottom_x() - seg_x0,
                  dr.fire_x() - seg_x0, dr.fire_x() - dr.bottom_x());
    }
    if (dl.seq() != seq_l) {
      seq_l = dl.seq();
      std::printf("  %5d   L   %s  %2d  %6.2f  %8.2f      %8.2f  %5.2f\n", idx,
                  dl.state() == 2 ? "early" : "confm", ms, dl.bottom(), dl.bottom_x() - seg_x0,
                  dl.fire_x() - seg_x0, dl.fire_x() - dl.bottom_x());
    }
  }
  (void)seg_start_x_i;
}

int main(int argc, char **argv) {
  test_fixed();
  for (int i = 1; i < argc; i++) replay_csv(argv[i]);
  if (g_fail) { std::printf("FAILED: %d check(s)\n", g_fail); return 1; }
  std::printf("PASS\n");
  return 0;
}
