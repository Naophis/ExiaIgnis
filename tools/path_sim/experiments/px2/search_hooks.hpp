// 実験 4: 探索のサブゴールを「タイム最小の経路(未知の壁は無いものとする)の上の未知区画」だけにする。
// 計算を軽くする工夫:
//   1. 覚えた経路は、その上に壁が見つかるまで使い続ける(新しく分かるのは壁だけなので、
//      ほかの経路が速くなることは無い = 塞がれない限り最小のまま)
//   2. 探すのは「既知の区画だけで作れる最小タイムより速い経路」だけ(上限つき + 残り時間の下限)
//   3. 計算に時間がかかる想定(lag 秒後に結果が届く)。届くまでは前の経路の未知区画を使う
// px_search.cpp(build.py が作る)の dp_subgoals の後ろで読み込む。
#pragma once

struct HEdge {
  int x, y, d;
};
struct HPlan {
  bool none = false;   // 既知だけの最小より速い経路は無い(このモードは完了)
  bool proven = true;  // 厳密に求めた(false = 節点の上限を超えたので粗く求めた)
  std::vector<HEdge> path;
};
struct HCache {
  bool has = false;
  HPlan plan;
  bool pending = false;
  double ready = 0;
  HPlan next;
};
static std::vector<HCache> g_hc;
static double *g_now = nullptr;
static double g_lag = 0;
static int g_cap = 0;
static float g_w0 = 1.0f;     // 走りながら使う探索の、下限に掛ける倍率(1 = 厳密)
static bool g_collapse = false; // 走りながら使う探索は、節点をまとめた軽い版
static bool g_verify = false; // スタートへ戻ったときに厳密に確かめる
static int g_verify_cap = 0;
static std::vector<std::vector<double>> g_v_log; // 帰還時の確認: ms, 節点, 辺, 見つかった未知区画
static int g_recompute = 0, g_cap_hit = 0, g_unproven_end = 0;
static double g_wait_home = 0;
static std::vector<std::vector<double>> g_h_log; // ms, 節点, 辺, 既知の節点, proven

static std::vector<HEdge> h_edges(const std::string &moves) {
  std::vector<HEdge> e;
  tp::Pos p{0, 0, 0};
  const std::string mv = "F" + moves; // (0,0) → (0,1) の 1 歩
  for (char c : mv) {
    const int d = c == 'F' ? p.d : c == 'R' ? (p.d + 1) % 4 : (p.d + 3) % 4;
    e.push_back(HEdge{p.x, p.y, d});
    p = tp::Pos{p.x + tp::DX[d], p.y + tp::DY[d], d};
  }
  return e;
}
static bool h_blocked(const std::vector<HEdge> &path) {
  for (const auto &e : path)
    if (g_lgc0->existWall(e.x, e.y, tp::DIRS[e.d]))
      return true;
  return false;
}
static int h_unknown(const std::vector<HEdge> &path, std::unordered_map<unsigned int, unsigned char> *list) {
  int n = 0;
  for (const auto &e : path) {
    if (g_lgc0->is_unknown(e.x, e.y, tp::DIRS[e.d])) {
      n++;
      if (list)
        (*list)[(e.x + tp::DX[e.d]) + (e.y + tp::DY[e.d]) * g_N0] = 1;
    }
  }
  return n;
}
static HPlan h_compute(int k, float w0 = -1, int cap = -1, bool verify = false) {
  const auto t0 = std::chrono::steady_clock::now();
  auto &ps = g_psets[k];
  HPlan plan;
  if (w0 < 0)
    w0 = g_w0;
  if (cap < 0)
    cap = g_cap;
  // 既知の区画だけで作れる最小タイム
  tp::C = tp::Ctx{g_lgc0, g_pc0, &ps, false, true, false, 1.0f};
  const auto known = tp::solve(g_goals0, g_N0);
  const float upper = known.found ? known.time : 1e30f;
  // それより速い経路を、未知の壁は無いものとして探す
  const bool col = g_collapse && !verify;
  tp::C = tp::Ctx{g_lgc0, g_pc0, &ps, true, true, true, w0, upper, cap, col};
  auto r = tp::solve(g_goals0, g_N0);
  int nodes = r.n_nodes;
  long edges = r.n_edge;
  plan.proven = w0 <= 1.0f && !col;
  if (r.overflow) {
    // 節点が足りない: 下限を大きめに見て(厳密でなくなる)、入る所まで粗くする
    g_cap_hit++;
    plan.proven = false;
    for (float w : {1.5f, 2.0f, 3.0f, 5.0f, 8.0f}) {
      if (w <= w0)
        continue;
      tp::C = tp::Ctx{g_lgc0, g_pc0, &ps, true, true, true, w, upper, cap, col};
      r = tp::solve(g_goals0, g_N0);
      nodes = std::max(nodes, r.n_nodes);
      edges += r.n_edge;
      if (!r.overflow)
        break;
    }
  }
  if (r.found)
    plan.path = h_edges(r.moves);
  else
    plan.none = true; // 見つからない = 既知だけの最小が最小(粗く求めたときは未確定)
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (verify) {
    g_v_log.push_back({ms, (double)nodes, (double)edges, (double)h_unknown(plan.path, nullptr), plan.proven ? 1.0 : 0.0});
  } else {
    g_recompute++;
    g_h_log.push_back({ms, (double)nodes, (double)edges, (double)known.n_nodes, plan.proven ? 1.0 : 0.0});
  }
  return plan;
}
// 覚えた経路がまだ使えるか見て、だめなら計算し直す(結果は lag 秒後に届く)
static void h_refresh() {
  if (g_hc.size() != g_psets.size())
    g_hc.assign(g_psets.size(), HCache{});
  const double now = g_now ? *g_now : 0;
  for (size_t k = 0; k < g_hc.size(); k++) {
    auto &c = g_hc[k];
    if (c.pending && now >= c.ready) {
      c.plan = c.next;
      c.has = true;
      c.pending = false;
    }
    bool need = !c.has;
    if (c.has && !c.plan.none) {
      if (h_blocked(c.plan.path))
        need = true; // 経路の上に壁が見つかった
      else if (!c.plan.proven && h_unknown(c.plan.path, nullptr) == 0)
        need = true; // 粗く求めた経路を見終わった: 厳密に確かめ直す
    }
    if (c.has && c.plan.none && !c.plan.proven)
      need = !c.pending && need; // 粗い探索で見つからなかった: これ以上は探せない
    if (need && !c.pending) {
      c.next = h_compute((int)k);
      if (g_lag <= 0) {
        c.plan = c.next;
        c.has = true;
      } else {
        c.pending = true;
        c.ready = now + g_lag;
      }
    }
  }
}
static bool h_pending() {
  for (const auto &c : g_hc)
    if (c.pending)
      return true;
  return false;
}
static void lazy_subgoals(std::unordered_map<unsigned int, unsigned char> &list) {
  const auto t0 = std::chrono::steady_clock::now();
  h_refresh();
  for (const auto &c : g_hc) {
    if (c.has && !c.plan.none)
      h_unknown(c.plan.path, &list);
  }
  g_dp_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
// スタートへ戻ったとき: 計算待ちがあれば届くまで待ち、未知区画が残っていれば出直す
static bool h_at_home(std::unordered_map<unsigned int, unsigned char> &list) {
  for (int guard = 0; guard < 16; guard++) {
    list.clear();
    lazy_subgoals(list);
    if (!h_pending())
      break;
    double ready = 0;
    for (const auto &c : g_hc)
      if (c.pending)
        ready = std::max(ready, c.ready);
    if (g_now && ready > *g_now) {
      g_wait_home += ready - *g_now;
      *g_now = ready;
    }
  }
  if (list.empty() && g_verify) {
    // 止まっているので、厳密に確かめる。未知区画を通る、もっと速い経路が残っていれば出直す
    for (size_t k = 0; k < g_hc.size(); k++) {
      auto &c = g_hc[k];
      if (c.has && c.plan.proven)
        continue;
      c.plan = h_compute((int)k, 1.0f, g_verify_cap, true);
      c.has = true;
      if (!c.plan.none)
        h_unknown(c.plan.path, &list);
    }
  }
  if (list.empty()) {
    for (const auto &c : g_hc)
      if (c.has && !c.plan.proven)
        g_unproven_end++;
  }
  return !list.empty();
}

// ---- 重みパターンの経路を覚えておき、壁で塞がれたときだけ作り直す
extern std::vector<int> g_sgp_path;
struct PCache {
  bool has = false;
  std::vector<int> path; // x | y << 8 | 向き << 16
};
static std::vector<PCache> g_pcache;
static int g_p_recompute = 0, g_p_calls = 0;
static double g_p_ms = 0, g_p_check_ms = 0;
static int g_p_hist[8] = {0};  // 1 回の更新で作り直した枚数の分布
static int g_p_limit = 0;      // 1 回の更新で作り直す枚数の上限(0 = なし)。超えた分は次の更新へ回す
static void pattern_lazy(std::unordered_map<unsigned int, unsigned char> &list) {
  const auto t0 = std::chrono::steady_clock::now();
  double ms_re = 0;
  if (g_pcache.size() != g_sub_multi.size())
    g_pcache.assign(g_sub_multi.size(), PCache{});
  g_p_calls++;
  int rebuilt = 0;
  for (size_t i = 0; i < g_sub_multi.size(); i++) {
    auto &c = g_pcache[i];
    bool ok = c.has;
    if (ok) {
      for (const int e : c.path) {
        if (g_lgc0->existWall(e & 255, (e >> 8) & 255, static_cast<Direction>(e >> 16))) {
          ok = false;
          break;
        }
      }
    }
    if (!ok && g_p_limit > 0 && rebuilt >= g_p_limit) {
      c.has = false; // 次の更新で作り直す。それまでこのパターンのサブゴールは無し
      continue;
    }
    if (!ok) {
      rebuilt++;
      const auto t1 = std::chrono::steady_clock::now();
      std::unordered_map<unsigned int, unsigned char> tmp;
      g_lgc0->set_param_num(g_sub_multi[i]);
      g_lgc0->set_param();
      g_lgc0->searchGoalPosition(true, tmp);
      c.path = g_sgp_path;
      c.has = true;
      g_p_recompute++;
      ms_re += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
    }
    for (const int e : c.path) {
      const int x = e & 255, y = (e >> 8) & 255;
      const Direction d = static_cast<Direction>(e >> 16);
      if (g_lgc0->is_unknown(x, y, d)) {
        const int nx = x + (d == Direction::East) - (d == Direction::West);
        const int ny = y + (d == Direction::North) - (d == Direction::South);
        list[nx + ny * g_N0] = 1;
      }
    }
  }
  g_p_hist[std::min(rebuilt, 7)]++;
  g_lgc0->set_param_num(1);
  g_lgc0->set_param();
  g_p_ms += ms_re;
  g_p_check_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() - ms_re;
}
