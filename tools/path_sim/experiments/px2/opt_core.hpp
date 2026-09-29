// 実験 3 の本体: タイム最小の経路探索(px_opt.cpp / px_search.cpp 共用)
#pragma once
struct OptCtx { MazeSolverBaseLgc *lgc; PathCreator *pc; param_set_t *ps; bool search; };
static OptCtx CTX;
static TrajectoryCreator TC;

// ---- ファームの変換とタイム計算に通す
struct FwEval {
  float time;
  std::vector<float> s;
  std::vector<int> t;
};
static FwEval fw_eval(const std::vector<float> &rs, const std::vector<int> &rt) {
  auto &pc = *CTX.pc;
  pc.path_s.assign(rs.begin(), rs.end());
  pc.path_t.clear();
  for (int v : rt)
    pc.path_t.push_back((unsigned char)v);
  pc.path_size = (int)rt.size() - 1;
  for (int k = 0; k < 8; k++) { // 終端の先は 0(ファームは終端の先も読む)
    pc.path_s.push_back(0);
    pc.path_t.push_back(0);
  }
  pc.convert_large_path(true);
  pc.diagonalPath(false, true);
  FwEval e;
  e.time = pc.calc_goal_time((*CTX.ps));
  for (size_t i = 0; i < pc.path_t.size(); i++) {
    e.s.push_back(pc.path_s[i]);
    e.t.push_back(pc.path_t[i]);
    if (pc.path_t[i] == 255 || pc.path_t[i] == 0)
      break;
  }
  return e;
}

// ---- calc_goal_time の 1 区間ぶんの写し
static std::map<std::tuple<float, float, float, float, float, float>, float> str_memo;
static float straight_time(float v1, float vmax, float v2, float ac, float diac, float dist) {
  const auto key = std::make_tuple(v1, vmax, v2, ac, diac, dist);
  const auto it = str_memo.find(key);
  if (it != str_memo.end())
    return it->second;
  planning_time_t tmp;
  const float t = CTX.pc->go_straight_dummy(v1, vmax, v2, ac, diac, dist, tmp, false);
  str_memo[key] = t;
  return t;
}
struct SegOut {
  float time;
  float v_now;
  bool dia;
};
// next_type: 次の区間のターン種別(dia を見ない get_turn_type)。FN = 次の直線 > 0 かつ次が Orval / Large
static SegOut seg_cost(bool first, bool dia, float s, int tcode, float v_now, bool FN, int next_tcode) {
  auto &p = (*CTX.ps);
  const float cell = p.cell_size;
  const float dist = !dia ? (0.5 * s - 1) * cell : (0.5 * s - 1) * cell * ROOT2;
  const auto turn_dir = TC.get_turn_dir(tcode);
  const auto turn_type = TC.get_turn_type(tcode, dia);
  bool fast_mode = !first || dist > 0;
  bool start_turn = false, fast_turn = false;
  const auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
  float v_max = p.str_map[st].v_max;
  float v_end = fast_mode ? p.map[turn_type].v : p.map_slow[turn_type].v;
  float accl = p.str_map[st].accl;
  float decel = p.str_map[st].decel;
  SegOut o{0, v_now, dia};
  if (dist > 0 || first) {
    float d = dist;
    if (FN) {
      v_end = p.map_fast[turn_type].v;
      fast_turn = true;
    }
    if (first) {
      if (dist == 0)
        start_turn = true;
      d += p.start_offset;
      const auto tmp_v2 = 2 * accl * d;
      if (v_end * v_end > tmp_v2) {
        accl = (v_end * v_end) / (2 * d) + 1000;
        decel = -accl;
      }
    }
    if (turn_type == TurnType::Finish) {
      d -= cell / 2;
      v_end = p.suction ? 3500 : p.map[TurnType::Large].v;
    }
    o.time += straight_time(v_now, v_max, v_end, accl, decel, d);
    v_now = v_end;
    if (turn_type == TurnType::Finish) {
      o.v_now = v_now;
      return o;
    }
  }
  if (!(turn_type == TurnType::None || turn_type == TurnType::Finish)) {
    float tt;
    if (fast_turn)
      tt = CTX.pc->slalom_dummy(turn_type, turn_dir, p.map_fast);
    else if (start_turn)
      tt = CTX.pc->slalom_dummy(turn_type, turn_dir, p.map_slow);
    else
      tt = CTX.pc->slalom_dummy(turn_type, turn_dir, p.map);
    if (first && start_turn) {
      v_end = p.map[TC.get_turn_type(next_tcode)].v;
    } else if (dist == 0) {
      v_end = p.map[TC.get_turn_type(tcode)].v;
    }
    v_now = v_end;
    o.time += tt;
    // 斜めかどうか(入る: 7-10 を直進中に、出る: 7-10 を斜め中に。11/12 は斜めのまま)
    if (tcode >= 7 && tcode <= 10)
      o.dia = !dia;
  }
  o.v_now = v_now;
  return o;
}
// 変換後の経路(s, t)を写しで積み上げる(写しが calc_goal_time と一致するかの確認用)
static float model_time(const std::vector<float> &s, const std::vector<int> &t) {
  float v = 0, time = 0;
  bool dia = false;
  for (size_t i = 0; i < t.size(); i++) {
    bool FN = false;
    int nt = 255;
    if (i + 1 < t.size()) {
      nt = t[i + 1];
      const auto ntt = TC.get_turn_type(nt);
      FN = (0.5 * s[i + 1] - 1) > 0 && (ntt == TurnType::Orval || ntt == TurnType::Large);
    }
    const auto o = seg_cost(i == 0, dia, s[i], t[i], v, FN, nt);
    time += o.time;
    v = o.v_now;
    dia = o.dia;
    if (t[i] == 255 || t[i] == 0)
      break;
  }
  return time;
}

// ---- 探索
static int N;
static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0}; // N E S W
static const Direction DIRS[4] = {Direction::North, Direction::East, Direction::South, Direction::West};
static std::vector<char> is_goal;
struct Pos {
  int x, y, d;
};
static bool can_move(const Pos &p, int m, Pos &q) { // m: 0 = F, 1 = R, 2 = L
  const int d = m == 0 ? p.d : m == 1 ? (p.d + 1) % 4 : (p.d + 3) % 4;
  if (CTX.lgc->existWall(p.x, p.y, DIRS[d]) || (!CTX.search && !CTX.lgc->isStep(p.x, p.y, DIRS[d])))
    return false;
  q = Pos{p.x + DX[d], p.y + DY[d], d};
  return q.x >= 0 && q.y >= 0 && q.x < N && q.y < N;
}
static bool goal_at(const Pos &p) { return is_goal[p.x + p.y * N]; }

struct Node {
  Pos p;
  int kind; // 0 = 直進中(ターンの組が終わった後), 1 = 斜め(直前のターン R), 2 = 斜め(直前のターン L), 3 = スタート
  float v;
  int b;  // 直前のターンを「速いターン」として計算したか(次の区間が 直線 > 0 + Large / Orval の約束)
  int nt; // スタート直後のターンが見た「次のターン種別」の約束(0 = なし)
  float cost = 1e30f;
  int prev = -1;
  std::string moves; // prev からの素の動き(F / R / L)
  bool done = false;
};
static std::vector<Node> nodes;
static std::map<std::tuple<int, int, int, int, float, int, int>, int> node_idx;
static long n_edges = 0;
static int get_node(const Pos &p, int kind, float v, int b, int nt) {
  const auto key = std::make_tuple(p.x, p.y, p.d, kind, v, b, nt);
  const auto it = node_idx.find(key);
  if (it != node_idx.end())
    return it->second;
  nodes.push_back(Node{p, kind, v, b, nt});
  node_idx[key] = (int)nodes.size() - 1;
  return (int)nodes.size() - 1;
}
using QE = std::pair<float, int>;
static std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
static float best_goal = 1e30f;
static int best_goal_prev = -1;
static std::string best_goal_moves;

struct EdgeRec { std::string mv; int to; float cost; };
static bool g_collect = false;
static std::vector<EdgeRec> g_edges;
static void relax(int from, float cost, const std::string &mv, const Pos &p, int kind, float v, int b, int nt) {
  n_edges++;
  const int id = get_node(p, kind, v, b, nt);
  if (g_collect) { g_edges.push_back({mv, id, cost}); return; }
  if (cost < nodes[id].cost) {
    nodes[id].cost = cost;
    nodes[id].prev = from;
    nodes[id].moves = mv;
    pq.push({cost, id});
  }
}
static void finish(int from, float cost, const std::string &mv) {
  n_edges++;
  if (g_collect) { g_edges.push_back({mv, -1, cost}); return; }
  if (cost < best_goal) {
    best_goal = cost;
    best_goal_prev = from;
    best_goal_moves = mv;
  }
}
static bool g_count_final = true;
static bool is_LO(int tcode) { return tcode >= 3 && tcode <= 6; }
// この区間(直線 s + ターン tcode)が、直前のターンの約束(b, nt)に合うか
static bool meets(const Node &n, float s, int tcode) {
  if (n.kind == 3)
    return true; // スタートには直前のターンが無い
  const bool fn = (0.5 * s - 1) > 0 && is_LO(tcode);
  if (n.b >= 0 && (n.b != 0) != fn)
    return false; // b = -1: 直前の区間は直線が無く、先読みをしていない(約束なし)
  if (n.nt != 0 && n.nt != tcode)
    return false;
  return true;
}
// 直線 s + ターン tcode を from から出す。着地が直進なら b = 0 / 1 の両方、斜めなら b = 0 だけ。
// スタート直後でいきなりターン(start_turn)のときは、次のターン種別ごとに分ける。
static void emit(int from, bool first, bool dia, float s, int tcode, const std::string &mv, const Pos &land, int land_kind) {
  const Node n = nodes[from];
  if (!meets(n, s, tcode))
    return;
  const bool start_turn = first && (0.5 * s - 1) == 0;
  static const int NEXT_ALL[] = {3, 4, 5, 6, 7, 8, 9, 10, 255};
  static const int NEXT_DIA[] = {7, 8, 9, 10, 11, 12};
  const bool looks_ahead = (0.5 * s - 1) > 0 || first; // calc_goal_time が次の区間を見るのは直線があるときだけ
  for (int bb = 0; bb < 2; bb++) {
    if (bb == 1 && (land_kind != 0 || !looks_ahead))
      continue;
    // 斜めへ着地: 次は Large / Orval にならないので約束は要らない
    const int b = (land_kind != 0 || !looks_ahead) ? -1 : bb;
    const bool fnv = bb == 1;
    if (start_turn) {
      const int *list = land_kind == 0 ? NEXT_ALL : NEXT_DIA;
      const int cnt = land_kind == 0 ? 9 : 6;
      for (int k = 0; k < cnt; k++) {
        const int nt = list[k];
        if (b == 1 && !is_LO(nt))
          continue;
        const auto o = seg_cost(first, dia, s, tcode, n.v, fnv, nt);
        relax(from, n.cost + o.time, mv, land, land_kind, o.v_now, b, nt);
      }
    } else {
      const auto o = seg_cost(first, dia, s, tcode, n.v, fnv, 255);
      relax(from, n.cost + o.time, mv, land, land_kind, o.v_now, b, 0);
    }
  }
}
// 直線 s + ターン tcode のあと、着地がゴール(最後の直線 s_fin = 3)
static void emit_final_turn(int from, bool first, bool dia, float s, int tcode, const std::string &mv) {
  const Node n = nodes[from];
  if (!meets(n, s, tcode))
    return;
  const auto o = seg_cost(first, dia, s, tcode, n.v, false, 255);
  const auto f = seg_cost(false, o.dia, 3, 255, o.v_now, false, 255);
  finish(from, n.cost + o.time + (g_count_final ? f.time : 0), mv);
}

static void expand(int id) {
  const Node n = nodes[id];
  const bool first = n.kind == 3;
  if (n.kind == 0 || n.kind == 3) {
    Pos q = n.p;
    std::string mv;
    for (int f = 0;; f++) {
      if (f > 0 || first) {
        for (int t = 1; t <= 2; t++) {
          Pos r1;
          if (!can_move(q, t, r1))
            continue;
          const float s = first ? 3 + 2 * f - 1 : 2 + 2 * f - 2;
          const char tc = t == 1 ? 'R' : 'L';
          const std::string m1 = mv + tc;
          if (goal_at(r1)) {
            emit_final_turn(id, first, false, s, 4 + t, m1); // Large
            continue;
          }
          Pos tmp;
          if (can_move(r1, 0, tmp))
            emit(id, first, false, s, 4 + t, m1, r1, 0); // Large(次は直進)
          if (can_move(r1, t == 1 ? 2 : 1, tmp))
            emit(id, first, false, s, 6 + t, m1, r1, t); // Dia45 で斜めへ(次は逆向きのターン)
          Pos r2;
          if (can_move(r1, t, r2)) {
            const std::string m2 = m1 + tc;
            if (goal_at(r2)) {
              emit_final_turn(id, first, false, s, 2 + t, m2); // Orval
            } else {
              if (can_move(r2, 0, tmp))
                emit(id, first, false, s, 2 + t, m2, r2, 0); // Orval
              if (can_move(r2, t == 1 ? 2 : 1, tmp))
                emit(id, first, false, s, 8 + t, m2, r2, t); // Dia135 で斜めへ
            }
          }
        }
      }
      Pos nx;
      if (!can_move(q, 0, nx))
        break;
      mv += 'F';
      q = nx;
      if (goal_at(q)) {
        const int fcnt = f + 1;
        const float s = first ? 3 + 2 * fcnt + 2 : 2 + 2 * fcnt + 2 - 1;
        if (meets(n, s, 255)) {
          const auto o = seg_cost(first, false, s, 255, n.v, false, 255);
          finish(id, n.cost + (g_count_final ? o.time : 0), mv);
        }
        break;
      }
    }
  } else {
    Pos cur = n.p;
    int u = n.kind == 1 ? 2 : 1; // 次は逆向き
    std::string mv;
    for (int a = 1;; a++) {
      Pos nx;
      if (!can_move(cur, u, nx))
        break;
      const char uc = u == 1 ? 'R' : 'L';
      mv += uc;
      const float s = 1 + a;
      if (goal_at(nx)) {
        emit_final_turn(id, false, true, s, 6 + u, mv); // Dia45 で出てゴール
        break;
      }
      Pos tmp;
      if (can_move(nx, 0, tmp))
        emit(id, false, true, s, 6 + u, mv, nx, 0); // Dia45 で出る
      Pos r;
      if (can_move(nx, u, r)) {
        const std::string m2 = mv + uc;
        if (goal_at(r)) {
          emit_final_turn(id, false, true, s, 8 + u, m2); // Dia135 で出てゴール
        } else {
          if (can_move(r, 0, tmp))
            emit(id, false, true, s, 8 + u, m2, r, 0); // Dia135 で出る
          if (can_move(r, u == 1 ? 2 : 1, tmp))
            emit(id, false, true, s, 10 + u, m2, r, u); // Dia90(斜めのまま)
        }
      }
      cur = nx;
      u = u == 1 ? 2 : 1;
    }
  }
}

// 素の動きの列を、この探索の辺でなぞれるか。なぞれた所までの位置と、そこでの辺の一覧を返す
static size_t g_trace_best = 0;
static std::string g_trace_info;
static float g_trace_cost = -1;
static bool trace(int id, float cost, const std::string &moves, size_t pos, int depth) {
  g_collect = true;
  g_edges.clear();
  nodes[id].cost = cost;
  expand(id);
  const std::vector<EdgeRec> edges = g_edges;
  g_collect = false;
  if (pos >= g_trace_best) {
    g_trace_best = pos;
    const auto &n = nodes[id];
    g_trace_info = "node (" + std::to_string(n.p.x) + "," + std::to_string(n.p.y) + ") d" + std::to_string(n.p.d) + " kind " + std::to_string(n.kind) + " b" + std::to_string(n.b) + " nt" + std::to_string(n.nt) + " edges:";
    for (const auto &e : edges)
      g_trace_info += " " + e.mv + (e.to < 0 ? "*" : "");
  }
  for (const auto &e : edges) {
    if (moves.compare(pos, e.mv.size(), e.mv) != 0)
      continue;
    if (e.to < 0) {
      if (pos + e.mv.size() == moves.size()) {
        g_trace_cost = e.cost;
        g_trace_best = moves.size();
        return true;
      }
      continue;
    }
    if (pos + e.mv.size() >= moves.size())
      continue;
    if (trace(e.to, e.cost, moves, pos + e.mv.size(), depth + 1))
      return true;
  }
  return false;
}
static void raw_from_moves(const std::string &mv, std::vector<float> &rs, std::vector<int> &rt) {
  rs = {3};
  rt.clear();
  for (char c : mv) {
    if (c == 'F') {
      rs.back() += 2;
    } else {
      rt.push_back(c == 'R' ? 1 : 2);
      rs.push_back(2);
    }
  }
  rs.back() += 2;
  rt.push_back(255);
}


// 探索 1 回ぶん。見つかれば moves(素の動き)とタイムを返す
struct OptResult { bool found = false; float time = 0; std::string moves; int n_nodes = 0; long n_edge = 0; };
static OptResult opt_solve(const std::vector<point_t> &goals, int maze_size) {
  N = maze_size;
  is_goal.assign(N * N, 0);
  for (const auto &g : goals)
    is_goal[g.x + g.y * N] = 1;
  nodes.clear();
  node_idx.clear();
  while (!pq.empty())
    pq.pop();
  best_goal = 1e30f;
  best_goal_prev = -1;
  best_goal_moves.clear();
  n_edges = 0;
  const int st = get_node(Pos{0, 1, 0}, 3, 0.0f, 0, 0);
  nodes[st].cost = 0;
  pq.push({0, st});
  while (!pq.empty()) {
    const auto [c, id] = pq.top();
    pq.pop();
    if (nodes[id].done || c > nodes[id].cost)
      continue;
    nodes[id].done = true;
    if (c >= best_goal)
      break;
    expand(id);
  }
  OptResult r;
  r.n_nodes = (int)nodes.size();
  r.n_edge = n_edges;
  if (best_goal_prev < 0)
    return r;
  r.found = true;
  r.time = best_goal;
  std::vector<std::string> parts = {best_goal_moves};
  for (int id = best_goal_prev; id >= 0 && nodes[id].prev >= 0; id = nodes[id].prev)
    parts.push_back(nodes[id].moves);
  for (auto it = parts.rbegin(); it != parts.rend(); ++it)
    r.moves += *it;
  return r;
}
