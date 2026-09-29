// 実験 3 の本体(速い版)。opt_core.hpp と同じ探索を、整数の鍵・表引き・文字列なしで書いたもの。
// ファームへ持っていくときの形を想定: 節点はハッシュ表、区間のタイムは (直線の長さ, ターン, 入口速度, 先読み)
// で引く表(最初に使ったときに calc_goal_time と同じ計算で埋める)。
#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace tp {

struct Ctx {
  MazeSolverBaseLgc *lgc;
  PathCreator *pc;
  param_set_t *ps;
  bool search; // true: 未知の壁は無いものとして通る(探索中のサブゴール用)
  bool count_final = true;
  bool astar = false;  // 残り時間の下限を使う
  float weight = 1.0f; // 下限に掛ける倍率(1 より大きいと厳密でなくなる)
  float upper = 1e30f; // これ以上のタイムの経路は要らない(既知だけの最小タイム等)
  int node_cap = 0;    // 節点の上限(0 = なし)。超えたら Result::overflow
  // 節点を「位置・向き・直進か斜めか」だけにまとめる(速度と約束は、その節点へいちばん速く着いた
  // 経路のものを使う。速いターンは使わない)。節点が数分の 1 になるが、厳密ではなくなる
  bool collapse = false;
};
static Ctx C;
static TrajectoryCreator TCR;

static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};
static const Direction DIRS[4] = {Direction::North, Direction::East, Direction::South, Direction::West};

// ---- 区間のタイム(calc_goal_time の 1 区間ぶん)。表に覚える
static std::vector<float> vlist; // 出てくる速度
static int vidx_of(float v) {
  for (size_t i = 0; i < vlist.size(); i++)
    if (vlist[i] == v)
      return (int)i;
  vlist.push_back(v);
  return (int)vlist.size() - 1;
}
struct Seg {
  float time;
  int v;
};
static float straight_sim(float v1, float vmax, float v2, float ac, float diac, float dist) {
  planning_time_t tmp;
  return C.pc->go_straight_dummy(v1, vmax, v2, ac, diac, dist, tmp, false);
}
static Seg seg_raw(bool first, bool dia, int s, int tcode, int vin, bool FN, int next_tcode) {
  auto &p = *C.ps;
  const float cell = p.cell_size;
  const float dist = !dia ? (0.5f * s - 1) * cell : (0.5f * s - 1) * cell * ROOT2;
  const auto turn_dir = TCR.get_turn_dir(tcode);
  const auto turn_type = TCR.get_turn_type(tcode, dia);
  const bool fast_mode = !first || dist > 0;
  bool start_turn = false, fast_turn = false;
  const auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
  const float v_max = p.str_map[st].v_max;
  float v_end = fast_mode ? p.map[turn_type].v : p.map_slow[turn_type].v;
  float accl = p.str_map[st].accl;
  float decel = p.str_map[st].decel;
  float v_now = vlist[vin];
  float time = 0;
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
    time += straight_sim(v_now, v_max, v_end, accl, decel, d);
    v_now = v_end;
    if (turn_type == TurnType::Finish)
      return Seg{time, vidx_of(v_now)};
  }
  if (!(turn_type == TurnType::None || turn_type == TurnType::Finish)) {
    if (fast_turn)
      time += C.pc->slalom_dummy(turn_type, turn_dir, p.map_fast);
    else if (start_turn)
      time += C.pc->slalom_dummy(turn_type, turn_dir, p.map_slow);
    else
      time += C.pc->slalom_dummy(turn_type, turn_dir, p.map);
    if (first && start_turn)
      v_end = p.map[TCR.get_turn_type(next_tcode)].v;
    else if (dist == 0)
      v_end = p.map[TCR.get_turn_type(tcode)].v;
    v_now = v_end;
  }
  return Seg{time, vidx_of(v_now)};
}
// 表: [dia][s 0..79][tcode 0..13(13 = 終端)][vin 0..15][FN]。スタートの区間は毎回計算する(1 本しか無い)
static const int S_MAX = 80, V_MAX = 16;
static Seg seg_tab[2][S_MAX][14][V_MAX][2];
static bool seg_has[2][S_MAX][14][V_MAX][2];
static long n_seg_filled = 0;
static inline int tc_i(int tcode) { return tcode == 255 ? 13 : tcode; }
static inline Seg seg(bool first, bool dia, int s, int tcode, int vin, bool FN, int next_tcode) {
  if (first)
    return seg_raw(true, dia, s, tcode, vin, FN, next_tcode);
  bool &has = seg_has[dia][s][tc_i(tcode)][vin][FN];
  Seg &e = seg_tab[dia][s][tc_i(tcode)][vin][FN];
  if (!has) {
    e = seg_raw(false, dia, s, tcode, vin, FN, 255);
    has = true;
    n_seg_filled++;
  }
  return e;
}

// ---- 節点
struct Node {
  uint32_t key;
  float cost; // スタートからのタイム
  float pri;  // 並べる値(cost + 下限)
  int32_t prev;
  uint16_t run; // prev からの直進 / 斜めの数
  uint8_t prim; // prev からの区間の終わりのターン(tcode)
  uint8_t done;
  uint8_t v;    // collapse のとき: いちばん速く着いた経路の速度
};
static std::vector<Node> nodes;
static std::vector<int32_t> table; // ハッシュ表(開番地)
static uint32_t table_mask;
static int N;
static std::vector<uint8_t> is_goal;
static std::vector<float> hcell; // 区画ごとの残り時間の下限
static long n_edges, n_expand;

// kind: 0 = 直進, 1 = 斜め(直前 R), 2 = 斜め(直前 L), 3 = スタート。b: 0 / 1 / 2(= 約束なし)。nt: 0 = なし、他は tcode
static inline uint32_t make_key(int x, int y, int d, int kind, int v, int b, int nt) {
  return (uint32_t)x | ((uint32_t)y << 5) | ((uint32_t)d << 10) | ((uint32_t)kind << 12) | ((uint32_t)v << 14) | ((uint32_t)b << 18) | ((uint32_t)nt << 20);
}
static inline int kx(uint32_t k) { return k & 31; }
static inline int ky(uint32_t k) { return (k >> 5) & 31; }
static inline int kd(uint32_t k) { return (k >> 10) & 3; }
static inline int kkind(uint32_t k) { return (k >> 12) & 3; }
static inline int kv(uint32_t k) { return (k >> 14) & 15; }
static inline int kb(uint32_t k) { return (k >> 18) & 3; }
static inline int knt(uint32_t k) { return (k >> 20) & 255; }
static bool g_overflow = false;
static int find_node(uint32_t key) {
  uint32_t h = (key * 2654435761u) & table_mask;
  while (true) {
    const int32_t id = table[h];
    if (id < 0) {
      if (C.node_cap > 0 && (int)nodes.size() >= C.node_cap) {
        g_overflow = true;
        return -1;
      }
      nodes.push_back(Node{key, 1e30f, 1e30f, -1, 0, 0, 0, 0});
      table[h] = (int32_t)nodes.size() - 1;
      return (int)nodes.size() - 1;
    }
    if (nodes[id].key == key)
      return id;
    h = (h + 1) & table_mask;
  }
}
// 2 分ヒープ(値, 節点)
struct HeapE {
  float pri;
  int32_t id;
};
static std::vector<HeapE> heap;
static void heap_push(float pri, int id) {
  heap.push_back(HeapE{pri, id});
  size_t i = heap.size() - 1;
  while (i > 0) {
    const size_t p = (i - 1) / 2;
    if (heap[p].pri <= heap[i].pri)
      break;
    std::swap(heap[p], heap[i]);
    i = p;
  }
}
static HeapE heap_pop() {
  const HeapE top = heap[0];
  heap[0] = heap.back();
  heap.pop_back();
  size_t i = 0;
  const size_t n = heap.size();
  while (true) {
    size_t l = i * 2 + 1, r = l + 1, m = i;
    if (l < n && heap[l].pri < heap[m].pri)
      m = l;
    if (r < n && heap[r].pri < heap[m].pri)
      m = r;
    if (m == i)
      break;
    std::swap(heap[m], heap[i]);
    i = m;
  }
  return top;
}

struct Pos {
  int x, y, d;
};
// 壁の表(毎回 existWall を呼ばない): open[cell * 4 + d]
static std::vector<uint8_t> open_tab;
static void build_open() {
  open_tab.assign(N * N * 4, 0);
  for (int y = 0; y < N; y++)
    for (int x = 0; x < N; x++)
      for (int d = 0; d < 4; d++) {
        const int nx = x + DX[d], ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= N || ny >= N)
          continue;
        if (C.lgc->existWall(x, y, DIRS[d]))
          continue;
        if (!C.search && !C.lgc->isStep(x, y, DIRS[d]))
          continue;
        open_tab[(x + y * N) * 4 + d] = 1;
      }
}
static inline bool can_move(const Pos &p, int m, Pos &q) { // m: 0 = F, 1 = R, 2 = L
  const int d = m == 0 ? p.d : m == 1 ? (p.d + 1) & 3 : (p.d + 3) & 3;
  if (!open_tab[(p.x + p.y * N) * 4 + d])
    return false;
  q = Pos{p.x + DX[d], p.y + DY[d], d};
  return true;
}
static inline bool goal_at(const Pos &p) { return is_goal[p.x + p.y * N]; }

static float best_goal;
static int best_prev;
static int best_run, best_prim;

static inline bool is_LO(int tcode) { return tcode >= 3 && tcode <= 6; }
static inline bool meets(uint32_t key, int s, int tcode) {
  if (kkind(key) == 3 || C.collapse)
    return true;
  const int b = kb(key);
  if (b != 2) {
    const bool fn = s > 2 && is_LO(tcode);
    if ((b != 0) != fn)
      return false;
  }
  const int nt = knt(key);
  return nt == 0 || nt == tcode;
}
static inline void relax(int from, float cost, int run, int prim, const Pos &p, int kind, int v, int b, int nt) {
  n_edges++;
  // 残りを最短で行っても、もう分かっている経路(best_goal)より遅いなら要らない
  if (cost + (C.astar ? hcell[p.x + p.y * N] : 0) >= best_goal)
    return;
  const int id = find_node(C.collapse ? make_key(p.x, p.y, p.d, kind, 0, 0, 0) : make_key(p.x, p.y, p.d, kind, v, b, nt));
  if (id < 0)
    return;
  Node &n = nodes[id];
  if (cost < n.cost) {
    n.cost = cost;
    n.v = (uint8_t)v;
    n.prev = from;
    n.run = (uint16_t)run;
    n.prim = (uint8_t)prim;
    n.pri = cost + (C.astar ? C.weight * hcell[p.x + p.y * N] : 0);
    heap_push(n.pri, id);
  }
}
static inline void finish(int from, float cost, int run, int prim) {
  n_edges++;
  if (cost < best_goal) {
    best_goal = cost;
    best_prev = from;
    best_run = run;
    best_prim = prim;
  }
}
static void emit(int from, bool first, bool dia, int s, int tcode, int run, const Pos &land, int land_kind) {
  const uint32_t key = nodes[from].key;
  const float base = nodes[from].cost;
  if (!meets(key, s, tcode))
    return;
  const int vin = C.collapse ? nodes[from].v : kv(key);
  const bool start_turn = first && s == 2;
  if (C.collapse) {
    const Seg o = seg(first, dia, s, tcode, vin, false, 5);
    relax(from, base + o.time, run, tcode, land, land_kind, o.v, 2, 0);
    return;
  }
  const bool looks_ahead = s > 2 || first;
  static const int NEXT_ALL[] = {3, 4, 5, 6, 7, 8, 9, 10, 255};
  static const int NEXT_DIA[] = {7, 8, 9, 10, 11, 12};
  for (int bb = 0; bb < 2; bb++) {
    if (bb == 1 && (land_kind != 0 || !looks_ahead))
      continue;
    const int b = (land_kind != 0 || !looks_ahead) ? 2 : bb;
    if (start_turn) {
      const int *list = land_kind == 0 ? NEXT_ALL : NEXT_DIA;
      const int cnt = land_kind == 0 ? 9 : 6;
      for (int k = 0; k < cnt; k++) {
        const int nt = list[k];
        if (bb == 1 && !is_LO(nt))
          continue;
        const Seg o = seg(first, dia, s, tcode, vin, bb == 1, nt);
        relax(from, base + o.time, run, tcode, land, land_kind, o.v, b, nt);
      }
    } else {
      const Seg o = seg(first, dia, s, tcode, vin, bb == 1, 255);
      relax(from, base + o.time, run, tcode, land, land_kind, o.v, b, 0);
    }
  }
}
static void emit_final_turn(int from, bool first, bool dia, int s, int tcode, int run) {
  const uint32_t key = nodes[from].key;
  if (!meets(key, s, tcode))
    return;
  const Seg o = seg(first, dia, s, tcode, C.collapse ? nodes[from].v : kv(key), false, 255);
  float t = nodes[from].cost + o.time;
  if (C.count_final)
    t += seg(false, false, 3, 255, o.v, false, 255).time;
  finish(from, t, run, tcode);
}
static void expand(int id) {
  n_expand++;
  const uint32_t key = nodes[id].key;
  const int kind = kkind(key);
  const bool first = kind == 3;
  const Pos start{kx(key), ky(key), kd(key)};
  if (kind == 0 || kind == 3) {
    Pos q = start;
    for (int f = 0;; f++) {
      if (f > 0 || first) {
        for (int t = 1; t <= 2; t++) {
          Pos r1;
          if (!can_move(q, t, r1))
            continue;
          const int s = first ? 3 + 2 * f - 1 : 2 + 2 * f - 2;
          if (goal_at(r1)) {
            emit_final_turn(id, first, false, s, 4 + t, f);
            continue;
          }
          Pos tmp;
          if (can_move(r1, 0, tmp))
            emit(id, first, false, s, 4 + t, f, r1, 0);
          if (can_move(r1, t == 1 ? 2 : 1, tmp))
            emit(id, first, false, s, 6 + t, f, r1, t);
          Pos r2;
          if (can_move(r1, t, r2)) {
            if (goal_at(r2)) {
              emit_final_turn(id, first, false, s, 2 + t, f);
            } else {
              if (can_move(r2, 0, tmp))
                emit(id, first, false, s, 2 + t, f, r2, 0);
              if (can_move(r2, t == 1 ? 2 : 1, tmp))
                emit(id, first, false, s, 8 + t, f, r2, t);
            }
          }
        }
      }
      Pos nx;
      if (!can_move(q, 0, nx))
        break;
      q = nx;
      if (goal_at(q)) {
        const int fcnt = f + 1;
        const int s = first ? 3 + 2 * fcnt + 2 : 2 + 2 * fcnt + 2 - 1;
        if (meets(key, s, 255)) {
          const Seg o = seg(first, false, s, 255, C.collapse ? nodes[id].v : kv(key), false, 255);
          finish(id, nodes[id].cost + (C.count_final ? o.time : 0), fcnt, 255);
        }
        break;
      }
    }
  } else {
    Pos cur = start;
    int u = kind == 1 ? 2 : 1;
    for (int a = 1;; a++) {
      Pos nx;
      if (!can_move(cur, u, nx))
        break;
      const int s = 1 + a;
      if (goal_at(nx)) {
        emit_final_turn(id, false, true, s, 6 + u, a);
        break;
      }
      Pos tmp;
      if (can_move(nx, 0, tmp))
        emit(id, false, true, s, 6 + u, a, nx, 0);
      Pos r;
      if (can_move(nx, u, r)) {
        if (goal_at(r)) {
          emit_final_turn(id, false, true, s, 8 + u, a);
        } else {
          if (can_move(r, 0, tmp))
            emit(id, false, true, s, 8 + u, a, r, 0);
          if (can_move(r, u == 1 ? 2 : 1, tmp))
            emit(id, false, true, s, 10 + u, a, r, u);
        }
      }
      cur = nx;
      u = u == 1 ? 2 : 1;
    }
  }
}
// 区間 1 本ぶんの素の動き
static std::string seg_moves(int from_kind, int run, int prim) {
  std::string mv;
  if (from_kind == 0 || from_kind == 3) {
    mv.assign(run, 'F');
    if (prim == 255)
      return mv;
    const char c = (prim & 1) ? 'R' : 'L';
    mv += c;
    if (prim == 3 || prim == 4 || prim == 9 || prim == 10)
      mv += c; // Orval / Dia135 は同じ向きに 2 回
    return mv;
  }
  char u = from_kind == 1 ? 'L' : 'R';
  for (int a = 0; a < run; a++) {
    mv += u;
    u = u == 'R' ? 'L' : 'R';
  }
  if (prim >= 9 && prim <= 12)
    mv += mv.back(); // Dia135 で出る / Dia90 は最後と同じ向きをもう 1 回
  return mv;
}
// 残り時間の下限: 区画を 1 つ進むのに最低かかる時間 × ゴールまでの区画数(壁を見た歩数)
static void build_h(float per_cell) {
  hcell.assign(N * N, 1e30f);
  std::vector<int> q;
  for (int i = 0; i < N * N; i++)
    if (is_goal[i]) {
      hcell[i] = 0;
      q.push_back(i);
    }
  for (size_t h = 0; h < q.size(); h++) {
    const int c = q[h];
    const int x = c % N, y = c / N;
    for (int d = 0; d < 4; d++) {
      if (!open_tab[c * 4 + d])
        continue;
      const int nc = (x + DX[d]) + (y + DY[d]) * N;
      if (hcell[nc] > 1e29f) {
        hcell[nc] = hcell[c] + per_cell;
        q.push_back(nc);
      }
    }
  }
}
struct Result {
  bool found = false;
  float time = 0;
  std::string moves;
  int n_nodes = 0;
  long n_edge = 0, n_exp = 0;
  int heap_max = 0;
  bool overflow = false;
  long seg_filled = 0;
  int n_speeds = 0;
};
static param_set_t *tab_ps = nullptr;
static Result solve(const std::vector<point_t> &goals, int maze_size) {
  N = maze_size;
  if (tab_ps != C.ps) { // 走行パラメータが変わったら表を作り直す
    memset(seg_has, 0, sizeof(seg_has));
    n_seg_filled = 0;
    vlist.clear();
    vidx_of(0.0f);
    tab_ps = C.ps;
  }
  is_goal.assign(N * N, 0);
  for (const auto &g : goals)
    is_goal[g.x + g.y * N] = 1;
  build_open();
  if (C.astar) {
    // 素の動き 1 個(区画を 1 つ進む)に最低かかる時間。直線は最高速で抜ける時間。ターンは
    // 前後の直線を 1 区画ぶん取るので、ターンの時間 / (使う動きの数 + 1) も下限に入れる。
    auto &p = *C.ps;
    const float vs = p.str_map[StraightType::FastRun].v_max;
    const float vd = p.str_map[StraightType::FastRunDia].v_max;
    float per = std::min(p.cell_size / vs, p.cell_size * ROOT2 / 2 / vd);
    const auto turn_min = [&](TurnType t, int moves) {
      for (auto *m : {&p.map, &p.map_fast}) {
        for (auto dir : {TurnDirection::Right, TurnDirection::Left}) {
          const float tt = C.pc->slalom_dummy(t, dir, *m);
          if (tt > 0)
            per = std::min(per, tt / (moves + 1));
        }
      }
    };
    turn_min(TurnType::Large, 1);
    turn_min(TurnType::Orval, 2);
    turn_min(TurnType::Dia45, 1);
    turn_min(TurnType::Dia135, 2);
    turn_min(TurnType::Dia45_2, 1);
    turn_min(TurnType::Dia135_2, 2);
    turn_min(TurnType::Dia90, 2);
    build_h(per);
  }
  nodes.clear();
  if (table.empty()) {
    table.assign(1 << 20, -1);
    table_mask = (1 << 20) - 1;
  } else {
    std::fill(table.begin(), table.end(), -1);
  }
  heap.clear();
  best_goal = C.upper;
  best_prev = -1;
  n_edges = n_expand = 0;
  g_overflow = false;
  const int st = find_node(make_key(0, 1, 0, 3, 0, 0, 0));
  nodes[st].cost = 0;
  heap_push(0, st);
  Result r;
  while (!heap.empty()) {
    r.heap_max = std::max(r.heap_max, (int)heap.size());
    const HeapE e = heap_pop();
    Node &n = nodes[e.id];
    if (n.done || e.pri > n.pri)
      continue;
    n.done = 1;
    if (e.pri >= best_goal)
      break;
    expand(e.id);
    if (g_overflow)
      break;
  }
  r.overflow = g_overflow;
  r.n_nodes = (int)nodes.size();
  r.n_edge = n_edges;
  r.n_exp = n_expand;
  r.seg_filled = n_seg_filled;
  r.n_speeds = (int)vlist.size();
  if (best_prev < 0 || g_overflow)
    return r;
  r.found = true;
  r.time = best_goal;
  std::vector<std::string> parts = {seg_moves(kkind(nodes[best_prev].key), best_run, best_prim)};
  for (int id = best_prev; nodes[id].prev >= 0; id = nodes[id].prev)
    parts.push_back(seg_moves(kkind(nodes[nodes[id].prev].key), nodes[id].run, nodes[id].prim));
  for (auto it = parts.rbegin(); it != parts.rend(); ++it)
    r.moves += *it;
  return r;
}
} // namespace tp
