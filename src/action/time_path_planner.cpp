#include "include/action/time_path_planner.hpp"

#include <cstdio>

#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
#include <malloc.h>
#include <unistd.h>
extern "C" {
extern char __HeapLimit;
}
#endif

namespace {

constexpr int DX[4] = {0, 1, 0, -1}; // N E S W
constexpr int DY[4] = {1, 0, -1, 0};
constexpr Direction DIRS[4] = {Direction::North, Direction::East,
                               Direction::South, Direction::West};

// 節点の種類
constexpr int KIND_STR = 0;   // 直進中(ターンの組が終わった後)
constexpr int KIND_DIA_R = 1; // 斜め(直前のターンが右)
constexpr int KIND_DIA_L = 2; // 斜め(直前のターンが左)
constexpr int KIND_START = 3; // スタート

// 次の区間への約束
constexpr int B_NOT_FAST = 0; // 速いターンとして計算していない(次は 直線 > 0 + Large / Orval ではない)
constexpr int B_FAST = 1;     // 速いターンとして計算した(次は 直線 > 0 + Large / Orval)
constexpr int B_FREE = 2;     // 先読みをしていない(直線が無かった / 斜めへ入った)

constexpr int T_FINISH = 255;

// 節点の鍵(24bit): x 5 / y 5 / 向き 2 / 種類 2 / 速度 4 / 約束 2 / 次のターン 4
inline uint32_t make_key(int x, int y, int d, int kind, int v, int b, int nt) {
  return (uint32_t)x | ((uint32_t)y << 5) | ((uint32_t)d << 10) |
         ((uint32_t)kind << 12) | ((uint32_t)v << 14) | ((uint32_t)b << 18) |
         ((uint32_t)nt << 20);
}
inline int kx(uint32_t k) { return k & 31; }
inline int ky(uint32_t k) { return (k >> 5) & 31; }
inline int kd(uint32_t k) { return (k >> 10) & 3; }
inline int kkind(uint32_t k) { return (k >> 12) & 3; }
inline int kv(uint32_t k) { return (k >> 14) & 15; }
inline int kb(uint32_t k) { return (k >> 18) & 3; }
inline int knt(uint32_t k) { return (k >> 20) & 15; }

// 次のターンの約束(スタート直後にターンしたときだけ使う): 0 = なし、1〜10 = 3〜12、11 = 終端
inline int nt_of(int tcode) { return tcode == T_FINISH ? 11 : tcode - 2; }
inline int tcode_of_nt(int nt) { return nt == 11 ? T_FINISH : nt + 2; }

inline bool is_large_or_orval(int tcode) { return tcode >= 3 && tcode <= 6; }

// 機体の空きメモリ(ヒープの上端から続いている分)。ここから取る分は必ず確保できる。
int free_heap_bytes() {
#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
  const char *brk = (const char *)sbrk(0);
  const struct mallinfo mi = mallinfo();
  return (int)(&__HeapLimit - brk) + (int)mi.keepcost;
#else
  return 0;
#endif
}

} // namespace

const char *TimePathPlanner::result_str(Result r) {
  switch (r) {
  case Result::Ok:
    return "ok";
  case Result::NoPath:
    return "no path";
  case Result::NoMemory:
    return "no memory";
  case Result::Overflow:
    return "overflow";
  case Result::Aborted:
    return "aborted";
  }
  return "?";
}

void TimePathPlanner::release() {
  std::vector<Node>().swap(nodes_);
  std::vector<uint16_t>().swap(table_);
  std::vector<HeapE>().swap(heap_);
  std::vector<SegE>().swap(seg_);
  std::vector<uint8_t>().swap(open_);
}

int TimePathPlanner::v_index(float v) {
  for (int i = 0; i < n_v_; i++) {
    if (vlist_[i] == v)
      return i;
  }
  if (n_v_ >= V_MAX) {
    overflow_ = true;
    return 0;
  }
  vlist_[n_v_] = v;
  return n_v_++;
}

// 区間のタイム。calc_goal_time() と同じ計算(PathCreator::calc_segment_time())を呼ぶ。
// 次の区間は「先読みの結果が fast_next になる」ものを渡す。
__attribute__((noinline, section(".time_critical.path_creator")))
TimePathPlanner::Seg TimePathPlanner::seg_raw(bool first, bool dia, int s,
                                              int tcode, int vin,
                                              bool fast_next, int next_tcode) {
  planning_time_t tmp;
  // fast_next: 次は 直線 > 0(path_s = 4)+ Large / Orval。そうでなければ直線なし(path_s = 2)
  const float next_s = fast_next ? 4 : 2;
  const int nt =
      (fast_next && !is_large_or_orval(next_tcode)) ? 5 : next_tcode;
  const auto r = pc_->calc_segment_time(*ps_, first, dia, (float)s, tcode,
                                        vlist_[vin], true, next_s, nt, tmp);
  Seg o;
  o.time = r.str_time + r.turn_time;
  // 直線が進まない(速度・加速度が 0 等で go_straight_dummy が 10000 を返した)区間は使わない
  o.ok = o.time >= 0 && o.time < 100;
  o.v = v_index(r.v_now);
  return o;
}

__attribute__((noinline, section(".time_critical.path_creator")))
TimePathPlanner::Seg TimePathPlanner::seg(bool first, bool dia, int s,
                                          int tcode, int vin, bool fast_next,
                                          int next_tcode) {
  if (first || s > 127) // スタートの区間は 1 本しか無いので覚えない
    return seg_raw(first, dia, s, tcode, vin, fast_next, next_tcode);
  const int tc = tcode == T_FINISH ? 13 : tcode;
  const uint32_t k = (1u << 17) | ((uint32_t)dia << 16) | ((uint32_t)s << 9) |
                     ((uint32_t)tc << 5) | ((uint32_t)vin << 1) |
                     (uint32_t)fast_next;
  const uint32_t mask = SEG_CAP - 1;
  uint32_t h = (k * 2654435761u) >> 16 & mask;
  while (true) {
    SegE &e = seg_[h];
    if (e.key == 0) {
      const Seg o = seg_raw(false, dia, s, tcode, vin, fast_next, next_tcode);
      if (stats.seg_cached < SEG_CAP * 3 / 4) {
        e.key = (k << 4) | (uint32_t)o.v | (o.ok ? 0 : 0x80000000u);
        e.time = o.time;
        stats.seg_cached++;
      }
      return o;
    }
    if (((e.key & 0x7fffffffu) >> 4) == k)
      return Seg{e.time, (int)(e.key & 15), (e.key & 0x80000000u) == 0};
    h = (h + 1) & mask;
  }
}

__attribute__((noinline, section(".time_critical.path_creator")))
int TimePathPlanner::find_node(uint32_t key) {
  uint32_t h = (key * 2654435761u) >> 12 & table_mask_;
  while (true) {
    const uint16_t id = table_[h];
    if (id == NONE) {
      if ((int)nodes_.size() >= stats.node_cap) {
        overflow_ = true;
        return NONE;
      }
      Node n;
      n.cost = 1e30f;
      n.key = key;
      n.run = 0;
      n.done = 0;
      n.prev = NONE;
      n.prim = 0;
      n.pad = 0;
      nodes_.push_back(n);
      table_[h] = (uint16_t)(nodes_.size() - 1);
      return (int)nodes_.size() - 1;
    }
    if (nodes_[id].key == key)
      return id;
    h = (h + 1) & table_mask_;
  }
}

__attribute__((noinline, section(".time_critical.path_creator")))
void TimePathPlanner::heap_push(float cost, int id) {
  if ((int)heap_.size() >= heap_cap_) {
    overflow_ = true;
    return;
  }
  heap_.push_back(HeapE{cost, (uint16_t)id});
  size_t i = heap_.size() - 1;
  while (i > 0) {
    const size_t p = (i - 1) / 2;
    if (heap_[p].cost <= heap_[i].cost)
      break;
    const HeapE t = heap_[p];
    heap_[p] = heap_[i];
    heap_[i] = t;
    i = p;
  }
  if ((int)heap_.size() > stats.heap_max)
    stats.heap_max = (int)heap_.size();
}

__attribute__((noinline, section(".time_critical.path_creator")))
TimePathPlanner::HeapE TimePathPlanner::heap_pop() {
  const HeapE top = heap_[0];
  heap_[0] = heap_.back();
  heap_.pop_back();
  size_t i = 0;
  const size_t n = heap_.size();
  while (true) {
    const size_t l = i * 2 + 1, r = l + 1;
    size_t m = i;
    if (l < n && heap_[l].cost < heap_[m].cost)
      m = l;
    if (r < n && heap_[r].cost < heap_[m].cost)
      m = r;
    if (m == i)
      break;
    const HeapE t = heap_[m];
    heap_[m] = heap_[i];
    heap_[i] = t;
    i = m;
  }
  return top;
}

// m: 0 = 直進、1 = 右、2 = 左。区画 p から向き d の壁を抜けて隣の区画 q へ。
inline bool TimePathPlanner::can_move(const Pos &p, int m, Pos &q) const {
  const int d = m == 0 ? p.d : m == 1 ? (p.d + 1) & 3 : (p.d + 3) & 3;
  if (!(open_[p.x + p.y * n_] & (1 << d)))
    return false;
  q.x = (int8_t)(p.x + DX[d]);
  q.y = (int8_t)(p.y + DY[d]);
  q.d = (int8_t)d;
  return true;
}

inline bool TimePathPlanner::goal_at(const Pos &p) const {
  return (open_[p.x + p.y * n_] & 0x80) != 0;
}

// この区間(直線 s + ターン tcode)が、直前のターンを計算したときの約束に合うか
inline bool TimePathPlanner::meets(uint32_t key, int s, int tcode) const {
  if (kkind(key) == KIND_START)
    return true;
  const int b = kb(key);
  if (b != B_FREE) {
    const bool fast_next = s > 2 && is_large_or_orval(tcode);
    if ((b == B_FAST) != fast_next)
      return false;
  }
  const int nt = knt(key);
  return nt == 0 || tcode_of_nt(nt) == tcode;
}

__attribute__((noinline, section(".time_critical.path_creator")))
void TimePathPlanner::relax(int from, float cost, int run, int prim,
                            const Pos &p, int kind, int v, int b, int nt) {
  stats.edges++;
  if (cost >= best_goal_)
    return; // もうゴールに着いた経路より遅い
  const int id = find_node(make_key(p.x, p.y, p.d, kind, v, b, nt));
  if (id == NONE)
    return;
  Node &n = nodes_[id];
  if (cost < n.cost) {
    n.cost = cost;
    n.prev = (uint16_t)from;
    n.run = (uint32_t)run;
    n.prim = (uint8_t)prim;
    heap_push(cost, id);
  }
}

inline void TimePathPlanner::finish(int from, float cost, int run, int prim) {
  stats.edges++;
  if (cost < best_goal_) {
    best_goal_ = cost;
    best_prev_ = from;
    best_run_ = run;
    best_prim_ = prim;
  }
}

// 直線 s + ターン tcode の辺を from から出す。land = ターンを終えた区画、land_kind = そこが
// 直進か斜めか。直進へ着地するなら「速いターンとして計算する / しない」の両方を出す。
__attribute__((noinline, section(".time_critical.path_creator")))
void TimePathPlanner::emit(int from, bool first, bool dia, int s, int tcode,
                           int run, const Pos &land, int land_kind) {
  const uint32_t key = nodes_[from].key;
  const float base = nodes_[from].cost;
  if (!meets(key, s, tcode))
    return;
  const int vin = kv(key);
  // スタート直後にいきなりターン: 出口の速度が「次のターンの速度」になるので、次のターンごとに分ける
  const bool start_turn = first && s == 2;
  // calc_segment_time() が次の区間を見るのは、直線があるとき(か最初の区間)だけ
  const bool looks_ahead = s > 2 || first;
  static const uint8_t NEXT_STR[] = {3, 4, 5, 6, 7, 8, 9, 10, T_FINISH};
  static const uint8_t NEXT_DIA[] = {7, 8, 9, 10, 11, 12};
  for (int fast = 0; fast < 2; fast++) {
    // 斜めへ着地するなら、次は Large / Orval にならない
    if (fast == 1 && (land_kind != KIND_STR || !looks_ahead))
      continue;
    const int b = (land_kind != KIND_STR || !looks_ahead)
                      ? B_FREE
                      : (fast ? B_FAST : B_NOT_FAST);
    if (start_turn) {
      const uint8_t *list = land_kind == KIND_STR ? NEXT_STR : NEXT_DIA;
      const int cnt = land_kind == KIND_STR ? 9 : 6;
      for (int k = 0; k < cnt; k++) {
        const int nt = list[k];
        if (fast == 1 && !is_large_or_orval(nt))
          continue;
        const Seg o = seg(first, dia, s, tcode, vin, fast == 1, nt);
        if (o.ok)
          relax(from, base + o.time, run, tcode, land, land_kind, o.v, b,
                nt_of(nt));
      }
    } else {
      const Seg o = seg(first, dia, s, tcode, vin, fast == 1, T_FINISH);
      if (o.ok)
        relax(from, base + o.time, run, tcode, land, land_kind, o.v, b, 0);
    }
  }
}

// 直線 s + ターン tcode で着地した区画がゴール。最後の直線は path_s = 3。
__attribute__((noinline, section(".time_critical.path_creator")))
void TimePathPlanner::emit_final_turn(int from, bool first, bool dia, int s,
                                      int tcode, int run) {
  const uint32_t key = nodes_[from].key;
  if (!meets(key, s, tcode))
    return;
  const Seg o = seg(first, dia, s, tcode, kv(key), false, T_FINISH);
  if (!o.ok)
    return;
  const Seg f = seg(false, false, 3, T_FINISH, o.v, false, T_FINISH);
  if (!f.ok)
    return;
  finish(from, nodes_[from].cost + o.time + f.time, run, tcode);
}

__attribute__((noinline, section(".time_critical.path_creator")))
void TimePathPlanner::expand(int id) {
  const uint32_t key = nodes_[id].key;
  const int kind = kkind(key);
  const bool first = kind == KIND_START;
  const Pos start{(int8_t)kx(key), (int8_t)ky(key), (int8_t)kd(key)};
  if (kind == KIND_STR || kind == KIND_START) {
    // 直進を f 区画 → ターン。ターンの組の後は必ず 1 区画は直進する(f >= 1)。
    Pos q = start;
    for (int f = 0;; f++) {
      if (f > 0 || first) {
        // 変換後の path_s: 素は 2 + 2f(最初は 3 + 2f)。前後のターンが 1 ずつ取る
        const int s = first ? 3 + 2 * f - 1 : 2 + 2 * f - 2;
        for (int t = 1; t <= 2; t++) {
          const int rev = t == 1 ? 2 : 1;
          Pos r1, r2, tmp;
          if (!can_move(q, t, r1))
            continue;
          if (goal_at(r1)) {
            emit_final_turn(id, first, false, s, 4 + t, f); // Large でゴール
            continue;
          }
          if (can_move(r1, 0, tmp))
            emit(id, first, false, s, 4 + t, f, r1, KIND_STR); // Large
          if (can_move(r1, rev, tmp))
            emit(id, first, false, s, 6 + t, f, r1, t); // Dia45 で斜めへ
          if (!can_move(r1, t, r2))
            continue;
          if (goal_at(r2)) {
            emit_final_turn(id, first, false, s, 2 + t, f); // Orval でゴール
            continue;
          }
          if (can_move(r2, 0, tmp))
            emit(id, first, false, s, 2 + t, f, r2, KIND_STR); // Orval
          if (can_move(r2, rev, tmp))
            emit(id, first, false, s, 8 + t, f, r2, t); // Dia135 で斜めへ
        }
      }
      Pos nx;
      if (!can_move(q, 0, nx))
        break;
      q = nx;
      if (goal_at(q)) {
        // 直進のままゴール。素の最後の直線は +2(ゴールの区画ぶん)
        const int fcnt = f + 1;
        const int s = first ? 3 + 2 * fcnt + 2 : 2 + 2 * fcnt + 2 - 1;
        if (meets(key, s, T_FINISH)) {
          const Seg o = seg(first, false, s, T_FINISH, kv(key), false, T_FINISH);
          if (o.ok)
            finish(id, nodes_[id].cost + o.time, fcnt, T_FINISH);
        }
        break;
      }
    }
  } else {
    // 斜め: 向きが交互のターンを a 個 → 出る(Dia45 / Dia135)か折り返す(Dia90)。
    // 斜めへ入ったターン(か Dia90)が 1 個目なので、斜めの path_s は 1 + a。
    Pos cur = start;
    int u = kind == KIND_DIA_R ? 2 : 1; // 次は逆向き
    for (int a = 1;; a++) {
      const int rev = u == 1 ? 2 : 1;
      Pos nx, r, tmp;
      if (!can_move(cur, u, nx))
        break;
      const int s = 1 + a;
      if (goal_at(nx)) {
        emit_final_turn(id, false, true, s, 6 + u, a); // Dia45 で出てゴール
        break;
      }
      if (can_move(nx, 0, tmp))
        emit(id, false, true, s, 6 + u, a, nx, KIND_STR); // Dia45 で出る
      if (can_move(nx, u, r)) {
        if (goal_at(r)) {
          emit_final_turn(id, false, true, s, 8 + u, a); // Dia135 で出てゴール
        } else {
          if (can_move(r, 0, tmp))
            emit(id, false, true, s, 8 + u, a, r, KIND_STR); // Dia135 で出る
          if (can_move(r, rev, tmp))
            emit(id, false, true, s, 10 + u, a, r, u); // Dia90(斜めのまま)
        }
      }
      cur = nx;
      u = rev;
    }
  }
}

// 求めた経路を、path_create() が作るのと同じ素の形(直進は +2、ターンは R / L、最後に +2 と 255)で
// pc の path_s / path_t へ入れる。
void TimePathPlanner::write_path() {
  // ゴールからさかのぼって区間を並べる
  struct Edge {
    uint8_t kind, run, prim;
  };
  std::vector<Edge> edges;
  edges.push_back(Edge{(uint8_t)kkind(nodes_[best_prev_].key), (uint8_t)best_run_,
                       (uint8_t)best_prim_});
  for (int id = best_prev_; nodes_[id].prev != NONE; id = nodes_[id].prev) {
    const Node &n = nodes_[id];
    edges.push_back(Edge{(uint8_t)kkind(nodes_[n.prev].key), (uint8_t)n.run, n.prim});
  }

  auto &path_s = pc_->path_s;
  auto &path_t = pc_->path_t;
  path_s.clear();
  path_t.clear();
  path_s.emplace_back(3);
  const auto turn = [&](int t) {
    path_t.emplace_back((unsigned char)t);
    path_s.emplace_back(2);
  };
  for (int i = (int)edges.size() - 1; i >= 0; i--) {
    const Edge &e = edges[i];
    const int t = (e.prim & 1) ? R : L;
    if (e.kind == KIND_STR || e.kind == KIND_START) {
      path_s.back() += 2 * e.run;
      if (e.prim == T_FINISH)
        continue;
      turn(t);
      if (e.prim == 3 || e.prim == 4 || e.prim == 9 || e.prim == 10)
        turn(t); // Orval / Dia135 は同じ向きに 2 回
    } else {
      int u = e.kind == KIND_DIA_R ? L : R;
      for (int a = 0; a < e.run; a++) {
        turn(u);
        u = u == R ? L : R;
      }
      if (e.prim >= 9 && e.prim <= 12)
        turn(t); // Dia135 で出る / Dia90 は最後と同じ向きをもう 1 回
    }
  }
  path_s.back() += 2;
  path_t.emplace_back(T_FINISH);
  pc_->path_size = (int)path_t.size() - 1;
}

TimePathPlanner::Result TimePathPlanner::solve(PathCreator &pc,
                                               param_set_t &p_set) {
  pc_ = &pc;
  lgc_ = pc.lgc.get();
  ps_ = &p_set;
  n_ = lgc_->maze_size;
  stats = Stats{};
  overflow_ = false;
  n_v_ = 0;
  best_goal_ = 1e30f;
  best_prev_ = NONE;
  if (n_ <= 0 || n_ > 32)
    return Result::Overflow;

  // 節点の上限は 2 の累乗。1 個あたり 節点 12 + ハッシュ表 4 + ヒープ 4 バイト
  // (ヒープは節点の半分。大会迷路 21 本 × 5 モードで、節点 3601 に対してヒープは最大 546)
  constexpr int PER_NODE = (int)sizeof(Node) + 2 * (int)sizeof(uint16_t) +
                           (int)sizeof(HeapE) / 2;
  const int fixed = SEG_CAP * (int)sizeof(SegE) + n_ * n_;
  int cap = node_cap_request;
  stats.free_bytes = free_heap_bytes();
  if (cap <= 0) {
#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE
    // 走行の準備(パラメータの読み直し等)に残す分
    constexpr int RESERVE = 24 * 1024;
    const int budget = stats.free_bytes - RESERVE - fixed;
    cap = 8192;
    while (cap >= 1024 && cap * PER_NODE > budget)
      cap /= 2;
    if (cap < 1024)
      return Result::NoMemory;
#else
    cap = 32768;
#endif
  }
  if (cap > 32768)
    cap = 32768;
  stats.node_cap = cap;
  heap_cap_ = cap / 2;
  stats.mem_bytes = cap * PER_NODE + fixed;

  nodes_.reserve(cap);
  table_.assign((size_t)cap * 2, (uint16_t)NONE);
  table_mask_ = (uint32_t)cap * 2 - 1;
  heap_.reserve(heap_cap_);
  seg_.assign(SEG_CAP, SegE{0, 0});

  // 通れる向き(既知で壁が無い)とゴール
  open_.assign((size_t)n_ * n_, 0);
  for (int y = 0; y < n_; y++) {
    for (int x = 0; x < n_; x++) {
      uint8_t v = lgc_->arrival_goal_position(x, y) ? 0x80 : 0;
      for (int d = 0; d < 4; d++) {
        const int nx = x + DX[d], ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= n_ || ny >= n_)
          continue;
        if (!lgc_->existWall(x, y, DIRS[d]) && lgc_->isStep(x, y, DIRS[d]))
          v |= (uint8_t)(1 << d);
      }
      open_[x + y * n_] = v;
    }
  }

  Result res = Result::NoPath;
  // path_create() と同じく、(0, 0) から北へ 1 区画進んだ (0, 1) から始める
  const int st = find_node(make_key(0, 1, 0, KIND_START, v_index(0.0f), 0, 0));
  nodes_[st].cost = 0;
  heap_push(0, st);
  int cnt = 0;
  while (!heap_.empty()) {
    const HeapE e = heap_pop();
    Node &n = nodes_[e.id];
    if (n.done || e.cost > n.cost)
      continue;
    n.done = 1;
    if (e.cost >= best_goal_)
      break; // 残りは、もう着いた経路より遅い
    expand(e.id);
    if (overflow_) {
      res = Result::Overflow;
      break;
    }
    if (++cnt % ABORT_CHECK == 0 && pc_->ui && pc_->ui->button_state()) {
      res = Result::Aborted;
      break;
    }
  }
  stats.nodes = (int)nodes_.size();
  if (res == Result::NoPath && best_prev_ != NONE) {
    res = Result::Ok;
    stats.time = best_goal_;
    write_path();
  }
  release();
  return res;
}
