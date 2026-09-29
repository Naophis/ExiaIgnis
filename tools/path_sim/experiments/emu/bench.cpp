// 機体と同じ CPU 向けにビルドして、エミュレータ上で探索の計算を回す。
// 状態は 0x21000000 に置いた snap.bin(ホストの探索シミュレータが update() の直前に残したもの)。
#include "stdhdr.hpp"
#define private public
#include "include/search/adachi.hpp"
#undef private
#include <cstring>

extern "C" {
void __libc_init_array(void);
void _init(void) {}
void _fini(void) {}
void bench_begin(int id); // marks.c(別の翻訳単位。呼び出しを消されないように)
void bench_end(int id);
extern volatile int g_result[64];
int _write(int, const char *, int n) { return n; }
}
#ifdef OPT
void opt_search_goal_position(MazeSolverBaseLgc &l, std::unordered_map<unsigned int, unsigned char> &subgoal_list);
void opt_update_dist_map(MazeSolverBaseLgc &l, int mode, bool search_mode);
#endif
#ifdef FAST
void fast_search_goal_position(MazeSolverBaseLgc &l, std::unordered_map<unsigned int, unsigned char> &subgoal_list);
#endif

static const uint8_t *P;
static int rd32() {
  int v;
  memcpy(&v, P, 4);
  P += 4;
  return v;
}

extern "C" int bench_main(int first, int count, int stride) {
  __libc_init_array();
  P = (const uint8_t *)0x21000000;
  const int N = rd32();
  const int ng = rd32();
  std::vector<point_t> goals;
  for (int i = 0; i < ng; i++) {
    point_t p{};
    p.x = rd32();
    p.y = rd32();
    goals.push_back(p);
  }
  const int nsnap = rd32();

  auto lgc = std::make_shared<MazeSolverBaseLgc>();
  auto ego = std::make_shared<ego_t>();
  lgc->set_ego(ego);
  lgc->init(N, N * N - 1);
  lgc->set_goal_pos(goals);
  lgc->set_default_wall_data();
  std::unordered_map<unsigned int, unsigned char> sub, sub0;
  sub.max_load_factor(1);
  sub.reserve(1024);
  // 比べる用(元のまま)
  auto ref = std::make_shared<MazeSolverBaseLgc>();
  ref->set_ego(ego);
  ref->init(N, N * N - 1);
  ref->set_goal_pos(goals);
  ref->set_default_wall_data();
  std::unordered_map<unsigned int, unsigned char> rsub;
  rsub.max_load_factor(1);
  rsub.reserve(1024);

  int done = 0, mismatch = 0;
  for (int k = 0; k < nsnap; k++) {
    const uint8_t *map = P;
    P += N * N;
    const int ex = rd32(), ey = rd32(), ed = rd32(), stationary = rd32();
    (void)stationary;
    const int ns = rd32();
    sub0.clear();
    for (int i = 0; i < ns; i++) {
      const int idx = rd32(), age = rd32();
      sub0[idx] = (unsigned char)age;
    }
    const int np = rd32();
    std::vector<point_t> pts;
    for (int i = 0; i < np; i++) {
      point_t p{};
      p.x = rd32();
      p.y = rd32();
      pts.push_back(p);
    }
    if (k < first || (k - first) % stride != 0 || done >= count)
      continue;
    done++;
    ego->x = ex;
    ego->y = ey;
    ego->dir = static_cast<Direction>(ed);
    for (auto *l : {lgc.get(), ref.get()}) {
      for (int i = 0; i < N * N; i++)
        l->map[i] = map[i];
      l->set_param_num(1);
      l->set_param();
    }
    // 1 回目は前の表を作るだけ(サブゴールの期限切れ判定が前の表を見るため)
    sub = sub0;
    rsub = sub0;
    ref->searchGoalPosition(true, rsub);
    lgc->vector_dist = ref->vector_dist;
    lgc->updateMap = ref->updateMap;
    sub = sub0;
    rsub = sub0;
    // ---- 1: update() の中身(表づくり + 経路の上の未知区画)
    ref->searchGoalPosition(true, rsub);
    bench_begin(1);
#ifdef OPT
    opt_search_goal_position(*lgc, sub);
#else
    lgc->searchGoalPosition(true, sub);
#endif
    bench_end(1);
    if (sub != rsub)
      mismatch++;
    for (int i = 0; i < N * N; i++) {
      const auto &a = lgc->vector_dist[i];
      const auto &b = ref->vector_dist[i];
      if (a.n != b.n || a.e != b.e || a.w != b.w || a.s != b.s || a.N1 != b.N1 || a.NE != b.NE || a.E1 != b.E1 || a.SE != b.SE ||
          a.S1 != b.S1 || a.SW != b.SW || a.W1 != b.W1 || a.NW != b.NW) {
        mismatch++;
        break;
      }
    }
#ifdef FAST
    // ---- 3: 地図が前回と同じときの近道(表は作り直さない)。同じ状態でもう 1 回 update したのと比べる
    {
      auto s3 = sub, r3 = rsub;
      ref->searchGoalPosition(true, r3);
      bench_begin(3);
      fast_search_goal_position(*lgc, s3);
      bench_end(3);
      if (s3 != r3)
        mismatch++;
    }
#endif
    // ---- 2: exec() の中の歩数マップ(サブゴールへ向かう)
    for (auto *l : {lgc.get(), ref.get()}) {
      l->set_goal_pos2(pts);
      l->reset_dist_map();
    }
    ref->update_dist_map(0, true);
    bench_begin(2);
#ifdef OPT
    opt_update_dist_map(*lgc, 0, true);
#else
    lgc->update_dist_map(0, true);
#endif
    bench_end(2);
    for (int i = 0; i < N * N; i++) {
      if (lgc->dist[i] != ref->dist[i]) {
        mismatch++;
        break;
      }
    }
  }
  g_result[0] = done;
  g_result[1] = mismatch;
  return mismatch;
}
