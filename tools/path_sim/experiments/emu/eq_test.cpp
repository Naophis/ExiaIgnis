// 書き直した関数が、変更前と同じ結果を返すかを乱数の状態で確かめる(ホスト)。
// 同じソースを「変更前の写し(../frozen2)」と「いまのファーム」の 2 通りにビルドし、出力(状態ごとの
// チェックサム)を比べる(eq_test.py)。探索 1 本まるごとの比較(fw_test.py)では通りにくい組み合わせ
// (mode 1 の歩数マップ、isSearch = false の表づくり、行けないゴール、古いサブゴール)を見るため。
#include "stdhdr.hpp"
#define private public
#include "adachi.hpp" // -I の順で、変更前の写し(frozen2)かファームかが決まる
#undef private
#include <cstdio>
#include <cstring>
#include <random>

static uint32_t g_hash;
static void hmix(uint32_t v) { g_hash = (g_hash ^ v) * 16777619u; }
static void hmixf(float f) {
  uint32_t v;
  memcpy(&v, &f, 4);
  hmix(v);
}
static void hash_sub(const std::unordered_map<unsigned int, unsigned char> &m) {
  uint32_t sum = 0;
  for (const auto &kv : m)
    sum += (kv.first * 2654435761u) ^ (kv.second * 40503u + 1);
  hmix((uint32_t)m.size());
  hmix(sum);
}
static void hash_table(MazeSolverBaseLgc &l) {
  for (unsigned i = 0; i < l.maze_size * l.maze_size; i++) {
    const auto &a = l.vector_dist[i];
    hmixf(a.n), hmixf(a.e), hmixf(a.w), hmixf(a.s);
    hmix(a.N1 | (a.NE << 4) | (a.E1 << 8) | (a.SE << 12) | (a.S1 << 16) | (a.SW << 20) | (a.W1 << 24) | (a.NW << 28));
    hmix(l.updateMap[i]);
  }
}

// 表をスタートからたどって、ゴールに着くか行き止まりで終わるか。元の searchGoalPosition() のたどり方は
// 回数の上限が無く、乱数の状態(ゴールへ行けない表)では同じ所を回り続けることがあるので、先に確かめる。
static bool walk_ends(MazeSolverBaseLgc &l) {
  Direction next_dir = Direction::North, now_dir = Direction::North;
  int x = 0, y = 1;
  Direction dirLog[3] = {now_dir, now_dir, now_dir};
  const int N = l.maze_size;
  for (int guard = 0; guard < N * N * 4; guard++) {
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;
    next_dir = Direction::Undefined;
    if (l.arrival_goal_position(x, y))
      return true;
    float position = l.getDistVector(x, y, now_dir);
    if (now_dir == Direction::North)
      position = l.getDistVector(x, y, Direction::South);
    else if (now_dir == Direction::East)
      position = l.getDistVector(x, y, Direction::West);
    else if (now_dir == Direction::West)
      position = l.getDistVector(x, y, Direction::East);
    else if (now_dir == Direction::South)
      position = l.getDistVector(x, y, Direction::North);
    l.setNextRootDirectionPathUnKnown(x, y, Direction::North, now_dir, next_dir, position);
    l.setNextRootDirectionPathUnKnown(x, y, Direction::East, now_dir, next_dir, position);
    l.setNextRootDirectionPathUnKnown(x, y, Direction::West, now_dir, next_dir, position);
    l.setNextRootDirectionPathUnKnown(x, y, Direction::South, now_dir, next_dir, position);
    if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2])
      l.priorityStraight2(x, y, now_dir, dirLog[0], position, next_dir);
    else
      l.priorityStraight2(x, y, now_dir, dirLog[1], position, next_dir);
    if (next_dir == Direction::North)
      y++;
    else if (next_dir == Direction::East)
      x++;
    else if (next_dir == Direction::West)
      x--;
    else if (next_dir == Direction::South)
      y--;
    else
      return true;
    if (x < 0 || y < 0 || x >= N || y >= N)
      return false;
  }
  return false;
}

int main(int argc, char **argv) {
  const int cases = argc > 1 ? atoi(argv[1]) : 3000;
  std::mt19937 rng(12345);
  const auto rnd = [&](int n) { return (int)(rng() % (unsigned)n); };
  for (int c = 0; c < cases; c++) {
    const int N = (c % 3 == 0) ? 16 : 32;
    const int wall_pct = 10 + rnd(35);
    const int known_pct = rnd(101);
    const bool consistent = (c % 5) != 4; // 5 回に 1 回は、壁の両側が食い違う地図(ふつうは起きない)
    const bool open_edge = (c % 7) == 6;  // 7 回に 1 回は、外周の壁が抜けている地図(ふつうは起きない)
    auto lgc = std::make_shared<MazeSolverBaseLgc>();
    auto ego = std::make_shared<ego_t>();
    lgc->set_ego(ego);
    lgc->init(N, N * N - 1);
    std::vector<point_t> goals;
    const int gx = rnd(N - 3), gy = rnd(N - 3), gw = 1 + rnd(3), gh = 1 + rnd(3);
    for (int i = 0; i < gw; i++)
      for (int j = 0; j < gh; j++) {
        point_t p{};
        p.x = gx + i;
        p.y = gy + j;
        goals.push_back(p);
      }
    lgc->set_goal_pos(goals);
    lgc->set_default_wall_data();
    std::vector<unsigned char> m(N * N, 0);
    const auto put = [&](int x, int y, int bit, bool wall, bool known) {
      if (wall)
        m[x + y * N] |= bit;
      if (known)
        m[x + y * N] |= bit << 4;
    };
    for (int y = 0; y < N; y++)
      for (int x = 0; x < N; x++) {
        // 北と東の壁を決め、隣の区画の南・西へも同じものを入れる
        for (int k = 0; k < 2; k++) {
          const int nx = x + (k == 1), ny = y + (k == 0);
          const bool edge = nx >= N || ny >= N;
          const bool wall = (edge && !open_edge) || rnd(100) < wall_pct;
          const bool known = (edge && !open_edge) || rnd(100) < known_pct;
          put(x, y, k == 0 ? 1 : 2, wall, known);
          if (!edge) {
            const bool w2 = consistent ? wall : rnd(100) < wall_pct;
            const bool k2 = consistent ? known : rnd(100) < known_pct;
            put(nx, ny, k == 0 ? 8 : 4, w2, k2);
          }
        }
        if (x == 0)
          put(x, y, 4, !open_edge || rnd(100) < wall_pct, !open_edge || rnd(100) < known_pct);
        if (y == 0)
          put(x, y, 8, !open_edge || rnd(100) < wall_pct, !open_edge || rnd(100) < known_pct);
      }
    for (int i = 0; i < N * N; i++)
      lgc->map[i] = m[i];
    ego->x = rnd(N);
    ego->y = rnd(N);
    ego->dir = static_cast<Direction>(1 << rnd(4));
    std::unordered_map<unsigned int, unsigned char> sub;
    const int ns = rnd(40);
    for (int i = 0; i < ns; i++)
      sub[rnd(N * N)] = (unsigned char)rnd(50);
    if (c % 11 == 10)
      sub[N * N + rnd(100)] = (unsigned char)rnd(50); // 迷路の外の番号(ふつうは起きない)
    const int pn = 1 + rnd(5);
    const bool is_search = (c % 4) != 3;
    lgc->set_param_num(pn);
    lgc->set_param();

    g_hash = 2166136261u;
    // 表づくりを 3 回続ける(2 回目からは、サブゴールの期限切れが前の表を見る)
    int walked = 0;
    for (int k = 0; k < 3; k++) {
      hmix(lgc->updateVectorMap(is_search, sub));
      hash_sub(sub);
      hash_table(*lgc);
      if (walk_ends(*lgc)) {
        walked++;
        hmix(lgc->searchGoalPosition(is_search, sub));
        hash_sub(sub);
        hash_table(*lgc);
      }
      if (k == 1) { // 途中で壁が 1 枚分かった
        const int x = rnd(N - 1), y = rnd(N - 1);
        lgc->map[x + y * N] |= 0x11;
        lgc->map[x + (y + 1) * N] |= 0x88;
      }
    }
    hmix(lgc->updateVectorMap(is_search, sub));
    hash_sub(sub);
    hash_table(*lgc);
    hmix(lgc->clear_vector_distmap(sub));
    hash_sub(sub);
    hash_table(*lgc);
    hmix((uint32_t)lgc->vq_list.size());
    // 歩数マップ
    std::vector<point_t> pts;
    const int np = rnd(12);
    for (int i = 0; i < np; i++) {
      point_t p{};
      p.x = rnd(N);
      p.y = rnd(N);
      pts.push_back(p);
    }
    lgc->set_goal_pos2(pts);
    for (int mode = 0; mode < 2; mode++)
      for (int sm = 0; sm < 2; sm++)
        for (int reset = 0; reset < 2; reset++) {
          if (reset)
            lgc->reset_dist_map();
          lgc->update_dist_map(mode, sm);
          for (int i = 0; i < N * N; i++)
            hmix(lgc->dist[i]);
        }
    printf("%d %d %d %d %d %08x\n", c, N, consistent && !open_edge, is_search, walked, g_hash);
  }
  return 0;
}
