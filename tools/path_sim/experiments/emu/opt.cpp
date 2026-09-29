// 探索の表づくり(updateVectorMap / clear_vector_distmap)と歩数マップ(update_dist_map)を、
// 結果を変えずに速くした試作。やっていることは元と同じで、違うのは
//   ・区画の範囲確認(valid_map_list_idx)を、配列を読むたびではなく 1 か所にまとめた
//   ・向きごとの if の連鎖を、表引きにした
//   ・小さい関数の呼び出しをやめた(元は noinline が付いていて、毎回関数呼び出しになる)
//   ・サブゴールの期限切れを、全区画ではなくサブゴールの数だけ回す
// 取り出す順番(priority_queue)と書き込む値は元と同じ。
#include "stdhdr.hpp"
#define private public
#include "adachi.hpp" // 変更前の写し(frozen2)
#undef private

namespace {
// Direction(N=1 E=2 W=4 S=8)→ n / e / w / s の並び(0..3)
inline int di(int dir) { return dir == 1 ? 0 : dir == 2 ? 1 : dir == 4 ? 2 : 3; }
struct Nb {
  int8_t i, j;      // 進む先の区画
  uint8_t d2[3];    // 出ていく壁の向き(Direction の値)
  uint8_t vd[3];    // 進む向きの番号: 0 N / 1 NE / 2 E / 3 SE / 4 S / 5 SW / 6 W / 7 NW
};
// 元の updateVectorMap() の d[] / d2[] と同じ並び
const Nb NB[4] = {
    {0, 1, {1, 2, 4}, {0, 1, 7}},  // North: N, NE, NW / N, E, W
    {1, 0, {2, 8, 1}, {2, 3, 1}},  // East:  E, SE, NE / E, S, N
    {-1, 0, {4, 1, 8}, {6, 7, 5}}, // West:  W, NW, SW / W, N, S
    {0, -1, {8, 4, 2}, {4, 5, 3}}, // South: S, SW, SE / S, W, E
};
inline float &distv(vector_map_t &m, int k) { return (&m.n)[k]; }
inline unsigned get_cnt(const vector_map_t &m, int vd) {
  switch (vd) {
  case 0: return m.N1;
  case 1: return m.NE;
  case 2: return m.E1;
  case 3: return m.SE;
  case 4: return m.S1;
  case 5: return m.SW;
  case 6: return m.W1;
  default: return m.NW;
  }
}
inline void set_cnt(vector_map_t &m, int vd, unsigned v) {
  switch (vd) {
  case 0: m.N1 = v; break;
  case 1: m.NE = v; break;
  case 2: m.E1 = v; break;
  case 3: m.SE = v; break;
  case 4: m.S1 = v; break;
  case 5: m.SW = v; break;
  case 6: m.W1 = v; break;
  default: m.NW = v; break;
  }
}
} // namespace

static void opt_clear(MazeSolverBaseLgc &l, std::unordered_map<unsigned int, unsigned char> &subgoal_list) {
  const int N = l.maze_size;
  const float vmax = l.vector_max_step_val;
  // サブゴールの期限切れ(元は全区画を回って contains() していた)
  const unsigned char limit = N > 20 ? 45 : 25;
  for (auto it = subgoal_list.begin(); it != subgoal_list.end();) {
    const unsigned idx = it->first;
    bool erase = false;
    if (idx < (unsigned)(N * N)) {
      const auto &m = l.vector_dist[idx];
      if (m.n == vmax && m.e == vmax && m.w == vmax && m.s == vmax) {
        erase = true;
      } else {
        it->second++;
        if (it->second > limit)
          erase = true;
      }
    }
    it = erase ? subgoal_list.erase(it) : std::next(it);
  }
  vector_map_t z{};
  z.n = z.e = z.w = z.s = vmax;
  for (int idx = 0; idx < N * N; idx++) {
    l.vector_dist[idx] = z;
    l.updateMap[idx] = 0;
  }
  while (!l.vq_list.empty())
    l.vq_list.pop();
  for (const auto p : l.goal_list) {
    const unsigned char x = p.x, y = p.y;
    const int idx = x + y * N;
    const unsigned char w = l.map[idx];
    if (!(w & 1)) {
      l.vector_dist[idx].n = 0;
      l.vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::North, .dist2 = 0.0f});
      if (y < N - 1)
        l.vector_dist[idx + N].s = 0;
    }
    if (!(w & 2)) {
      l.vector_dist[idx].e = 0;
      l.vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::East, .dist2 = 0.0f});
      if (x < N - 1)
        l.vector_dist[idx + 1].w = 0;
    }
    if (!(w & 4)) {
      l.vector_dist[idx].w = 0;
      l.vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::West, .dist2 = 0.0f});
      if (x > 0)
        l.vector_dist[idx - 1].e = 0;
    }
    if (!(w & 8)) {
      l.vector_dist[idx].s = 0;
      l.vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::South, .dist2 = 0.0f});
      if (y > 0)
        l.vector_dist[idx - N].n = 0;
    }
  }
}

static void opt_update_vector_map(MazeSolverBaseLgc &l, const bool isSearch,
                                  std::unordered_map<unsigned int, unsigned char> &subgoal_list) {
  opt_clear(l, subgoal_list);
  const int N = l.maze_size;
  const unsigned bl1 = l.borderLv1, bl2 = l.borderLv2, bl1d = l.borderLv1d, bl2d = l.borderLv2d;
  const float St1 = l.St1, St2 = l.St2, St3 = l.St3, Dia = l.Dia, Dia2 = l.Dia2, Dia3 = l.Dia3;
  auto *vd = l.vector_dist.data();
  const unsigned char *map = l.map.data();
  unsigned char *upd = l.updateMap.data();
  bool has_sub = !subgoal_list.empty();
  while (!l.vq_list.empty()) {
    const auto now_pos = l.vq_list.top();
    l.vq_list.pop();
    const int X = now_pos.x, Y = now_pos.y;
    const int dir = static_cast<int>(now_pos.dir);
    const int idx = X + Y * N;
    if (has_sub && (map[idx] & 0xf0) == 0xf0) {
      subgoal_list.erase(idx);
      has_sub = !subgoal_list.empty();
    }
    const Nb &nb = NB[di(dir)];
    const float now = distv(vd[idx], di(dir));
    const int nx = X + nb.i, ny = Y + nb.j;
    if (nx < 0 || ny < 0 || nx >= N || ny >= N)
      continue; // 範囲の外は元も「壁」扱いで何もしない
    const int nidx = nx + ny * N;
    const unsigned char w = map[nidx];
    for (int k = 0; k < 3; k++) {
      const int d2 = nb.d2[k];
      if ((w & d2) || !(isSearch || (w & (d2 << 4))))
        continue;
      const int v = nb.vd[k];
      const unsigned cnt = get_cnt(vd[idx], v);
      float tmp = now;
      if (dir == d2) {
        if (cnt > bl2)
          tmp += St3;
        else if (cnt > bl1)
          tmp += St2;
        else
          tmp += St1;
      } else {
        // 元の haveVectorLv(): 直進の向き(N / E / S / W)は borderLv、斜めは borderLv?d
        const bool diag = v & 1;
        const unsigned lv = cnt > (diag ? bl2d : bl2) ? 2 : cnt > (diag ? bl1d : bl1) ? 1 : 0;
        if (lv == 2)
          tmp += Dia3;
        else if (lv == 1)
          tmp += Dia2;
        else
          tmp += Dia;
      }
      const int k2 = di(d2);
      if (tmp <= distv(vd[nidx], k2)) {
        if (!(upd[nidx] & d2)) {
          // setDistV(): 壁の両側へ書く
          distv(vd[nidx], k2) = tmp;
          if (d2 == 1) {
            if (ny < N - 1)
              vd[nidx + N].s = tmp;
          } else if (d2 == 2) {
            if (nx < N - 1)
              vd[nidx + 1].w = tmp;
          } else if (d2 == 4) {
            if (nx > 0)
              vd[nidx - 1].e = tmp;
          } else {
            if (ny > 0)
              vd[nidx - N].n = tmp;
          }
          l.vq_list.push(dir_pt_t{.x = (unsigned char)nx, .y = (unsigned char)ny, .dir = static_cast<Direction>(d2), .dist2 = tmp});
          upd[nidx] |= d2;
        }
      }
      // addVector(): 15 未満なら +1 して、進む先の区画へ書く
      set_cnt(vd[nidx], v, cnt < 15 ? cnt + 1 : cnt);
    }
  }
}

// searchGoalPosition(isSearch = true) と同じ。表づくりだけ速い版に差し替え、経路のたどり方は元の関数を呼ぶ
void opt_search_goal_position(MazeSolverBaseLgc &l, std::unordered_map<unsigned int, unsigned char> &subgoal_list) {
  Direction next_dir = Direction::North;
  Direction now_dir = Direction::North;
  int x = 0, y = 1;
  Direction dirLog[3] = {now_dir, now_dir, now_dir};
  const int N = l.maze_size;
  opt_update_vector_map(l, true, subgoal_list);
  while (true) {
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;
    l.Value = l.vector_max_step_val;
    next_dir = Direction::Undefined;
    if (l.arrival_goal_position(x, y))
      break;
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
    if (next_dir == Direction::North) {
      if (l.is_unknown(x, y, Direction::North))
        subgoal_list[x + (y + 1) * N] = 1;
    } else if (next_dir == Direction::East) {
      if (l.is_unknown(x, y, Direction::East))
        subgoal_list[x + 1 + y * N] = 1;
    } else if (next_dir == Direction::West) {
      if (l.is_unknown(x, y, Direction::West))
        subgoal_list[x - 1 + y * N] = 1;
    } else if (next_dir == Direction::South) {
      if (l.is_unknown(x, y, Direction::South))
        subgoal_list[x + (y - 1) * N] = 1;
    }
    if (next_dir == Direction::North)
      y++;
    else if (next_dir == Direction::East)
      x++;
    else if (next_dir == Direction::West)
      x--;
    else if (next_dir == Direction::South)
      y--;
    if (next_dir == Direction::Undefined)
      break;
  }
}

// update_dist_map() と同じ(幅優先の歩数マップ)
void opt_update_dist_map(MazeSolverBaseLgc &l, const int mode, const bool search_mode) {
  const int N = l.maze_size;
  const unsigned maxv = l.max_step_val;
  unsigned *dist = l.dist.data();
  const unsigned char *map = l.map.data();
  if (!l.reset_done) {
    for (int c = 0; c < N * N; c++)
      dist[c] = maxv;
  }
  l.reset_done = false;
  int head = 0, tail = 0;
  auto seed = [&](const std::vector<point_t> &list) {
    for (const auto g : list) {
      dist[g.x + g.y * N] = 0;
      l.q_list[tail].x = g.x;
      l.q_list[tail].y = g.y;
      tail++;
    }
  };
  if (!search_mode) {
    seed(l.goal_list);
  } else {
    seed(l.goal_list2);
    seed(l.goal_list3);
  }
  static const int8_t DI[4] = {0, 1, -1, 0}, DJ[4] = {1, 0, 0, -1}; // N E W S(direction_list の順)
  static const uint8_t DB[4] = {1, 2, 4, 8};
  const int ex = l.ego->x, ey = l.ego->y;
  while (head != tail) {
    const int Y = l.q_list[head].y;
    const int X = l.q_list[head].x;
    head++;
    const int idx = X + Y * N;
    const unsigned pt1 = dist[idx] + 1;
    const unsigned char w = map[idx];
    for (int k = 0; k < 4; k++) {
      const bool b = mode == 1 ? ((w & DB[k]) == 0 && (w & (DB[k] << 4)) != 0) : (w & DB[k]) == 0;
      if (!b)
        continue;
      const int nx = X + DI[k], ny = Y + DJ[k];
      if (nx < 0 || ny < 0 || nx >= N || ny >= N)
        continue;
      const int nidx = nx + ny * N;
      if (dist[nidx] == maxv) {
        dist[nidx] = pt1;
        l.q_list[tail].x = nx;
        l.q_list[tail].y = ny;
        tail++;
      }
    }
    if (search_mode && ex == X && ey == Y)
      break;
  }
}
