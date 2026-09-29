
#include "logic.hpp"
int g_n_reuse = 0, g_n_rebuild = 0;
#include <algorithm>

// ---- 探索の表づくり(updateVectorMap)と歩数マップ(update_dist_map)用の表引き(2026-09-29)
namespace {
// Direction(N=1 E=2 W=4 S=8)→ vector_map_t の n / e / w / s の並び(0..3)
inline int dir_index(int dir) { return dir == 1 ? 0 : dir == 2 ? 1 : dir == 4 ? 2 : 3; }
struct vector_step_t {
  int8_t i, j;       // 進む先の区画
  uint8_t exit[3];   // 出ていく壁の向き(Direction の値)。0 番目が直進
  uint8_t vec[3];    // 進む向きの番号: 0 N / 1 NE / 2 E / 3 SE / 4 S / 5 SW / 6 W / 7 NW
};
// 並びは dir_index() の順。中身は元の updateVectorMap() の d[] / d2[] と同じ
constexpr vector_step_t VECTOR_STEP[4] = {
    {0, 1, {1, 2, 4}, {0, 1, 7}},  // North: N, NE, NW / N, E, W
    {1, 0, {2, 8, 1}, {2, 3, 1}},  // East:  E, SE, NE / E, S, N
    {-1, 0, {4, 1, 8}, {6, 7, 5}}, // West:  W, NW, SW / W, N, S
    {0, -1, {8, 4, 2}, {4, 5, 3}}, // South: S, SW, SE / S, W, E
};
inline float &dist_of(vector_map_t &m, int k) {
  switch (k) {
  case 0: return m.n;
  case 1: return m.e;
  case 2: return m.w;
  default: return m.s;
  }
}
inline unsigned vector_count(const vector_map_t &m, int vec) {
  switch (vec) {
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
inline void set_vector_count(vector_map_t &m, int vec, unsigned v) {
  switch (vec) {
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


void MazeSolverBaseLgc::init(const int _maze_size, const int _max_step_val) {
  maze_size = _maze_size;
  max_step_val = _max_step_val;
  maze_list_size = maze_size * maze_size;

  map.resize(maze_list_size);
  dist.resize(maze_list_size);
  vector_dist.resize(maze_list_size);
  updateMap.resize(maze_list_size);
  q_list.resize(maze_list_size + 1);
  while (!vq_list.empty()) {
    vq_list.pop();
  }
  goal_list3.clear();
  search_table.valid = false;
}

void MazeSolverBaseLgc::data_economize() {
  map.clear();
  dist.clear();
  vector_dist.clear();
  updateMap.clear();
  q_list.clear();
  // vq_list.clear();
  while (!vq_list.empty()) {
    vq_list.pop();
  }
  goal_list3.clear();
}

void MazeSolverBaseLgc::set_ego(std::shared_ptr<ego_t> &_ego) { ego = _ego; }

void MazeSolverBaseLgc::set_goal_pos(const vector<point_t> &list) {
  goal_list.clear();
  goal_list_origin.clear();
  goal_list.reserve(list.size());
  goal_list_origin.reserve(list.size());
  for (const auto p : list) {
    goal_list.emplace_back(p);
    goal_list_origin.emplace_back(p);
  }
}

void MazeSolverBaseLgc::set_goal_pos2(const vector<point_t> &pt_list) {
  goal_list2.clear();
  goal_list2.reserve(pt_list.size());
  for (const auto p : pt_list)
    goal_list2.emplace_back(p);
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::valid_map_list_idx(const int x, const int y) {
  if (x < 0 || x >= maze_size || y < 0 || y >= maze_size)
    return false;

  if ((x + y * maze_size) >= maze_list_size)
    return false;

  return true;
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::candidate_end(const int x, const int y) {
  unsigned char temp = map[x + y * maze_size] & 0x0f;
  return (temp == 0x0e || temp == 0x0d || temp == 0x0b || temp == 0x07 ||
          temp == 0x0f);
}

void MazeSolverBaseLgc::updateWall(int x, int y, Direction dir) {
  if (dir == Direction::North) {
    set_wall_data(x, y, Direction::North, true);
    set_wall_data(x, y + 1, Direction::South, true);
  } else if (dir == Direction::East) {
    set_wall_data(x, y, Direction::East, true);
    set_wall_data(x + 1, y, Direction::West, true);
  } else if (dir == Direction::West) {
    set_wall_data(x, y, Direction::West, true);
    set_wall_data(x - 1, y, Direction::East, true);
  } else if (dir == Direction::South) {
    set_wall_data(x, y, Direction::South, true);
    set_wall_data(x, y - 1, Direction::North, true);
  }
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::remove_goal_pos3() {
  std::erase_if(goal_list3, [this](const point_t &p) {
    return is_stepped(p.x, p.y) || get_dist_val(p.x, p.y) == max_step_val;
  });
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::reset_dist_map() {
  int c = 0;
  for (int i = 0; i < maze_size; i++)
    for (int j = 0; j < maze_size; j++)
      dist[c++] = max_step_val;
  reset_done = true;
}

// 歩数マップ(幅優先)。2026-09-29: 結果はそのままで、区画の範囲確認(valid_map_list_idx)と
// 小さい関数の呼び出しを 1 区画 1 回にまとめた(機体向けビルドの命令数で約 1/6)。
__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::update_dist_map(const int mode,
                                        const bool search_mode) {
  const int N = maze_size;
  if (!reset_done) {
    for (int c = 0; c < N * N; c++)
      dist[c] = max_step_val;
  }
  reset_done = false;

  int head = 0;
  int tail = 0;
  const auto seed = [&](const vector<point_t> &list) {
    for (const auto g : list) {
      dist[g.x + g.y * N] = 0;
      q_list[tail].x = g.x;
      q_list[tail].y = g.y;
      tail++;
    }
  };
  if (!search_mode) {
    seed(goal_list);
  } else {
    seed(goal_list2);
    seed(goal_list3);
  }
  // direction_list と同じ順(North, East, West, South)
  static constexpr int8_t DI[4] = {0, 1, -1, 0};
  static constexpr int8_t DJ[4] = {1, 0, 0, -1};
  static constexpr uint8_t DB[4] = {1, 2, 4, 8};
  while (head != tail) {
    const int Y = q_list[head].y;
    const int X = q_list[head].x;
    head++;
    const int idx = X + Y * N;
    const unsigned int pt1 = dist[idx] + 1;
    const unsigned char w = map[idx];
    for (int k = 0; k < 4; k++) {
      // mode 1: 壁が無く、かつ既知(isProceed)。それ以外: 壁が無い(!existWall)
      const bool b = (mode == 1)
                         ? ((w & DB[k]) == 0 && (w & (DB[k] << 4)) != 0)
                         : ((w & DB[k]) == 0);
      if (!b)
        continue;
      const int nx = X + DI[k];
      const int ny = Y + DJ[k];
      if (nx < 0 || nx >= N || ny < 0 || ny >= N)
        continue;
      const int nidx = nx + ny * N;
      if (dist[nidx] == max_step_val) {
        dist[nidx] = pt1;
        q_list[tail].x = nx;
        q_list[tail].y = ny;
        tail++;
      }
    }
    if (search_mode) {
      if (ego->x == X && ego->y == Y) {
        break;
      }
    }
  }
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::set_dist_val(const int x, const int y, const int val) {
  if (valid_map_list_idx(x, y))
    dist[x + y * maze_size] = val;
}

__attribute__((noinline, section(".time_critical.search")))
int MazeSolverBaseLgc::get_dist_val(int x, int y) {
  if (valid_map_list_idx(x, y))
    return dist[x + y * maze_size];
  else
    return max_step_val;
}

__attribute__((noinline, section(".time_critical.search")))
float MazeSolverBaseLgc::get_diadist_n_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return vector_dist[x + y * maze_size].n;
  else
    return vector_max_step_val;
}

__attribute__((noinline, section(".time_critical.search")))
float MazeSolverBaseLgc::get_diadist_e_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return vector_dist[x + y * maze_size].e;
  else
    return vector_max_step_val;
}

__attribute__((noinline, section(".time_critical.search")))
int MazeSolverBaseLgc::get_map_val(const int x, const int y) {
  if (valid_map_list_idx(x, y))
    return map[x + y * maze_size];
  else
    return 0xff;
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::isProceed(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y))
    return ((get_map_val(x, y) / static_cast<int>(dir)) & 0x11) == 0x10;
  else
    return false;
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::existWall(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y))
    return ((get_map_val(x, y) / static_cast<int>(dir)) & 0x01) == 0x01;
  else
    return true;
}

void MazeSolverBaseLgc::set_map_val(const int x, const int y, const int val) {
  if (valid_map_list_idx(x, y))
    map[x + y * maze_size] = val;
}
void MazeSolverBaseLgc::set_map_val(int idx, int val) { map[idx] = val; }

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::set_wall_data(const int x, const int y, Direction dir,
                                      const bool isWall) {
  if (valid_map_list_idx(x, y)) {
    int idx = x + y * maze_size;
    map[idx] |= (0x10 * static_cast<int>(dir));
    if (isWall)
      map[idx] |= 0x01 * static_cast<int>(dir);
    else
      map[idx] = (map[idx] & 0xf0) |
                 (map[idx] & (~(0x01 * static_cast<int>(dir)) & 0x0f));
  }
}
void MazeSolverBaseLgc::set_native_wall_data(const int idx,
                                             const uint8_t data) {
  if (idx >= maze_list_size)
    return;
  map[idx] = data;
}

void MazeSolverBaseLgc::set_default_wall_data() {
  for (int i = 0; i < maze_size; i++) {
    // set_map
    set_wall_data(i, maze_size - 1, Direction::North, true);
    set_wall_data(maze_size - 1, i, Direction::East, true);
    set_wall_data(0, i, Direction::West, true);
    set_wall_data(i, 0, Direction::South, true);
  }
  set_wall_data(0, 0, Direction::East, true);
  set_wall_data(0, 0, Direction::North, false);
  set_wall_data(1, 0, Direction::West, true);
  set_wall_data(0, 1, Direction::South, false);
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::isStep(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y))
    return ((map[x + y * maze_size] / static_cast<int>(dir)) & 0x10) == 0x10;
  return false;
}

void MazeSolverBaseLgc::back_home() {
  clear_goal();
  goal_list.emplace_back(point_t{.x = 0, .y = 0});
}

void MazeSolverBaseLgc::clear_goal() {
  goal_list.clear();
  // goal_list.shrink_to_fit();
}

void MazeSolverBaseLgc::append_goal(const int x, const int y) {
  goal_list.emplace_back(point_t{.x = 0, .y = 0});
}

int MazeSolverBaseLgc::get_max_step_val() { return max_step_val; }

__attribute__((noinline, section(".time_critical.search")))
int MazeSolverBaseLgc::clear_vector_distmap() {
  int tail = 0;
  vector_map_serial++;
  for (char i = 0; i < maze_size; i++) {
    for (char j = 0; j < maze_size; j++) {
      int idx = i + j * maze_size;
      vector_dist[idx].n = vector_max_step_val;
      vector_dist[idx].e = vector_max_step_val;
      vector_dist[idx].w = vector_max_step_val;
      vector_dist[idx].s = vector_max_step_val;
      vector_dist[idx].v = 0;
      vector_dist[idx].N1 = 0;
      vector_dist[idx].NE = 0;
      vector_dist[idx].E1 = 0;
      vector_dist[idx].SE = 0;
      vector_dist[idx].S1 = 0;
      vector_dist[idx].SW = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].NW = 0;
      vector_dist[idx].W1 = 0;
      vector_dist[idx].step = 0;
      updateMap[idx] = 0;
    }
  }
  while (!vq_list.empty()) {
    vq_list.pop();
  }
  for (const auto p : goal_list) {
    unsigned char x = p.x;
    unsigned char y = p.y;
    int idx = x + y * maze_size;
    if (!existWall(x, y, Direction::North)) {
      vector_dist[idx].n = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::North, .dist2 = 0.0f});
      tail++;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = 0;
    }
    if (!existWall(x, y, Direction::East)) {
      vector_dist[idx].e = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::East, .dist2 = 0.0f});
      tail++;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = 0;
    }
    if (!existWall(x, y, Direction::West)) {
      vector_dist[idx].w = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::West, .dist2 = 0.0f});
      tail++;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = 0;
    }
    if (!existWall(x, y, Direction::South)) {
      vector_dist[idx].s = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::South, .dist2 = 0.0f});
      tail++;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = 0;
    }
  }

  return tail;
}

// 2026-09-29: 結果はそのままで、サブゴールの期限切れを「全区画を回って contains()」から
// 「サブゴールの数だけ回す」に変え、表の初期化と種まきの範囲確認・関数呼び出しを減らした。
__attribute__((noinline, section(".time_critical.search")))
int MazeSolverBaseLgc::clear_vector_distmap(
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  int tail = 0;
  vector_map_serial++;
  age_subgoal(subgoal_list);

  const int N = maze_size;
  vector_map_t clear{};
  clear.n = clear.e = clear.w = clear.s = vector_max_step_val;
  for (int idx = 0; idx < N * N; idx++) {
    vector_dist[idx] = clear;
    updateMap[idx] = 0;
  }
  while (!vq_list.empty()) {
    vq_list.pop();
  }

  for (const auto p : goal_list) {
    const unsigned char x = p.x;
    const unsigned char y = p.y;
    const int idx = x + y * N;
    const unsigned char w = map[idx];
    if (!(w & 0x01)) {
      vector_dist[idx].n = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::North, .dist2 = 0.0f});
      tail++;
      if (y < N - 1)
        vector_dist[idx + N].s = 0;
    }
    if (!(w & 0x02)) {
      vector_dist[idx].e = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::East, .dist2 = 0.0f});
      tail++;
      if (x < N - 1)
        vector_dist[idx + 1].w = 0;
    }
    if (!(w & 0x04)) {
      vector_dist[idx].w = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::West, .dist2 = 0.0f});
      tail++;
      if (x > 0)
        vector_dist[idx - 1].e = 0;
    }
    if (!(w & 0x08)) {
      vector_dist[idx].s = 0;
      vq_list.push(dir_pt_t{.x = x, .y = y, .dir = Direction::South, .dist2 = 0.0f});
      tail++;
      if (y > 0)
        vector_dist[idx - N].n = 0;
    }
  }

  return tail;
}

// サブゴールの期限切れ。前に作った表(vector_dist)で、どの壁にも値が入らなかった区画(行けない区画)は
// 外す。残りは 1 つ歳を取らせ、上限を超えたら外す。clear_vector_distmap() が表を消す前に呼ぶ。
__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::age_subgoal(
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  const float vmax = vector_max_step_val;
  const unsigned char limit = (maze_size > 20) ? 45 : 25;
  for (auto it = subgoal_list.begin(); it != subgoal_list.end();) {
    const unsigned int idx = it->first;
    bool erase = false;
    if (idx < maze_list_size) {
      const auto &m = vector_dist[idx];
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
}

__attribute__((noinline, section(".time_critical.search")))
int MazeSolverBaseLgc::haveVectorLv(const int x, const int y, Direction dir) {
  const int idx = x + y * maze_size;
  if (dir == Direction::North) {
    if (vector_dist[idx].N1 > borderLv2)
      return 2;
    else if (vector_dist[idx].N1 > borderLv1)
      return 1;
  } else if (dir == Direction::NorthEast) {
    if (vector_dist[idx].NE > borderLv2d)
      return 2;
    else if (vector_dist[idx].NE > borderLv1d)
      return 1;
  } else if (dir == Direction::East) {
    if (vector_dist[idx].E1 > borderLv2)
      return 2;
    else if (vector_dist[idx].E1 > borderLv1)
      return 1;
  } else if (dir == Direction::SouthEast) {
    if (vector_dist[idx].SE > borderLv2d)
      return 2;
    else if (vector_dist[idx].SE > borderLv1d)
      return 1;
  } else if (dir == Direction::South) {
    if (vector_dist[idx].S1 > borderLv2)
      return 2;
    else if (vector_dist[idx].S1 > borderLv1)
      return 1;
  } else if (dir == Direction::SouthWest) {
    if (vector_dist[idx].SW > borderLv2d)
      return 2;
    else if (vector_dist[idx].SW > borderLv1d)
      return 1;
  } else if (dir == Direction::West) {
    if (vector_dist[idx].W1 > borderLv2)
      return 2;
    else if (vector_dist[idx].W1 > borderLv1)
      return 1;
  } else if (dir == Direction::NorthWest) {
    if (vector_dist[idx].NW > borderLv2d)
      return 2;
    else if (vector_dist[idx].NW > borderLv1d)
      return 1;
  }
  return 0;
}

__attribute__((noinline, section(".time_critical.search")))
float MazeSolverBaseLgc::getDistV(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    if (dir == Direction::North)
      return vector_dist[x + y * maze_size].n;
    else if (dir == Direction::East)
      return vector_dist[x + y * maze_size].e;
    else if (dir == Direction::West)
      return vector_dist[x + y * maze_size].w;
    else if (dir == Direction::South)
      return vector_dist[x + y * maze_size].s;
  }
  return vector_max_step_val;
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::setDistV(const int x, const int y, Direction dir,
                                 const float val) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == Direction::North) {
      vector_dist[idx].n = val;
      if (y < maze_size - 1)
        vector_dist[(x) + (y + 1) * maze_size].s = val;
    } else if (dir == Direction::East) {
      vector_dist[idx].e = val;
      if (x < maze_size - 1)
        vector_dist[(x + 1) + (y)*maze_size].w = val;
    } else if (dir == Direction::West) {
      vector_dist[idx].w = val;
      if (x > 0)
        vector_dist[(x - 1) + (y)*maze_size].e = val;
    } else if (dir == Direction::South) {
      vector_dist[idx].s = val;
      if (y > 0)
        vector_dist[(x) + (y - 1) * maze_size].n = val;
    }
  }
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::isUpdated(const int x, const int y, Direction dir) {
  if (!valid_map_list_idx(x, y))
    return true;

  const int idx = x + y * maze_size;
  if (dir == Direction::North)
    return (updateMap[idx] & 0x01) == 0x01;
  else if (dir == Direction::East)
    return (updateMap[idx] & 0x02) == 0x02;
  else if (dir == Direction::West)
    return (updateMap[idx] & 0x04) == 0x04;
  else if (dir == Direction::South)
    return (updateMap[idx] & 0x08) == 0x08;
  else if (dir == Direction::NorthEast)
    return (updateMap[idx] & 0x10) == 0x10;
  else if (dir == Direction::SouthEast)
    return (updateMap[idx] & 0x20) == 0x20;
  else if (dir == Direction::SouthWest)
    return (updateMap[idx] & 0x40) == 0x40;
  else if (dir == Direction::NorthWest)
    return (updateMap[idx] & 0x80) == 0x80;

  return false;
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::addVector(const int x, const int y, Direction dir,
                                  float val) {
  if (valid_map_list_idx(x, y)) {
    if (val < 15)
      val++;
    const int idx = x + y * maze_size;
    if (dir == Direction::North)
      vector_dist[idx].N1 = val;
    else if (dir == Direction::NorthEast)
      vector_dist[idx].NE = val;
    else if (dir == Direction::East)
      vector_dist[idx].E1 = val;
    else if (dir == Direction::SouthEast)
      vector_dist[idx].SE = val;
    else if (dir == Direction::South)
      vector_dist[idx].S1 = val;
    else if (dir == Direction::SouthWest)
      vector_dist[idx].SW = val;
    else if (dir == Direction::West)
      vector_dist[idx].W1 = val;
    else if (dir == Direction::NorthWest)
      vector_dist[idx].NW = val;
  }
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::updateMapCheck(const int x, const int y,
                                       Direction dir) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == Direction::North)
      updateMap[idx] |= 0x01;
    else if (dir == Direction::East)
      updateMap[idx] |= 0x02;
    else if (dir == Direction::West)
      updateMap[idx] |= 0x04;
    else if (dir == Direction::South)
      updateMap[idx] |= 0x08;
    else if (dir == Direction::NorthEast)
      updateMap[idx] |= 0x10;
    else if (dir == Direction::SouthEast)
      updateMap[idx] |= 0x20;
    else if (dir == Direction::SouthWest)
      updateMap[idx] |= 0x40;
    else if (dir == Direction::NorthWest)
      updateMap[idx] |= 0x80;
  }
}

__attribute__((noinline, section(".time_critical.search")))
int MazeSolverBaseLgc::getVector(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    const int idx = x + y * maze_size;
    if (dir == Direction::North)
      return vector_dist[idx].N1;
    else if (dir == Direction::NorthEast)
      return vector_dist[idx].NE;
    else if (dir == Direction::East)
      return vector_dist[idx].E1;
    else if (dir == Direction::SouthEast)
      return vector_dist[idx].SE;
    else if (dir == Direction::South)
      return vector_dist[idx].S1;
    else if (dir == Direction::SouthWest)
      return vector_dist[idx].SW;
    else if (dir == Direction::West)
      return vector_dist[idx].W1;
    else if (dir == Direction::NorthWest)
      return vector_dist[idx].NW;
  }
  return 0;
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::is_unknown(const int x, const int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    // return (map[x + y * maze_size] & 0xf0) != 0xf0;
    if (dir == Direction::North)
      return (map[x + y * maze_size] & 0x10) == 0x00;
    else if (dir == Direction::East)
      return (map[x + y * maze_size] & 0x20) == 0x00;
    else if (dir == Direction::West)
      return (map[x + y * maze_size] & 0x40) == 0x00;
    else if (dir == Direction::South)
      return (map[x + y * maze_size] & 0x80) == 0x00;
  }
  return false;
}

void MazeSolverBaseLgc::simplesort(const int tail) {}

__attribute__((noinline, section(".time_critical.search")))
unsigned int MazeSolverBaseLgc::updateVectorMap(const bool isSearch) {
  unsigned int head = 0;
  unsigned int tail = clear_vector_distmap();
  unsigned int c = 0;

  while (!vq_list.empty()) {
    const auto now_pos = vq_list.top();
    vq_list.pop();
    int X = now_pos.x;
    int Y = now_pos.y;
    Direction dir = now_pos.dir;
    int i = 0;
    int j = 0;
    Direction d[3] = {Direction::North, Direction::North, Direction::North};
    Direction d2[3] = {Direction::North, Direction::North, Direction::North};
    float now = getDistV(X, Y, dir);
    if (dir == Direction::North) {
      j = 1;
      d[0] = Direction::North;     // N
      d[1] = Direction::NorthEast; // NE
      d[2] = Direction::NorthWest; // NW
      d2[0] = Direction::North;    // N
      d2[1] = Direction::East;     // E
      d2[2] = Direction::West;     // W
    } else if (dir == Direction::East) {
      i = 1;
      d[0] = Direction::East;      // E
      d[1] = Direction::SouthEast; // SE
      d[2] = Direction::NorthEast; // NE
      d2[0] = Direction::East;     // E
      d2[1] = Direction::South;    // S
      d2[2] = Direction::North;    // N
    } else if (dir == Direction::West) {
      i = -1;
      d[0] = Direction::West;      // W
      d[1] = Direction::NorthWest; // NW
      d[2] = Direction::SouthWest; // SW
      d2[0] = Direction::West;     // W
      d2[1] = Direction::North;    // N
      d2[2] = Direction::South;    // S
    } else if (dir == Direction::South) {
      j = -1;
      d[0] = Direction::South;     // S
      d[1] = Direction::SouthWest; // SW
      d[2] = Direction::SouthEast; // SE
      d2[0] = Direction::South;    // S
      d2[1] = Direction::West;     // W
      d2[2] = Direction::East;     // E
    }
    // c++;
    for (int k = 0; k < 3; k++) {
      c++;

      if (!existWall(X + i, Y + j, d2[k]) &&
          (isSearch || isStep(X + i, Y + j, d2[k]))) {
        int v = haveVectorLv(X, Y, d[k]);
        float tmp = now;
        if (dir == d2[k]) {
          if (v >= 2) {
            tmp += St3;
          } else if (v == 1) {
            tmp += St2;
          } else {
            tmp += St1;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list.push(dir_pt_t{.x = (unsigned char)(X + i), .y = (unsigned char)(Y + j), .dir = d2[k], .dist2 = tmp});
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        } else {
          if (v == 2) {
            tmp += Dia3;
          } else if (v == 1) {
            tmp += Dia2;
          } else {
            tmp += Dia;
          }
          if (tmp <= getDistV(X + i, Y + j, d2[k])) {
            if (!isUpdated(X + i, Y + j, d2[k])) {
              setDistV(X + i, Y + j, d2[k], tmp);
              vq_list.push(dir_pt_t{.x = (unsigned char)(X + i), .y = (unsigned char)(Y + j), .dir = d2[k], .dist2 = tmp});
              simplesort(tail);
              tail++;
              updateMapCheck(X + i, Y + j, d2[k]);
            }
          }
          addVector(X + i, Y + j, d[k], getVector(X, Y, d[k]));
        }
      }
    }
    head++;
  }
  return c;
}

// 探索のサブゴール選び用の表づくり。
// 2026-09-29: 結果(vector_dist / updateMap / サブゴール)はそのままで速くした。取り出す順番
// (vq_list)・書き込む値・書き込む順番は元と同じ。変えたのは、
//   ・区画の範囲確認と壁・既知の確認を、進む先の区画ごとに 1 回にまとめた
//     (元は getDistV / existWall / isUpdated … がそれぞれ valid_map_list_idx を呼び、1 回の表づくりで
//      約 2.8 万回になっていた)
//   ・向きごとの if の連鎖を表引き(VECTOR_STEP)にした
//   ・小さい関数(どれも noinline)の呼び出しをやめた
// 機体向けビルドの命令数で 137 万 → 59 万(tools/path_sim/experiments/README.md の実験 6)。
// 確認は tools/path_sim/check_search.py(探索 1 本まるごとの結果が基準と同じか)。
__attribute__((noinline, section(".time_critical.search")))
unsigned int MazeSolverBaseLgc::updateVectorMap(
    const bool isSearch,
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  clear_vector_distmap(subgoal_list);
  unsigned int c = 0;
  const int N = maze_size;
  bool has_subgoal = !subgoal_list.empty();

  while (!vq_list.empty()) {
    const auto now_pos = vq_list.top();
    vq_list.pop();
    const int X = now_pos.x;
    const int Y = now_pos.y;
    const int dir = static_cast<int>(now_pos.dir);
    const int idx = X + Y * N;

    if (has_subgoal && (map[idx] & 0xf0) == 0xf0) {
      subgoal_list.erase(idx);
      has_subgoal = !subgoal_list.empty();
    }

    const vector_step_t &st = VECTOR_STEP[dir_index(dir)];
    const float now = dist_of(vector_dist[idx], dir_index(dir));
    c += 3;
    const int nx = X + st.i;
    const int ny = Y + st.j;
    if (nx < 0 || nx >= N || ny < 0 || ny >= N)
      continue; // 迷路の外(元も existWall() が「壁」を返して何もしない)
    const int nidx = nx + ny * N;
    const unsigned char w = map[nidx];

    for (int k = 0; k < 3; k++) {
      const int exit = st.exit[k];
      // 壁がある、または(探索でないとき)未知の壁は通らない
      if ((w & exit) || !(isSearch || (w & (exit << 4))))
        continue;
      const int vec = st.vec[k];
      const unsigned int cnt = vector_count(vector_dist[idx], vec);
      float tmp = now;
      if (dir == exit) {
        if (cnt > borderLv2)
          tmp += St3;
        else if (cnt > borderLv1)
          tmp += St2;
        else
          tmp += St1;
      } else {
        if (cnt > borderLv2d)
          tmp += Dia3;
        else if (cnt > borderLv1d)
          tmp += Dia2;
        else
          tmp += Dia;
      }
      const int k2 = dir_index(exit);
      if (tmp <= dist_of(vector_dist[nidx], k2)) {
        if (!(updateMap[nidx] & exit)) {
          // setDistV(): 壁の両側の区画へ書く
          dist_of(vector_dist[nidx], k2) = tmp;
          if (exit == 1) {
            if (ny < N - 1)
              vector_dist[nidx + N].s = tmp;
          } else if (exit == 2) {
            if (nx < N - 1)
              vector_dist[nidx + 1].w = tmp;
          } else if (exit == 4) {
            if (nx > 0)
              vector_dist[nidx - 1].e = tmp;
          } else {
            if (ny > 0)
              vector_dist[nidx - N].n = tmp;
          }
          vq_list.push(dir_pt_t{.x = (unsigned char)nx, .y = (unsigned char)ny, .dir = static_cast<Direction>(exit), .dist2 = tmp});
          updateMap[nidx] |= exit;
        }
      }
      // addVector(): 続けて進んだ回数(15 まで)を、進む先の区画へ書く
      set_vector_count(vector_dist[nidx], vec, cnt < 15 ? cnt + 1 : cnt);
    }
  }
  return c;
}

void MazeSolverBaseLgc::step_cell(int x, int y, Direction d) {
  if (valid_map_list_idx(x, y)) {
    if (d == Direction::North)
      map[x + y * maze_size] |= 0x10;
    else if (d == Direction::East)
      map[x + y * maze_size] |= 0x20;
    else if (d == Direction::West)
      map[x + y * maze_size] |= 0x40;
    else if (d == Direction::South)
      map[x + y * maze_size] |= 0x80;
  }
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::is_stepped(int x, int y) {
  if (valid_map_list_idx(x, y))
    return ((map[x + y * maze_size]) & 0xf0) == 0xf0;
  return false;
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::is_front_cell_stepped(int x, int y, Direction dir) {
  if (valid_map_list_idx(x, y)) {
    if (dir == Direction::North) {
      if (valid_map_list_idx(x, y + 1)) {
        return ((map[x + (y + 1) * maze_size]) & 0xf0) == 0xf0;
      }
    } else if (dir == Direction::East) {
      if (valid_map_list_idx(x + 1, y)) {
        return ((map[(x + 1) + y * maze_size]) & 0xf0) == 0xf0;
      }
    } else if (dir == Direction::West) {
      if (valid_map_list_idx(x - 1, y)) {
        return ((map[(x - 1) + y * maze_size]) & 0xf0) == 0xf0;
      }
    } else if (dir == Direction::South) {
      if (valid_map_list_idx(x, y - 1)) {
        return ((map[x + (y - 1) * maze_size]) & 0xf0) == 0xf0;
      }
    }
  }
  return false;
}

__attribute__((noinline, section(".time_critical.search")))
float MazeSolverBaseLgc::getDistVector(const int x, const int y,
                                       Direction dir) {
  if (valid_map_list_idx(x, y)) {
    if (existWall(x, y, dir))
      return VectorMax;
    if (dir == Direction::North)
      return vector_dist[x + y * maze_size].n;
    else if (dir == Direction::East)
      return vector_dist[x + y * maze_size].e;
    else if (dir == Direction::West)
      return vector_dist[x + y * maze_size].w;
    else if (dir == Direction::South)
      return vector_dist[x + y * maze_size].s;
  }
  return VectorMax;
}

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::setNextRootDirectionPathUnKnown(
    int x, int y, Direction dir, Direction now_dir, Direction &nextDirection,
    float &Value) {
  const bool isWall = existWall(x, y, dir);
  // const bool step = isStep(x, y, dir);
  const float dist = isWall ? vector_max_step_val : getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && dist < Value) {
    nextDirection = dir;
    Value = dist;
  }
}

__attribute__((noinline, section(".time_critical.search")))
bool MazeSolverBaseLgc::arrival_goal_position(const int x, const int y) {
  return std::ranges::any_of(goal_list, [&](const auto &p) {
    return p.x == x && p.y == y;
  });
}

__attribute__((noinline, section(".time_critical.search")))
unsigned int MazeSolverBaseLgc::searchGoalPosition(
    const bool isSearch,
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  unsigned int cnt = updateVectorMap(isSearch, subgoal_list);
  walk_goal_route(subgoal_list);
  return cnt;
}

// searchGoalPosition(true, subgoal_list) と同じ結果を返す。違いは、地図・ゴール・重みパターンが
// 「前回この関数で表を作ったとき」から変わっていなければ、表(vector_dist / updateMap)を作り直さないこと
// (2026-09-29)。探索で既知の区画を走っている間は新しい壁が分からないので、表は前回と同じになる
// (ゴール後の update() の 44 %)。そのときはサブゴールの手入れだけを、元と同じ順でやる:
//   1. 期限切れ(clear_vector_distmap() がやっていた分)
//   2. 4 辺とも既知になった区画を外す(updateVectorMap() が区画を取り出すたびにやっていた分)
//   3. 経路の上の未知区画を足す
// ほかの誰かが表を作り直していたら(vector_map_serial が進んでいたら)使い回さない。
__attribute__((noinline, section(".time_critical.search")))
unsigned int MazeSolverBaseLgc::searchGoalPositionReuse(
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  const auto same_goal = [&]() {
    if (search_table.goal.size() != goal_list.size())
      return false;
    for (size_t i = 0; i < goal_list.size(); i++) {
      if (search_table.goal[i].x != goal_list[i].x ||
          search_table.goal[i].y != goal_list[i].y)
        return false;
    }
    return true;
  };
  const bool reuse = search_table.valid &&
                     search_table.serial == vector_map_serial &&
                     search_table.param_num == param_num &&
                     search_table.map == map && same_goal();
  if (!reuse) {
    g_n_rebuild++;
    const unsigned int cnt = searchGoalPosition(true, subgoal_list);
    search_table.valid = true;
    search_table.serial = vector_map_serial;
    search_table.param_num = param_num;
    search_table.map = map;
    search_table.goal = goal_list;
    return cnt;
  }

  g_n_reuse++;
  age_subgoal(subgoal_list);
  // 表づくりで取り出される区画 = 壁のどれかが表に入った区画か、出口のあるゴール区画
  for (auto it = subgoal_list.begin(); it != subgoal_list.end();) {
    const unsigned int idx = it->first;
    bool erase = false;
    if (idx < maze_list_size && (map[idx] & 0xf0) == 0xf0) {
      erase = updateMap[idx] != 0;
      if (!erase) {
        for (const auto p : goal_list) {
          if ((unsigned int)(p.x + p.y * maze_size) == idx &&
              (map[idx] & 0x0f) != 0x0f)
            erase = true;
        }
      }
    }
    it = erase ? subgoal_list.erase(it) : std::next(it);
  }
  walk_goal_route(subgoal_list);
  return 0;
}

// 表(vector_dist)をスタートからゴールまでたどり、通る壁が未知なら、その先の区画をサブゴールに足す
__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::walk_goal_route(
    unordered_map<unsigned int, unsigned char> &subgoal_list) {
  Direction next_dir = Direction::North;
  Direction now_dir = Direction::North;
  int x = 0;
  int y = 1;
  // int position = 0;
  // int idx;

  Direction dirLog[3] = {now_dir, now_dir, now_dir};
  point_t pt;
  pt.x = 0;
  pt.y = 0;
  // search_log.clear();
  // search_log.shrink_to_fit();

  while (true) {
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;
    Value = vector_max_step_val;
    next_dir = Direction::Undefined;
    pt.x = x;
    pt.y = y;
    // search_log.emplace_back(pt);

    if (arrival_goal_position(x, y))
      break;

    // const unsigned int position = getDistVector(x, y, now_dir);
    float position = getDistVector(x, y, now_dir);

    if (now_dir == Direction::North) {
      position = getDistVector(x, y, Direction::South);
    } else if (now_dir == Direction::East) {
      position = getDistVector(x, y, Direction::West);
    } else if (now_dir == Direction::West) {
      position = getDistVector(x, y, Direction::East);
    } else if (now_dir == Direction::South) {
      position = getDistVector(x, y, Direction::North);
    }
    setNextRootDirectionPathUnKnown(x, y, Direction::North, now_dir, next_dir,
                                    position);
    setNextRootDirectionPathUnKnown(x, y, Direction::East, now_dir, next_dir,
                                    position);
    setNextRootDirectionPathUnKnown(x, y, Direction::West, now_dir, next_dir,
                                    position);
    setNextRootDirectionPathUnKnown(x, y, Direction::South, now_dir, next_dir,
                                    position);

    if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2])
      priorityStraight2(x, y, now_dir, dirLog[0], position, next_dir);
    else
      priorityStraight2(x, y, now_dir, dirLog[1], position, next_dir);

    if (next_dir == Direction::North) {
      if (is_unknown(x, y, Direction::North))
        subgoal_list[x + (y + 1) * maze_size] = 1;
    } else if (next_dir == Direction::East) {
      if (is_unknown(x, y, Direction::East))
        subgoal_list[x + 1 + y * maze_size] = 1;
    } else if (next_dir == Direction::West) {
      if (is_unknown(x, y, Direction::West))
        subgoal_list[x - 1 + y * maze_size] = 1;
    } else if (next_dir == Direction::South) {
      if (is_unknown(x, y, Direction::South))
        subgoal_list[x + (y - 1) * maze_size] = 1;
    }
    // if (next_dir == Direction::North) {
    //   if (!is_stepped(x, y + 1))
    //     subgoal_list[x + (y + 1) * maze_size] = 1;
    // } else if (next_dir == Direction::East) {
    //   if (!is_stepped(x + 1, y))
    //     subgoal_list[x + 1 + y * maze_size] = 1;
    // } else if (next_dir == Direction::West) {
    //   if (!is_stepped(x - 1, y))
    //     subgoal_list[x - 1 + y * maze_size] = 1;
    // } else if (next_dir == Direction::South) {
    //   if (!is_stepped(x, y - 1))
    //     subgoal_list[x + (y - 1) * maze_size] = 1;
    // }

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

__attribute__((noinline, section(".time_critical.search")))
void MazeSolverBaseLgc::priorityStraight2(int x, int y, Direction now_dir,
                                          Direction dir, float &dist_val,
                                          Direction &next_dir) {
  const bool isWall = existWall(x, y, dir);
  const bool step = isStep(x, y, dir);
  const float dist = isWall ? vector_max_step_val : getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && step && dist <= dist_val) {
    next_dir = dir;
    dist_val = dist;
  }
}
MazeSolverBaseLgc::MazeSolverBaseLgc(/* args */) {}

MazeSolverBaseLgc::~MazeSolverBaseLgc() {}
