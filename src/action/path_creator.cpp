#include "include/action/path_creator.hpp"

PathCreator::PathCreator() {
  const int capacity = MAX_MAZE_SIZE * MAX_MAZE_SIZE;
  other_route_map.reserve(capacity);
  other_route_map_bk.reserve(capacity);
  stepped.reserve(capacity);
}

PathCreator::~PathCreator() {}

int PathCreator::get_dist_val(int x, int y) { return lgc->get_dist_val(x, y); }
void PathCreator::set_logic(std::shared_ptr<MazeSolverBaseLgc> &_lgc) {
  lgc = _lgc;
}
void PathCreator::set_userinterface(std::shared_ptr<UserInterface> &_ui) {
  ui = _ui;
}
void PathCreator::updateVectorMap(const bool isSearch) {
  lgc->updateVectorMap(isSearch);
}

void PathCreator::path_reflash() {
  path_s.clear();
  path_t.clear();
}

void PathCreator::append_path_s(float val) { path_s.emplace_back(val); }
void PathCreator::append_path_t(int val) { path_t.emplace_back(val); }
void PathCreator::add_path_s(int idx, float val) { path_s[idx] += val; }

void PathCreator::setNextRootDirectionPath(int x, int y, Direction now_dir,
                                           Direction dir, float &val,
                                           Direction &next_dir) {
  const bool isWall = lgc->existWall(x, y, dir);
  const float dist =
      isWall ? vector_max_step_val : lgc->getDistVector(x, y, dir);

  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && dist < val) {
    next_dir = dir;
    val = dist;
  }
}
Motion PathCreator::get_next_motion(Direction now_dir,
                                    Direction next_direction) {
  if (now_dir == next_direction)
    return Motion::Straight;

  if (now_dir == Direction::North) {
    if (next_direction == Direction::East)
      return Motion::TurnRight;
    else if (next_direction == Direction::West)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::East) {
    if (next_direction == Direction::South)
      return Motion::TurnRight;
    else if (next_direction == Direction::North)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::West) {
    if (next_direction == Direction::North)
      return Motion::TurnRight;
    else if (next_direction == Direction::South)
      return Motion::TurnLeft;
  } else if (now_dir == Direction::South) {
    if (next_direction == Direction::West)
      return Motion::TurnRight;
    else if (next_direction == Direction::East)
      return Motion::TurnLeft;
  }
  return Motion::Back;
}
Direction PathCreator::get_next_pos(int &x, int &y, Direction dir,
                                    Direction next_direction) {
  if (next_direction == Direction::Undefined) {
    if (dir == Direction::North)
      next_direction = Direction::South;
    else if (dir == Direction::East)
      next_direction = Direction::West;
    else if (dir == Direction::West)
      next_direction = Direction::East;
    else if (dir == Direction::South)
      next_direction = Direction::North;
  }
  if (next_direction == Direction::North)
    y++;
  else if (next_direction == Direction::East)
    x++;
  else if (next_direction == Direction::West)
    x--;
  else if (next_direction == Direction::South)
    y--;

  return next_direction;
}

void PathCreator::priorityStraight2(int x, int y, Direction now_dir,
                                    Direction dir, float &dist_val,
                                    Direction &next_dir) {
  const bool isWall = lgc->existWall(x, y, dir);
  const bool step = lgc->isStep(x, y, dir);
  const float dist = isWall ? MAX : lgc->getDistVector(x, y, dir);
  if (static_cast<int>(now_dir) * static_cast<int>(dir) == 8)
    return;
  if (!isWall && step && dist <= dist_val) {
    next_dir = dir;
    dist_val = dist;
  }
}
bool PathCreator::path_create(bool is_search) {
  bool use;
  return path_create(is_search, 0, 0, Direction::Null, use);
}

__attribute__((noinline, section(".time_critical.path_creator")))
bool PathCreator::path_create(bool is_search, int tgt_x, int tgt_y,
                              Direction tgt_dir, bool &use) {
  Direction next_dir = Direction::North;
  Direction now_dir = next_dir;
  unsigned int idx = 0;
  Direction dirLog[3] = {
      Direction::North,
      Direction::North,
      Direction::North,
  };

  int x = 0;
  int y = 1;

  path_reflash();
  if (!reuse_vector_map) {
    lgc->set_param();
    lgc->updateVectorMap(is_search);
  }

  path_s.emplace_back(3);
  float dist_val = MAX;
  float old_dist_val = MAX;
  stepped.clear();
  int cnt = 0;
  while (true) {
    cnt++;
    if (ui->button_state()) {
      return false;
    }
    if (cnt > 1023) {
      return false;
    }
    now_dir = next_dir;
    dirLog[2] = dirLog[1];
    dirLog[1] = dirLog[0];
    dirLog[0] = now_dir;
    if (stepped.contains(x + lgc->maze_size * y)) {
      return false;
    }
    stepped[x + lgc->maze_size * y] = 1;
    dist_val = MAX;
    next_dir = Direction::Undefined;

    if (lgc->arrival_goal_position(x, y)) {
      add_path_s(idx, 2);
      path_t.emplace_back(255);
      path_size = idx;
      return true;
    }
    float position = lgc->VectorMaxF;
    if (now_dir == Direction::North) {
      position = lgc->getDistVector(x, y, Direction::South);
    } else if (now_dir == Direction::East) {
      position = lgc->getDistVector(x, y, Direction::West);
    } else if (now_dir == Direction::West) {
      position = lgc->getDistVector(x, y, Direction::East);
    } else if (now_dir == Direction::South) {
      position = lgc->getDistVector(x, y, Direction::North);
    }

    setNextRootDirectionPath(x, y, now_dir, Direction::North, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::East, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::West, dist_val,
                             next_dir);
    setNextRootDirectionPath(x, y, now_dir, Direction::South, dist_val,
                             next_dir);
    if (dist_val == old_dist_val) {
      break;
    }
    if ((dirLog[0] == dirLog[1]) || (dirLog[0] != dirLog[2])) {
      priorityStraight2(x, y, now_dir, dirLog[0], dist_val, next_dir);
    } else {
      priorityStraight2(x, y, now_dir, dirLog[1], dist_val, next_dir);
    }

    if (tgt_dir != Direction::Null) {
      if (x == tgt_x && y == tgt_y) {
        next_dir = tgt_dir;
        dist_val = lgc->getDistV(x, y, next_dir);
        use = true;
      }
      if (other_route_map.contains(x + lgc->maze_size * y)) {
        if (other_route_map[x + lgc->maze_size * y].selected) {
          next_dir = other_route_map[x + lgc->maze_size * y].select_dir;
          dist_val = lgc->getDistV(x, y, next_dir);
        }
      }
    }
    Motion nextMotion = get_next_motion(now_dir, next_dir);
    checkOtherRoot(x, y, now_dir, position);

    if (nextMotion == Motion::Straight) {
      add_path_s(idx, 2);
    } else if (nextMotion == Motion::TurnRight) {
      path_t.emplace_back(static_cast<int>(TurnDirection::Right));
      path_s.emplace_back(2);
      idx++;
    } else if (nextMotion == Motion::TurnLeft) {
      path_t.emplace_back(static_cast<int>(TurnDirection::Left));
      path_s.emplace_back(2);
      idx++;
    } else {
      path_t.emplace_back(0);
      break;
    }
    next_dir = get_next_pos(x, y, now_dir, next_dir);
  }
  path_size = idx;
  return false;
}
__attribute__((noinline, section(".time_critical.path_creator")))
void PathCreator::convert_large_path(bool b1) {
  int i = 0;
  int finish = 0;
  for (i = 0; i < path_size; i++) {
    if (path_s[i + 1] == 2) {
      if ((path_s[i] > 2) && path_s[i + 2] > 2) {
        if (path_t[i] == path_t[i + 1]) {
          if (path_t[i] == 1) {
            path_t[i] = (unsigned char)3;
            path_t[i + 1] = (unsigned char)254;
          } else if (path_t[i] == 2) {
            path_t[i] = (unsigned char)4;
            path_t[i + 1] = (unsigned char)254;
          }
          path_s[i] -= 1;
          path_s[i + 2] -= 1;
          i++;
        }
      }
    }
    if (path_t[i] == 0) {
      break;
    }
  }
  for (i = 0; i < path_size; i++) {
    if ((path_s[i] > 2) && (path_s[i + 1] > 2)) {
      if (path_t[i] == 1) {
        path_t[i] = (unsigned char)5;
        path_s[i + 1] -= 1;
        path_s[i] -= 1;
      } else if (path_t[i] == 2) {
        path_t[i] = (unsigned char)6;
        path_s[i + 1] -= 1;
        path_s[i] -= 1;
      }
    }
    if (path_t[i] == 0) {
      break;
    }
  }
  if (b1) {
    if (path_size >= 2 && path_s[0] == 2 && path_t[0] == 1 && path_s[1] > 2) {
      path_s[0] -= 1;
      path_s[1] -= 1;
      path_t[0] = (unsigned char)5;
    }
    // 2026-09-29: 終端は 255 で 0 は入らないので、0 を探すと配列の先(他のメモリ)まで読み、
    // そこの値しだいで配列の外へ書いていた。配列の中だけを見る。
    i = 0;
    while (i < (int)path_t.size() && path_t[i] != 0) {
      i++;
    }
    finish = i;
    if (finish >= (int)path_s.size() || finish >= (int)path_t.size()) {
      return;
    }
    if (finish >= 1) {
      if (path_s[finish] == 2 && path_s[finish - 1] > 2 &&
          (path_t[finish - 1] == 1 || path_t[finish - 1] == 2)) {
        path_s[finish - 1] -= 1;
        path_s[finish] -= 1;
        if (path_t[finish - 1] == 1) {
          path_t[finish - 1] = (unsigned char)5;
        } else if (path_t[finish - 1] == 2) {
          path_t[finish - 1] = (unsigned char)6;
        }
      }
    }
    if (finish > 2) {
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] > 2 && path_t[finish - 1] == 1 &&
          path_t[finish - 2] == 1) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = (unsigned char)3;
        path_t[finish - 1] = (unsigned char)254;
      }
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] > 2 && path_t[finish - 1] == 2 &&
          path_t[finish - 2] == 2) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = (unsigned char)4;
        path_t[finish - 1] = (unsigned char)254;
      }
    }
    if (finish == 2) {
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] >= 2 && path_t[finish - 1] == 1 &&
          path_t[finish - 2] == 1) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = (unsigned char)3;
        path_t[finish - 1] = (unsigned char)254;
      }
      if (path_s[finish] == 2 && path_s[finish - 1] == 2 &&
          path_s[finish - 2] >= 2 && path_t[finish - 1] == 2 &&
          path_t[finish - 2] == 2) {
        path_s[finish - 2] -= 1;
        path_s[finish] -= 1;
        path_t[finish - 2] = (unsigned char)4;
        path_t[finish - 1] = (unsigned char)254;
      }
    }
  }
}

__attribute__((noinline, section(".time_critical.path_creator")))
void PathCreator::diagonalPath(bool isFull, bool a1) {
  int i = 0;
  int j = 0;
  int dir = 0;
  int check = 0;
  bool diaMode = false;
  int m = 0;
  bool check2 = false;
  bool check3 = false;

  bool _virtual = !a1;
  bool flag = false;
  // 2026-09-29: 配列の先を読まない(終端は 255 なので、0 を探すと配列の外まで進んでいた)。
  // 配列の外は「終端の先 = 0」として扱う。配列の中の結果は変わらない。
  const int t_size = (int)path_t.size();
  const int s_size = (int)path_s.size();
  // 読むときは T() / S() を通す(書くときは path_t[] / path_s[] のまま)
  const auto T = [&](int k) -> int {
    return (k >= 0 && k < t_size) ? path_t[k] : 0;
  };
  const auto S = [&](int k) -> float {
    return (k >= 0 && k < s_size) ? path_s[k] : 0;
  };
  while (T(i) != 0) {
    check = 0;
    if (_virtual) {
      a1 = false;
      if (S(i) > 2) {
        flag = true;
      }
      if (flag) {
        for (m = i + 1;; m++) {
          if (S(m) > 2) {
            a1 = true;
            break;
          }
          if (T(m) == 0) {
            a1 = false;
            break;
          }
        }
      }
    }

    if (T(i) == R && (a1 ? true : S(i) > 2)) {
      dir = R;
      for (j = i + 1; T(j) != dir; j++) {
        if (T(j) == R && S(j) == 2) {
          dir = R;
        } else if (T(j) == L && S(j) == 2) {
          dir = L;
        } else {
          break;
        }
        check++;
      }
    } else if (T(i) == L && (a1 ? true : S(i) > 2)) {
      dir = L;
      for (j = i + 1; T(j) != dir; j++) {
        if (T(j) == R && S(j) == 2) {
          dir = R;
        } else if (T(j) == L && S(j) == 2) {
          dir = L;
        } else {
          break;
        }
        check++;
      }
    }
    if (check != 0) {
      j -= 1;
      if ((i != 0 && S(i) == 2 && T(i) == T(i - 1) && S(i - 1) > 2)) {
        if (T(i) == R) {
          path_t[i - 1] = (unsigned char)9;
          check3 = true;
        } else if (T(i) == L) {
          path_t[i - 1] = (unsigned char)10;
          check3 = true;
        }
        if (j != 0 && S(j + 2) > 2 && T(j) == T(j + 1)) {
          if (T(j) == R) {
            path_t[j + 1] = (unsigned char)9;
            check2 = true;
          } else if (T(j) == L) {
            path_t[j + 1] = (unsigned char)10;
            check2 = true;
          }
          path_s[j + 1] = check + 1;
        } else {
          if (T(j) == R) {
            path_t[j] = (unsigned char)7;
          } else if (T(j) == L) {
            path_t[j] = (unsigned char)8;
          }
          path_s[j] = check + 1;
        }
      } else {
        int memory = 0;
        if (T(i) == R) {
          path_t[i] = (unsigned char)7;
          memory = R;
        } else if (T(i) == L) {
          path_t[i] = (unsigned char)8;
          memory = L;
        }
        if (j != 0 && S(j + 2) > 2 && T(j) == T(j + 1)) {
          if (T(j) == R) {
            path_t[j + 1] = (unsigned char)9;
            check2 = true;
          } else if (T(j) == L) {
            path_t[j + 1] = (unsigned char)10;
            check2 = true;
          }
          path_s[j + 1] = check + 1;
        } else {
          if (!a1) {
            if (S(j + 1) > 2) {
              if (T(j) == R) {
                path_t[j] = (unsigned char)7;
              } else if (T(j) == L) {
                path_t[j] = (unsigned char)8;
              }
              path_s[j] = check + 1;
            } else {
              path_t[i] = memory;
              check3 = false;
              check2 = false;
              i = j;
              i++;
              continue;
            }
          } else {
            if (T(j) == R) {
              path_t[j] = (unsigned char)7;
            } else if (T(j) == L) {
              path_t[j] = (unsigned char)8;
            }
            path_s[j] = check + 1;
          }
        }
      }
      if (check3) {
        for (int k = i; k < j; k++) {
          path_t[k] = (unsigned char)254;
        }
      } else {
        for (int k = i + 1; k < j; k++) {
          path_t[k] = (unsigned char)254;
        }
      }
      if (check2) {
        path_t[j] = (unsigned char)254;
      }
      check3 = false;
      check2 = false;
      i = j;
    }
    i++;
  }
  i = 0;

  while (T(i) != 0) {
    if (T(i) == 7 && T(i + 1) == 7 && S(i + 1) == 2) {
      path_t[i] = (unsigned char)11;
      path_t[i + 1] = (unsigned char)254;
    }
    if (T(i) == 8 && T(i + 1) == 8 && S(i + 1) == 2) {
      path_t[i] = (unsigned char)12;
      path_t[i + 1] = (unsigned char)254;
    }
    i++;
  }
  i = 0;
  diaMode = false;
  while (T(i) != 0) {
    if (!diaMode) {
      if (T(i) == 7 || T(i) == 8 || T(i) == 9 || T(i) == 10) {
        path_s[i] -= 1;
        diaMode = true;
      }
    } else if (diaMode) {
      if (T(i) == 7 || T(i) == 8 || T(i) == 9 || T(i) == 10) {
        if (i + 1 < s_size) {
          path_s[i + 1] -= 1;
        }
        diaMode = false;
      }
    }
    i++;
  }
  pathOffset();
}

__attribute__((noinline, section(".time_critical.path_creator")))
void PathCreator::pathOffset() {
  int i = 0;
  for (i = 0; i < path_size; i++) {
    if (path_t[i] == 0) {
      path_t[i] = (unsigned char)0xff;
      break;
    }
  }
  i = 0;
  while (i < path_size && path_t[i] != 0xff) {
    if (i > 0) {
      if (path_t[i] == 254) {
        path_s[i] = 0;
        path_t[i] = (unsigned char)0;
      }
    }
    i++;
  }
  i = 0;
  while (i < path_size && path_t[i] != 0xff) {
    while (path_t[i] == 0) {
      for (int j = i; path_t[j] != 0xff; j++) {
        path_s[j] = path_s[j + 1];
        path_t[j] = (unsigned char)path_t[j + 1];
      }
    }
    i++;
  }
}

__attribute__((noinline, section(".time_critical.path_creator")))
void PathCreator::print_path() {
  printf("path size: %d\n", path_size);
  auto size = path_s.size();
  bool dia = false;
  for (int i = 0; i < (int)size; i++) {
    float dist2 = 0.5 * path_s[i] - 1;
    if (path_time_s.size() > (size_t)i) {
      printf("[%d]: %0.2f,\t%s [%s]\tt=(%0.3f, %0.3f, "
             "%0.3f)\t[v,d]=(%4.1f, %4.1f, "
             "%4.1f, "
             "%4.3f[mm])\n",
             i, dist2, tc.get_turn_type_string(path_t[i], dia).c_str(),
             tc.get_turn_dir_string(path_t[i]).c_str(), path_time_s[i],
             path_time_t[i], path_time_total[i].total_time,
             path_time_total[i].v_start, path_time_total[i].v_max,
             path_time_total[i].v_end, path_time_total[i].dist);
      if (path_t[i] == 7 || path_t[i] == 8 || path_t[i] == 9 ||
          path_t[i] == 10) {
        dia = !dia;
      }
    } else {
      printf("[%d]: %0.2f,\t%d\n", i, dist2, path_t[i]);
    }
    if (path_t[i] == 255 || path_t[i] == 0) {
      break;
    }
  }
}

__attribute__((noinline, section(".time_critical.path_creator")))
void PathCreator::print_path2() {
  auto size = path_s2.size();
  for (int i = 0; i < (int)size; i++) {
    float dist2 = 0.5 * path_s2[i] - 1;
    printf("[%d]: %0.2f,\t%d\n", i, dist2, path_t2[i]);
  }
}

__attribute__((noinline, section(".time_critical.path_creator")))
bool PathCreator::path_create_with_change(bool is_search, int tgt_x, int tgt_y,
                                          Direction tgt_dir,
                                          path_create_status_t &pc_state,
                                          param_set_t &p_set) {
  pc_result.time = 10000;
  pc_result.use = false;
  pc_result.state =
      path_create(is_search, tgt_x, tgt_y, tgt_dir, pc_result.use);
  if (!pc_result.state) {
    return false;
  }
  convert_large_path(true);
  diagonalPath(false, true);
  pc_result.time = calc_goal_time(p_set);
  if (pc_result.time > 100) {
    return false;
  }
  return true;
}

__attribute__((noinline, section(".time_critical.path_creator")))
float PathCreator::timebase_path_create(bool is_search, param_set_t &p_set,
                                        path_set_t &p) {
  // 2026-09-29: 候補を 1 つ試すたびに path_create() が近似コストの表(updateVectorMap、迷路全体の
  // Dijkstra)を作り直していた。この間に変わるのは「(x, y) でこの向きへ行く」という指示だけで、
  // 表を決める壁・ゴール・重みパターンは直前の path_create() と同じなので、そのとき作った表を
  // 使い回す。どの return でも元に戻す。path_sim の 57 件で経路は同じ、計算は半分以下。
  struct ReuseVectorMap {
    bool &flag;
    explicit ReuseVectorMap(bool &f) : flag(f) { flag = true; }
    ~ReuseVectorMap() { flag = false; }
  } reuse_guard(reuse_vector_map);
  bool next_end = false;
  candidate_route_info_t tmp_cand_route;

  for (int i = 0; i < 5; i++) {
    const auto before = other_route_map.size();
    for (auto itr = other_route_map.begin(); itr != other_route_map.end();
         itr++) {
      auto tmp_cand = (itr->second);
      if (tmp_cand.selected) {
        tmp_cand.candidate_dir_set.clear();
        tmp_cand.candidate_dir_set.insert(tmp_cand.select_dir);
      } else {
        route_q.push((itr->second));
      }
    }
    while (!route_q.empty()) {
      const auto cand = route_q.top();
      route_q.pop();
      int x = cand.x;
      int y = cand.y;
      while (!route_list.empty()) {
        route_list.pop();
      }
      for (const auto dir : cand.candidate_dir_set) {
        bool goal =
            path_create_with_change(is_search, x, y, dir, pc_result, p_set);
        if (ui->button_state_hold()) {
          p.result = false;
          return 0;
        }
        if (!pc_result.state || !goal) {
          other_route_map[x + lgc->maze_size * y].candidate_dir_set.erase(dir);
        } else {
          route.time = pc_result.time;
          if (p.time > route.time) {
            p.time = route.time;
            p.path_s.clear();
            p.path_t.clear();
            for (int i = 0; i < (int)path_t.size(); i++) {
              p.path_s.push_back(path_s[i]);
              p.path_t.push_back(path_t[i]);
            }
          }
          route.dir = dir;
          route.use = pc_result.use;
          route_list.push(route);
        }
      }
      if (route_list.size() > 0) {
        const auto top = route_list.top();
        if (top.use) {
          other_route_map[x + lgc->maze_size * y].selected = true;
          other_route_map[x + lgc->maze_size * y].select_dir = top.dir;
        }
      }
    }

    const auto after = other_route_map.size();
    if (next_end) {
      p.result = true;
      return 1;
    }
    if (before == after) {
      next_end = true;
    }
  }
  p.result = false;
  return 0;
}

// 分岐候補に入れる近似コストの幅(区画数)。checkOtherRoot() 参照。
static constexpr float OTHER_ROUTE_MARGIN_CELLS = 0.5f;

__attribute__((noinline, section(".time_critical.path_creator")))
void PathCreator::checkOtherRoot(int x, int y, Direction now_dir, float now) {
  if (other_route_map.contains(x + y * lgc->maze_size)) {
    return;
  }

  const int now_d = static_cast<int>(now_dir);
  candidate_route_info_t cand;
  // 2026-09-29: 分岐候補を「今の値より小さい向き」から「今の値 + 区画の OTHER_ROUTE_MARGIN_CELLS
  // ぶん以下の向き」にゆるめた。近似コスト(updateVectorMap)はターンの種類を見ないので、
  // 手数が多くても実際は速い経路(きついターンを避けて斜め + Large で入る等)が「下り」に
  // ならず、timebase_path_create の時間比較に載らなかった。同点も落ちていた。
  // 例: log_log_20260927_155341.maze の (8,4) 西向きで、南が param_num 1 で 24.8 と今の値
  // 21.9 より大きく(差は 0.4 区画)、param_num 2 では同点 → 候補外で 2.017s、入れると 1.982s。
  // 幅は重みパターンごとの 1 区画のコスト(cell_cost)で測る(倍率だとパターンやゴールからの
  // 距離で幅が変わり、候補が増えすぎて計算が 5〜6 倍になった)。
  const float margin = lgc->cell_cost() * OTHER_ROUTE_MARGIN_CELLS;
  for (const auto d : direction_list) {
    const auto dist = lgc->getDistVector(x, y, d);
    const auto d_int = static_cast<int>(d);
    if (now_d * d_int != 8 && !lgc->existWall(x, y, d) &&
        dist <= now + margin && lgc->isStep(x, y, d)) {
      other_route_map[x + y * lgc->maze_size].candidate_dir_set.insert(d);
    }
  }
  if (other_route_map[x + y * lgc->maze_size].candidate_dir_set.size() <= 1) {
    other_route_map.erase(x + y * lgc->maze_size);
  } else {
    other_route_map[x + y * lgc->maze_size].from_dist = now;
    other_route_map[x + y * lgc->maze_size].x = x;
    other_route_map[x + y * lgc->maze_size].y = y;
  }
}

// 区間 1 個(i 番目の直線 + i 番目のターン)のタイム。calc_goal_time() の 1 周ぶんで、
// TimePathPlanner(タイム最小の経路探索)も辺の重みとしてこれを呼ぶ。速度や加速度の選び方を
// 変えるときはここだけ直せば、見積もりと経路探索の両方に入る。
//   first       経路の最初の区間(スタートからの直線)
//   dia         この区間の直線が斜めか
//   s, tcode    path_s[i], path_t[i](変換後)
//   v_now       区間の入口の速度
//   exist_next  次の区間があるか。next_s / next_tcode = path_s[i + 1], path_t[i + 1]
//   tmp_time    直線の内訳(v_start / v_max / v_end / dist)を入れる
__attribute__((noinline, section(".time_critical.path_creator")))
segment_time_t PathCreator::calc_segment_time(param_set_t &p_set, bool first,
                                              bool dia, float s, int tcode,
                                              float v_now, bool exist_next,
                                              float next_s, int next_tcode,
                                              planning_time_t &tmp_time,
                                              bool debug) {
  segment_time_t res;
  const float cell_size = p_set.cell_size;
  const float dist = !dia ? (0.5 * s - 1) * p_set.cell_size
                          : (0.5 * s - 1) * p_set.cell_size * ROOT2;
  const auto turn_dir = tc.get_turn_dir(tcode);
  const auto turn_type = tc.get_turn_type(tcode, dia);
  bool start_turn = false;
  bool fast_turn_mode = false;
  tmp_time.dist = dist;
  tmp_time.v_start = v_now;
  tmp_time.v_max = v_now;
  tmp_time.v_end = v_now;
  tmp_time.lap_time = 0;
  tmp_time.total_time = 0;
  // 最初の区間で直線が無い(スタート直後にターン)ときだけ遅いターン
  const bool fast_mode = !first || dist > 0;

  const auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
  ps.v_max = p_set.str_map[st].v_max;
  ps.v_end = fast_mode ? p_set.map[turn_type].v : p_set.map_slow[turn_type].v;
  ps.accl = p_set.str_map[st].accl;
  ps.decel = p_set.str_map[st].decel;
  if ((dist > 0) || first) {

    ps.dist = dist;

    if (exist_next) {
      float next_dist = 0.5 * next_s - 1;
      auto next_turn_type = tc.get_turn_type(next_tcode);
      if (next_dist > 0 && (next_turn_type == TurnType::Orval ||
                            next_turn_type == TurnType::Large)) {
        ps.v_end = p_set.map_fast[turn_type].v;
        fast_turn_mode = true;
      }
    }

    tmp_time.v_max = ps.v_max;
    tmp_time.v_end = ps.v_end;

    if (first) {
      if (dist == 0) {
        start_turn = true;
      }
      ps.dist += p_set.start_offset;
      auto tmp_v2 = 2 * ps.accl * ps.dist;
      if (ps.v_end * ps.v_end > tmp_v2) {
        ps.accl = (ps.v_end * ps.v_end) / (2 * ps.dist) + 1000;
        ps.decel = -ps.accl;
      }
    }
    if (turn_type == TurnType::Finish) {
      ps.dist -= p_set.cell_size / 2;
      if (p_set.suction) {
        ps.v_end = 3500;
      } else {
        ps.v_end = p_set.map[TurnType::Large].v;
      }
    }
    res.str_time = go_straight_dummy(v_now, ps.v_max, ps.v_end, ps.accl,
                                     ps.decel, ps.dist, tmp_time, debug);
    res.has_straight = true;
    v_now = ps.v_end;
    if (turn_type == TurnType::Finish) {
      res.v_now = v_now;
      return res;
    }
  }
  if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
    float dist3 = 0;
    if (exist_next) {
      dist3 = 0.5 * next_s * cell_size;
    }

    if (fast_turn_mode) {
      res.turn_time = slalom_dummy(turn_type, turn_dir, p_set.map_fast);
    } else if (start_turn) {
      res.turn_time = slalom_dummy(turn_type, turn_dir, p_set.map_slow);
    } else {
      res.turn_time = slalom_dummy(turn_type, turn_dir, p_set.map);
    }

    if (first && start_turn) {
      auto next_turn_type = tc.get_turn_type(next_tcode);
      ps.v_end = p_set.map[next_turn_type].v;
      v_now = ps.v_end;
    } else if (dist3 == 0) {
      auto next_turn_type = tc.get_turn_type(next_tcode);
      ps.v_end       = p_set.map[next_turn_type].v;
      tmp_time.v_end = ps.v_end;
      tmp_time.v_max = ps.v_end;
    } else if (dist == 0) {
      auto turn_type = tc.get_turn_type(tcode);
      ps.v_end       = p_set.map[turn_type].v;
      tmp_time.v_end = ps.v_end;
      tmp_time.v_max = ps.v_end;
    }
    v_now = ps.v_end;
  }
  res.v_now = v_now;
  return res;
}

__attribute__((noinline, section(".time_critical.path_creator")))
float PathCreator::calc_goal_time(param_set_t &p_set, bool debug) {
  if (debug) {
    printf("calc_goal_time start\n");
  }
  bool dia = false;
  float v_now = 0;
  float time = 0;
  Direction ego_dir = Direction::North;
  planning_time_t tmp_time;
  path_time_s.clear();
  path_time_t.clear();
  path_time_total.clear();
  float lap_time = 0;
  auto path_size = path_s.size();
  for (int i = 0; i < (int)path_t.size(); i++) {
    auto turn_dir = tc.get_turn_dir(path_t[i]);
    auto turn_type = tc.get_turn_type(path_t[i], dia);
    lap_time = 0;

    const bool exist_next_idx = (i + 1) < (int)path_size;
    const float next_s = exist_next_idx ? path_s[i + 1] : 0;
    const int next_tcode = (i + 1) < (int)path_t.size() ? path_t[i + 1] : 0;
    const auto seg =
        calc_segment_time(p_set, i == 0, dia, path_s[i], path_t[i], v_now,
                          exist_next_idx, next_s, next_tcode, tmp_time, debug);

    if (seg.has_straight) {
      time += seg.str_time;
      lap_time += seg.str_time;
      if (time > 100) {
        break;
      }
    }
    v_now = seg.v_now;
    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
      time += seg.turn_time;
      lap_time += seg.str_time;
      ego_dir = tc.get_next_dir(ego_dir, turn_type, turn_dir);
      dia =
          (ego_dir == Direction::NorthEast || ego_dir == Direction::NorthWest ||
           ego_dir == Direction::SouthEast || ego_dir == Direction::SouthWest);
    }

    // 2026-09-29: 最後の直線(最後のターン〜ゴール)もここを通して合計に入れる。以前は
    // Finish の直線を time に足した直後に break しており、返す値(最後に積んだ区間の
    // total_time)に入っていなかった(大会迷路 21 本で平均 45 ms、最大 514 ms 短く出ていた)。
    path_time_s.push_back(seg.str_time);
    path_time_t.push_back(seg.turn_time);
    tmp_time.total_time = time;
    tmp_time.lap_time = lap_time;
    path_time_total.push_back(tmp_time);

    if (turn_type == TurnType::None) {
      break;
    }
    if (turn_type == TurnType::Finish) {
      break;
    }
  }
  if (path_time_total.size() == 0) {
    return 100000;
  }
  return path_time_total.back().total_time;
}

__attribute__((noinline, section(".time_critical.path_creator")))
char PathCreator::asc(float d, float d2) {
  if (d < d2) {
    return 2;
  }
  return 1;
}

__attribute__((noinline, section(".time_critical.path_creator")))
float PathCreator::go_straight_dummy(float v1, float vmax, float v2, float ac,
                                     float diac, float dist,
                                     planning_time_t &planning_time,
                                     bool debug) {
  float acc = ac;
  float distance = 0;
  float time = 0;
  float V_now = v1;
  int sequence = 1;
  float d2;

  planning_time.v_start = planning_time.v_max = V_now;

  float dist2 = std::abs((v1 * v1 - v2 * v2) / (2 * diac));
  if (dist2 > dist) {
    acc = std::abs((v1 * v1 - v2 * v2) / (2 * dist)) + 1000;
  }

  // 無限ループしたときに人が止めるためのボタン確認は、計算上 5 ms(ループ 5 回)ごと
  // (2026-09-29。1 ms ごとだと 1 回の path_run で数千万回 GPIO を読んでいた。ループ 1 回は
  // 実時間では µs 程度なので、5 回ごとでも押してからの遅れは分からない)。
  constexpr int BUTTON_CHECK_INTERVAL = 5;
  int loop_cnt = 0;
  while (distance <= dist) {
    time += 1;
    if (++loop_cnt % BUTTON_CHECK_INTERVAL == 0 && ui->button_state()) {
      return 10000;
    }
    d2 = std::abs((V_now + v2) * (V_now - v2) / (2.0 * diac));
    auto diac2 = -std::abs((V_now + v2) * (V_now - v2)) /
                 (2 * std::abs(dist - distance));

    auto tmpv = V_now + acc * dt;
    switch (sequence) {
    case 3:
      acc = 0;
      break;
    case 1:
      sequence = asc(dist - distance, d2);
      if (tmpv >= vmax) {
        acc = 0;
        V_now = vmax;
      } else {
        acc = ac;
      }
      if (sequence != 3) {
        break;
      }
    case 2:
      if (tmpv <= v2) {
        acc = 0;
        V_now = v2;
      } else {
        acc = diac;
        if (diac2 < diac) {
          acc = diac2;
        }
      }
      break;
    }
    V_now += acc * dt;
    distance += V_now * dt;
    if (V_now > planning_time.v_max) {
      planning_time.v_max = V_now;
    }
    // 速度が 0 以下のまま距離が残っていると、ここから先は進まず抜けられない(パラメータの
    // 抜けで速度・加速度が 0 のとき等)。ボタンを待たずに失敗(ボタンで止めたときと同じ
    // 10000)を返す(2026-09-29。以前は負のときだけ break して途中までの時間を返しており、
    // 走り切れない直線が速く見えていた)。
    if (V_now <= 0 && distance <= dist) {
      return 10000;
    }
  }
  planning_time.v_end = V_now;
  planning_time.dist = distance;
  return time / 1000;
}

__attribute__((noinline, section(".time_critical.path_creator")))
float PathCreator::slalom_dummy(
    TurnType turn_type, TurnDirection td,
    std::unordered_map<TurnType, slalom_param2_t> &turn_param) {
  float turn_time = turn_param[turn_type].time * 2;
  float turn_front_dist = 0;
  float turn_back_dist = 0;
  float v = turn_param[turn_type].v;

  if (td == TurnDirection::Right) {
    turn_front_dist = turn_param[turn_type].front.right;
    turn_back_dist = turn_param[turn_type].back.right;
  } else {
    turn_front_dist = turn_param[turn_type].front.left;
    turn_back_dist = turn_param[turn_type].back.left;
  }
  // 2026-09-28: backからoffset_after_turnを引かない。実走(slalom())は引いた
  // 距離を次ターン手前のWALL_OFFでターン速度のまま走るので、ここで引くと
  // その区間が丸ごと抜ける(20260925_015557.csv: 15ターンで壁切れ237mm/123ms、
  // 仮想1623msに対し実走1787ms)。

  if (turn_front_dist < 0)
    turn_front_dist = 0;
  if (turn_back_dist < 0)
    turn_back_dist = 0;

  float front_time = turn_front_dist / v;
  float back_time = turn_back_dist / v;

  return turn_time + front_time + back_time;
}
