#pragma once
#ifndef PATH_CREATOR_H
#define PATH_CREATOR_H

#include "pico/stdlib.h"
#include "defines.hpp"
#include "stdio.h"
#include <vector>

#include "ui.hpp"
#include "logic.hpp"
#include "maze_solver.hpp"
#include "action/trajectory_creator.hpp"

constexpr int R = 1;
constexpr int L = 2;
constexpr unsigned int vector_max_step_val = 180 * 1024;
constexpr float MAX = vector_max_step_val;

// calc_segment_time() の結果(区間 1 個 = 直線 + ターン)
struct segment_time_t {
  float str_time = 0;        // 直線の時間
  float turn_time = 0;       // ターンの時間
  float v_now = 0;           // 区間を終えたときの速度(次の区間の入口)
  bool has_straight = false; // 直線を計算したか(長さ 0 で最初の区間でもないときは false)
};

class PathCreator {
private:
  int get_dist_val(int x, int y);
  void updateVectorMap(const bool isSearch);

  void add_path_s(int idx, float val);
  void append_path_s(float val);
  void append_path_t(int val);
  void setNextRootDirectionPath(int x, int y, Direction now_dir, Direction dir,
                                float &val, Direction &next_dir);
  Direction get_next_pos(int &x, int &y, Direction dir,
                         Direction next_direction);

  Motion get_next_motion(Direction now_dir, Direction next_direction);

  void priorityStraight2(int x, int y, Direction now_dir, Direction dir,
                         float &dist_val, Direction &next_dir);

  void pathOffset();

  priority_queue<candidate_route_info_t, vector<candidate_route_info_t>,
                 CompairCandiRoute>
      route_q;
  priority_queue<route_t, vector<route_t>, CompairRoute> route_list;

  std::unordered_map<int, int> stepped;

  TrajectoryCreator tc;
  void checkOtherRoot(int x, int y, float now, Direction now_dir);
  void checkOtherRoot(int x, int y, Direction now_dir, float now_pos);

  param_straight_t ps;

  // true の間、path_create() は近似コストの表(lgc の vector_dist)を作り直さない。
  // timebase_path_create() が候補を試している間だけ立てる(timebase_path_create() 参照)。
  bool reuse_vector_map = false;

public:
  std::unordered_map<int, candidate_route_info_t> other_route_map;
  std::unordered_map<int, candidate_route_info_t> other_route_map_bk;
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  std::shared_ptr<UserInterface> ui;
  void set_logic(std::shared_ptr<MazeSolverBaseLgc> &_lgc);
  void set_userinterface(std::shared_ptr<UserInterface> &_ui);

  std::vector<float> path_s;
  std::vector<unsigned char> path_t;
  std::vector<float> path_time_s;
  std::vector<float> path_time_t;
  std::vector<planning_time_t> path_time_total;

  vector<float> path_s2;
  vector<unsigned char> path_t2;

  struct CompairPath {
    bool operator()(path_set_t const &p1, path_set_t const &p2) {
      if (p1.time > p2.time) {
        return true;
      } else if (p1.time == p2.time) {
        if (p1.path_t.size() > p2.path_t.size()) {
          return true;
        }
      }
      return false;
    };
  };
  std::priority_queue<path_set_t, vector<path_set_t>, CompairPath> path_set_map;

  int path_size;

  PathCreator();
  ~PathCreator();
  bool path_create(bool is_search);
  bool path_create(bool is_search, int tgt_x, int tgt_y, Direction tgt_dir,
                   bool &use);
  bool path_create_with_change(bool is_search, int tgt_x, int tgt_y,
                               Direction tgt_dir,
                               path_create_status_t &pc_state,
                               param_set_t &p_set);
  void path_reflash();
  void convert_large_path(bool b1);
  void diagonalPath(bool isFull, bool a1);
  void print_path();
  void print_path2();

  float calc_goal_time(param_set_t &p_set, bool debug = false);
  // 区間 1 個ぶんのタイム。calc_goal_time() と TimePathPlanner が同じものを使う。
  segment_time_t calc_segment_time(param_set_t &p_set, bool first, bool dia,
                                   float s, int tcode, float v_now,
                                   bool exist_next, float next_s,
                                   int next_tcode, planning_time_t &tmp_time,
                                   bool debug = false);
  // 呼ぶ前に、同じ重みパターン(lgc->set_param_num)で path_create() しておくこと。
  // 候補の評価ではそのとき作った近似コストの表を使い回す。
  float timebase_path_create(bool is_search, param_set_t &p_set, path_set_t &p);
  path_create_status_t pc_result;
  route_t route;
  float go_straight_dummy(float v1, float vmax, float v2, float ac, float diac,
                          float dist, planning_time_t &time,
                          bool debug = false);
  float slalom_dummy(TurnType turn_type, TurnDirection td,
                     std::unordered_map<TurnType, slalom_param2_t> &turn_param);

  char asc(float d, float d2);
};

#endif
