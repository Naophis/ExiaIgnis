// search_sim — SearchController::exec()(メインモード mode_num == 0 の探索)をホストで
// 再現し、足立法の判断 1 回ごとの記録と探索時間の試算を出す。
//
// 足立法(Adachi)と迷路ロジック(MazeSolverBaseLgc)はファームのソースそのもの。
// 探索ループは SearchController::exec() の写しで、ユーザー指定の簡略化をしている:
//   - 移動方向は adachi->exec() が決めたとおり
//   - モーションは exec() の選び方で固定。ターンは常にスラローム(judge2() の pivot90 は
//     選ばない)。直進中の壁切れ補正(wall_off)はなし
//   - 壁は正解の迷路から理想どおりに見える(読み違い・横ずれ・前壁距離の補正はなし)
// 時間は各モーションの手順を足したもの:
//   直進        PathCreator::go_straight_dummy()(calc_goal_time と同じ加減速モデル)
//   スラローム  前の直線(ターン速度へ)+ sp.time × 2 + 後ろの直線をターン速度で
//   後退        pivot() の手順(中央へ → 前壁合わせ → 超信地 → 後退 → 半区画)と sleep_ms。
//               超信地は角速度の台形(w_max / alpha)、前壁合わせは即収束の front_ctrl_th + 1 ms
// 写し元を変えたらここも合わせること:
//   run_main_mode() の mode_num == 0 の分岐     src/main/main_task_run.cpp
//   SearchController::exec / reset / judge_wall / go_straight_wrapper / slalom /
//   front_wall_ctrl / pivot / finish            src/search/search_controller.cpp
//   MotionPlanning::go_straight() の探索時の adachi->update()   src/action/motion_planning.cpp
//   MazeSolverBaseLgc::searchGoalPosition() の経路のたどり方(capture_route、画面表示用)
//                                                              src/search/logic.cpp
//
// 入力(stdin、JSON 1 個):
//   files  LittleFS と同じ名前 → 中身の JSON 文字列(path_sim と同じ)
//   truth  正解の迷路。ファームの並び map[x + y * n] の下位 4bit。n < maze_size なら
//          左下に置いて外側を壁で埋める
//   goals  省略時は system.txt の goals
// 出力: stdout に結果の JSON 1 行(steps と時間の内訳)。ファームの printf は stderr。

#include <algorithm>
#include <cmath>

#include "host_common.hpp"
#include "include/action/path_creator.hpp"
#include "include/search/adachi.hpp"
#include "include/ui.hpp"

// go_straight_dummy() はボタンで打ち切れる。ホストでは押されない。
bool UserInterface::button_state() { return false; }
bool UserInterface::button_state_hold() { return false; }

// 無限ループよけ(実機は seach_timer で止まるが、念のため)
static constexpr int STEP_LIMIT = 20000;

struct Step {
  ego_t from; // 判断した区画(入口の境界にいる)と向き
  ego_t to;   // adachi->exec() が決めた行き先
  char motion; // S 探索直進 / F 既知区間の直進(FastRun)/ D 既知区間→ターン(FastRunDia)/ R / L / B 後退
  double t0 = 0, t1 = 0;
  bool goal = false;
  int subgoals = 0;
  std::vector<std::pair<int, int>> changes; // 前の判断からの lgc->map の変化 (idx, 値)
  // 足立法が見ている候補の経路と、サブゴールの区画(前の判断から変わったときだけ入れる)
  bool has_route = false;
  std::string route; // スタート (0,0) からの向きの列(N / E / S / W)。空 = 候補なし
  // 比較用: 重みパターン 2〜4 ならどの経路を見るか(ファームは使わない)。変わったものだけ
  std::vector<std::pair<int, std::string>> other_routes;
  bool has_subgoals = false;
  std::vector<int> subgoal_cells; // x + y * N
};

class SearchSim : public MainTaskCopy {
public:
  std::shared_ptr<MazeSolverBaseLgc> lgc = std::make_shared<MazeSolverBaseLgc>();
  std::shared_ptr<Adachi> adachi = std::make_shared<Adachi>();
  std::shared_ptr<ego_t> ego = std::make_shared<ego_t>();
  std::shared_ptr<UserInterface> ui_ = std::make_shared<UserInterface>();
  PathCreator pc; // go_straight_dummy() だけ使う

  std::vector<uint8_t> truth; // 正解の迷路(ファームの並び、下位 4bit)
  int N = 0;
  double now = 0; // 仮想時計 [s]
  float v = 0;    // 現在の速度 [mm/s]
  bool saved = false;

  std::vector<Step> steps;
  std::vector<uint8_t> snapshot;
  // 画面に出す「候補の経路」(Adachi::update() が作った表から引いた経路)。update() のたびに取る
  std::string cur_route, sent_route;
  // 比較用の重みパターン 2〜4 の経路(ファームのサブゴール選びはパターン 1 だけ)
  static constexpr int OTHER_PATTERNS[3] = {2, 3, 4};
  std::string cur_other[3], sent_other[3];
  std::vector<int> sent_subgoals;
  std::string end_reason;
  double finish_time = 0;

  SearchSim() {
    adachi->set_logic(lgc);
    adachi->set_ego(ego);
    lgc->set_ego(ego);
    pc.set_logic(lgc);
    pc.set_userinterface(ui_);
  }

  bool truth_wall(int x, int y, Direction d) const {
    if (x < 0 || y < 0 || x >= N || y >= N)
      return true;
    return (truth[x + y * N] & static_cast<int>(d)) != 0;
  }

  static Direction turn(Direction d, bool right) {
    switch (d) {
    case Direction::North: return right ? Direction::East : Direction::West;
    case Direction::East: return right ? Direction::South : Direction::North;
    case Direction::South: return right ? Direction::West : Direction::East;
    default: return right ? Direction::North : Direction::South;
    }
  }

  // ===== 時間 =====

  void sleep(double ms) { now += ms / 1000.0; }

  void straight(float vmax, float vend, float accl, float decel, float dist) {
    if (dist <= 0)
      return;
    planning_time_t t{};
    now += pc.go_straight_dummy(v, std::fabs(vmax), std::fabs(vend), std::fabs(accl), -std::fabs(decel), dist, t);
    v = t.v_end;
  }

  void pivot_turn(float ang, float w_max, float alpha) {
    const float a = std::fabs(ang), w = std::fabs(w_max), al = std::fabs(alpha);
    now += (a >= w * w / al) ? a / w + w / al : 2.0f * std::sqrt(a / al);
    v = 0;
  }

  // SearchController::front_wall_ctrl()。理想の位置なので front_ctrl() は即収束し、
  // cnt > front_ctrl_th で抜ける。
  void front_wall_ctrl() {
    sleep(param_->sen_ref_p.search_exist.front_ctrl_th + 1);
    sleep(5);
    v = 0;
  }

  // ===== SearchController の写し =====

  void judge_wall() {
    bool wall_n = false, wall_e = false, wall_w = false, wall_s = false;
    const int x = ego->x, y = ego->y;
    const Direction d = ego->dir;
    // 前・右・左だけ見える(後ろは来た道で false のまま。実機と同じ)
    const bool front = truth_wall(x, y, d);
    const bool right = truth_wall(x, y, turn(d, true));
    const bool left = truth_wall(x, y, turn(d, false));
    if (d == Direction::North) {
      wall_n = front, wall_e = right, wall_w = left;
    } else if (d == Direction::East) {
      wall_e = front, wall_s = right, wall_n = left;
    } else if (d == Direction::West) {
      wall_w = front, wall_n = right, wall_s = left;
    } else if (d == Direction::South) {
      wall_s = front, wall_w = right, wall_e = left;
    }
    lgc->set_wall_data(x, y, Direction::North, wall_n);
    lgc->set_wall_data(x, y + 1, Direction::South, wall_n);
    lgc->set_wall_data(x, y, Direction::East, wall_e);
    lgc->set_wall_data(x + 1, y, Direction::West, wall_e);
    lgc->set_wall_data(x, y, Direction::West, wall_w);
    lgc->set_wall_data(x - 1, y, Direction::East, wall_w);
    lgc->set_wall_data(x, y, Direction::South, wall_s);
    lgc->set_wall_data(x, y - 1, Direction::North, wall_s);
    lgc->set_wall_data(0, 0, Direction::East, true);
    lgc->set_wall_data(1, 0, Direction::West, true);
  }

  // 直進は MotionPlanning::go_straight(p, adachi, true) が最初に adachi->update() を呼ぶ。
  void go_straight_wrapper(StraightType st) {
    auto &sm = param_set.str_map;
    float v_max = sm[st].v_max;
    if (v_max + 10 < v)
      v_max = sm[StraightType::FastRun].v_max;
    float v_end = sm[st].v_max;
    if (st == StraightType::FastRunDia)
      v_end = sm[StraightType::Search].v_max;
    adachi->update();
    capture_route();
    straight(v_max, v_end, sm[st].accl, sm[st].decel, param_->cell);
  }

  void slalom(bool right) {
    const auto &sp = param_set.map[TurnType::Normal];
    const auto &s = param_set.str_map[StraightType::Search];
    straight(sp.v, sp.v, s.accl, s.decel, right ? sp.front.right : sp.front.left);
    now += sp.time * 2;
    v = sp.v;
    const float back = right ? sp.back.right : sp.back.left;
    if (back > 0 && sp.v > 0)
      now += back / sp.v;
  }

  // SearchController::pivot()。判断した区画(from)の壁で分岐する。
  void pivot(const ego_t &from) {
    const auto &s = param_set.str_map[StraightType::Search];
    const bool left_exist = truth_wall(from.x, from.y, turn(from.dir, false));
    const bool right_exist = truth_wall(from.x, from.y, turn(from.dir, true));
    const bool front_wall = truth_wall(from.x, from.y, from.dir);
    // 前壁があれば back_enable(境界で前壁が pivot_back_enable_front_th より近い)。
    // 理想の位置なので前壁補正 (front_mid_dist − front_dist_offset0) は 0。
    const bool back_enable = front_wall;
    float dist = param_->pivot_straight;
    if (dist < 10)
      dist = 10;
    straight(s.v_max, 20, s.accl, s.decel, dist);
    straight(20, 5, s.accl, s.decel, 2);

    const bool flag = left_exist || right_exist;
    const float ang = (flag ? param_->pivot_angle_90 : param_->pivot_angle_180) * M_PI / 180.0;
    if (adachi->goal_step && !saved) {
      sleep(2);
      saved = true;
    }
    sleep(5);
    if (front_wall) // 中央で前壁が front_dist_offset_pivot_th より近い
      front_wall_ctrl();
    if (flag) {
      pivot_turn(ang, s.w_max, s.alpha);
      sleep(2);
      front_wall_ctrl(); // front_ctrl2: 横壁のほうを向いたので、その壁で合わせる
      sleep(1);
      straight(20, 5, s.accl, s.decel, 0.1f);
      sleep(5);
      pivot_turn(ang, s.w_max, s.alpha);
      sleep(1);
    } else {
      pivot_turn(ang, s.w_max, s.alpha);
      sleep(1);
    }
    sleep(5);
    straight(s.v_max, 10, s.accl, s.decel, back_enable ? param_->pivot_back_dist0 : param_->pivot_back_dist1);
    v = 0; // 後退して止まった
    sleep(10);
    adachi->update();
    capture_route();
    sleep(5);
    const float d2 = back_enable ? param_->cell / 2 + param_->pivot_back_dist0 - param_->pivot_back_offset
                                 : param_->cell / 2 + param_->pivot_back_dist1;
    straight(s.v_max, s.v_max, s.accl, s.decel, d2);
  }

  // Adachi::update() の直後に呼ぶ。searchGoalPosition() が表(lgc の vector_dist)をたどるのと
  // 同じ歩き方の写しで、足立法がいま「未知の壁は無いものとして最短」と見ている経路を取る
  // (この上の未知区画がサブゴールになる)。ゴール前は表を作らないので空。
  // 写し元: MazeSolverBaseLgc::searchGoalPosition()(src/search/logic.cpp)
  // いま lgc にある表(vector_dist)を、searchGoalPosition() と同じ歩き方でたどる
  std::string walk_route() {
    Direction next_dir = Direction::North;
    Direction now_dir = Direction::North;
    int x = 0, y = 1;
    Direction dirLog[3] = {now_dir, now_dir, now_dir};
    std::string r = "N"; // (0,0) → (0,1)
    for (int guard = 0; guard < N * N; guard++) {
      now_dir = next_dir;
      dirLog[2] = dirLog[1];
      dirLog[1] = dirLog[0];
      dirLog[0] = now_dir;
      next_dir = Direction::Undefined;
      if (lgc->arrival_goal_position(x, y))
        break;
      float position = lgc->getDistVector(x, y, now_dir);
      if (now_dir == Direction::North)
        position = lgc->getDistVector(x, y, Direction::South);
      else if (now_dir == Direction::East)
        position = lgc->getDistVector(x, y, Direction::West);
      else if (now_dir == Direction::West)
        position = lgc->getDistVector(x, y, Direction::East);
      else if (now_dir == Direction::South)
        position = lgc->getDistVector(x, y, Direction::North);
      lgc->setNextRootDirectionPathUnKnown(x, y, Direction::North, now_dir, next_dir, position);
      lgc->setNextRootDirectionPathUnKnown(x, y, Direction::East, now_dir, next_dir, position);
      lgc->setNextRootDirectionPathUnKnown(x, y, Direction::West, now_dir, next_dir, position);
      lgc->setNextRootDirectionPathUnKnown(x, y, Direction::South, now_dir, next_dir, position);
      if (dirLog[0] == dirLog[1] || dirLog[0] != dirLog[2])
        lgc->priorityStraight2(x, y, now_dir, dirLog[0], position, next_dir);
      else
        lgc->priorityStraight2(x, y, now_dir, dirLog[1], position, next_dir);
      if (next_dir == Direction::North)
        y++, r += 'N';
      else if (next_dir == Direction::East)
        x++, r += 'E';
      else if (next_dir == Direction::West)
        x--, r += 'W';
      else if (next_dir == Direction::South)
        y--, r += 'S';
      else
        break;
    }
    return r;
  }

  void capture_route() {
    cur_route.clear();
    for (auto &r : cur_other)
      r.clear();
    if (!(adachi->goal_step && adachi->sm == SearchMode::ALL))
      return;
    cur_route = walk_route(); // ファームが update() で作った表(重みパターン 1)
    // 比較用: ほかの重みパターンの表を作ってたどる。最後にパターン 1 の表を作り直して、
    // lgc をファームが update() した直後と同じ状態へ戻す(次の update() が前の表を見るため)
    std::unordered_map<unsigned int, unsigned char> tmp;
    for (int i = 0; i < 3; i++) {
      tmp.clear();
      lgc->set_param_num(OTHER_PATTERNS[i]);
      lgc->set_param();
      lgc->searchGoalPosition(true, tmp);
      cur_other[i] = walk_route();
    }
    tmp = adachi->subgoal_list;
    lgc->set_param_num(1);
    lgc->set_param();
    lgc->searchGoalPosition(true, tmp);
  }

  void record(const ego_t &from, char motion) {
    Step st;
    st.from = from;
    st.to = *ego;
    st.motion = motion;
    st.t0 = now;
    st.goal = adachi->goal_step;
    st.subgoals = (int)adachi->subgoal_list.size();
    // この判断が使った候補の経路(直前の update() のもの)とサブゴール。変わったときだけ出す
    if (cur_route != sent_route || steps.empty()) {
      st.has_route = true;
      st.route = cur_route;
      sent_route = cur_route;
    }
    for (int i = 0; i < 3; i++) {
      if (cur_other[i] != sent_other[i] || steps.empty()) {
        st.other_routes.emplace_back(OTHER_PATTERNS[i], cur_other[i]);
        sent_other[i] = cur_other[i];
      }
    }
    std::vector<int> sg;
    for (const auto &kv : adachi->subgoal_list)
      sg.push_back((int)kv.first);
    std::sort(sg.begin(), sg.end());
    if (sg != sent_subgoals || steps.empty()) {
      st.has_subgoals = true;
      st.subgoal_cells = sg;
      sent_subgoals = sg;
    }
    for (int i = 0; i < (int)lgc->map.size(); i++) {
      if (lgc->map[i] != snapshot[i]) {
        st.changes.emplace_back(i, lgc->map[i]);
        snapshot[i] = lgc->map[i];
      }
    }
    steps.push_back(std::move(st));
  }

  // run_main_mode() の mode_num == 0 → SearchController::exec(param_set, SearchMode::ALL)
  void run(const std::vector<point_t> &goals) {
    lgc->init(sys_.maze_size, sys_.maze_size * sys_.maze_size - 1);
    lgc->set_goal_pos(goals);
    load_slalom_param(0, 0, 0);
    param_set.cell_size = param_->cell;
    snapshot.assign(lgc->map.size(), 0);

    adachi->lgc->reset_dist_map();
    // reset()
    ego->dir = Direction::North;
    ego->x = 0;
    ego->y = 1;
    ego->prev_motion = 0;
    lgc->set_default_wall_data();
    adachi->reset_goal();
    for (const auto p : lgc->goal_list)
      lgc->map[p.x + p.y * lgc->maze_size] = lgc->map[p.x + p.y * lgc->maze_size] & 0x0f;

    const auto &s = param_set.str_map[StraightType::Search];
    ego_t start = *ego;
    start.y = 0;
    record(start, 'S');
    straight(s.v_max, s.v_max, s.accl, s.decel, param_->cell / 2 + param_->offset_start_dist_search);
    steps.back().t1 = now;

    int back_cnt = 0;
    adachi->sm = SearchMode::ALL;
    while (true) {
      if ((int)steps.size() > STEP_LIMIT) {
        end_reason = "step_limit";
        break;
      }
      const bool front_is_stepped = lgc->is_front_cell_stepped(ego->x, ego->y, ego->dir);
      judge_wall();
      const ego_t from = *ego;
      const Motion next = adachi->exec(false, false);
      adachi->diff = 0;

      if (next == Motion::Straight) {
        Motion next2 = Motion::NONE;
        if (front_is_stepped)
          next2 = adachi->exec(true, false);
        const StraightType st = !front_is_stepped            ? StraightType::Search
                                : next2 == Motion::Straight ? StraightType::FastRun
                                                            : StraightType::FastRunDia;
        record(from, st == StraightType::Search ? 'S' : st == StraightType::FastRun ? 'F' : 'D');
        go_straight_wrapper(st);
      } else if (next == Motion::TurnRight || next == Motion::TurnLeft) {
        record(from, next == Motion::TurnRight ? 'R' : 'L');
        slalom(next == Motion::TurnRight);
      } else if (next == Motion::Back) {
        record(from, 'B');
        pivot(from);
      } else {
        end_reason = "none"; // adachi が NONE(スタートへ戻って打ち切り)
        break;
      }
      steps.back().t1 = now;

      back_cnt = next == Motion::Back ? back_cnt + 1 : 0;
      if (back_cnt >= 4) {
        end_reason = "back4";
        break;
      }
      if (adachi->goal_step && ego->x == 0 && ego->y == 0) {
        end_reason = "home";
        break;
      }
      if (now > param_->seach_timer) {
        end_reason = "timeup";
        break;
      }
    }
    // finish()
    const double t = now;
    straight(s.v_max, 10, s.accl, s.decel, param_->cell / 2 - 5);
    finish_time = now - t;
  }
};

int main() {
  host::JsonOut jo;
  JsonDocument &out = jo.doc;
  JsonDocument in;
  if (!host::read_input(in, out))
    return jo.finish(1);

  SearchSim sim;
  sim.load_params();
  sim.load_turn_param_profiles();

  std::vector<uint8_t> truth;
  for (JsonVariantConst v : in["truth"].as<JsonArrayConst>())
    truth.push_back(v.as<int>() & 0x0f);
  sim.N = sim.sys_.maze_size;
  // 小さい迷路は左下に置き、外側は全部壁(実機を maze_size のまま走らせるのと同じ)
  sim.truth = host::embed_maze(truth, sim.N, 0x0f);
  if (sim.truth.empty()) {
    out["ok"] = false;
    out["error"] = "迷路 (" + std::to_string(truth.size()) + " マス) が system の maze_size=" +
                   std::to_string(sim.N) + " より大きいか、正方形ではありません";
    return jo.finish(1);
  }
  std::vector<point_t> goals = sim.sys_.goals;
  if (in["goals"].is<JsonArrayConst>()) {
    goals.clear();
    for (JsonVariantConst g : in["goals"].as<JsonArrayConst>()) {
      point_t p{};
      convertFromJson(g, p);
      goals.push_back(p);
    }
  }

  sim.run(goals);

  out["ok"] = true;
  out["maze_size"] = sim.N;
  out["end_reason"] = sim.end_reason;
  out["total_time"] = sim.now;
  out["finish_time"] = sim.finish_time;
  out["search_timer"] = sim.param_->seach_timer;
  const auto &s = sim.param_set.str_map[StraightType::Search];
  const auto &nt = sim.param_set.map[TurnType::Normal];
  JsonObject prm = out["params"].to<JsonObject>();
  prm["search_v"] = s.v_max;
  prm["search_accl"] = s.accl;
  prm["fast_v"] = sim.param_set.str_map[StraightType::FastRun].v_max;
  prm["turn_v"] = nt.v;
  prm["turn_time"] = nt.time * 2;
  prm["w_max"] = s.w_max;
  prm["alpha"] = s.alpha;
  if (!sim.tpp.file_list.empty()) {
    const int fi = sim.tpp.profile_map[0][TurnType::Normal];
    if (fi >= 0 && fi < (int)sim.tpp.file_list.size())
      prm["turn_file"] = sim.tpp.file_list[fi];
  }

  // ゴール区画に最初に入った判断の時刻。1 区画だけ残ったゴールは 4 辺が分かれば
  // 入らずに到達扱い(Adachi::goal_step_check)なので、入らなかったときは判定した時刻。
  double goal_time = -1;
  const char *goal_by = "none";
  for (const auto &st : sim.steps) {
    for (const auto &g : goals) {
      if (goal_time < 0 && st.from.x == g.x && st.from.y == g.y)
        goal_time = st.t0, goal_by = "enter";
    }
  }
  if (goal_time < 0) {
    for (const auto &st : sim.steps) {
      if (st.goal) {
        goal_time = st.t0, goal_by = "known";
        break;
      }
    }
  }
  out["goal_time"] = goal_time;
  out["goal_by"] = goal_by;

  JsonArray steps = out["steps"].to<JsonArray>();
  for (const auto &st : sim.steps) {
    JsonObject o = steps.add<JsonObject>();
    JsonArray f = o["f"].to<JsonArray>();
    f.add(st.from.x), f.add(st.from.y), f.add(static_cast<int>(st.from.dir));
    JsonArray t = o["to"].to<JsonArray>();
    t.add(st.to.x), t.add(st.to.y), t.add(static_cast<int>(st.to.dir));
    o["m"] = std::string(1, st.motion);
    o["t0"] = st.t0;
    o["t1"] = st.t1;
    o["g"] = st.goal;
    o["sg"] = st.subgoals;
    if (st.has_route)
      o["r"] = st.route;
    if (!st.other_routes.empty()) {
      JsonObject rp = o["rp"].to<JsonObject>();
      for (const auto &[pn, r] : st.other_routes)
        rp[std::to_string(pn)] = r;
    }
    if (st.has_subgoals) {
      JsonArray sg = o["s"].to<JsonArray>();
      for (const int v : st.subgoal_cells)
        sg.add(v);
    }
    JsonArray c = o["c"].to<JsonArray>();
    for (const auto &[i, val] : st.changes) {
      JsonArray pair = c.add<JsonArray>();
      pair.add(i), pair.add(val);
    }
  }
  return jo.finish(0);
}
