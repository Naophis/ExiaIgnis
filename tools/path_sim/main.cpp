// path_sim — MainTask::path_run() の経路生成(走行の手前まで)をホストで再現する。
//
// 経路生成そのもの(PathCreator / MazeSolverBaseLgc / TrajectoryCreator)は
// ファームのソースをそのままビルドする(Makefile)。ここにあるのは、Pico の
// 周辺機能ごとでないと持ち出せない MainTask のメンバー関数の写しだけ
// (読込関数は host_common.hpp の MainTaskCopy)。写し元を変えたらここも合わせること:
//   run_main_mode() の lgc 初期化 + read_maze_data()  src/main/main_task_run*.cpp
//   path_run() の exec_path_running() より前      src/main/main_task_run.cpp
//
// 入力(stdin、JSON 1 個):
//   files     LittleFS と同じ名前 → 中身の JSON 文字列("hardware.txt", "t_1200.hf" …)。
//             Param Console が機体へ送るときと同じ変換(yaml → JSON)をしたもの。
//   map       ファームの並び map[x + y * n] の 1 バイト(/maze.txt と同じ)。n < maze_size なら
//             左下に置いて外側を壁で埋める
//   exec      run_prf の exec_prof の番号(メインモードの mode_num - 2)
//   direction "right" = タイムで候補を比べる / "left" = 単純な経路
//             (path_run の ui_->select_direction())
//   goals     省略時は system.txt の goals
// 出力: stdout に結果の JSON 1 行。ファームの printf は stderr(実機のコンソールと同じ内容)。

#include "host_common.hpp"
#include "include/action/path_creator.hpp"
#include "include/ui.hpp"


// 実機はボタンで計算を打ち切れる(path_create / timebase_path_create /
// go_straight_dummy)。ホストでは押されない。
bool UserInterface::button_state() { return false; }
bool UserInterface::button_state_hold() { return false; }

// ===== MainTask の写し =====

class PathSim : public MainTaskCopy {
public:

  std::shared_ptr<MazeSolverBaseLgc> lgc = std::make_shared<MazeSolverBaseLgc>();
  std::shared_ptr<PathCreator> pc = std::make_shared<PathCreator>();
  std::shared_ptr<UserInterface> ui_ = std::make_shared<UserInterface>();

  // path_run() の候補(timebase_path_create の結果)。ファームは捨てるだけ。
  std::vector<path_set_t> candidates;
  int selected_type = -1; // -1 = 候補が全部失敗して単純な経路に戻った / left
  std::string error;


  // run_main_mode() の lgc 初期化と read_maze_data()。
  void setup_maze(const std::vector<uint8_t> &map, const std::vector<point_t> &goals) {
    lgc->init(sys_.maze_size, sys_.maze_size * sys_.maze_size - 1);
    lgc->set_goal_pos(goals);
    pc->set_logic(lgc);
    pc->set_userinterface(ui_);
    for (int i = 0; i < (int)map.size(); i++)
      lgc->set_native_wall_data(i, map[i]);
  }

  // path_run() が失敗で ui_->error() して return する経路は false。
  bool simple_path(const char *what) {
    pc->other_route_map.clear();
    if (!pc->path_create(false)) {
      error = std::string("path_create に失敗 (") + what + ")";
      return false;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
    return true;
  }

  bool path_run(int idx, int idx2, int idx3, bool right) {
    load_slalom_param(idx, idx2, idx3);
    param_set.cell_size = param_->cell;
    param_set.start_offset = param_->offset_start_dist;
    if (sys_.circuit_mode != 0) {
      error = "circuit_mode=1(load_circuit_path)は未対応";
      return false;
    }
    if (right) {
      //速度ベース経路導出
      for (int i = 1; i <= 5; i++) {
        lgc->set_param_num(i);
        pc->other_route_map.clear();
        const bool res = pc->path_create(false);
        printf("other route size = %d\n", (int)pc->other_route_map.size());
        if (!res) {
          error = "path_create に失敗 (param_num=" + std::to_string(i) + ")";
          return false;
        }
        pc->convert_large_path(true);
        pc->diagonalPath(true, true);
        // (ファームの `if (i == 0)` のブロックはループが 1 からなので通らない)
        path_set_t p;
        p.type = i;
        p.time = 10000;
        pc->timebase_path_create(false, param_set, p);
        pc->path_set_map.push(p);
      }
      // 先頭に最適な経路を持ってくる
      const auto top_p = pc->path_set_map.top();
      while (!pc->path_set_map.empty()) {
        candidates.push_back(pc->path_set_map.top());
        pc->path_set_map.pop();
      }
      if (top_p.result) { //成功
        pc->path_s.assign(top_p.path_s.begin(), top_p.path_s.end());
        pc->path_t.assign(top_p.path_t.begin(), top_p.path_t.end());
        selected_type = top_p.type;
      } else { //失敗
        if (!simple_path("候補がすべて失敗した後"))
          return false;
      }
    } else {
      if (!simple_path("left"))
        return false;
      pc->print_path();
    }
    printf("----------------\n");
    if (pc->path_s.size() == 0) {
      if (!simple_path("経路が空"))
        return false;
      pc->print_path();
    }
    pc->calc_goal_time(param_set, true);
    pc->print_path();
    return true;
  }
};

// ===== 入出力 =====

// path_t の終端(255 か 0)まで。その先は pathOffset() の詰め残り。
static size_t path_len(const std::vector<unsigned char> &t) {
  for (size_t i = 0; i < t.size(); i++)
    if (t[i] == 255 || t[i] == 0)
      return i + 1;
  return t.size();
}

static void put_path(JsonObject o, const std::vector<float> &path_s,
                     const std::vector<unsigned char> &path_t) {
  const size_t n = std::min(path_s.size(), path_len(path_t));
  JsonArray s = o["path_s"].to<JsonArray>();
  JsonArray t = o["path_t"].to<JsonArray>();
  for (size_t i = 0; i < n; i++) {
    s.add(path_s[i]);
    t.add((int)path_t[i]);
  }
}

// load_slalom_param() が読んだ値(fast / normal / slow の 3 組)と直線(normal の vel_prof)。
static void put_params(JsonObject out, PathSim &sim, const exec_pram_t &ep) {
  const std::pair<const char *, int> sets[] = {
      {"fast", ep.fast_idx}, {"normal", ep.normal_idx}, {"slow", ep.slow_idx}};
  const std::unordered_map<TurnType, slalom_param2_t> *maps[] = {
      &sim.param_set.map_fast, &sim.param_set.map, &sim.param_set.map_slow};
  JsonObject turns = out["turn_params"].to<JsonObject>();
  for (int k = 0; k < 3; k++) {
    JsonArray arr = turns[sets[k].first].to<JsonArray>();
    for (const auto &p : sim.turn_name_list) {
      if (p.first == TurnType::None)
        continue;
      JsonObject o = arr.add<JsonObject>();
      o["type"] = p.second;
      const int file_idx = sim.tpp.profile_map[sets[k].second][p.first];
      if (file_idx >= 0 && file_idx < (int)sim.tpp.file_list.size())
        o["file"] = sim.tpp.file_list[file_idx];
      const auto it = maps[k]->find(p.first);
      if (it == maps[k]->end())
        continue; // ファイルにこのターンの項目が無い
      const auto &sp = it->second;
      o["v"] = sp.v;
      o["end_v"] = sp.end_v;
      o["ang"] = sp.ang * 180.0f / m_PI;
      o["rad"] = sp.rad;
      o["rad2"] = sp.rad2;
      o["pow_n"] = sp.pow_n;
      o["time"] = sp.time;
      o["time2"] = sp.time2;
      o["front_l"] = sp.front.left;
      o["front_r"] = sp.front.right;
      o["back_l"] = sp.back.left;
      o["back_r"] = sp.back.right;
    }
  }
  JsonObject str = out["straight_params"].to<JsonObject>();
  for (const auto &p : sim.straight_name_list) {
    const auto it = sim.param_set.str_map.find(p.first);
    if (it == sim.param_set.str_map.end())
      continue;
    JsonObject o = str[p.second].to<JsonObject>();
    o["v_max"] = it->second.v_max;
    o["accl"] = it->second.accl;
    o["decel"] = it->second.decel;
  }
}

int main() {
  host::JsonOut jo; // ファームの printf はすべて stderr へ。stdout は結果の JSON だけ
  JsonDocument &out = jo.doc;
  const auto finish = [&](int code) { return jo.finish(code); };
  JsonDocument in;
  if (!host::read_input(in, out))
    return finish(1);

  PathSim sim;
  sim.load_params();
  sim.load_turn_param_profiles();
  sim.exec_param_prof();

  const int exec = in["exec"] | 0;
  const bool right = std::string(in["direction"] | "right") != "left";
  out["exec"]["index"] = exec;
  out["direction"] = right ? "right" : "left";
  if (exec < 0 || exec >= (int)sim.exec_param_list.size()) {
    out["ok"] = false;
    out["error"] = "exec が run_prf の範囲外です (exec_prof は " +
                   std::to_string(sim.exec_param_list.size()) + " 件)";
    return finish(1);
  }
  const auto ep = sim.exec_param_list[exec];
  out["exec"]["fast"] = (int)ep.fast_idx;
  out["exec"]["normal"] = (int)ep.normal_idx;
  out["exec"]["slow"] = (int)ep.slow_idx;

  std::vector<uint8_t> src;
  for (JsonVariantConst v : in["map"].as<JsonArrayConst>())
    src.push_back(v.as<int>() & 0xff);
  // 小さい迷路は左下に置き、外側は全部壁・踏破済み(実機を maze_size のまま走らせるのと同じ)
  const std::vector<uint8_t> map = host::embed_maze(src, sim.sys_.maze_size, 0xff);
  if (map.empty()) {
    out["ok"] = false;
    out["error"] = "迷路 (" + std::to_string(src.size()) + " マス) が system の maze_size=" +
                   std::to_string(sim.sys_.maze_size) + " より大きいか、正方形ではありません";
    return finish(1);
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
  sim.setup_maze(map, goals);

  const bool ok = sim.path_run(ep.fast_idx, ep.normal_idx, ep.slow_idx, right);
  out["ok"] = ok;
  put_params(out.as<JsonObject>(), sim, ep);
  if (!ok) {
    out["error"] = sim.error;
    return finish(1);
  }

  out["suction"] = (int)sim.param_set.suction;
  out["cell_size"] = sim.param_set.cell_size;
  out["start_offset"] = sim.param_set.start_offset;
  out["selected_type"] = sim.selected_type;
  JsonArray cands = out["candidates"].to<JsonArray>();
  for (const auto &c : sim.candidates) {
    JsonObject o = cands.add<JsonObject>();
    o["type"] = (int)c.type;
    o["result"] = c.result;
    o["time"] = c.time;
    put_path(o, c.path_s, c.path_t);
  }
  put_path(out.as<JsonObject>(), sim.pc->path_s, sim.pc->path_t);

  // calc_goal_time() の区間ごとの内訳(print_path() が出しているもの)
  const auto &pc = *sim.pc;
  JsonArray segs = out["segments"].to<JsonArray>();
  for (size_t i = 0; i < pc.path_time_total.size(); i++) {
    JsonObject o = segs.add<JsonObject>();
    o["str_time"] = pc.path_time_s[i];
    o["turn_time"] = pc.path_time_t[i];
    o["total_time"] = pc.path_time_total[i].total_time;
    o["v_start"] = pc.path_time_total[i].v_start;
    o["v_max"] = pc.path_time_total[i].v_max;
    o["v_end"] = pc.path_time_total[i].v_end;
    o["dist"] = pc.path_time_total[i].dist;
  }
  out["goal_time"] = pc.path_time_total.empty() ? 0.0f : pc.path_time_total.back().total_time;
  return finish(0);
}
