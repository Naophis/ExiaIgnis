// path_sim — MainTask::path_run() の経路生成(走行の手前まで)をホストで再現する。
//
// 経路生成そのもの(PathCreator / MazeSolverBaseLgc / TrajectoryCreator)は
// ファームのソースをそのままビルドする(Makefile)。ここにあるのは、Pico の
// 周辺機能ごとでないと持ち出せない MainTask のメンバー関数の写しだけ。
// 写し元を変えたらここも合わせること:
//   load_params()                     src/main/main_task.cpp
//   load_turn_param_profiles(false,0) src/main/main_task_util.cpp
//   exec_param_prof()                 src/main/main_task_run_profile.cpp
//   load_slalom_param/load_slas/load_straight  src/main/main_task_util.cpp
//   run_main_mode() の lgc 初期化 + read_maze_data()  src/main/main_task_run*.cpp
//   path_run() の exec_path_running() より前      src/main/main_task_run.cpp
//   turn_name_list / straight_name_list / cast_turn_type  include/main/main_task.hpp
//
// 入力(stdin、JSON 1 個):
//   files     LittleFS と同じ名前 → 中身の JSON 文字列("hardware.txt", "t_1200.hf" …)。
//             Param Console が機体へ送るときと同じ変換(yaml → JSON)をしたもの。
//   map       ファームの並び map[x + y * maze_size] の 1 バイト(/maze.txt と同じ)
//   exec      run_prf の exec_prof の番号(メインモードの mode_num - 2)
//   direction "right" = タイムで候補を比べる / "left" = 単純な経路
//             (path_run の ui_->select_direction())
//   goals     省略時は system.txt の goals
// 出力: stdout に結果の JSON 1 行。ファームの printf は stderr(実機のコンソールと同じ内容)。

#include <cstdio>
#include <iostream>
#include <iterator>
#include <string>
#include <unistd.h>

#include "include/action/path_creator.hpp"
#include "include/config_loader.hpp"
#include "include/config_mapping.hpp"
#include "include/ui.hpp"

// ===== ホスト用の差し替え =====

static std::unordered_map<std::string, std::string> g_files;

bool ConfigLoader::load_file(const char *path, JsonDocument &dst) {
  std::string name = path;
  if (!name.empty() && name[0] == '/')
    name.erase(0, 1);
  const auto it = g_files.find(name);
  if (it == g_files.end())
    return false;
  const auto err = deserializeJson(dst, it->second);
  if (err) {
    printf("[path_sim] %s: JSON parse error: %s\n", path, err.c_str());
    return false;
  }
  return true;
}

// 実機はボタンで計算を打ち切れる(path_create / timebase_path_create /
// go_straight_dummy)。ホストでは押されない。
bool UserInterface::button_state() { return false; }
bool UserInterface::button_state_hold() { return false; }

// ===== MainTask の写し =====

class PathSim {
public:
  std::shared_ptr<input_param_t> param_ = std::make_shared<input_param_t>();
  system_t sys_;
  turn_param_profile_t tpp;
  std::unordered_map<TurnType, int> p_idx;
  std::vector<exec_pram_t> exec_param_list;
  param_set_t param_set;
  bool silent_load = false;

  std::shared_ptr<MazeSolverBaseLgc> lgc = std::make_shared<MazeSolverBaseLgc>();
  std::shared_ptr<PathCreator> pc = std::make_shared<PathCreator>();
  std::shared_ptr<UserInterface> ui_ = std::make_shared<UserInterface>();

  // path_run() の候補(timebase_path_create の結果)。ファームは捨てるだけ。
  std::vector<path_set_t> candidates;
  int selected_type = -1; // -1 = 候補が全部失敗して単純な経路に戻った / left
  std::string error;

  std::vector<std::pair<TurnType, std::string>> turn_name_list = {
      {TurnType::None, "straight"},    //
      {TurnType::Normal, "normal"},    //
      {TurnType::Large, "large"},      //
      {TurnType::Orval, "orval"},      //
      {TurnType::Dia45, "dia45"},      //
      {TurnType::Dia135, "dia135"},    //
      {TurnType::Dia90, "dia90"},      //
      {TurnType::Dia45_2, "dia45_2"},  //
      {TurnType::Dia135_2, "dia135_2"} //
  };
  std::vector<std::pair<StraightType, std::string>> straight_name_list = {
      {StraightType::Search, "search"}, //
      {StraightType::FastRun, "fast"},  //
      {StraightType::FastRunDia, "dia"} //
  };

  TurnType cast_turn_type(const std::string &str) {
    if (str == "normal")
      return TurnType::Normal;
    if (str == "large")
      return TurnType::Large;
    if (str == "orval")
      return TurnType::Orval;
    if (str == "dia45")
      return TurnType::Dia45;
    if (str == "dia45_2")
      return TurnType::Dia45_2;
    if (str == "dia135")
      return TurnType::Dia135;
    if (str == "dia135_2")
      return TurnType::Dia135_2;
    if (str == "dia90")
      return TurnType::Dia90;
    return TurnType::None;
  }

  // load_params() のうち経路に関わる部分(load_param_after() は走行制御の設定)。
  void load_params() {
    ConfigLoader::load_as("/hardware.txt", *param_);
    ConfigLoader::load_as("/sensor.hf", *param_);
    ConfigLoader::load_as("/offset.hf", *param_);
    ConfigLoader::load_as("/system.txt", sys_);
  }

  void load_turn_param_profiles() {
    const char *fileName = (sys_.hf_cl == 0) ? "/profiles.hf" : "/profiles.cl";
    JsonDocument doc;
    if (!ConfigLoader::load_file(fileName, doc)) {
      printf("[main] %s not found, profiles empty.\n", fileName);
      return;
    }
    tpp.file_list.clear();
    tpp.file_list_size = 0;
    for (JsonVariantConst v : doc["list"].as<JsonArrayConst>()) {
      tpp.file_list.emplace_back(v.as<const char *>());
      tpp.file_list_size++;
    }
    tpp.profile_idx_size = doc["profile_idx_size"] | 0;
    tpp.profile_map.clear();
    int i = 0;
    for (JsonVariantConst entry : doc["profile_idx"].as<JsonArrayConst>()) {
      p_idx[TurnType::None] = entry["run_param"] | 0;
      p_idx[TurnType::Finish] = entry["suction"] | 0;
      p_idx[TurnType::Normal] = entry["normal"] | 0;
      p_idx[TurnType::Large] = entry["large"] | 0;
      p_idx[TurnType::Orval] = entry["orval"] | 0;
      p_idx[TurnType::Dia45] = entry["dia45"] | 0;
      p_idx[TurnType::Dia45_2] = entry["dia45_2"] | 0;
      p_idx[TurnType::Dia135] = entry["dia135"] | 0;
      p_idx[TurnType::Dia135_2] = entry["dia135_2"] | 0;
      p_idx[TurnType::Dia90] = entry["dia90"] | 0;
      tpp.profile_map[i++] = p_idx;
    }
  }

  void exec_param_prof() {
    const char *fileName = (sys_.hf_cl == 0) ? "/run_prf.hf" : "/run_prf.cl";
    JsonDocument doc;
    if (!ConfigLoader::load_file(fileName, doc)) {
      printf("not found\n");
      return;
    }
    exec_param_list.clear();
    for (JsonVariantConst item : doc["exec_prof"].as<JsonArrayConst>()) {
      exec_pram_t ep{};
      ep.fast_idx = item["fast"] | 0;
      ep.normal_idx = item["normal"] | 0;
      ep.slow_idx = item["slow"] | 0;
      exec_param_list.emplace_back(ep);
    }
  }

  void load_slalom_param(int idx, int idx2, int idx3) {
    printf("load_slalom_param: %d, %d, %d\n", idx, idx2, idx3);
    param_set.suction = tpp.profile_map[idx][TurnType::Finish];
    if (param_set.suction == 1) {
      param_set.suction_duty = sys_.test.suction_duty;
      param_set.suction_duty_low = sys_.test.suction_duty_low;
    } else if (param_set.suction == 2) {
      param_set.suction_duty = sys_.test.suction_duty_burst;
      param_set.suction_duty_low = sys_.test.suction_duty_burst_low;
    }
    param_set.map.clear();
    param_set.map_slow.clear();
    param_set.map_fast.clear();
    param_set.str_map.clear();

    const auto load_set = [&](int profile, std::unordered_map<TurnType, slalom_param2_t> &dst) {
      std::unordered_map<int, std::vector<std::pair<TurnType, std::string>>> turn_map;
      for (const auto &p : turn_name_list) {
        if (p.first == TurnType::None)
          continue;
        turn_map[tpp.profile_map[profile][p.first]].emplace_back(p);
      }
      for (auto &kv : turn_map)
        load_slas(kv.first, kv.second, dst);
    };
    load_set(idx, param_set.map_fast); // fast
    printf("------\n");
    load_set(idx2, param_set.map); // normal
    printf("------\n");
    load_set(idx3, param_set.map_slow); // slow

    load_straight(idx2, param_set.str_map);
  }

  void load_slas(int idx, std::vector<std::pair<TurnType, std::string>> &turn_list,
                 std::unordered_map<TurnType, slalom_param2_t> &sla_map) {
    if (idx < 0 || idx >= (int)tpp.file_list.size()) {
      printf("[main] load_slas: idx=%d out of range (size=%d)\n", idx,
             (int)tpp.file_list.size());
      return;
    }
    const auto &file_name = tpp.file_list[idx];
    const auto path = std::string("/") + file_name;
    JsonDocument doc;
    if (!ConfigLoader::load_file(path.c_str(), doc)) {
      printf("[main] load_slas: %s not found\n", path.c_str());
      return;
    }
    if (!silent_load)
      printf("%s\n", file_name.c_str());
    for (const auto &p : turn_list) {
      JsonVariantConst entry = doc[p.second.c_str()];
      if (entry.isNull())
        continue;
      slalom_param2_t sp{};
      convertFromJson(entry, sp);
      sp.ref_ang = sp.ang;
      sp.type = cast_turn_type(p.second);
      sla_map[p.first] = sp;
      if (!silent_load) {
        printf(" - %s: v=%f  rad=%f  time=%f", p.second.c_str(), sp.v, sp.rad, sp.time);
        if (p.first == TurnType::Orval)
          printf("  rad2=%f  time2=%f", sp.rad2, sp.time2);
        printf("  front=[%0.2f, %0.2f]  back=[%0.2f, %0.2f]\n", sp.front.left,
               sp.front.right, sp.back.left, sp.back.right);
      }
    }
  }

  void load_straight(int idx, std::unordered_map<StraightType, straight_param_t> &str_map) {
    const char *fileName = (sys_.hf_cl == 0) ? "/vel_prof.hf" : "/vel_prof.cl";
    JsonDocument doc;
    if (!ConfigLoader::load_file(fileName, doc)) {
      printf("[main] load_straight: %s not found\n", fileName);
      return;
    }
    JsonArrayConst vel_prof = doc["v_prof"].as<JsonArrayConst>();
    if (idx < 0 || idx >= (int)vel_prof.size()) {
      printf("[main] load_straight: idx=%d out of range (size=%d)\n", idx,
             (int)vel_prof.size());
      return;
    }
    JsonVariantConst entry = vel_prof[idx];
    for (const auto &p : straight_name_list) {
      JsonVariantConst sp_json = entry[p.second.c_str()];
      if (sp_json.isNull())
        continue;
      straight_param_t sp{};
      convertFromJson(sp_json, sp);
      str_map[p.first] = sp;
      if (!silent_load) {
        printf("[%d][%s]: v_max=%4.1f  accl=%4.1f  decel=%4.1f"
               "  w_max=%4.1f  w_end=%4.1f  alpha=%4.1f\n",
               idx, p.second.c_str(), sp.v_max, sp.accl, sp.decel, sp.w_max,
               sp.w_end, sp.alpha);
      }
    }
  }

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
  // ファームの printf はすべて stderr へ。stdout は結果の JSON だけにする。
  fflush(stdout);
  const int json_fd = dup(STDOUT_FILENO);
  dup2(STDERR_FILENO, STDOUT_FILENO);

  JsonDocument out;
  const auto finish = [&](int code) {
    fflush(stdout);
    std::string text;
    serializeJson(out, text);
    text += "\n";
    if (write(json_fd, text.data(), text.size()) < 0)
      return 2;
    return code;
  };

  const std::string input((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
  JsonDocument in;
  if (const auto err = deserializeJson(in, input)) {
    out["ok"] = false;
    out["error"] = std::string("入力 JSON を読めません: ") + err.c_str();
    return finish(1);
  }
  for (JsonPairConst kv : in["files"].as<JsonObjectConst>()) {
    if (kv.value().is<const char *>())
      g_files[kv.key().c_str()] = kv.value().as<const char *>();
    else
      serializeJson(kv.value(), g_files[kv.key().c_str()]);
  }

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

  std::vector<uint8_t> map;
  for (JsonVariantConst v : in["map"].as<JsonArrayConst>())
    map.push_back(v.as<int>() & 0xff);
  if ((int)map.size() != sim.sys_.maze_size * sim.sys_.maze_size) {
    out["ok"] = false;
    out["error"] = "迷路の大きさ (" + std::to_string(map.size()) + " マス) が system の maze_size=" +
                   std::to_string(sim.sys_.maze_size) + " と合いません";
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
