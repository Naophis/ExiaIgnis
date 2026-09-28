#pragma once
// path_sim / search_sim 共通。
// - 入力 JSON の files(LittleFS と同じ名前 → 中身)を LittleFS の代わりにする
//   ConfigLoader::load_file(host_common.cpp)
// - ファームの printf を stderr へ回し、stdout を結果の JSON だけにする JsonOut
// - MainTask の読込関数の写し MainTaskCopy。写し元を変えたらここも合わせること:
//     load_params()                     src/main/main_task.cpp
//     load_turn_param_profiles(false,0) src/main/main_task_util.cpp
//     exec_param_prof()                 src/main/main_task_run_profile.cpp
//     load_slalom_param/load_slas/load_straight  src/main/main_task_util.cpp
//     turn_name_list / straight_name_list / cast_turn_type  include/main/main_task.hpp

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "include/config_loader.hpp"
#include "include/config_mapping.hpp"
#include "include/defines.hpp"

namespace host {

// ファームの並び(map[x + y * n])の迷路を、maze_size の地図の左下に置く。外側は fill で埋める
// (maze_size = 32 の設定のまま 16x16 の迷路を走るのと同じ)。大きすぎる・正方形でないなら空。
std::vector<uint8_t> embed_maze(const std::vector<uint8_t> &src, int maze_size, uint8_t fill);

// 入力 JSON を読み、files を ConfigLoader::load_file から見えるようにする。
// 失敗したら out に ok=false / error を書いて false。
bool read_input(JsonDocument &in, JsonDocument &out);

// 生成時に stdout を stderr へ付け替え(ファームの printf 用)、finish() で結果の
// JSON を元の stdout へ 1 行で書く。
class JsonOut {
public:
  JsonOut();
  int finish(int code);
  JsonDocument doc;

private:
  int fd_;
};

} // namespace host

class MainTaskCopy {
public:
  std::shared_ptr<input_param_t> param_ = std::make_shared<input_param_t>();
  system_t sys_;
  turn_param_profile_t tpp;
  std::unordered_map<TurnType, int> p_idx;
  std::vector<exec_pram_t> exec_param_list;
  param_set_t param_set;
  bool silent_load = false;

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
};
