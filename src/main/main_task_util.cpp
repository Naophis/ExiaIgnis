#include "config_loader.hpp"
#include "config_mapping.hpp"
#include "define.hpp"
#include "main/main_task.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

// ─── exec パラメータロード ────────────────────────────────────────────────────
// /exec.json (JSON 配列) を読み込んで exec_param_list を構築する。
// 本走行・テスト走行の両モードに入る前に呼ぶこと。
bool MainTask::load_exec_params() {
  JsonDocument doc;
  if (!ConfigLoader::load_file("/exec.json", doc)) {
    printf("[main] /exec.json not found, exec_param_list empty.\n");
    return false;
  }

  exec_param_list.clear();
  for (JsonVariantConst v : doc.as<JsonArrayConst>()) {
    exec_pram_t ep{};
    convertFromJson(v, ep);
    exec_param_list.push_back(ep);
  }

  printf("[main] exec_params loaded: %u entries\n",
         (unsigned)exec_param_list.size());
  return !exec_param_list.empty();
}

// ─── 走行前共通セットアップ ────────────────────────────────────────────────────
void MainTask::setup_before_run() {
  // planning_->reset_gyro_ref_with_check();
}

// ─── LED ヘルパー ─────────────────────────────────────────────────────────────
void MainTask::show_mode_led(int mode) {
  int v = mode + 1;
  ui_.LED_bit((v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1,
              (v >> 4) & 1, (v >> 5) & 1);
}

// ─── ボタン待機ヘルパー ───────────────────────────────────────────────────────
void MainTask::wait_button() {
  while (!ui_.button_state_hold())
    sleep_ms(10);
}

// ─── LittleFS マウント管理 ────────────────────────────────────────────────────
// ConfigLoader が起動時に init() 済みのため、ここでは再初期化は不要。
// 将来的にファイルシステムの排他制御が必要になった際にここに実装する。
void MainTask::mount() {}
void MainTask::umount() {}

// ─── プロファイルロード ───────────────────────────────────────────────────────
// /profiles.hf または /profiles.cl から TurnType ごとのファイルインデックスを構築する。
void MainTask::load_turn_param_profiles(bool const_mode, int const_index) {
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
  printf("[main] tpp.file_list.size() = %d\n", (int)tpp.file_list.size());

  tpp.profile_idx_size = doc["profile_idx_size"] | 0;
  printf("[main] tpp.profile_idx_size=%d  const_index=%d\n",
         tpp.profile_idx_size, const_index);

  tpp.profile_map.clear();

  if (const_mode) {
    p_idx[TurnType::None] = p_idx[TurnType::Finish] = p_idx[TurnType::Normal] =
        p_idx[TurnType::Large] = p_idx[TurnType::Orval] =
            p_idx[TurnType::Dia45] = p_idx[TurnType::Dia45_2] =
                p_idx[TurnType::Dia135] = p_idx[TurnType::Dia135_2] =
                    p_idx[TurnType::Dia90] = const_index;
    tpp.profile_map[const_index] = p_idx;
  } else {
    int i = 0;
    for (JsonVariantConst entry : doc["profile_idx"].as<JsonArrayConst>()) {
      p_idx[TurnType::None]       = entry["run_param"]  | 0;
      p_idx[TurnType::Finish]     = entry["suction"]    | 0;
      p_idx[TurnType::Normal]     = entry["normal"]     | 0;
      p_idx[TurnType::Large]      = entry["large"]      | 0;
      p_idx[TurnType::Orval]      = entry["orval"]      | 0;
      p_idx[TurnType::Dia45]      = entry["dia45"]      | 0;
      p_idx[TurnType::Dia45_2]    = entry["dia45_2"]    | 0;
      p_idx[TurnType::Dia135]     = entry["dia135"]     | 0;
      p_idx[TurnType::Dia135_2]   = entry["dia135_2"]   | 0;
      p_idx[TurnType::Dia90]      = entry["dia90"]      | 0;
      tpp.profile_map[i++] = p_idx;
    }
  }
}

// ─── スラロームパラメータロード (fast/normal/slow 3セット) ───────────────────
void MainTask::load_slalom_param(int idx, int idx2, int idx3) {
  printf("load_slalom_param: %d, %d, %d\n", idx, idx2, idx3);

  param_set.suction = tpp.profile_map[idx][TurnType::Finish];
  if (param_set.suction == 1) {
    param_set.suction_duty     = sys_.test.suction_duty;
    param_set.suction_duty_low = sys_.test.suction_duty_low;
  } else if (param_set.suction == 2) {
    param_set.suction_duty     = sys_.test.suction_duty_burst;
    param_set.suction_duty_low = sys_.test.suction_duty_burst_low;
  }

  param_set.map.clear();
  param_set.map_slow.clear();
  param_set.map_fast.clear();
  param_set.str_map.clear();

  // fast
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None) continue;
    turn_map[tpp.profile_map[idx][p.first]].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map_fast);
  printf("------\n");

  // normal
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None) continue;
    turn_map[tpp.profile_map[idx2][p.first]].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map);
  printf("------\n");

  // slow
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None) continue;
    turn_map[tpp.profile_map[idx3][p.first]].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map_slow);

  load_straight(idx2, param_set.str_map);
}

// ─── スラロームパラメータロード (単一インデックス) ────────────────────────────
void MainTask::load_slalom_param2(int idx) {
  printf("load_slalom_param2: %d\n", idx);

  param_set.suction = tpp.profile_map[idx][TurnType::Finish];
  if (param_set.suction == 1) {
    param_set.suction_duty     = sys_.test.suction_duty;
    param_set.suction_duty_low = sys_.test.suction_duty_low;
  } else if (param_set.suction == 2) {
    param_set.suction_duty     = sys_.test.suction_duty_burst;
    param_set.suction_duty_low = sys_.test.suction_duty_burst_low;
  }

  param_set.map.clear();
  param_set.map_slow.clear();
  param_set.map_fast.clear();
  param_set.str_map.clear();

  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None) continue;
    turn_map[idx].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map);

  load_straight(idx, param_set.str_map);
}

// ─── スラロームファイル読み込み ───────────────────────────────────────────────
// tpp.file_list[idx] の JSON ファイルから TurnType ごとの slalom_param2_t を構築する。
void MainTask::load_slas(
    int idx,
    std::vector<std::pair<TurnType, std::string>> &turn_list,
    std::unordered_map<TurnType, slalom_param2_t> &sla_map) {
  if (idx < 0 || idx >= (int)tpp.file_list.size()) {
    printf("[main] load_slas: idx=%d out of range (size=%d)\n",
           idx, (int)tpp.file_list.size());
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
    if (entry.isNull()) continue;

    slalom_param2_t sp{};
    convertFromJson(entry, sp);
    sp.ref_ang = sp.ang;
    sp.type = cast_turn_type(p.second);
    sla_map[p.first] = sp;

    if (!silent_load) {
      printf(" - %s: v=%f  rad=%f  time=%f", p.second.c_str(), sp.v, sp.rad, sp.time);
      if (p.first == TurnType::Orval)
        printf("  rad2=%f  time2=%f", sp.rad2, sp.time2);
      printf("  front=[%0.2f, %0.2f]  back=[%0.2f, %0.2f]\n",
             sp.front.left, sp.front.right, sp.back.left, sp.back.right);
    }
  }
}

// ─── 直線パラメータロード ─────────────────────────────────────────────────────
// /vel_prof.hf または /vel_prof.cl の v_prof[idx] から StraightType ごとの
// straight_param_t を構築する。
void MainTask::load_straight(
    int idx, std::unordered_map<StraightType, straight_param_t> &str_map) {
  const char *fileName =
      (sys_.hf_cl == 0) ? "/vel_prof.hf" : "/vel_prof.cl";

  JsonDocument doc;
  if (!ConfigLoader::load_file(fileName, doc)) {
    printf("[main] load_straight: %s not found\n", fileName);
    return;
  }

  JsonArrayConst vel_prof = doc["v_prof"].as<JsonArrayConst>();
  if (idx < 0 || idx >= (int)vel_prof.size()) {
    printf("[main] load_straight: idx=%d out of range (size=%d)\n",
           idx, (int)vel_prof.size());
    return;
  }

  JsonVariantConst entry = vel_prof[idx];
  for (const auto &p : straight_name_list) {
    JsonVariantConst sp_json = entry[p.second.c_str()];
    if (sp_json.isNull()) continue;

    straight_param_t sp{};
    convertFromJson(sp_json, sp);
    str_map[p.first] = sp;

    if (!silent_load) {
      printf("[%d][%s]: v_max=%4.1f  accl=%4.1f  decel=%4.1f"
             "  w_max=%4.1f  w_end=%4.1f  alpha=%4.1f\n",
             idx, p.second.c_str(),
             sp.v_max, sp.accl, sp.decel, sp.w_max, sp.w_end, sp.alpha);
    }
  }
}

void MainTask::reset_tgt_data() { mp->reset_tgt_data(); }

void MainTask::reset_ego_data() { mp->reset_ego_data(); }

// ─── コンポーネント間配線 ─────────────────────────────────────────────────────
// params ロード後・mode 実行前に呼ぶ。
// shared_ptr に格納済みの各コンポーネントに依存オブジェクトを注入する。
void MainTask::setup_components() {
  auto sensing_entity = sensing_->get_sensing_entity();

  // UserInterface は値メンバーのため no-op deleter で shared_ptr 化
  auto ui_ptr = std::shared_ptr<UserInterface>(&ui_, [](UserInterface *) {});

  // MotionPlanning
  mp->set_tgt_val(tgt_val_);
  mp->set_sensing_entity(sensing_entity);
  mp->set_input_param_entity(param_);
  mp->set_userinterface(ui_ptr);
  mp->set_planning_task(planning_);
  mp->set_logging_task(lt_);

  // MazeSolverBaseLgc: maze_size と max_step_val で初期化
  const int msize = (sys_.maze_size > 0) ? sys_.maze_size : 16;
  lgc->init(msize, msize * msize - 1);
  lgc->set_goal_pos(sys_.goals);

  // SearchController
  search_ctrl->set_lgc(lgc);
  search_ctrl->set_motion_plannning(mp);
  search_ctrl->set_planning_task(planning_);
  search_ctrl->set_sensing_entity(sensing_entity);
  search_ctrl->set_logging_task(lt_);
  search_ctrl->set_userinterface(ui_ptr);
  search_ctrl->set_input_param_entity(param_);
}

