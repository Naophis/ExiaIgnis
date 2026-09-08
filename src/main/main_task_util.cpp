#include "config_loader.hpp"
#include "config_mapping.hpp"
#include "define.hpp"
#include "main/main_task.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

// ─── LED ヘルパー───────────────────────────────────────────────────
void MainTask::show_mode_led(int mode) {
  int v = mode + 1;
  ui_->LED_bit((v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1,
               (v >> 4) & 1, (v >> 5) & 1);
}

// ─── ボタン待機ヘルパー────────────────────────────────
void MainTask::wait_button() {
  while (!ui_->button_state_hold())
    sleep_ms(10);
}

// ─── v_max→decel絶対値LUT ──────────────────────────────
// param_->decel_v_max_x/y(hardware.yaml)を線形補間して引く。
// decel_v_max_enable=0(デフォルト)なら常にbase_decelをそのまま返す。
// [2026-08-23修正] 当初「配列が空(size<2)なら無効」としていたが、
// from_json_vector()はJSONキーが無いとdst.clear()まで到達せず前回値が
// 残ってしまい、yamlから行を消す/コメントアウトしても無効化できない実害を
// 確認した。配列の空/非空に頼らず、明示的なenableフラグで確実にON/OFFする。
// 減速中に値を変えるのではなく、区間開始前にv_max(既知)から1回だけ選ぶ
// ため、距離から逆算する既存のclosed-form計算(go_straight_dummy等)は
// 一切変更しない。
float MainTask::apply_decel_v_max_lut(float v_max, float base_decel) const {
  if (!param_->decel_v_max_enable) {
    return base_decel;
  }
  const auto &vx = param_->decel_v_max_x;
  const auto &vy = param_->decel_v_max_y;
  if (vx.size() < 2 || vx.size() != vy.size()) {
    return base_decel;
  }
  float decel_mag;
  if (v_max <= vx.front()) {
    decel_mag = vy.front();
  } else if (v_max >= vx.back()) {
    decel_mag = vy.back();
  } else {
    decel_mag = vy.back();
    for (size_t i = 0; i + 1 < vx.size(); ++i) {
      if (v_max >= vx[i] && v_max <= vx[i + 1]) {
        const float t = (v_max - vx[i]) / (vx[i + 1] - vx[i]);
        decel_mag = vy[i] + t * (vy[i + 1] - vy[i]);
        break;
      }
    }
  }
  return (base_decel < 0) ? -decel_mag : decel_mag;
}

// ─── LittleFS マウント管理───────────────────────────────
// ConfigLoader が起動時に init() 済みのため、ここでは再初期化は不要。
// 将来的にファイルシステムの排他制御が必要になった際にここに実装する。
void MainTask::mount() {}
void MainTask::umount() {}

// ─── プロファイルロード────────────────────────────────
//  /profiles.hf または /profiles.cl から TurnType ごとのファイルインデックスを構築する。
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
}

// ─── スラロームパラメータロード (fast/normal/slow 3セット) ───────────────────
void MainTask::load_slalom_param(int idx, int idx2, int idx3) {
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

  // fast
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None)
      continue;
    turn_map[tpp.profile_map[idx][p.first]].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map_fast);
  printf("------\n");

  // normal
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None)
      continue;
    turn_map[tpp.profile_map[idx2][p.first]].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map);
  printf("------\n");

  // slow
  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None)
      continue;
    turn_map[tpp.profile_map[idx3][p.first]].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map_slow);

  load_straight(idx2, param_set.str_map);
}

// ─── スラロームパラメータロード (単一インデックス)
// ────────────────────────────
void MainTask::load_slalom_param2(int idx) {
  printf("load_slalom_param2: %d\n", idx);

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

  turn_map.clear();
  for (const auto &p : turn_name_list) {
    if (p.first == TurnType::None)
      continue;
    turn_map[idx].emplace_back(p);
  }
  for (auto &kv : turn_map)
    load_slas(kv.first, kv.second, param_set.map);

  load_straight(idx, param_set.str_map);
}

// ─── スラロームファイル読み込み
// ─────────────────────────────────────────────── tpp.file_list[idx] の JSON
// ファイルから TurnType ごとの slalom_param2_t を構築する。
void MainTask::load_slas(
    int idx, std::vector<std::pair<TurnType, std::string>> &turn_list,
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
      printf(" - %s: v=%f  rad=%f  time=%f", p.second.c_str(), sp.v, sp.rad,
             sp.time);
      if (p.first == TurnType::Orval)
        printf("  rad2=%f  time2=%f", sp.rad2, sp.time2);
      printf("  front=[%0.2f, %0.2f]  back=[%0.2f, %0.2f]\n", sp.front.left,
             sp.front.right, sp.back.left, sp.back.right);
    }
  }
}

// ─── スラロームオフセット書き戻し
// ─────────────────────────────────────────────── tpp.file_list[idx] の
// JSONファイルを読み込み、対象TurnTypeのfront.left/rightだけを書き換えて
// 同じファイルへ書き戻す(他のTurnTypeのエントリはそのまま保持)。
bool MainTask::save_slalom_front_offset(int idx, TurnType type,
                                        const slalom_offset_t &front) {
  if (idx < 0 || idx >= (int)tpp.file_list.size()) {
    printf("[main] save_slalom_front_offset: idx=%d out of range (size=%d)\n",
           idx, (int)tpp.file_list.size());
    return false;
  }

  std::string type_name;
  for (const auto &p : turn_name_list) {
    if (p.first == type) {
      type_name = p.second;
      break;
    }
  }
  if (type_name.empty()) {
    printf("[main] save_slalom_front_offset: unknown TurnType\n");
    return false;
  }

  const auto &file_name = tpp.file_list[idx];
  const auto path = std::string("/") + file_name;

  JsonDocument doc;
  if (!ConfigLoader::load_file(path.c_str(), doc)) {
    printf("[main] save_slalom_front_offset: %s not found\n", path.c_str());
    return false;
  }

  JsonVariant entry = doc[type_name.c_str()];
  if (entry.isNull()) {
    printf("[main] save_slalom_front_offset: %s not in %s\n",
           type_name.c_str(), path.c_str());
    return false;
  }
  entry["front"]["left"] = front.left;
  entry["front"]["right"] = front.right;

  size_t need = measureJson(doc) + 1;
  char *buf = static_cast<char *>(malloc(need));
  if (!buf) {
    printf("[main] save_slalom_front_offset: no memory\n");
    return false;
  }
  serializeJson(doc, buf, need);

  // flash_range_erase/prog は割り込みを ~100ms 禁止し USB CDC を切断する。
  // 直前のprintfがUSBへ届く猶予を空けてから書き込む(main_task_usb.cppと同じ作法)。
  fflush(stdout);
  sleep_ms(80);
  bool ok = ConfigLoader::write_file(path.c_str(), reinterpret_cast<uint8_t *>(buf),
                                     need - 1);
  free(buf);
  return ok;
}

// ─── スラロームオフセット書き戻し確認用読み直し
// ─────────────────────────────── save_slalom_front_offset()で書いた直後、
// 実際にLittleFS上のファイルへ反映されているかをその場で確認するために
// 同じファイルを読み直してfront.left/rightだけ取り出す(検証専用、
// param_set等の実行時状態には触れない)。
bool MainTask::read_slalom_front_offset(int idx, TurnType type,
                                        slalom_offset_t &out) {
  if (idx < 0 || idx >= (int)tpp.file_list.size())
    return false;

  std::string type_name;
  for (const auto &p : turn_name_list) {
    if (p.first == type) {
      type_name = p.second;
      break;
    }
  }
  if (type_name.empty())
    return false;

  const auto &file_name = tpp.file_list[idx];
  const auto path = std::string("/") + file_name;

  JsonDocument doc;
  if (!ConfigLoader::load_file(path.c_str(), doc))
    return false;

  JsonVariantConst entry = doc[type_name.c_str()];
  if (entry.isNull())
    return false;

  convertFromJson(entry["front"], out);
  return true;
}

// ─── 直線パラメータロード
// ───────────────────────────────────────────────────── /vel_prof.hf または
// /vel_prof.cl の v_prof[idx] から StraightType ごとの straight_param_t
// を構築する。
void MainTask::load_straight(
    int idx, std::unordered_map<StraightType, straight_param_t> &str_map) {
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
    // decel_v_max LUTは吸引ON時限定の対策(main_task_test_run.cpp参照)。
    // load_straight()は設定ロード時に1回だけ呼ばれ、その時点では対象走行が
    // 吸引ONかどうか分からないため、ここでは適用しない。
    str_map[p.first] = sp;

    if (!silent_load) {
      printf("[%d][%s]: v_max=%4.1f  accl=%4.1f  decel=%4.1f"
             "  w_max=%4.1f  w_end=%4.1f  alpha=%4.1f\n",
             idx, p.second.c_str(), sp.v_max, sp.accl, sp.decel, sp.w_max,
             sp.w_end, sp.alpha);
    }
  }
}

void MainTask::reset_tgt_data() { mp->reset_tgt_data(); }

void MainTask::reset_ego_data() { mp->reset_ego_data(); }

// ─── コンポーネント間配線
// ───────────────────────────────────────────────────── params ロード後・mode
// 実行前に呼ぶ。 shared_ptr
// に格納済みの各コンポーネントに依存オブジェクトを注入する。
void MainTask::setup_components() {
  auto sensing_entity = sensing_->get_sensing_entity();

  auto ui_ptr = ui_;

  // MotionPlanning
  printf("[main] wiring MotionPlanning\n");
  mp->set_tgt_val(tgt_val_);
  mp->set_sensing_entity(sensing_entity);
  mp->set_input_param_entity(param_);
  mp->set_userinterface(ui_ptr);
  mp->set_planning_task(planning_);
  mp->set_logging_task(lt_);

  // MazeSolverBaseLgc: maze_size と max_step_val で初期化
  const int msize = (sys_.maze_size > 0) ? sys_.maze_size : 16;
  printf("[main] lgc init msize=%d\n", msize);
  lgc->init(msize, msize * msize - 1);
  lgc->set_goal_pos(sys_.goals);

  // SearchController
  printf("[main] wiring SearchController\n");
  search_ctrl->set_lgc(lgc);
  search_ctrl->set_motion_plannning(mp);
  search_ctrl->set_planning_task(planning_);
  search_ctrl->set_sensing_entity(sensing_entity);
  search_ctrl->set_logging_task(lt_);
  search_ctrl->set_userinterface(ui_ptr);
  search_ctrl->set_input_param_entity(param_);
}

void MainTask::req_error_reset() {

  // printf("kf_batt:\n");
  // pt->kf_batt.print_state();

  // printf("kf_v:\n");
  // pt->kf_v.print_state();

  // printf("kf_enc_r:\n");
  // pt->kf_v_r.print_state();

  // printf("kf_enc_l:\n");
  // pt->kf_v_l.print_state();

  // printf("kf_dist:\n");
  // pt->kf_dist.print_state();

  // printf("kf_w:\n");
  // pt->kf_w.print_state();

  // printf("kf_ang:\n");
  // pt->kf_ang.print_state();

  tgt_val_->pl_req.error_vel_reset = 1;
  tgt_val_->pl_req.error_gyro_reset = 1;
  tgt_val_->pl_req.error_ang_reset = 1;
  tgt_val_->pl_req.error_dist_reset = 1;
  tgt_val_->nmr.timstamp = tgt_val_->nmr.timstamp + 1;
  planning_->send_command(*tgt_val_);
}

void MainTask::check_battery() {

  const auto se = get_sensing_entity();
  sleep_ms(100); // センサー値の更新待ち

  printf("battery= %f\n", se->ego.battery_raw);
  if (se->ego.battery_raw > LOW_BATTERY_TH || se->ego.battery_raw < 10.5)
    return;
  while (1) {
    ui_->music_sync(MUSIC::G5_, 250);
    sleep_ms(250);
    bool break_btn = ui_->button_state_hold();
    if (break_btn) {
      ui_->coin(100);
      break;
    }
  }
}

int MainTask::select_mode() {
  int mode_num = 0;
  lbit.byte = 0;

  char max_mode_idx = 2 + exec_param_list.size() + 4;
  while (1) {
    int res = ui_->encoder_operation();
    mode_num += res;
    if (mode_num == -1) {
      mode_num = max_mode_idx - 1;
    } else if (mode_num == max_mode_idx) {
      mode_num = 0;
    }
    lbit.byte = mode_num + 1;
    ui_->LED_bit(lbit.b0, lbit.b1, lbit.b2, lbit.b3, lbit.b4, lbit.b5);
    if (ui_->button_state_hold()) {
      ui_->coin(100);
      break;
    }
    sleep_ms(10);
  }
  return mode_num;
}