#pragma once
#include "defines.hpp"
#include "logging/logging_task.hpp"
#include "planning/planning_task.hpp"
#include "action/motion_planning.hpp"
#include "search/logic.hpp"
#include "search/search_controller.hpp"

#include "sensing_task.hpp"
#include "structs.hpp"
#include "ui.hpp"
#include <memory>

// Core0 で動作するメインタスク。
// シリアル出力・ボタン処理・planning への指示を担う。
// TinyUSB が Core0 固定のため printf を直接呼べる。
//
// パラメータの読み書きは本クラスのみが行い、sensing/planning は read-only
// で参照する。
class MainTask {
public:
  static std::shared_ptr<MainTask>
  create(std::shared_ptr<SensingTask> sensing,
         std::shared_ptr<PlanningTask> planning,
         std::shared_ptr<input_param_t> param);

  // Core0 のメインループとして呼ぶ (ブロッキング)。
  static void start();

  // param_ / sys_ の全フィールドを JSON でシリアルに出力する。
  static void dump_all_params();

  // create() 後・start() 前に呼ぶ。
  void set_logging_task(std::shared_ptr<LoggingTask> lt) { lt_ = lt; }
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> _tgt_val) {
    tgt_val_ = _tgt_val;
  }

private:
  MainTask() = default;
  void run();

  bool load_params();

  std::shared_ptr<sensing_result_entity_t> get_sensing_entity();

  static std::shared_ptr<MainTask> s_instance;

  std::shared_ptr<SensingTask> sensing_;
  std::shared_ptr<PlanningTask> planning_;
  std::shared_ptr<LoggingTask> lt_;
  UserInterface ui_;

  std::vector<exec_pram_t> exec_param_list;
  turn_param_profile_t tpp;
  std::unordered_map<TurnType, int> p_idx;
  std::shared_ptr<input_param_t> param_;
  std::shared_ptr<motion_tgt_val_t> tgt_val_;
  system_t sys_{};
  LED_bit lbit;

  // ─── 共通ユーティリティ (main_task_util.cpp) ─────────────────────────────
  bool load_exec_params();
  void setup_before_run();
  void show_mode_led(int mode);
  void wait_button();
  void mount();
  void umount();
  void load_turn_param_profiles(bool const_mode, int const_index);
  void load_slalom_param(int idx, int idx2, int idx3);
  void load_slalom_param2(int idx);
  void load_slas(int idx, std::vector<std::pair<TurnType, std::string>> &turn_list,
                 std::unordered_map<TurnType, slalom_param2_t> &sla_map);
  void load_straight(int idx,
                     std::unordered_map<StraightType, straight_param_t> &str_map);

  bool silent_load = false;
  void setup_components();
  // 走行用関数
  std::shared_ptr<MotionPlanning> mp;
  std::shared_ptr<MazeSolverBaseLgc> lgc;
  std::shared_ptr<SearchController> search_ctrl;
  std::shared_ptr<PathCreator> pc;
  void reset_tgt_data();
  void reset_ego_data();


  // ─── モード dispatch ──────────────────────────────────────────────────────
  void run_main_mode();
  void run_test_mode(int mode);

  // ─── 本走行サブモード選択 ─────────────────────────────────────────────────
  // 短押し: 次のモードへ(LED 2進表示更新) / 長押し(>=1秒): 決定
  int select_run_mode(int max_mode);

  // ─── 個別モード ───────────────────────────────────────────────────────────
  void test_sla();
  void test_run();
  void test_turn();
  void test_run_sla();
  void test_search_sla(bool wall_off);
  void test_front_wall_offset();
  void test_front_ctrl(bool enable);
  void test_sla_walloff();
  void test_back();
  void keep_pivot();
  void dump1();
  void dump2();
  void test_dia_walloff();
  void test_pivot_n();
  void test_pivot_n2();
  void encoder_test();
  void test_search_pivot();
  void test_system_identification(bool para);

  std::unordered_map<unsigned char, std::vector<std::pair<TurnType, std::string>>> turn_map;
  param_set_t p_set;
  std::vector<param_set_t> paramset_list;
  param_set_t param_set;
  TurnType cast_turn_type(std::string str) {
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
  // stub
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
  std::vector<std::pair<ExecParamType, std::string>> exec_param_name_list = {
      {ExecParamType::Fast, "fast"},     //
      {ExecParamType::Normal, "normal"}, //
      {ExecParamType::Slow, "slow"},     //
  };
};
