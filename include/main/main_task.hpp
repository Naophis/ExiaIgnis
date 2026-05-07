#pragma once
#include "defines.hpp"
#include "logging/logging_task.hpp"
#include "planning/planning_task.hpp"
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

  std::shared_ptr<input_param_t> param_;
  system_t sys_{};
  LED_bit lbit;

  void print_system_params();
  void print_hardware_params();
  void print_offset_params();
  void print_sensor_params();

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
};
