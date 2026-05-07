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

  std::shared_ptr<input_param_t> param_;
  std::shared_ptr<motion_tgt_val_t> tgt_val_;
  system_t sys_{};
  LED_bit lbit;

  void print_system_params();
  void print_hardware_params();
  void print_offset_params();
  void print_sensor_params();

  // ─── 共通ユーティリティ (main_task_util.cpp) ─────────────────────────────
  // /exec.json を読み込んで exec_param_list を構築する。
  bool load_exec_params();
  // 両モード共通の事前準備 (ジャイロリセット・センサーウォームアップ等)。
  void setup_before_run();
  // mode+1 を 6bit 2進数で LED 表示する。
  void show_mode_led(int mode);
  // ボタン長押しまで待機する。
  void wait_button();

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

  // util
  void load_turn_param_profiles(bool const_mode, int const_index);
  void load_slalom_param(int idx, int idx2, int idx3);
  void load_slalom_param2(int idx);
  void load_slas(int idx, vector<pair<TurnType, string>> &turn_list,
                 std::unordered_map<TurnType, slalom_param2_t> &turn_map);
  void
  load_straight(int idx,
                std::unordered_map<StraightType, straight_param_t> &str_map);
};
