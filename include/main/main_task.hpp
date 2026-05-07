#pragma once
#include <memory>
#include "logging/logging_task.hpp"
#include "sensing_task.hpp"
#include "planning/planning_task.hpp"
#include "ui.hpp"
#include "structs.hpp"

// Core0 で動作するメインタスク。
// シリアル出力・ボタン処理・planning への指示を担う。
// TinyUSB が Core0 固定のため printf を直接呼べる。
//
// パラメータの読み書きは本クラスのみが行い、sensing/planning は read-only で参照する。
class MainTask {
public:
    static std::shared_ptr<MainTask> create(
        std::shared_ptr<SensingTask>   sensing,
        std::shared_ptr<PlanningTask>  planning,
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

    std::shared_ptr<SensingTask>    sensing_;
    std::shared_ptr<PlanningTask>   planning_;
    std::shared_ptr<LoggingTask>    lt_;
    UserInterface ui_;

    std::shared_ptr<input_param_t> param_;
    system_t sys_{};

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
    void mode_sensor_monitor(); // mode 14: echo_sensor_csv (dump1 相当)
    void mode_sensor_csv();     // mode 15: echo_printf     (dump2 相当)
    void mode_suction_test();   // 本走行サブモード用: インタラクティブ吸引テスト
    void mode_straight_test();  // mode 2:  test_run 相当 (stub)
    void mode_pivot_test();     // mode 3:  test_turn 相当 (stub)
};
