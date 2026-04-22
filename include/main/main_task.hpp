#pragma once
#include <memory>
#include "sensing_task.hpp"
#include "planning/planning_task.hpp"
#include "ui.hpp"

// Core1 で動作するメインタスク。
// シリアル出力・ボタン処理・planning への指示を担う。
class MainTask {
public:
    static std::shared_ptr<MainTask> create(
        std::shared_ptr<SensingTask>  sensing,
        std::shared_ptr<PlanningTask> planning);

    // Core1 エントリポイント。multicore_launch_core1() に渡す。
    static void core1_entry();

    // Core1 が snprintf で組み立てた文字列。Core0 のメインループが fputs する。
    // TinyUSB はシングルコア前提のため Core1 から直接 printf/USB 関数を呼ばない。
    static char          s_print_buf[1024];
    static volatile bool s_print_ready;

private:
    MainTask() = default;
    void run();

    static std::shared_ptr<MainTask> s_instance;

    std::shared_ptr<SensingTask>  sensing_;
    std::shared_ptr<PlanningTask> planning_;
    UserInterface ui_;
};
