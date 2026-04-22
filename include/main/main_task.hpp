#pragma once
#include <memory>
#include "sensing_task.hpp"
#include "planning/planning_task.hpp"
#include "ui.hpp"

// Core0 で動作するメインタスク。
// シリアル出力・ボタン処理・planning への指示を担う。
// TinyUSB が Core0 固定のため printf を直接呼べる。
class MainTask {
public:
    static std::shared_ptr<MainTask> create(
        std::shared_ptr<SensingTask>  sensing,
        std::shared_ptr<PlanningTask> planning);

    // Core0 のメインループとして呼ぶ (ブロッキング)。
    static void start();

private:
    MainTask() = default;
    void run();

    static std::shared_ptr<MainTask> s_instance;

    std::shared_ptr<SensingTask>  sensing_;
    std::shared_ptr<PlanningTask> planning_;
    UserInterface ui_;
};
