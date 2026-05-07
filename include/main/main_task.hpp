#pragma once
#include <memory>
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

private:
    MainTask() = default;
    void run();

    // LittleFS 上の各パラメータファイルを読み込み param_ / sys_ に反映する。
    // ファイルが存在しない場合はそのフィールドをスキップする。
    // 戻り値: 1 ファイル以上の読み込みに成功した場合 true。
    bool load_params();

    std::shared_ptr<sensing_result_entity_t> get_sensing_entity();

    static std::shared_ptr<MainTask> s_instance;

    std::shared_ptr<SensingTask>   sensing_;
    std::shared_ptr<PlanningTask>  planning_;
    UserInterface ui_;

    // sensing_ / planning_ と共有する入力パラメータ (本クラスのみ書き込み可)
    std::shared_ptr<input_param_t> param_;
    // 走行モード・ゴール設定など主タスク管理の系統パラメータ
    system_t sys_{};
    void print_system_params();
};
