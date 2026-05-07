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

    // param_ / sys_ の全フィールドを JSON でシリアルに出力する。
    // シリアルコマンド "DUMP\n" から呼ばれる。
    static void dump_all_params();

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
    void print_hardware_params();
    void print_offset_params();
    void print_sensor_params();

    // ─── モード dispatch ──────────────────────────────────────────────────────
    // user_mode==0: 本走行モード / user_mode!=0: テストモード番号に応じて分岐
    void run_main_mode();
    void run_test_mode(int mode);

    // ─── 本走行サブモード選択 ─────────────────────────────────────────────────
    // 短押し: 次のモードへ(LED 2進表示更新) / 長押し(>=1秒): 決定
    // Astraea の select_mode()+encoder_operation() に相当。
    int select_run_mode(int max_mode);

    // ─── 個別モード ───────────────────────────────────────────────────────────
    void mode_sensor_monitor(); // センサーモニター / mode 1 (dump1/dump2 相当)
    void mode_suction_test();   // 吸引 PWM テスト  / mode 5 (Astraea mode 11 相当)
    void mode_straight_test();  // 直進テスト        / mode 2 (test_run 相当)
    void mode_pivot_test();     // 旋回テスト        / mode 3 (test_turn 相当)
};
