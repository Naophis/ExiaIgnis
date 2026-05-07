#include "config_loader.hpp"
#include "config_mapping.hpp"
#include "define.hpp"
#include "main/main_task.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

// ─── exec パラメータロード
// ──────────────────────────────────────────────────── /exec.json (JSON 配列)
// を読み込んで exec_param_list を構築する。
// 本走行・テスト走行の両モードに入る前に呼ぶこと。
bool MainTask::load_exec_params() {
  JsonDocument doc;
  if (!ConfigLoader::load_file("/exec.json", doc)) {
    printf("[main] /exec.json not found, exec_param_list empty.\n");
    return false;
  }

  exec_param_list.clear();
  JsonArrayConst arr = doc.as<JsonArrayConst>();
  for (JsonVariantConst v : arr) {
    exec_pram_t ep{};
    convertFromJson(v, ep);
    exec_param_list.push_back(ep);
  }

  printf("[main] exec_params loaded: %u entries\n",
         (unsigned)exec_param_list.size());
  return !exec_param_list.empty();
}

// ─── 走行前共通セットアップ
// ────────────────────────────────────────────────────
// ジャイロゼロオフセット取得・センサーウォームアップ等、
// 本走行・テスト走行どちらも必要な初期化をまとめる。
void MainTask::setup_before_run() {
  // planning_->reset_gyro_ref_with_check();
}

// ─── LED ヘルパー
// ───────────────────────────────────────────────────────────── mode+1 を 6bit
// 2進数で表示する (Astraea の lbit.byte = mode+1 と同等)。
void MainTask::show_mode_led(int mode) {
  int v = mode + 1;
  ui_.LED_bit((v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1,
              (v >> 4) & 1, (v >> 5) & 1);
}

// ─── ボタン待機ヘルパー
// ───────────────────────────────────────────────────────
void MainTask::wait_button() {
  while (!ui_.button_state_hold())
    sleep_ms(10);
}

void MainTask::load_turn_param_profiles(bool const_mode, int const_index) {}
void MainTask::load_slalom_param(int idx, int idx2, int idx3) {}
void MainTask::load_slalom_param2(int idx) {}

void MainTask::load_slas(
    int idx, vector<pair<TurnType, string>> &turn_list,
    std::unordered_map<TurnType, slalom_param2_t> &turn_map) {}
void MainTask::load_straight(
    int idx, std::unordered_map<StraightType, straight_param_t> &str_map) {}