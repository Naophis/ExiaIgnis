#include "define.hpp"
#include "main/main_task.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

// ============================================================
// 本走行サブモード選択 (Astraea の select_mode() 相当)
// 短押し: 次のモードへ / 長押し(>=1秒): 決定
// ============================================================
int MainTask::select_run_mode(int max_mode) {
  int mode_num = 0;
  lbit.byte = 0;

  char max_mode_idx = 2 + exec_param_list.size() + 4;
  while (1) {
    int res = ui_.encoder_operation();
    mode_num += res;
    if (mode_num == -1) {
      mode_num = max_mode_idx - 1;
    } else if (mode_num == max_mode_idx) {
      mode_num = 0;
    }
    show_mode_led(mode_num);
    if (ui_.button_state_hold()) {
      ui_.coin(100);
      break;
    }
    sleep_ms(1);
  }
  return mode_num;
}

// ============================================================
// 本走行モード (user_mode == 0)
// Astraea task() の else ブロック相当。
// ============================================================
//
// サブモード:
//   0: 探索走行    (stub)
//   1: 最速走行    (stub)
//   2: 吸引テスト  (インタラクティブ)
//   3: センサーモニター
//
void MainTask::run_main_mode() {
  printf("[main] === MAIN RUN MODE ===\n");
  ui_.coin(200);

  ui_.coin(200);
  lgc->init(sys_.maze_size, sys_.maze_size * sys_.maze_size - 1);
  lgc->set_goal_pos(sys_.goals);
  search_ctrl->set_lgc(lgc);
  search_ctrl->set_motion_plannning(mp);
  pc->set_logic(lgc);
  pc->set_userinterface(ui_);
  // read_maze_data();
  search_ctrl->print_maze();

  int mode_num = 0;
  static constexpr int SUB_MODE_COUNT = 4;
  SearchResult sr;

  while (true) {
    planning_->set_mode_select(true);
    mode_num = select_mode();
    planning_->set_mode_select(false);
    exec_param_prof();
    printf("mode_num: %d, exec_param_list.size(): %d\n", mode_num,
           exec_param_list.size());
    if (mode_num == 0) {
      lgc->set_goal_pos(sys_.goals);
      rorl2 = ui_.select_direction();
      int idx = 0;
      if (rorl2 == TurnDirection::Right) {
        idx = 0;
      } else {
        idx = 1;
      }
      load_slalom_param(idx, idx, idx);
      sr = search_ctrl->exec(param_set, SearchMode::ALL);
      if (sr == SearchResult::SUCCESS)
        save_maze_data(true);
      while (1) {
        if (ui_.button_state_hold())
          break;
        sleep_ms(10);
      }
      search_ctrl->print_maze();
    } else if (mode_num == 1) {
      lgc->set_goal_pos(sys_.goals);
      rorl = ui_.select_direction();
      rorl2 = ui_.select_direction();
      int idx = 0;
      if (rorl2 == TurnDirection::Right) {
        idx = 0;
      } else {
        idx = 1;
      }
      sr = SearchResult::SUCCESS;
      load_slalom_param(idx, idx, idx);
      if (rorl == TurnDirection::Right)
        sr = search_ctrl->exec(param_set, SearchMode::Kata);
      else
        sr = search_ctrl->exec(param_set, SearchMode::Return);
      if (sr == SearchResult::SUCCESS) {
        save_maze_data(true);
      }
      while (1) {
        if (ui_.button_state_hold())
          break;
        sleep_ms(10);
      }
      search_ctrl->print_maze();
    } else if (2 <= mode_num && mode_num <= exec_param_list.size() + 1) {
      const auto p = exec_param_list[mode_num - 2];
      printf("mode_num: %d\n", mode_num - 2);
      exec_param_list.clear();
      path_run(p.fast_idx, p.normal_idx, p.slow_idx);
    } else if (mode_num == (2 + exec_param_list.size())) {
      printf("keep_pivot\n");
      keep_pivot();
    } else if (mode_num == (2 + exec_param_list.size() + 1)) {
      // dump1(); // taskの最終行に配置すること
      printf("suction\n");
      mp->reset_gyro_ref_with_check();
      planning_->suction_enable(sys_.test.suction_duty,
                               sys_.test.suction_duty_low);
      sleep_ms(10000);
      planning_->suction_disable();
    } else if (mode_num == (2 + exec_param_list.size() + 2)) {
      sim_run_time_all();
    } else if (mode_num == (2 + exec_param_list.size() + 3)) {
      save_maze_data(false);
      ui_.coin(25);
      save_maze_kata_data(false);
      ui_.coin(25);
      save_maze_return_data(false);
      ui_.coin(25);
      lgc->init(sys_.maze_size, sys_.maze_size * sys_.maze_size - 1);
      lgc->set_goal_pos(sys_.goals);
    }
    sleep_ms(10);
  }
}
