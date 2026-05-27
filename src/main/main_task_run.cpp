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
    int res = ui_->encoder_operation();
    mode_num += res;
    if (mode_num == -1) {
      mode_num = max_mode_idx - 1;
    } else if (mode_num == max_mode_idx) {
      mode_num = 0;
    }
    show_mode_led(mode_num);
    if (ui_->button_state_hold()) {
      ui_->coin(100);
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
  ui_->coin(200);

  ui_->coin(200);
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
      rorl2 = ui_->select_direction();
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
        if (ui_->button_state_hold())
          break;
        sleep_ms(10);
      }
      search_ctrl->print_maze();
    } else if (mode_num == 1) {
      lgc->set_goal_pos(sys_.goals);
      rorl = ui_->select_direction();
      rorl2 = ui_->select_direction();
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
        if (ui_->button_state_hold())
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
      ui_->coin(25);
      save_maze_kata_data(false);
      ui_->coin(25);
      save_maze_return_data(false);
      ui_->coin(25);
      lgc->init(sys_.maze_size, sys_.maze_size * sys_.maze_size - 1);
      lgc->set_goal_pos(sys_.goals);
    }
    sleep_ms(10);
  }
}

void MainTask::path_run(int idx, int idx2, int idx3) {
  planning_->set_mode_select(false);
  load_slalom_param(idx, idx2, idx3);
  param_set.cell_size = param_->cell;
  param_set.start_offset = param_->offset_start_dist;
  if (sys_.circuit_mode == 0) {
    const auto rorl = ui_->select_direction();
    if (rorl == TurnDirection::Right) {
      //速度ベース経路導出
      for (int i = 1; i <= 5; i++) {
        lgc->set_param_num(i);
        pc->other_route_map.clear();
        const bool res = pc->path_create(false);
        printf("other route size = %d\n", pc->other_route_map.size());
        if (!res) {
          ui_->error();
          return;
        }
        pc->convert_large_path(true);
        pc->diagonalPath(true, true);
        if (i == 0) {
          pc->path_s2.clear();
          pc->path_t2.clear();
          for (int i = 0; i < pc->path_t.size(); i++) {
            pc->path_s2.push_back(pc->path_s[i]);
            pc->path_t2.push_back(pc->path_t[i]);
          }
        }
        path_set_t p;
        p.type = i;
        p.time = 10000;
        pc->timebase_path_create(false, param_set, p);
        pc->path_set_map.push(p);
      }

      // 先頭に最適な経路を持ってくる
      const auto top_p = pc->path_set_map.top();
      if (top_p.result) { //成功
        pc->path_s.clear();
        pc->path_t.clear();
        for (int i = 0; i < top_p.path_s.size(); i++) {
          pc->path_s.push_back(top_p.path_s[i]);
          pc->path_t.push_back(top_p.path_t[i]);
        }
        // clear map
        while (!pc->path_set_map.empty()) {
          auto p2 = pc->path_set_map.top();
          // printf("type: %d, time: %f, turn: %d\n", p2.type, p2.time,
          //        p2.path_t.size());
          p2.path_s.clear();
          p2.path_t.clear();
          pc->path_set_map.pop();
        }

      } else { //失敗
        pc->other_route_map.clear();
        const bool res = pc->path_create(false);
        if (!res) {
          ui_->error();
          return;
        }
        pc->convert_large_path(true);
        pc->diagonalPath(true, true);
      }
    } else {
      pc->other_route_map.clear();
      const bool res = pc->path_create(false);
      if (!res) {
        ui_->error();
        return;
      }
      pc->convert_large_path(true);
      pc->diagonalPath(true, true);
      pc->print_path();
    }
  } else {
    load_circuit_path();
  }
  printf("----------------\n");
  if (pc->path_s.size() == 0) {
    ui_->error();
    ui_->error();
    pc->other_route_map.clear();
    const bool res = pc->path_create(false);
    if (!res) {
      ui_->error();
      return;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
    pc->print_path();
  }

  pc->calc_goal_time(param_set, true);
  pc->print_path();
  // printf("after: %d bytes\n",
  // heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  const auto backup_l45 = param_->sen_ref_p.normal.exist.left45;
  const auto backup_r45 = param_->sen_ref_p.normal.exist.right45;

  mp->exec_path_running(param_set);

  pc->print_path();

  dump1();
  param_->sen_ref_p.normal.exist.left45 = backup_l45;
  param_->sen_ref_p.normal.exist.right45 = backup_r45;

  // param->fast_log_enable = 0; //１回きり
}

void MainTask::sim_run_time_all() {
  param_set.cell_size = param_->cell;
  param_set.start_offset = param_->offset_start_dist;
  const auto rorl = ui_->select_direction();
  silent_load = true;
  sim_run_time(2, 2, 2, 2, false);
  sim_run_time(3, 3, 3, 3, false);
  sim_run_time(4, 6, 4, 3, false);
  sim_run_time(6, 10, 6, 3, false);
  sim_run_time(8, 10, 8, 3, false);
  sim_run_time(10, 11, 10, 3, false);
  sim_run_time(12, 12, 12, 3, false);
  silent_load = false;
  sim_run_time(13, 13, 12, 3, true);
  // sim_run_time(14, 24, 23, 14, true);
}


void MainTask::sim_run_time(int mode_num, int idx, int idx2, int idx3,
                            bool dump_all) {
  planning_->set_search_mode(false);
  printf("[sim_run_time] mode_num = %d\n", mode_num);
  load_slalom_param(idx, idx2, idx3);

  if (!silent_load) {
    // fast
    printf("fast: \n");
    printf("  - Large: %0.2f\n", param_set.map_fast[TurnType::Large].v);
    printf("  - Orval: %0.2f\n", param_set.map_fast[TurnType::Orval].v);
    printf("  - Dia45: %0.2f\n", param_set.map_fast[TurnType::Dia45].v);
    printf("  - Dia45_2: %0.2f\n", param_set.map_fast[TurnType::Dia45_2].v);
    printf("  - Dia135: %0.2f\n", param_set.map_fast[TurnType::Dia135].v);
    printf("  - Dia135_2: %0.2f\n", param_set.map_fast[TurnType::Dia135_2].v);
    printf("  - Dia90: %0.2f\n", param_set.map_fast[TurnType::Dia90].v);

    // normal
    printf("normal: \n");
    printf("  - Large: %0.2f\n", param_set.map[TurnType::Large].v);
    printf("  - Orval: %0.2f\n", param_set.map[TurnType::Orval].v);
    printf("  - Dia45: %0.2f\n", param_set.map[TurnType::Dia45].v);
    printf("  - Dia45_2: %0.2f\n", param_set.map[TurnType::Dia45_2].v);
    printf("  - Dia135: %0.2f\n", param_set.map[TurnType::Dia135].v);
    printf("  - Dia135_2: %0.2f\n", param_set.map[TurnType::Dia135_2].v);
    printf("  - Dia90: %0.2f\n", param_set.map[TurnType::Dia90].v);

    // slow
    printf("slow: \n");
    printf("  - Large: %0.2f\n", param_set.map_slow[TurnType::Large].v);
    printf("  - Orval: %0.2f\n", param_set.map_slow[TurnType::Orval].v);
    printf("  - Dia45: %0.2f\n", param_set.map_slow[TurnType::Dia45].v);
    printf("  - Dia45_2: %0.2f\n", param_set.map_slow[TurnType::Dia45_2].v);
    printf("  - Dia135: %0.2f\n", param_set.map_slow[TurnType::Dia135].v);
    printf("  - Dia135_2: %0.2f\n", param_set.map_slow[TurnType::Dia135_2].v);
    printf("  - Dia90: %0.2f\n", param_set.map_slow[TurnType::Dia90].v);
  }

  for (int i = 1; i <= 5; i++) {
    lgc->set_param_num(i);
    pc->other_route_map.clear();
    const bool res = pc->path_create(false);
    if (dump_all) {
      printf("other route size = %d\n", pc->other_route_map.size());
    }
    if (!res) {
      ui_->error();
      return;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
    if (i == 0) {
      pc->path_s2.clear();
      pc->path_t2.clear();
      for (int i = 0; i < pc->path_t.size(); i++) {
        pc->path_s2.push_back(pc->path_s[i]);
        pc->path_t2.push_back(pc->path_t[i]);
      }
    }
    path_set_t p;
    p.type = i;
    p.time = 10000;
    pc->timebase_path_create(false, param_set, p);
    pc->path_set_map.push(p);
  }

  // 先頭に最適な経路を持ってくる
  const auto top_p = pc->path_set_map.top();
  if (top_p.result) { //成功
    pc->path_s.clear();
    pc->path_t.clear();
    for (int i = 0; i < top_p.path_s.size(); i++) {
      pc->path_s.push_back(top_p.path_s[i]);
      pc->path_t.push_back(top_p.path_t[i]);
    }
    // clear map
    while (!pc->path_set_map.empty()) {
      auto p2 = pc->path_set_map.top();
      p2.path_s.clear();
      p2.path_t.clear();
      pc->path_set_map.pop();
    }

  } else { //失敗
    pc->other_route_map.clear();
    const bool res = pc->path_create(false);
    if (!res) {
      ui_->error();
      return;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
  }
  if (pc->path_s.size() == 0) {
    pc->other_route_map.clear();
    const bool res = pc->path_create(false);
    if (!res) {
      ui_->error();
      return;
    }
    pc->convert_large_path(true);
    pc->diagonalPath(true, true);
    pc->print_path();
  }

  pc->calc_goal_time(param_set, true);
  pc->print_path();

  printf("----------------\n");
}