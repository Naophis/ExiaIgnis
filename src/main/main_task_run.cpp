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

  static constexpr int SUB_MODE_COUNT = 4;

  while (true) {
    int sub = select_run_mode(SUB_MODE_COUNT);
    printf("[main] sub_mode=%d\n", sub);

    sleep_ms(10);
  }
}
