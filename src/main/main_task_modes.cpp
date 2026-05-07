#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

// ─── LED ヘルパー
// ────────────────────────────────────────────────────────────── mode_num+1 を
// 6bit 2進数で表示。Astraea の lbit.byte = mode_num+1 と同等。
static void show_mode_led(UserInterface &ui, int mode) {
  int v = mode + 1;
  ui.LED_bit((v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1,
             (v >> 4) & 1, (v >> 5) & 1);
}

// ─── 共通センサー表示
// ─────────────────────────────────────────────────────────
static void print_sensing(const SensingTask::Data &d,
                          const sensing_result_entity_t &se,
                          const PlanningTask::State &ps, const system_t &sys,
                          bool suction_test_on) {
  const char ESC = '\033';
  printf("%c[2J", ESC);
  printf("%c[0;0H", ESC);

  printf("=== ExiaIgnis sensor monitor ===\n");

  printf("[timing]  dt=%4lu us  |  sense=%4lu us"
         "  |  gz->encR: %4llu us"
         "  |  encR->encL: %4llu us\n",
         d.dt_us, d.sense_duration_us, d.enc_r_ts - d.gz_ts,
         d.enc_l_ts - d.enc_r_ts);

  printf("[sensor]  %4u"
         "  |  %4u\t%4u\t%4u"
         "  |  %4u\t%4u\t%4u"
         "  |  %4u\n",
         se.led_sen.left90.raw, se.led_sen.left45_3.raw,
         se.led_sen.left45_2.raw, se.led_sen.left45.raw, se.led_sen.right45.raw,
         se.led_sen.right45_2.raw, se.led_sen.right45_3.raw,
         se.led_sen.right90.raw);

  printf("[motion]  gz=%6d (dt=%4llu us)"
         "  |  encL=%5u (dt=%4llu us)"
         "  |  encR=%5u (dt=%4llu us)\n",
         se.gyro.raw, d.gz_dt, d.enc_l, d.enc_l_dt, d.enc_r, d.enc_r_dt);

  printf("[power]   bat=%4u, %.4f\n", se.battery.raw, se.ego.batt_kf);

  printf("[planning] mode=%u  v=%6.1f  w=%6.3f"
         "  dist=%7.1f  ang=%6.3f\n",
         (uint8_t)ps.mode, ps.img_v, ps.img_w, ps.img_dist, ps.img_ang);
  printf("[control]  duty_l=%6.1f%%  duty_r=%6.1f%%"
         "  suction=%5.1f%%  tick=%lu  %s\n",
         ps.duty_l, ps.duty_r, ps.duty_suction, ps.tick,
         suction_test_on ? "[SUCTION TEST]" : "");

  printf("[sys]      mode=%d  maze=%d  goals=%u\n", sys.user_mode,
         sys.maze_size, (unsigned)sys.goals.size());
}

// ─── ボタン待機ヘルパー
// ───────────────────────────────────────────────────────
static void wait_button(UserInterface &ui) {
  while (!ui.button_state_hold())
    sleep_ms(10);
}

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
    lbit.byte = mode_num + 1;
    ui_.LED_bit(lbit.b0, lbit.b1, lbit.b2, lbit.b3, lbit.b4, lbit.b5);
    if (ui_.button_state_hold()) {
      ui_.coin(100);
      break;
    }
    // vTaskDelay(xDelay1);
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

// ============================================================
// テストモード dispatch (user_mode != 0)
// Astraea task() の if(mode != 0) ブロックを踏襲。
// ============================================================
void MainTask::run_test_mode(int mode) {
  if (mode != 0) {
    // mount();
    // mock slalom();
    // test_search_sla(false);
    if (mode == 1) {
      printf("test_sla\n");
      test_sla();
    } else if (mode == 2) {
      printf("test_run\n");
      test_run();
    } else if (mode == 3) {
      printf("test_turn\n");
      test_turn();
    } else if (mode == 4) {
      printf("test_run_sla\n");
      test_run_sla();
    } else if (mode == 5) {
      printf("test_search_sla\n");
      test_search_sla(true);
    } else if (mode == 6) {
      printf("test_front_wall_offset\n");
      test_front_wall_offset();
    } else if (mode == 7) {
      printf("test_front_ctrl hold\n");
      test_front_ctrl(true);
    } else if (mode == 8) {
      printf("test_front_ctrl \n");
      test_front_ctrl(false);
    } else if (mode == 9) {
      printf("test_sla_walloff\n");
      test_sla_walloff();
    } else if (mode == 10) {
      printf("back\n");
      test_back();
    } else if (mode == 11) {
      printf("suction\n");
      // mp->reset_gyro_ref_with_check();
    } else if (mode == 12) {
      printf("hold\n");
    } else if (mode == 13) {
      printf("keep_pivot\n");
      keep_pivot();
    } else if (mode == 14) {
      printf("echo_sensor_csv\n");
      dump1();
    } else if (mode == 15) {
      printf("echo_printf\n");
      dump2();
    } else if (mode == 16) {
      printf("sys id para\n");
      test_system_identification(false);
    } else if (mode == 17) {
      printf("sys id roll\n");
      test_system_identification(true);
    } else if (mode == 18) {
    } else if (mode == 19) {
      printf("test_dia_walloff\n");
      test_dia_walloff();
    } else if (mode == 20) {
      printf("test_pivot_n\n");
      test_pivot_n();
    } else if (mode == 21) {
      printf("test_pivot_n2\n");
      test_pivot_n2();
    } else if (mode == 22) {
      encoder_test();
    } else if (mode == 23) {
      printf("load_circuit_path\n");
      test_search_pivot();
    }
  }
}

// ============================================================

void MainTask::test_sla() {}

void MainTask::test_run() {}

void MainTask::test_turn() {}

void MainTask::test_run_sla() {}

void MainTask::test_search_sla(bool wall_off) {}

void MainTask::test_front_wall_offset() {}

void MainTask::test_front_ctrl(bool enable) {}

void MainTask::test_sla_walloff() {}

void MainTask::test_back() {}

void MainTask::keep_pivot() {}

void MainTask::dump1() {
  
}

void MainTask::dump2() {}

void MainTask::test_dia_walloff() {}

void MainTask::test_pivot_n() {}

void MainTask::test_pivot_n2() {}

void MainTask::encoder_test() {}

void MainTask::test_search_pivot() {}

void MainTask::test_system_identification(bool para) {}