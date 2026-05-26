#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

// ============================================================
// テストモード dispatch (user_mode != 0)
// ============================================================
void MainTask::run_test_mode(int mode) {
  if (mode != 0) {
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
      test_suction();
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
