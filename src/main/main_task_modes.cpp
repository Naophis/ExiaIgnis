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
  const auto se = get_sensing_entity();
  // tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
  // tgt_val->nmr.timstamp++;

  const char ESC = '\033';
  while (1) {

    printf("%c[2J", ESC);   /* 画面消去 */
    printf("%c[0;0H", ESC); /* 戦闘戻す*/

    printf("SW1 %d \n", gpio_get(BTN_PIN));
    printf("Temp %f \n", se->ego.temp);

    printf("gyro: %d\t(%0.3f)\n", se->gyro.raw,
           planning_->tgt_val->gyro_zero_p_offset);
    printf("gyro2: %d\t(%0.3f)\n", se->gyro2.raw,
           planning_->tgt_val->gyro2_zero_p_offset);
    printf("accel_x: %f\t(%f)\n", se->ego.accel_x_raw,
           se->ego.accel_x_raw / 9806.65 * param_->accel_x_param.gain);
    // printf("accel_y: %d\n", se->accel_y.raw);
    printf("battery: %0.3f (%d)\n", se->ego.battery_lp, se->battery.raw);
    printf("encoder: %ld, %ld\n", (long)se->encoder.left,
           (long)se->encoder.right);
    printf("sensor: %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
           se->led_sen.left90.raw, se->led_sen.left45.raw,
           se->led_sen.left45_2.raw, se->led_sen.front.raw,
           se->led_sen.right45_2.raw, se->led_sen.right45.raw,
           se->led_sen.right90.raw);

    printf("sensor_lp: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n",
           se->ego.left90_lp,    //
           se->ego.left45_3_lp,  //
           se->ego.left45_2_lp,  //
           se->ego.left45_lp,    //
           se->ego.right45_lp,   //
           se->ego.right45_2_lp, //
           se->ego.right45_3_lp, //
           se->ego.right90_lp);

    printf("sensor_dist(near): %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, "
           "%3.2f, %3.2f, %3.2f\n",
           se->ego.left90_dist,    //
           se->ego.left45_3_dist,  //
           se->ego.left45_2_dist,  //
           se->ego.left45_dist,    //
           se->ego.front_dist,     //
           se->ego.right45_dist,   //
           se->ego.right45_2_dist, //
           se->ego.right45_3_dist, //
           se->ego.right90_dist);

    printf("sensor_dist(mid): %3.2f, %3.2f, %3.2f\n",
           se->ego.left90_mid_dist, //
           se->ego.front_mid_dist,  //
           se->ego.right90_mid_dist);
    printf("sensor_dist(far): %3.2f, %3.2f, %3.2f\n",
           se->ego.left90_far_dist, //
           se->ego.front_far_dist,  //
           se->ego.right90_far_dist);

    auto l90_b = planning_->adjust_b_to_target90(se->led_sen.left90.raw,
                                                 param_->sensor_gain.l90.a);
    auto l90_far_b = planning_->adjust_b_to_target90(
        se->led_sen.left90.raw, param_->sensor_gain.l90_far.a);
    auto l45_b = planning_->adjust_b_to_target45(se->led_sen.left45.raw,
                                                 param_->sensor_gain.l45.a);
    auto l45_2_b = planning_->adjust_b_to_target45(se->led_sen.left45_2.raw,
                                                   param_->sensor_gain.l45_2.a);
    auto l45_3_b = planning_->adjust_b_to_target45(se->led_sen.left45_3.raw,
                                                   param_->sensor_gain.l45_3.a);
    auto front_b = planning_->adjust_b_to_target90(se->led_sen.front.raw,
                                                   param_->sensor_gain.front.a);
    auto r45_3_b = planning_->adjust_b_to_target45(se->led_sen.right45_3.raw,
                                                   param_->sensor_gain.r45_3.a);
    auto r45_2_b = planning_->adjust_b_to_target45(se->led_sen.right45_2.raw,
                                                   param_->sensor_gain.r45_2.a);
    auto r45_b = planning_->adjust_b_to_target45(se->led_sen.right45.raw,
                                                 param_->sensor_gain.r45.a);
    auto r90_b = planning_->adjust_b_to_target90(se->led_sen.right90.raw,
                                                 param_->sensor_gain.r90.a);
    auto r90_far_b = planning_->adjust_b_to_target90(
        se->led_sen.right90.raw, param_->sensor_gain.r90_far.a);
    // printf("side_sensor_b: %f, %f, %f, %f, %f, %f\n", //
    //        l45_3_b, l45_2_b, l45_b, r45_b, r45_2_b, r45_3_b);
    printf("L45_3: [%f, %f]\n", param_->sensor_gain.l45_3.a, l45_3_b);
    printf("L45_2: [%f, %f]\n", param_->sensor_gain.l45_2.a, l45_2_b);
    printf("L45: [%f, %f]\n", param_->sensor_gain.l45.a, l45_b);
    printf("R45: [%f, %f]\n", param_->sensor_gain.r45.a, r45_b);
    printf("R45_2: [%f, %f]\n", param_->sensor_gain.r45_2.a, r45_2_b);
    printf("R45_3: [%f, %f]\n", param_->sensor_gain.r45_3.a, r45_3_b);

    printf("L90_near: [%f, %f]\n", param_->sensor_gain.l90.a, l90_b);
    printf("R90_near: [%f, %f]\n", param_->sensor_gain.r90.a, r90_b);
    printf("L90_mid: [%f, %f]\n", param_->sensor_gain.l90.a, l90_b);
    printf("R90_mid: [%f, %f]\n", param_->sensor_gain.r90.a, r90_b);
    printf("L90_far: [%f, %f]\n", param_->sensor_gain.l90_far.a, l90_far_b);
    printf("R90_far: [%f, %f]\n", param_->sensor_gain.r90_far.a, r90_far_b);

    printf("front_sensor_b: %f, %f, %f, %f\n", //
           l90_b, l90_far_b, r90_b, r90_far_b);

    printf("ego_v: %4.3f, %4.3f, %4.3f, %4.3f, (%4ld, %4ld)\n", se->ego.v_l,
           se->ego.v_c, se->ego.v_r, planning_->tgt_val->ego_in.dist,
           (long)se->encoder.left, (long)se->encoder.right);

    printf("calc_v: %4.3f, %3.3f\n", planning_->tgt_val->ego_in.v,
           planning_->tgt_val->ego_in.w);

    printf("ego_w: %2.3f, %2.3f, %2.3f, %3.3f deg\n", se->ego.w_raw,
           se->ego.w_lp, planning_->tgt_val->ego_in.ang,
           planning_->tgt_val->ego_in.ang * 180 / m_PI);

    printf("gyro_raw[]: %4d, %4d, %4d, %4d, %4d\n", se->gyro_list[0],
           se->gyro_list[1], se->gyro_list[2], se->gyro_list[3],
           se->gyro_list[4]);

    const float tgt_gain =
        1000.0 / (se->accel_x.raw - planning_->tgt_val->accel_x_zero_p_offset) *
        9.8;
    printf("accel: %3.3f, %6.6f\n", se->ego.accel_x_raw, tgt_gain);

    // printf("duty: %3.3f, %3.3f\n", se->ego.duty.duty_l,
    //        se->ego.duty.duty_r);

    printf("planning_time: %d\n", planning_->tgt_val->calc_time);
    printf("planning_time_diff: %d\n", planning_->tgt_val->calc_time_diff);
    printf("sensing_time: %d\n", se->calc_time);

    if (ui_.button_state()) {
      planning_->tgt_val->ego_in.ang = planning_->tgt_val->ego_in.dist = 0;
    }

    sleep_ms(100);
  }
}

void MainTask::dump2() {}

void MainTask::test_dia_walloff() {}

void MainTask::test_pivot_n() {}

void MainTask::test_pivot_n2() {}

void MainTask::encoder_test() {}

void MainTask::test_search_pivot() {}

void MainTask::test_system_identification(bool para) {}