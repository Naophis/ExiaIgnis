#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

// ============================================================
// テストモード dispatch (user_mode != 0)
// Astraea task() の if(mode != 0) ブロックを踏襲。
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

// ============================================================

void MainTask::test_sla() {
  //
}

void MainTask::test_run() {
  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  if (sys_.test.suction_active == 1) {
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);
    sleep_ms(500);
  } else if (sys_.test.suction_active == 2) {
    planning_->suction_enable(sys_.test.suction_duty_burst,
                              sys_.test.suction_duty_burst_low);
    sleep_ms(500);
  }

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  req_error_reset();
  if (param_->test_log_enable > 0) {
    lt_->start();
  }
  planning_->set_search_mode(test_search_mode > 0);

  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  ps.dist = sys_.test.dist - 5;
  // + param_->offset_start_dist;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
  if (sys_.test.dia == 1) {
    ps.sct = SensorCtrlType::Dia;
  }
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  mp->go_straight(ps);
  // lt->stop_slalom_log();
  // bool front_ctrl = (sensing_result->ego.front_dist < 60);
  // vTaskDelay(50.0 / portTICK_RATE_MS);
  // sleep_ms(50);
  planning_->motor_disable();
  lt_->stop();

  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  ui_.coin(120);
  planning_->set_search_mode(false);
  // 1回目: バイナリ dump (rx_term.js バイナリプロトコル)
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_csv();
  ui_.coin(120);
  // 2回目: テキスト dump (rx_term.js テキストプロトコル)
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_csv_text();
  ui_.coin(120);
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_turn() {}

void MainTask::test_run_sla() {}

void MainTask::test_search_sla(bool wall_off) {}

void MainTask::test_front_wall_offset() {}

void MainTask::test_front_ctrl(bool enable) {}

void MainTask::test_sla_walloff() {}

void MainTask::test_back() {}

void MainTask::test_suction() {
  const auto se = get_sensing_entity();
  const float target_v = sys_.test.suction_duty;

  mp->reset_gyro_ref_with_check();
  sleep_ms(250);
  planning_->suction_enable(sys_.test.suction_duty, sys_.test.suction_duty_low);

  while (!ui_.button_state()) {
    printf("suction target=%.2fV battery=%.3fV duty=%.1f%%\n", target_v,
           se->ego.batt_kf, planning_->tgt_val->duty_suction);
    sleep_ms(200);
  }

  planning_->suction_disable();
  printf("suction stopped\n");
}

void MainTask::keep_pivot() {
  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->motor_enable();
  req_error_reset();
  ps.v_max = 0.000000001;
  ps.v_end = 0.000000001;
  ps.dist = 1000;
  ps.accl = 0.1;
  ps.decel = -1;
  ps.sct = SensorCtrlType::NONE;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;
  mp->go_straight(ps);
  while (1) {
    sleep_ms(1);
    if (ui_.button_state_hold()) {
      break;
    }
  }
  planning_->motor_disable();
}

void MainTask::dump1() {

  mp->reset_gyro_ref_with_check();
  sleep_ms(250);

  const auto se = get_sensing_entity();

  const char ESC = '\033';
  while (1) {

    printf("%c[2J", ESC);
    printf("%c[0;0H", ESC);

    printf("SW1 %d \n", gpio_get(BTN_PIN));
    printf("Temp %f \n", se->ego.temp);

    printf("gyro: %d\t(%0.3f)\n", se->gyro.raw,
           planning_->tgt_val->gyro_zero_p_offset);
    printf("gyro2: %d\t(%0.3f)\n", se->gyro2.raw,
           planning_->tgt_val->gyro2_zero_p_offset);
    printf("accel_x: %f\t(%f)\n", se->ego.accel_x_raw,
           se->ego.accel_x_raw / 9806.65 * param_->accel_x_param.gain);
    printf("battery: %0.3f (%d)\n", se->ego.battery_lp, se->battery.raw);
    printf("encoder: %ld, %ld\n", (long)se->encoder.left,
           (long)se->encoder.right);
    printf("sensor: %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
           se->led_sen.left90.raw, se->led_sen.left45.raw,
           se->led_sen.left45_2.raw, se->led_sen.front.raw,
           se->led_sen.right45_2.raw, se->led_sen.right45.raw,
           se->led_sen.right90.raw);

    printf(
        "sensor_lp: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n",
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

    printf("planning_time: %d\t%d\n", planning_->tgt_val->calc_time_diff,
           planning_->tgt_val->calc_time);
    printf("sensing_time: %d\t%d\n", se->calc_time, se->calc_time2);

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
