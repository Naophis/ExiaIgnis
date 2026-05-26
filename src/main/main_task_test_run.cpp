#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

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
  if (param_->test_log_enable > 0) {
    lt_->start();
  }
  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();
  req_error_reset();

  planning_->set_search_mode(test_search_mode > 0);

  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  ps.dist = sys_.test.dist - 5;
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
  ui_.coin(120);
}

void MainTask::test_back() {
  mp->reset_gyro_ref_with_check();

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
    lt_->start_slalom_log();
  }
  ps.v_max = sys_.test.v_max;
  ps.v_end = -20;
  ps.dist = sys_.test.dist - 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  mp->go_straight(ps);
  sleep_ms(100);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  lt_->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  lt_->save(slalom_log_file);
  ui_.coin(120);

  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_front_wall_offset() {
  const auto se = get_sensing_entity();
  printf("search_walloff_offset= %f, %f\n",
         param_->sen_ref_p.search_exist.offset_l,
         param_->sen_ref_p.search_exist.offset_r);

  mp->reset_gyro_ref_with_check();

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
    lt_->start_slalom_log();
  }
  // planning_->active_logging(_f);

  ps.v_max = sys_.test.v_max;
  ps.v_end = sys_.test.v_max;
  ps.dist = param_->cell + param_->cell2 / 2;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  ps.dist = param_->cell2 / 2 - 5;
  if (se->ego.left90_mid_dist < param_->sensor_range_mid_max &&
      se->ego.right90_mid_dist < param_->sensor_range_mid_max) {
    ps.dist -= (param_->front_dist_offset2 - se->ego.front_mid_dist);
  }

  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  mp->go_straight(ps);

  sleep_ms(100);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  lt_->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt_->save(slalom_log_file);
  ui_.coin(120);

  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_front_ctrl(bool mode) {
  file_idx = sys_.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui_.error();
    return;
  }

  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();
  req_error_reset();

  mp->front_ctrl(mode);
  planning_->motor_disable();
  ui_.coin(120);

  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
}
