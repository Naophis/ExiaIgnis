#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

void MainTask::test_turn() {
  const auto rorl = ui_.select_direction();
  const auto se = get_sensing_entity();

  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();

  planning_->motor_enable();

  req_error_reset();

  load_slalom_param(0, 0, 0);
  auto str_p = param_set.str_map[StraightType::Search];

  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }
  // planning_->active_logging();
  pr.w_max = sys_.test.w_max;
  pr.alpha = sys_.test.alpha;
  pr.w_end = str_p.w_end;
  pr.ang = sys_.test.ang * m_PI / 180;
  pr.RorL = rorl;

  mp->pivot_turn(pr);
  sleep_ms(100);
  planning_->motor_disable();
  planning_->suction_disable();

  lt_->stop();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  ui_.coin(120);

  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_csv();
  while (1) {
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
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

void MainTask::test_pivot_n() {
  rorl = ui_.select_direction();

  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  req_error_reset();

  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }
  // planning_->active_logging();
  pr.w_max = sys_.test.w_max;
  pr.alpha = sys_.test.alpha;
  pr.w_end = 0;
  pr.ang = sys_.test.ang * m_PI / 180;
  pr.RorL = rorl;

  for (int i = 0; i < sys_.test.turn_times; i++) {
    mp->pivot_turn(pr);
    sleep_ms(50);
  }
  planning_->motor_disable();
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

void MainTask::test_pivot_n2() {
  // rorl = ui_.select_direction();
  // search_ctrl->set_lgc(lgc);
  // search_ctrl->set_motion_plannning(mp);
  // pc->set_logic(lgc);
  // pc->set_userinterface(ui);
  // load_slalom_param(0, 0, 0);
  // // sr = search_ctrl->exec(param_set, SearchMode::ALL);

  // mp->reset_gyro_ref_with_check();
  // reset_tgt_data();
  // reset_ego_data();
  // planning_->motor_enable();

  // req_error_reset();

  // if (param_->test_log_enable > 0) {
  //   lt_->start_slalom_log();
  // }
  // // planning_->active_logging();
  // pr.w_max = sys_.test.w_max;
  // pr.alpha = sys_.test.alpha;
  // pr.w_end = 0;
  // pr.ang = sys_.test.ang * m_PI / 180;
  // pr.RorL = rorl;

  // for (int i = 0; i < sys_.test.turn_times; i++) {
  //   search_ctrl->pivot(param_set, 0);
  //   sleep_ms(50);
  // }
  // planning_->motor_disable();
  // planning_->suction_disable();

  // lt_->stop_slalom_log();
  // reset_tgt_data();
  // reset_ego_data();
  // req_error_reset();
  // lt_->save(slalom_log_file);
  // ui_.coin(120);

  // while (1) {
  //   if (ui_.button_state_hold())
  //     break;
  //   sleep_ms(10);
  // }
  // lt_->dump_log(slalom_log_file);
  // while (1) {
  //   if (ui_.button_state_hold())
  //     break;
  //   sleep_ms(10);
  // }
}

void MainTask::test_search_pivot() {

  file_idx = 0;

  if (file_idx >= tpp.file_list_size) {
    ui_.error();
    return;
  }
  search_ctrl->set_motion_plannning(mp);
  load_slalom_param(0, 0, 0);
  sla_p = param_set.map[TurnType::Normal];
  str_p = param_set.str_map[StraightType::Search];

  printf("str: \n");
  printf("- v_max: %f\n", str_p.v_max);
  printf("- accl: %f\n", str_p.accl);
  printf("- decel: %f\n", str_p.decel);

  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  req_error_reset();

  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }
  // run 1.5cell
  ps.v_max = str_p.v_max;
  ps.v_end = str_p.v_max;
  ps.dist = param_->cell2 * 1.5 + param_->offset_start_dist_search;
  ps.accl = str_p.accl;
  ps.decel = str_p.decel;
  ps.sct = SensorCtrlType::Straight;
  planning_->set_search_mode(true);
  mp->go_straight(ps);

  // exec search pivot
  search_ctrl->adachi = nullptr;
  search_ctrl->pivot(param_set, 0);
  // run back
  ps.v_max = str_p.v_max;
  ps.v_end = 20;
  ps.dist = param_->cell2 / 2 - 5;
  ps.accl = str_p.accl;
  ps.decel = str_p.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = 10;
  ps.dist = 5;
  ps.accl = str_p.accl;
  ps.decel = str_p.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  sleep_ms(100);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();
  lt_->stop_slalom_log();

  lt_->save(slalom_log_file);
  ui_.coin(120);

  param_->sen_ref_p.normal.exist.right45 = backup_r;
  param_->sen_ref_p.normal.exist.left45 = backup_l;
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
  param_->sen_ref_p.normal.expand.right45 = backup_r_expand;
  param_->sen_ref_p.normal.expand.left45 = backup_l_expand;
}
