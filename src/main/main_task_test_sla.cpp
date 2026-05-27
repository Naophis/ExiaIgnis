#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

void MainTask::test_sla() {

  if (file_idx >= tpp.file_list_size) {
    printf("%d %d\n", file_idx, tpp.file_list_size);
    ui_->error();
    return;
  }
  // silent_load = false;
  load_turn_param_profiles(true, file_idx);
  load_slalom_param2(file_idx);
  sla_p = param_set.map[static_cast<TurnType>(sys_.test.sla_type)];
  auto sla_p2 = param_set.map[static_cast<TurnType>(sys_.test.sla_type2)];
  printf("slalom params[0]:\n");
  printf("  v: %f\n", sla_p.v);
  printf("  ang: %f\n", sla_p.ang * 180 / m_PI);
  printf("  ref_ang: %f\n", sla_p.ref_ang * 180 / m_PI);
  printf("  rad: %f\n", sla_p.rad);
  printf("  time:  %f\n", sla_p.time);
  printf("  n: %d\n", sla_p.pow_n);
  printf("  front: [%f, %f]\n", sla_p.front.left, sla_p.front.right);
  printf("  back: [%f, %f]\n", sla_p.back.left, sla_p.back.right);

  printf("slalom params[1]:\n");
  printf("  v: %f\n", sla_p2.v);
  printf("  ang: %f\n", sla_p2.ang * 180 / m_PI);
  printf("  rad: %f\n", sla_p2.rad);
  printf("  time:  %f\n", sla_p2.time);
  printf("  n: %d\n", sla_p2.pow_n);
  printf("  front: [%f, %f]\n", sla_p2.front.left, sla_p2.front.right);
  printf("  back: [%f, %f]\n", sla_p2.back.left, sla_p2.back.right);

  rorl = ui_->select_direction();
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);

  backup_r = param_->sen_ref_p.normal.exist.right45;
  backup_l = param_->sen_ref_p.normal.exist.left45;
  if (rorl == TurnDirection::Right) {
    param_->sen_ref_p.normal.exist.left45 += 10;
  } else {
    param_->sen_ref_p.normal.exist.right45 += 10;
  }

  mp->reset_gyro_ref_with_check();

  if (sys_.test.suction_active == 1) {
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);
    sleep_ms(1000);
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

  ps.v_max = sla_p.v;
  ps.v_end = sla_p.v;
  ps.dist = param_->cell + param_->offset_start_dist;
  nm.skip_wall_off = false;

  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;

  if (sys_.test.start_turn > 0) {
    if (rorl == TurnDirection::Left) {
      ps.dist = param_->offset_start_dist + sla_p.front.left;
    } else {
      ps.dist = param_->offset_start_dist + sla_p.front.right;
    }
    nm.skip_wall_off = true;
  }

  auto tmp_v2 = 2 * ps.accl * ps.dist;
  if (ps.v_end * ps.v_end > tmp_v2) {
    ps.accl = (ps.v_end * ps.v_end) / (2 * ps.dist) + 1000;
    ps.decel = -ps.accl;
  }

  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;
  mp->go_straight(ps);

  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys_.test.accl;
  nm.decel = sys_.test.decel;
  nm.is_turn = true;
  if (sys_.test.sla_return > 0) {
    nm.v_max = sla_p2.v;
    nm.v_end = sla_p2.v;
  }
  auto lim_size = param_->sensor_deg_limitter_v.size();
  // TODO: backup sensor_deg_limitter_str, sensor_deg_limitter_dia,
  // sensor_deg_limitter_piller values

  std::vector<float> bk_sensor_deg_limitter_str(lim_size);
  std::vector<float> bk_sensor_deg_limitter_dia(lim_size);
  std::vector<float> bk_sensor_deg_limitter_piller(lim_size);
  for (int i = 0; i < lim_size; i++) {
    bk_sensor_deg_limitter_str[i] = param_->sensor_deg_limitter_str[i];
    bk_sensor_deg_limitter_dia[i] = param_->sensor_deg_limitter_dia[i];
    bk_sensor_deg_limitter_piller[i] = param_->sensor_deg_limitter_piller[i];
    // param_->sensor_deg_limitter_str[i] = 0.0;
    param_->sensor_deg_limitter_dia[i] = 0.0;
    param_->sensor_deg_limitter_piller[i] = 0.0;
  }

  mp->slalom(sla_p, rorl, nm, false);
  mp->wall_off_controller->continuous_turn_flag = true;
  param_->sen_ref_p.normal.exist.right45 = backup_r;
  param_->sen_ref_p.normal.exist.left45 = backup_l;

  if (sys_.test.sla_return > 0) {
    const auto type2 = static_cast<TurnType>(sys_.test.sla_type2);
    bool dia = type2 == TurnType::Dia45_2 || type2 == TurnType::Dia135_2 ||
               type2 == TurnType::Dia90;
    param_->sen_ref_p.normal.exist.right45 = 1;
    param_->sen_ref_p.normal.exist.left45 = 1;
    // clear deg limitter for slalom return
    for (int i = 0; i < lim_size; i++) {
      param_->sensor_deg_limitter_str[i] = 0.0;
      param_->sensor_deg_limitter_dia[i] = 0.0;
      param_->sensor_deg_limitter_piller[i] = 0.0;
    }
    mp->slalom(sla_p2, rorl2, nm, dia);
    mp->wall_off_controller->continuous_turn_flag = true;
  } else if (sys_.test.turn_times > 0) {
    for (int i = 0; i < sys_.test.turn_times; i++) {

      if (static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia45 ||
          static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia135) {
        if ((i & 0x01) == 0x00) {
          mp->slalom(sla_p2, rorl, nm, true);
          mp->wall_off_controller->continuous_turn_flag = true;
        } else {
          mp->slalom(sla_p, rorl, nm, false);
          mp->wall_off_controller->continuous_turn_flag = true;
        }
      } else {
        mp->slalom(sla_p, rorl, nm);
        mp->wall_off_controller->continuous_turn_flag = true;
      }
    }
  }
  // restore sensor deg limitter values
  for (int i = 0; i < lim_size; i++) {
    param_->sensor_deg_limitter_str[i] = bk_sensor_deg_limitter_str[i];
    param_->sensor_deg_limitter_dia[i] = bk_sensor_deg_limitter_dia[i];
    param_->sensor_deg_limitter_piller[i] = bk_sensor_deg_limitter_piller[i];
  }
  ps.v_max = sla_p.v;
  ps.v_end = sys_.test.end_v;
  ps.dist = param_->cell;

  if (sys_.test.sla_return == 0) {
    if (static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia45 ||
        static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia135) {
      ps.dist = param_->cell / 2 * ROOT2;
    }
  } else if (sys_.test.sla_return == 1) {
    if (static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia90) {
      ps.dist = param_->cell / 2 * ROOT2;
    }
  }
  if (sys_.test.ignore_opp_sen > 0) {
    ps.v_max = std::max(sys_.test.v_max, sla_p.v);
    ps.dist = sys_.test.dist;
  }
  ps.sct = SensorCtrlType::NONE;
  if (static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia45 ||
      static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia135) {
    if (sys_.test.ignore_opp_sen == 2) {
      ps.sct = SensorCtrlType::Dia;
    }
  }
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  if (static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia45 ||
      static_cast<TurnType>(sys_.test.sla_type) == TurnType::Dia135) {
    ps.accl = sys_.test.dia_accl;
    ps.decel = sys_.test.dia_decel;
  }
  ps.motion_type = MotionType::STRAIGHT;
  // ps.motion_type = MotionType::SLA_BACK_STR;

  mp->go_straight(ps);
  mp->wall_off_controller->continuous_turn_flag = false;

  sleep_ms(25);

  lt_->stop_slalom_log();

  sleep_ms(75);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->motor_disable();
  planning_->suction_disable();

  lt_->save(slalom_log_file);
  ui_->coin(120);

  param_->sen_ref_p.normal.exist.right45 = backup_r;
  param_->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);

  sleep_ms(500);
  printf("----------------------------------\n");
  printf("offset: min(%f, %f) + %f = %f\n", mp->g_offset_y_l, mp->g_offset_y_r,
         mp->g_offset_x1, mp->g_total_offset);
  printf("theta: %f\n", mp->g_sen_ang * 180 / m_PI);
  printf("sensor dist: %f, %f\n", mp->g_sen_l_dist, mp->g_sen_r_dist);
  printf("----------------------------------\n");

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_run_sla() {
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
    lt_->start_slalom_log();
  }
  // planning_->active_logging(_f);

  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  ps.dist = param_->cell / 2 + param_->offset_start_dist + param_->cell;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
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
  lt_->save(slalom_log_file);
  ui_->coin(120);

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_search_sla(bool wall_off) {

  file_idx = 0;

  if (file_idx >= tpp.file_list_size) {
    ui_->error();
    return;
  }

  load_slalom_param(0, 0, 0);
  sla_p = param_set.map[TurnType::Normal];
  str_p = param_set.str_map[StraightType::Search];

  printf("str: \n");
  printf("- v_max: %f\n", str_p.v_max);
  printf("- accl: %f\n", str_p.accl);
  printf("- decel: %f\n", str_p.decel);

  rorl = ui_->select_direction();
  backup_r = param_->sen_ref_p.normal.exist.right45;
  backup_l = param_->sen_ref_p.normal.exist.left45;
  backup_r_expand = param_->sen_ref_p.normal.expand.right45;
  backup_l_expand = param_->sen_ref_p.normal.expand.left45;
  param_->sen_ref_p.normal.expand.right45 =
      param_->sen_ref_p.normal.exist.right45;
  param_->sen_ref_p.normal.expand.left45 =
      param_->sen_ref_p.normal.exist.left45;
  param_->clear_dist_ragne_from = param_->clear_dist_ragne_to = 0;
  // if (sys_.test.ignore_opp_sen) {
  //   if (rorl == TurnDirection::Right) {
  //     param_->sen_ref_p.normal.exist.right45 = 1;
  //   } else {
  //     param_->sen_ref_p.normal.exist.left45 = 1;
  //   }
  // }
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);
  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  req_error_reset();

  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }

  ps.v_max = str_p.v_max;
  ps.v_end = str_p.v_max;
  ps.dist = param_->cell2 / 2 + param_->offset_start_dist_search;
  ps.accl = str_p.accl;
  ps.decel = str_p.decel;
  ps.sct = SensorCtrlType::Straight;

  mp->go_straight(ps);

  nm.v_max = str_p.v_max;
  nm.v_end = sla_p.v;
  nm.accl = str_p.accl;
  nm.decel = str_p.decel;
  nm.is_turn = true;

  planning_->set_search_mode(true);
  mp->slalom(sla_p, rorl, nm);
  for (int i = 0; i < sys_.test.turn_times; i++) {
    mp->slalom(sla_p, rorl, nm);
  }

  ps.v_max = sla_p.v;
  ps.v_end = 20;
  ps.dist = param_->cell2 / 2 - 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
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
  ui_->coin(120);

  param_->sen_ref_p.normal.exist.right45 = backup_r;
  param_->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  param_->sen_ref_p.normal.expand.right45 = backup_r_expand;
  param_->sen_ref_p.normal.expand.left45 = backup_l_expand;
}

void MainTask::test_sla_walloff() {

  backup_r = param_->sen_ref_p.normal.exist.right45;
  backup_l = param_->sen_ref_p.normal.exist.left45;
  if (test_search_mode == 0) {
    rorl = ui_->select_direction();

    if (rorl == TurnDirection::Right) {
      param_->sen_ref_p.normal.exist.left45 += 10;
    } else {
      param_->sen_ref_p.normal.exist.right45 += 10;
    }

    if (rorl == TurnDirection::Left) {
      param_->sen_ref_p.normal.exist.left45 = 1;
      param_->sen_ref_p.normal.expand.left45 = 1;
      param_->sen_ref_p.normal.expand.left45_2 = 1;
      param_->right_keep_dist_th = -1;
    } else {
      param_->sen_ref_p.normal.exist.right45 = 1;
      param_->sen_ref_p.normal.expand.right45 = 1;
      param_->sen_ref_p.normal.expand.right45_2 = 1;
      param_->left_keep_dist_th = -1;
    }
  }
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
  if (test_search_mode > 0) {
    ps.v_max = sys_.test.v_max;
    ps.v_end = 300;
    ps.dist = param_->offset_start_dist_search;
    ps.accl = sys_.test.accl;
    ps.decel = sys_.test.decel;
    ps.sct = SensorCtrlType::Straight;
    ps.wall_off_req = WallOffReq::NONE;
    ps.wall_off_dist_r = 0;
    ps.wall_off_dist_l = 0;
    ps.dia_mode = false;
    mp->go_straight(ps, mp->fake_adachi, false);

    ps.v_max = sys_.test.v_max;
    ps.v_end = sys_.test.v_max;
    ps.dist = sys_.test.dist;
    ps.accl = sys_.test.accl;
    ps.decel = sys_.test.decel;
    ps.sct = SensorCtrlType::Straight;
    ps.wall_off_req = WallOffReq::NONE;
    ps.wall_off_dist_r = 0;
    ps.wall_off_dist_l = 0;
    ps.dia_mode = false;
    mp->go_straight(ps, mp->fake_adachi, true);

    ps.dist = param_->cell2 / 2 - 5;
    ps.v_max = sys_.test.v_max;
    ps.v_end = 20;
    ps.accl = sys_.test.accl;
    ps.decel = sys_.test.decel;
    ps.wall_off_req = WallOffReq::NONE;
    mp->go_straight(ps);
  } else {

    ps.motion_type = MotionType::NONE;
    ps.v_max = sys_.test.v_max;
    ps.v_end = sys_.test.v_max;
    ps.dist = 45 + 45 + param_->offset_start_dist;
    ps.accl = sys_.test.accl;
    ps.decel = sys_.test.decel;
    ps.sct = SensorCtrlType::Straight;
    ps.wall_off_req = WallOffReq::NONE;
    ps.wall_off_dist_r = 0;
    ps.wall_off_dist_l = 0;
    ps.dia_mode = false;
    mp->go_straight(ps);

    ps.dist = 90 - 5;
    mp->wall_off(rorl, ps);
    ps.motion_type = MotionType::NONE;
    ps.v_max = sys_.test.v_max;
    ps.v_end = 20;
    ps.accl = sys_.test.accl;
    ps.decel = sys_.test.decel;
    ps.wall_off_req = WallOffReq::NONE;
    ps.sct = SensorCtrlType::Straight;
    ps.wall_off_dist_r = 0;
    ps.wall_off_dist_l = 0;
    mp->go_straight(ps);

    ps.v_max = 20;
    ps.v_end = sys_.test.end_v;
    ps.dist = 5;
    ps.accl = sys_.test.accl;
    ps.decel = sys_.test.decel;
    ps.motion_type = MotionType::NONE;
    mp->go_straight(ps);
  }

  sleep_ms(100);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  param_->sen_ref_p.normal.exist.right45 = backup_r;
  param_->sen_ref_p.normal.exist.left45 = backup_l;

  lt_->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt_->save(slalom_log_file);
  ui_->coin(120);

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  sleep_ms(500);
  printf("----------------------------------\n");
  printf("offset: min(%f, %f) + %f = %f\n", mp->g_offset_y_l, mp->g_offset_y_r,
         mp->g_offset_x1, mp->g_total_offset);
  printf("theta: %f\n", mp->g_sen_ang * 180 / m_PI);
  printf("sensor dist: %f, %f\n", mp->g_sen_l_dist, mp->g_sen_r_dist);
  printf("----------------------------------\n");
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_dia_walloff() {
  rorl = ui_->select_direction();
  rorl2 = (rorl == TurnDirection::Right) ? (TurnDirection::Left)
                                         : (TurnDirection::Right);
  if (rorl == TurnDirection::Right) {
    param_->sen_ref_p.normal.exist.right45 = 1;
  } else {
    param_->sen_ref_p.normal.exist.left45 = 1;
  }
  mp->reset_gyro_ref_with_check();

  // if (sys_.test.suction_active) {
  //   planning_->suction_enable(sys_.test.suction_duty,
  //   sys_.test.suction_duty_low); vTaskDelay(500.0 / portTICK_PERIOD_MS);
  // }

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
  ps.dist = param_->cell + param_->offset_start_dist;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  sla_p = paramset_list[file_idx].map[TurnType::Dia45];
  nm.v_max = sla_p.v;
  nm.v_end = sla_p.v;
  nm.accl = sys_.test.accl;
  nm.decel = sys_.test.decel;
  nm.is_turn = false;
  mp->slalom(sla_p, rorl, nm, false);

  ps.dist = param_->cell / 2 * std::sqrt(2);
  ps.dia_mode = true;
  bool use_oppo_wall = false;
  bool exist_wall = false;
  mp->wall_off_dia(rorl2, ps, use_oppo_wall, exist_wall);

  ps.dist = ps.dist - 5;
  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.wall_off_req = WallOffReq::NONE;
  ps.sct = SensorCtrlType::NONE;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::NONE;
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
  ui_->coin(120);

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}
