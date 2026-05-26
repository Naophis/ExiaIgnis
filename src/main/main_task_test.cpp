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

  if (file_idx >= tpp.file_list_size) {
    printf("%d %d\n", file_idx, tpp.file_list_size);
    ui_.error();
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

  rorl = ui_.select_direction();
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
  ui_.coin(120);

  param_->sen_ref_p.normal.exist.right45 = backup_r;
  param_->sen_ref_p.normal.exist.left45 = backup_l;
  while (1) {
    if (ui_.button_state_hold())
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
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
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
  if (param_->test_log_enable > 0) {
    lt_->start();
  }
  reset_tgt_data();
  // printf("reset tgt/ego\n");
  reset_ego_data();
  // printf("reset_ego_data\n");
  planning_->motor_enable();
  // printf("motor enabled\n");
  req_error_reset();
  // printf("req_error_reset\n");

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

  // printf("go_straight[1]\n");
  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  // printf("go_straight[2]\n");
  mp->go_straight(ps);
  // lt_->stop_slalom_log();
  // bool front_ctrl = (se->ego.front_dist < 60);
  // sleep_ms(50);
  // sleep_ms(50);
  // printf("test_run: done\n");
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

void MainTask::test_search_sla(bool wall_off) {

  file_idx = 0;

  if (file_idx >= tpp.file_list_size) {
    ui_.error();
    return;
  }

  load_slalom_param(0, 0, 0);
  sla_p = param_set.map[TurnType::Normal];
  str_p = param_set.str_map[StraightType::Search];

  printf("str: \n");
  printf("- v_max: %f\n", str_p.v_max);
  printf("- accl: %f\n", str_p.accl);
  printf("- decel: %f\n", str_p.decel);

  rorl = ui_.select_direction();
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

void MainTask::test_sla_walloff() {

  backup_r = param_->sen_ref_p.normal.exist.right45;
  backup_l = param_->sen_ref_p.normal.exist.left45;
  if (test_search_mode == 0) {
    rorl = ui_.select_direction();

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
  ui_.coin(120);

  while (1) {
    if (ui_.button_state_hold())
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
    if (ui_.button_state_hold())
      break;
    sleep_ms(10);
  }
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
  // + param_->offset_start_dist;
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
  sleep_ms(10);
  planning_->tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
  planning_->tgt_val->nmr.timstamp++;
  planning_->send_command(*planning_->tgt_val);
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
    printf("encoder: %5ld, %5ld\n", (long)se->encoder.left,
           (long)se->encoder.right);
    printf("sensor: %4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d\n",
           se->led_sen.left90.raw,    //
           se->led_sen.left45_3.raw,  //
           se->led_sen.left45_2.raw,  //
           se->led_sen.left45.raw,    //
           se->led_sen.right45.raw,   //
           se->led_sen.right45_2.raw, //
           se->led_sen.right45_3.raw, //
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
    printf("planning_breakdown[us]: ego=%d sensor=%d trj=%d knym=%d copy=%d "
           "ctl=%d\n",
           planning_->tgt_val->pln_t_ego, planning_->tgt_val->pln_t_sensor,
           planning_->tgt_val->pln_t_trj, planning_->tgt_val->pln_t_kanayama,
           planning_->tgt_val->pln_t_copy, planning_->tgt_val->pln_t_ctl);
    printf("sensing_time: %d\t%d\n", se->calc_time, se->calc_time2);
    printf("sensing_breakdown[us]: spi=%d amb=%d r90=%d r45=%d l45=%d l90=%d "
           "total=%d\n",
           se->t_spi, se->t_ambient, se->t_r90, se->t_r45, se->t_l45, se->t_l90,
           se->calc_time2);
    printf("spi_breakdown[us]: gyro=%d encr=%d encl=%d bat+calc=%d\n",
           se->t_gyro, se->t_encr, se->t_encl, se->t_bat);

    if (ui_.button_state()) {
      planning_->tgt_val->ego_in.ang = planning_->tgt_val->ego_in.dist = 0;
    }

    sleep_ms(100);
  }
}

void MainTask::dump2() {
  const auto se = get_sensing_entity();
  mp->reset_gyro_ref_with_check();
  planning_->tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
  planning_->tgt_val->nmr.timstamp++;
  planning_->send_command(*planning_->tgt_val);

  while (1) {
    printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", se->led_sen.left90.raw,
           se->led_sen.left45_3.raw, se->led_sen.left45_2.raw,
           se->led_sen.left45.raw, se->led_sen.front.raw,
           se->led_sen.right45.raw, se->led_sen.right45_2.raw,
           se->led_sen.right45_3.raw, se->led_sen.right90.raw);

    if (ui_.button_state()) {
      planning_->tgt_val->ego_in.ang = planning_->tgt_val->ego_in.dist = 0;
    }

    sleep_ms(100);
  }
}

void MainTask::test_dia_walloff() {
  rorl = ui_.select_direction();
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

void MainTask::encoder_test() {}

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

void MainTask::test_system_identification(bool para) {}
