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

  // 2026-09-09: front offset自動調整ループ。1回走行するたびにleft90/right90_mid_dist
  // から区画中心とのズレを測り、sla_p.front.{left,right}へフィードバックして即再走行する
  // (手でスタート位置へ戻してボタンを押すのが「次のトライアルへ進む」合図になる)。
  // 終了操作は設けない: 電源を切る/リセットするまで無限ループし続ける。
  while (true) {
    printf("front (%s): [%f, %f]\n",
           (rorl == TurnDirection::Left) ? "left" : "right", sla_p.front.left,
           sla_p.front.right);

    backup_r = param_->sen_ref_p.normal.exist.right45;
    backup_l = param_->sen_ref_p.normal.exist.left45;
    // 2026-09-05: テスト開始直後の直進助走で位置・角度を素早く補正するため、
    // 旋回方向側の壁までexist閾値を緩めていたが、反対側の壁も検出範囲内なら
    // check_sen_error()は両方使ってしまう。反対側の応答が旋回方向によって
    // 非対称(duty_l>duty_rという同じハード起因の偏りに対しても、旋回方向に
    // よってこの助走中のang変化が符号反転する: 左ターンで-1.13/-0.15°、
    // 右ターンで+2.36/+1.40°、20260905_2316xx.csv)だったため、意図した側
    // だけで補正がかかるよう反対側の壁判定をここで実質無効化する
    // (exist閾値を0にすると check_sen_error() の `1 < dist && dist <
    // exist_right45` が常に偽になり、範囲チェックそのものが成立しない)。
    if (rorl == TurnDirection::Right) {
      param_->sen_ref_p.normal.exist.left45 += 10;
      param_->sen_ref_p.normal.exist.right45 = 0;
    } else {
      param_->sen_ref_p.normal.exist.right45 += 10;
      param_->sen_ref_p.normal.exist.left45 = 0;
    }

    // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
    if (sys_.test.suction_active != 0) {
      planning_->suction_power_on();
    }
    mp->reset_gyro_ref_with_check();

    // 2026-09-05: 吸引ファンの反動トルクで機体が回転してしまう問題への対策
    // (吸引動作中に機体が左を向いて見える不具合をユーザーが実機で確認)。
    // reset_pos()等でkim(自己位置)だけ後からゼロに戻しても機体は物理的に
    // 回転したまま走行を開始することになり、Kanayamaが「ソフトはゼロだと
    // 思っているが実機はズレている」状態を後追いで補正する羽目になる
    // (=症状を隠すだけで実害は残る、[[project_dia45_lr_asymmetry_2026-09-05]]
    // 参照)。回転そのものを起こさせないため、吸引のランプ〜セトリングの間だけ
    // hold()(v=0/w=0保持専用モーション)を挟む。

    if (sys_.test.suction_active != 0) {
      reset_tgt_data();
      reset_ego_data();
      planning_->motor_enable();
      mp->hold();
    }

    // if (param_->test_log_enable > 0) {
    //   lt_->start_slalom_log();
    // }

    if (sys_.test.suction_active == 1) {
      planning_->suction_enable(sys_.test.suction_duty,
                                sys_.test.suction_duty_low);
      while (planning_->is_suction_ramping()) {
        sleep_ms(10);
      }
      mp->hold_settle_wait();
    } else if (sys_.test.suction_active == 2) {
      planning_->suction_enable(sys_.test.suction_duty_burst,
                                sys_.test.suction_duty_burst_low);
      while (planning_->is_suction_ramping()) {
        sleep_ms(10);
      }
      mp->hold_settle_wait();
    }
    if (sys_.test.suction_active != 0) {
      mp->unhold();
    } else {
      reset_tgt_data();
      reset_ego_data();
      planning_->motor_enable();
    }

    // testモード用の速度→加速度LUTに切り替える。非吸引時はグリップ不足を
    // 想定し、LUTを使わず固定accl(sys_.test.accl)にフォールバックする。
    if (sys_.test.suction_active != 0) {
      param_->accl_v_x = sys_.test.accl_v_x;
      param_->accl_v_y = sys_.test.accl_v_y;
    } else {
      param_->accl_v_x.clear();
      param_->accl_v_y.clear();
    }

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
    printf("[wait] press button to dump log CSV\n");
    while (1) {
      if (ui_->button_state_hold())
        break;
      sleep_ms(10);
    }
    // USB非接続時はdump_csv()がホスト待ちで進まなくなるため、ボタンを
    // 押した時点でのUSB接続状態を見てCSVダンプ自体をスキップする
    // (無線走行中は事前にUSBが繋がっていないことが多いため、判定は
    // ボタン押下の直前まで引っ張る)。
    if (stdio_usb_connected()) {
      lt_->dump_log(slalom_log_file);
    } else {
      printf("[auto-tune] USB not connected - skip CSV dump\n");
    }

    // front offset自動調整: 走行ログ(sen_log_l45/r45、実際に壁のある区画を
    // 無線で走っていた時点の値)の末尾を平均し、基準距離との偏差をsla_p.
    // front.{left,right}へフィードバックしてLittleFSへ書き戻す。書き込み
    // 自体はdump完了後(CSV転送中にflash書き込みでUSB CDCを切断させない
    // ため)に行うが、測定値はログ(走行中=壁がある時点)から取るので、
    // USB接続の有無やボタン操作でロボットに触れた後でも測定値は変わらない。
    //
    // 基準距離はターン形状によって変える:
    //   - Normal: 45mm固定
    //   - Large/Orval/Dia45_2/Dia135_2: param_->sen_ref_p.normal.ref.left45/
    //     right45 (calc_large_offset()/calc_orval_offset()/
    //     calc_dia135_offset()が旋回進入時のmid-turn壁オフセット計算で実際に
    //     使っているのと同じ基準値を流用する)
    //
    // 幾何: front.{left,right}はslalom()内部(motion_planning.cpp:578)で
    // 旋回前の直進区間(front leg)の距離として毎回使われるパラメータ
    // (sys_.test.start_turnの有無とは無関係、start_turnは「開幕ターン」用に
    // 別途この値をtest_sla()の助走距離計算に流用しているだけ)。90度旋回では
    // 「旋回前の進行方向への並進」がそのまま「旋回後、旋回した側の壁からの
    // 距離」になる(旋回自体の形状はfrontに依存せず並行移動するだけのため)。
    //   左ターン: front.left を +delta すると旋回後は左壁から +delta 離れる
    //   右ターン: front.right を +delta すると旋回後は右壁から +delta 離れる
    // なので 新front = 旧front - (測定距離 - 基準距離) で基準距離に収束する。
    // (符号は幾何からの導出であり実機未検証。逆に動く場合は符号を反転すること)
    //
    // sla_return/turn_times併用時は複数旋回の影響が混ざりfrontへの寄与を
    // 切り分けられないため対象外とする。
    bool at_ran = false;
    bool at_saved = false;
    bool at_match = false;
    float at_old = 0, at_new = 0, at_mean = 0, at_err = 0;
    printf("[auto-tune] ==================================\n");
    printf("[auto-tune] gate: front_auto_tune=%d sla_return=%d turn_times=%d\n",
           sys_.test.front_auto_tune, sys_.test.sla_return,
           sys_.test.turn_times);
    if (sys_.test.front_auto_tune > 0 && sys_.test.sla_return == 0 &&
        sys_.test.turn_times == 0) {
      constexpr int n_sample = 30;
      float ref_dist = 45.0f;
      switch (sla_p.type) {
      case TurnType::Large:
      case TurnType::Orval:
      case TurnType::Dia45_2:
      case TurnType::Dia135_2:
        ref_dist = (rorl == TurnDirection::Left)
                       ? param_->sen_ref_p.normal.ref.left45
                       : param_->sen_ref_p.normal.ref.right45;
        break;
      default:
        break;
      }
      float tail_l45 = 0, tail_r45 = 0;
      const bool have_tail =
          lt_->tail_average_sen45(tail_l45, tail_r45, n_sample);
      at_mean = have_tail
                    ? ((rorl == TurnDirection::Left) ? tail_l45 : tail_r45)
                    : 0;
      if (have_tail && at_mean > 10 && at_mean < 90) {
        at_ran = true;
        at_err = at_mean - ref_dist;
        if (rorl == TurnDirection::Left) {
          at_old = sla_p.front.left;
          sla_p.front.left -= at_err;
          at_new = sla_p.front.left;
        } else {
          at_old = sla_p.front.right;
          sla_p.front.right -= at_err;
          at_new = sla_p.front.right;
        }
        at_saved = save_slalom_front_offset(file_idx, sla_p.type, sla_p.front);
      }
    }

    if (!at_ran) {
      printf("[auto-tune] not applied this trial (dist=%.2f out of range, "
             "or gate condition above is false)\n",
             at_mean);
    } else {
      printf("[auto-tune] side=%s mean_dist=%.3f err=%.3f front: %.3f -> "
             "%.3f\n",
             (rorl == TurnDirection::Left) ? "left" : "right", at_mean, at_err,
             at_old, at_new);
      printf("[auto-tune] save_slalom_front_offset() returned %s\n",
             at_saved ? "true" : "false");
      slalom_offset_t readback{};
      if (read_slalom_front_offset(file_idx, sla_p.type, readback)) {
        const float on_flash =
            (rorl == TurnDirection::Left) ? readback.left : readback.right;
        at_match = std::abs(on_flash - at_new) < 0.001f;
        printf("[auto-tune] readback from flash: front.%s=%.3f (%s)\n",
               (rorl == TurnDirection::Left) ? "left" : "right", on_flash,
               at_match ? "MATCH" : "MISMATCH");
      } else {
        printf("[auto-tune] readback from flash failed\n");
      }
      if (!at_saved || !at_match) {
        ui_->music_sync(MUSIC::G5_, 250);
      }
    }
    printf("[auto-tune] ==================================\n");

    sleep_ms(500);
    printf("----------------------------------\n");
    printf("offset: min(%f, %f) + %f = %f\n", mp->g_offset_y_l, mp->g_offset_y_r,
           mp->g_offset_x1, mp->g_total_offset);
    printf("theta: %f\n", mp->g_sen_ang * 180 / m_PI);
    printf("sensor dist: %f, %f\n", mp->g_sen_l_dist, mp->g_sen_r_dist);
    printf("----------------------------------\n");

    printf("[wait] press button to start next trial\n");
    while (1) {
      if (ui_->button_state_hold())
        break;
      sleep_ms(10);
    }
    // 次の走行に進めるようになった合図(USB未接続でも音で分かるように)。
    ui_->coin(50);
    printf("=== next trial: reposition at start, then press button OR "
           "center between walls (motion_check) ===\n");
  } // while (true) — 手でスタート位置へ戻してボタンを押すと次のトライアルへ
}

void MainTask::test_run_sla() {
  // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
  if (sys_.test.suction_active != 0) {
    planning_->suction_power_on();
  }
  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();
  if (sys_.test.suction_active == 1) {
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    sleep_ms(800);
  } else if (sys_.test.suction_active == 2) {
    planning_->suction_enable(sys_.test.suction_duty_burst,
                              sys_.test.suction_duty_burst_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    sleep_ms(800);
  }

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  // testモード用の速度→加速度LUTに切り替える。非吸引時はグリップ不足を
  // 想定し、LUTを使わず固定accl(sys_.test.accl)にフォールバックする。
  if (sys_.test.suction_active != 0) {
    param_->accl_v_x = sys_.test.accl_v_x;
    param_->accl_v_y = sys_.test.accl_v_y;
  } else {
    param_->accl_v_x.clear();
    param_->accl_v_y.clear();
  }

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
  load_turn_param_profiles(true, file_idx);
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

  // 探索用の速度→加速度LUTに切り替える。非吸引時はグリップ不足を想定し、
  // LUTを使わず固定accl(str_p.accl)にフォールバックする。
  if (param_set.suction != 0) {
    param_->accl_v_x = str_p.accl_v_x;
    param_->accl_v_y = str_p.accl_v_y;
  } else {
    param_->accl_v_x.clear();
    param_->accl_v_y.clear();
  }

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
  if (sys_.test.suction_active != 0) {
    planning_->suction_power_on();
  }
  mp->reset_gyro_ref_with_check();
  // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
  if (sys_.test.suction_active != 0) {
    reset_tgt_data();
    reset_ego_data();
    planning_->motor_enable();
    mp->hold();
  }

  if (sys_.test.suction_active == 1) {
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    mp->hold_settle_wait();
  } else if (sys_.test.suction_active == 2) {
    planning_->suction_enable(sys_.test.suction_duty_burst,
                              sys_.test.suction_duty_burst_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    mp->hold_settle_wait();
  }

  if (sys_.test.suction_active != 0) {
    mp->unhold();
  } else {
    reset_tgt_data();
    reset_ego_data();
    planning_->motor_enable();
  }

  // testモード用の速度→加速度LUTに切り替える。非吸引時はグリップ不足を
  // 想定し、LUTを使わず固定accl(sys_.test.accl)にフォールバックする。
  if (sys_.test.suction_active != 0) {
    param_->accl_v_x = sys_.test.accl_v_x;
    param_->accl_v_y = sys_.test.accl_v_y;
  } else {
    param_->accl_v_x.clear();
    param_->accl_v_y.clear();
  }

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

  // testモード用の速度→加速度LUTに切り替える。非吸引時はグリップ不足を
  // 想定し、LUTを使わず固定accl(sys_.test.accl)にフォールバックする。
  if (sys_.test.suction_active != 0) {
    param_->accl_v_x = sys_.test.accl_v_x;
    param_->accl_v_y = sys_.test.accl_v_y;
  } else {
    param_->accl_v_x.clear();
    param_->accl_v_y.clear();
  }

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
