#include "main/main_task.hpp"
#include "define.hpp"
#include "hardware/gpio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

// ─── LED ヘルパー ──────────────────────────────────────────────────────────────
// mode_num+1 を 6bit 2進数で表示。Astraea の lbit.byte = mode_num+1 と同等。
static void show_mode_led(UserInterface &ui, int mode) {
  int v = mode + 1;
  ui.LED_bit((v >> 0) & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1,
             (v >> 4) & 1, (v >> 5) & 1);
}

// ─── 共通センサー表示 ─────────────────────────────────────────────────────────
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
         se.led_sen.left45_2.raw, se.led_sen.left45.raw,
         se.led_sen.right45.raw, se.led_sen.right45_2.raw,
         se.led_sen.right45_3.raw, se.led_sen.right90.raw);

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

  printf("[sys]      mode=%d  maze=%d  goals=%u\n", sys.user_mode, sys.maze_size,
         (unsigned)sys.goals.size());
}

// ─── ボタン待機ヘルパー ───────────────────────────────────────────────────────
static void wait_button(UserInterface &ui) {
  while (!ui.button_state_hold())
    sleep_ms(10);
}

// ============================================================
// 本走行サブモード選択 (Astraea の select_mode() 相当)
// 短押し: 次のモードへ / 長押し(>=1秒): 決定
// ============================================================
int MainTask::select_run_mode(int max_mode) {
  int mode = 0;
  show_mode_led(ui_, mode);

  while (true) {
    if (!gpio_get(BTN_PIN)) {
      absolute_time_t press_t = get_absolute_time();
      while (!gpio_get(BTN_PIN))
        tight_loop_contents();
      int64_t held_ms =
          absolute_time_diff_us(press_t, get_absolute_time()) / 1000;

      if (held_ms >= 1000) {
        ui_.coin(100);
        return mode;
      } else {
        mode = (mode + 1) % max_mode;
        show_mode_led(ui_, mode);
      }
    }
    sleep_ms(10);
  }
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

    if (sub == 0) {
      printf("探索走行\n");
      // TODO: search_ctrl->exec() 実装後に有効化
      ui_.coin(100);
      wait_button(ui_);

    } else if (sub == 1) {
      printf("最速走行\n");
      // TODO: path_run() 実装後に有効化
      ui_.coin(100);
      wait_button(ui_);

    } else if (sub == 2) {
      printf("吸引テスト\n");
      mode_suction_test();

    } else if (sub == 3) {
      printf("センサーモニター\n");
      mode_sensor_monitor();
    }

    sleep_ms(10);
  }
}

// ============================================================
// テストモード dispatch (user_mode != 0)
// Astraea task() の if(sys.user_mode != 0) ブロックを踏襲。
// ============================================================
void MainTask::run_test_mode(int mode) {
  if (mode == 1) {
    printf("test_sla\n");
    // TODO: test_sla() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 2) {
    printf("test_run\n");
    mode_straight_test();

  } else if (mode == 3) {
    printf("test_turn\n");
    mode_pivot_test();

  } else if (mode == 4) {
    printf("test_run_sla\n");
    // TODO: test_run_sla() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 5) {
    printf("test_search_sla\n");
    // TODO: test_search_sla() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 6) {
    printf("test_front_wall_offset\n");
    // TODO: test_front_wall_offset() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 7) {
    printf("test_front_ctrl hold\n");
    // TODO: test_front_ctrl(true) 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 8) {
    printf("test_front_ctrl \n");
    // TODO: test_front_ctrl(false) 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 9) {
    printf("test_sla_walloff\n");
    // TODO: test_sla_walloff() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 10) {
    printf("back\n");
    // TODO: test_back() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 11) {
    printf("suction\n");
    printf("suction_active = %d\n", sys_.test.suction_active);
    if (sys_.test.suction_active == 1) {
      printf("duty = %f, duty_low = %f\n", sys_.test.suction_duty,
             sys_.test.suction_duty_low);
      planning_->set_suction_test(sys_.test.suction_duty);
    } else if (sys_.test.suction_active == 2) {
      printf("duty = %f, duty_low = %f\n", sys_.test.suction_duty_burst,
             sys_.test.suction_duty_burst_low);
      planning_->set_suction_test(sys_.test.suction_duty_burst);
    }
    sleep_ms(1000 * 10);
    planning_->set_suction_test(0.0f);

  } else if (mode == 12) {
    printf("hold\n");
    if (lt_) lt_->start_slalom_log();
    sleep_ms(1000 * 10);
    if (lt_) lt_->stop_slalom_log();
    if (lt_) lt_->save(slalom_log_file);
    ui_.coin(120);
    wait_button(ui_);
    if (lt_) lt_->dump_log(slalom_log_file);
    wait_button(ui_);

  } else if (mode == 13) {
    printf("keep_pivot\n");
    // TODO: keep_pivot() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 14) {
    printf("echo_sensor_csv\n");
    mode_sensor_monitor();

  } else if (mode == 15) {
    printf("echo_printf\n");
    mode_sensor_csv();

  } else if (mode == 16) {
    printf("sys id para\n");
    // TODO: test_system_identification(false) 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 17) {
    printf("sys id roll\n");
    // TODO: test_system_identification(true) 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 18) {
    if (lt_) lt_->dump_log(slalom_log_file);
    wait_button(ui_);

  } else if (mode == 19) {
    printf("test_dia_walloff\n");
    // TODO: test_dia_walloff() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 20) {
    printf("test_pivot_n\n");
    // TODO: test_pivot_n() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 21) {
    printf("test_pivot_n2\n");
    // TODO: test_pivot_n2() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 22) {
    // TODO: encoder_test() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 23) {
    printf("load_circuit_path\n");
    // TODO: test_search_pivot() 実装後に有効化
    ui_.coin(100);
    wait_button(ui_);
  }
}

// ============================================================
// mode 14: echo_sensor_csv (Astraea dump1 相当)
// 詳細センサーデータを printf で表示し続ける。
// ============================================================
void MainTask::mode_sensor_monitor() {
  const auto se = get_sensing_entity();

  while (true) {
    if (sensing_->data_ready && stdio_usb_connected()) {
      SensingTask::Data d = sensing_->data;
      sensing_->data_ready = false;
      PlanningTask::State ps = planning_->state;
      print_sensing(d, *se, ps, sys_, false);
    }
    sleep_ms(25);
  }
}

// ============================================================
// mode 15: echo_printf (Astraea dump2 相当)
// センサー値をカンマ区切りで出力し続ける。
// ============================================================
void MainTask::mode_sensor_csv() {
  const auto se = get_sensing_entity();

  while (true) {
    if (sensing_->data_ready) {
      sensing_->data_ready = false;
      printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
             se->led_sen.left90.raw, se->led_sen.left45_3.raw,
             se->led_sen.left45_2.raw, se->led_sen.left45.raw,
             se->led_sen.right45.raw, se->led_sen.right45_2.raw,
             se->led_sen.right45_3.raw, se->led_sen.right90.raw);
    }
    sleep_ms(100);
  }
}

// ============================================================
// 本走行サブモード用: インタラクティブ吸引テスト
// ボタン長押し(3秒)で ON / 再押下で OFF。
// ============================================================
static constexpr uint32_t SUCTION_DELAY_MS = 3000;

void MainTask::mode_suction_test() {
  const auto se = get_sensing_entity();

  const float duty =
      (sys_.test.suction_active == 2) ? sys_.test.suction_duty_burst
      : (sys_.test.suction_duty > 0)  ? sys_.test.suction_duty
                                       : 15.0f;

  printf("[suction] duty=%.1f%%  active=%d\n", duty, sys_.test.suction_active);

  bool prev_btn = false;
  bool suction_on = false;
  bool btn_held = false;
  absolute_time_t btn_press_time = nil_time;

  while (true) {
    bool btn = !gpio_get(BTN_PIN);

    if (btn != prev_btn) {
      if (btn) {
        if (suction_on) {
          planning_->set_suction_test(0.0f);
          suction_on = false;
          btn_held = false;
          ui_.stop_tone();
          ui_.LED_headlight();
        } else {
          btn_held = true;
          btn_press_time = get_absolute_time();
          ui_.LED_on_all();
        }
      }
      prev_btn = btn;
    }

    if (btn_held && !suction_on) {
      if (absolute_time_diff_us(btn_press_time, get_absolute_time()) >=
          (int64_t)SUCTION_DELAY_MS * 1000LL) {
        planning_->set_suction_test(duty);
        suction_on = true;
        btn_held = false;
        ui_.play_tone(BUZZER_FREQ_HZ);
      }
    }

    if (sensing_->data_ready && stdio_usb_connected()) {
      SensingTask::Data d = sensing_->data;
      sensing_->data_ready = false;
      PlanningTask::State ps = planning_->state;
      print_sensing(d, *se, ps, sys_, suction_on);
    }

    sleep_ms(25);
  }
}

// ============================================================
// mode 2: test_run 相当 (stub)
// sys_.test.v_max / accl / decel / dist を使用。
// ============================================================
void MainTask::mode_straight_test() {
  printf("[straight] v_max=%.1f  accl=%.1f  decel=%.1f  dist=%.1f\n",
         sys_.test.v_max, sys_.test.accl, sys_.test.decel, sys_.test.dist);

  // TODO: MotionPlanning::go_straight() 呼び出し
  // param_straight_t ps;
  // ps.v_max = sys_.test.v_max;
  // ps.v_end = sys_.test.end_v;
  // ps.dist  = sys_.test.dist;
  // ps.accl  = sys_.test.accl;
  // ps.decel = sys_.test.decel;
  // ps.motion_type = MotionType::STRAIGHT;
  // mp_->go_straight(ps);

  ui_.coin(100);
  wait_button(ui_);
}

// ============================================================
// mode 3: test_turn 相当 (stub)
// sys_.test.w_max / alpha / ang を使用。
// ============================================================
void MainTask::mode_pivot_test() {
  printf("[pivot] w_max=%.3f  alpha=%.3f  ang=%.3f\n", sys_.test.w_max,
         sys_.test.alpha, sys_.test.ang);

  // TODO: MotionPlanning::pivot_turn() 呼び出し
  // param_roll_t pr;
  // pr.w_max  = sys_.test.w_max;
  // pr.alpha  = sys_.test.alpha;
  // pr.ang    = sys_.test.ang;
  // mp_->pivot_turn(pr);

  ui_.coin(100);
  wait_button(ui_);
}
