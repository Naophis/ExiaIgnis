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

// ─── ボタン長押し待機ヘルパー ─────────────────────────────────────────────────
// ボタンが押して離されるまで待つ (短押し/長押し問わず)。
static void wait_button(UserInterface &ui) {
  while (!ui.button_state_hold())
    sleep_ms(10);
}

// ============================================================
// 本走行サブモード選択 (Astraea の select_mode() 相当)
// 短押し: 次のモードへ (LED 2進数更新)
// 長押し (>= 1秒): 選択確定 → coin + return
// ============================================================
int MainTask::select_run_mode(int max_mode) {
  int mode = 0;
  show_mode_led(ui_, mode);

  while (true) {
    if (!gpio_get(BTN_PIN)) { // ボタン押下検出 (active low)
      absolute_time_t press_t = get_absolute_time();
      while (!gpio_get(BTN_PIN))
        tight_loop_contents(); // 離されるまで待つ
      int64_t held_ms =
          absolute_time_diff_us(press_t, get_absolute_time()) / 1000;

      if (held_ms >= 1000) {
        // 長押し: 決定
        ui_.coin(100);
        return mode;
      } else {
        // 短押し: 次のモードへ
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
// select_run_mode() で走行種別を選択してループ。
// ============================================================
//
// サブモード一覧:
//   0: 探索走行    (stub: SearchController 実装後に有効化)
//   1: 最速走行    (stub: PathCreator / path_run 実装後に有効化)
//   2: 吸引テスト  (即実行可能)
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
      // ── 探索走行 ─────────────────────────────────────────────────────────
      // TODO: lgc/search_ctrl 実装後に有効化
      printf("[main] 探索走行 (not yet implemented)\n");
      ui_.coin(100);
      wait_button(ui_);

    } else if (sub == 1) {
      // ── 最速走行 ─────────────────────────────────────────────────────────
      // TODO: PathCreator / path_run 実装後に有効化
      printf("[main] 最速走行 (not yet implemented)\n");
      ui_.coin(100);
      wait_button(ui_);

    } else if (sub == 2) {
      // ── 吸引テスト ───────────────────────────────────────────────────────
      printf("[main] 吸引テスト\n");
      mode_suction_test();

    } else if (sub == 3) {
      // ── センサーモニター ─────────────────────────────────────────────────
      printf("[main] センサーモニター\n");
      mode_sensor_monitor();
    }

    sleep_ms(10);
  }
}

// ============================================================
// テストモード dispatch (user_mode != 0)
// Astraea task() の if(sys.user_mode != 0) ブロック相当。
//
// モード番号は Astraea に合わせた番号体系:
//   1: センサーモニター  (dump1/dump2 相当, Astraea mode 14/15)
//   2: 直進テスト        (test_run 相当)
//   3: 旋回テスト        (test_turn / test_pivot_n 相当)
//   4: 後退テスト        (test_back 相当)
//   5: 吸引テスト        (Astraea mode 11 相当)
// ============================================================
void MainTask::run_test_mode(int mode) {
  printf("[main] test mode=%d\n", mode);

  if (mode == 1) {
    printf("[main] test 1: sensor monitor\n");
    mode_sensor_monitor();

  } else if (mode == 2) {
    printf("[main] test 2: straight\n");
    mode_straight_test();

  } else if (mode == 3) {
    printf("[main] test 3: pivot\n");
    mode_pivot_test();

  } else if (mode == 4) {
    // TODO: 後退テスト (test_back 相当)
    printf("[main] test 4: back (not yet implemented)\n");
    ui_.coin(100);
    wait_button(ui_);

  } else if (mode == 5) {
    printf("[main] test 5: suction\n");
    mode_suction_test();

  } else {
    printf("[main] unknown test mode %d, sensor monitor\n", mode);
    mode_sensor_monitor();
  }
}

// ============================================================
// mode 1: センサーモニター
// Astraea dump1/dump2 相当。USB 接続時にセンサーデータを表示し続ける。
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
// mode 5: 吸引 PWM テスト (Astraea mode 11 相当)
// sys_.test.suction_duty / suction_active を参照して吸引を制御する。
// ボタン1回目長押し(3秒)で吸引 ON、2回目押下で吸引 OFF。
// ============================================================
static constexpr uint32_t SUCTION_DELAY_MS = 3000;

void MainTask::mode_suction_test() {
  const auto se = get_sensing_entity();

  const float duty =
      (sys_.test.suction_active == 2) ? sys_.test.suction_duty_burst
      : (sys_.test.suction_duty > 0)  ? sys_.test.suction_duty
                                       : 15.0f; // フォールバック

  printf("[suction] duty=%.1f%%  active=%d\n", duty, sys_.test.suction_active);

  bool prev_btn = false;
  bool suction_test_on = false;
  bool btn_held = false;
  absolute_time_t btn_press_time = nil_time;

  while (true) {
    bool btn = !gpio_get(BTN_PIN);

    if (btn != prev_btn) {
      if (btn) {
        if (suction_test_on) {
          // 2 回目押下: 吸引 OFF
          planning_->set_suction_test(0.0f);
          suction_test_on = false;
          btn_held = false;
          ui_.stop_tone();
          ui_.LED_headlight();
        } else {
          // 1 回目押下: 3 秒タイマー開始
          btn_held = true;
          btn_press_time = get_absolute_time();
          ui_.LED_on_all();
        }
      }
      prev_btn = btn;
    }

    if (btn_held && !suction_test_on) {
      if (absolute_time_diff_us(btn_press_time, get_absolute_time()) >=
          (int64_t)SUCTION_DELAY_MS * 1000LL) {
        planning_->set_suction_test(duty);
        suction_test_on = true;
        btn_held = false;
        ui_.play_tone(BUZZER_FREQ_HZ);
      }
    }

    if (sensing_->data_ready && stdio_usb_connected()) {
      SensingTask::Data d = sensing_->data;
      sensing_->data_ready = false;
      PlanningTask::State ps = planning_->state;
      print_sensing(d, *se, ps, sys_, suction_test_on);
    }

    sleep_ms(25);
  }
}

// ============================================================
// mode 2: 直進テスト (Astraea test_run 相当)
// sys_.test.v_max / accl / decel / dist を使用。
// MotionPlanning 実装後に有効化予定。
// ============================================================
void MainTask::mode_straight_test() {
  printf("[straight] v_max=%.1f  accl=%.1f  decel=%.1f  dist=%.1f\n",
         sys_.test.v_max, sys_.test.accl, sys_.test.decel, sys_.test.dist);
  printf("[straight] not yet implemented. press button to exit.\n");

  // TODO: MotionPlanning::go_straight() 呼び出しに置き換える
  //
  // param_straight_t ps;
  // ps.v_max = sys_.test.v_max;
  // ps.v_end = sys_.test.end_v;
  // ps.dist  = sys_.test.dist;
  // ps.accl  = sys_.test.accl;
  // ps.decel = sys_.test.decel;
  // ps.motion_type = MotionType::STRAIGHT;
  // mp_->reset_gyro_ref_with_check();
  // mp_->reset_tgt_data();
  // mp_->reset_ego_data();
  // mp_->go_straight(ps);

  ui_.coin(100);
  wait_button(ui_);
}

// ============================================================
// mode 3: 旋回テスト (Astraea test_turn / test_pivot_n 相当)
// sys_.test.w_max / alpha / ang を使用。
// MotionPlanning 実装後に有効化予定。
// ============================================================
void MainTask::mode_pivot_test() {
  printf("[pivot] w_max=%.3f  alpha=%.3f  ang=%.3f\n", sys_.test.w_max,
         sys_.test.alpha, sys_.test.ang);
  printf("[pivot] not yet implemented. press button to exit.\n");

  // TODO: MotionPlanning::pivot_turn() 呼び出しに置き換える
  //
  // param_roll_t pr;
  // pr.w_max  = sys_.test.w_max;
  // pr.alpha  = sys_.test.alpha;
  // pr.ang    = sys_.test.ang;
  // mp_->pivot_turn(pr);

  ui_.coin(100);
  wait_button(ui_);
}
