#include "main/main_task.hpp"
#include "define.hpp"
#include "hardware/pwm.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include <stdio.h>

std::shared_ptr<MainTask> MainTask::s_instance;

std::shared_ptr<MainTask>
MainTask::create(std::shared_ptr<SensingTask> sensing,
                 std::shared_ptr<PlanningTask> planning) {
  s_instance = std::shared_ptr<MainTask>(new MainTask());
  s_instance->sensing_ = sensing;
  s_instance->planning_ = planning;
  return s_instance;
}

void MainTask::start() { s_instance->run(); }

std::shared_ptr<sensing_result_entity_t> MainTask::get_sensing_entity() {
  return sensing_->get_sensing_entity();
}
void MainTask::run() {
  // ブザー PWM 設定 (GPIO はどのコアからでも設定可)
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  uint pwm_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
  uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
  pwm_set_enabled(pwm_slice, false);

  const auto se = get_sensing_entity();

  ui_.init(pwm_slice, pwm_channel);
  ui_.LED_headlight();
  ui_.hello_exia();

  bool prev_btn = false;
  // bool suction_started = false;
  // bool suction_stopped = false;
  // absolute_time_t start_time = get_absolute_time();
  // absolute_time_t suction_start_time{};

  while (true) {
    // if (!suction_started && absolute_time_diff_us(start_time,
    // get_absolute_time()) >= 3000000LL) {
    //     PlanningTask::Command cmd;
    //     cmd.duty_suction = 42.0f;
    //     planning_->send_command(cmd);
    //     suction_started = true;
    //     suction_start_time = get_absolute_time();
    // }

    // if (suction_started && !suction_stopped &&
    //     absolute_time_diff_us(suction_start_time, get_absolute_time()) >=
    //     5000000LL) { PlanningTask::Command cmd; cmd.duty_suction = 0.0f;
    //     planning_->send_command(cmd);
    //     suction_stopped = true;
    // }

    // ---- ボタン処理 ----
    bool btn = !gpio_get(BTN_PIN); // active low

    if (btn != prev_btn) {
      // if (btn) {
      //     ui_.play_tone(BUZZER_FREQ_HZ);
      //     ui_.LED_on_all();

      //     // ボタン押下: planning に直進コマンドを投入
      //     PlanningTask::Command cmd;
      //     cmd.mode         = PlanningTask::MotionMode::STRAIGHT;
      //     cmd.v_max        = 200.0f;   // 200 mm/s
      //     cmd.accl         = 1000.0f;  // 1000 mm/s^2
      //     cmd.dist         = 1e9f;     // 実質無制限
      //     cmd.duty_suction = 5.0f;     // 5%
      //     planning_->send_command(cmd);
      // } else {
      //     ui_.stop_tone();
      //     ui_.LED_headlight();

      //     // ボタン離し: 減速停止
      //     PlanningTask::Command cmd;
      //     cmd.mode = PlanningTask::MotionMode::STOP;
      //     planning_->send_command(cmd);
      // }
      prev_btn = btn;
    }

    // ---- シリアル出力 (sensing が更新されたタイミングで表示) ----
    if (sensing_->data_ready && stdio_usb_connected()) {
      SensingTask::Data d = sensing_->data; // volatile → コピー
      sensing_->data_ready = false;

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
             se->led_sen.left90.raw,    //
             se->led_sen.left45_3.raw,  //
             se->led_sen.left45_2.raw,  //
             se->led_sen.left45.raw,    //
             se->led_sen.right45.raw,   //
             se->led_sen.right45_2.raw, //
             se->led_sen.right45_3.raw, //
             se->led_sen.right90.raw);

      printf("[motion]  gz=%6d (dt=%4llu us)"
             "  |  encL=%5u (dt=%4llu us)"
             "  |  encR=%5u (dt=%4llu us)\n",
             se->gyro.raw, d.gz_dt, d.enc_l, d.enc_l_dt, d.enc_r, d.enc_r_dt);

      printf("[power]   bat=%4u, %.4f\n", se->battery.raw, se->ego.batt_kf);

      PlanningTask::State ps = planning_->state; // volatile → コピー
      printf("[planning] mode=%u  v=%6.1f  w=%6.3f"
             "  dist=%7.1f  ang=%6.3f\n",
             (uint8_t)ps.mode, ps.img_v, ps.img_w, ps.img_dist, ps.img_ang);
      printf("[control]  duty_l=%6.1f%%  duty_r=%6.1f%%"
             "  suction=%5.1f%%  tick=%lu\n",
             ps.duty_l, ps.duty_r, ps.duty_suction, ps.tick);
    }

    sleep_ms(25);
  }
}
