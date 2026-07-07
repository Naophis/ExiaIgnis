#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

void MainTask::test_suction() {
  const auto se = get_sensing_entity();
  const float target_v = sys_.test.suction_duty;

  mp->reset_gyro_ref_with_check();
  sleep_ms(250);

  // --- 実行前に BLDC 関連パラメータをダンプ ---
  printf("== BLDC params ==\n");
  printf("suction_bldc_hz(base)=%.0f  test.suction_bldc_hz=%.0f  "
         "-> target_hz=%.0f\n",
         sys_.suction_bldc_hz, sys_.test.suction_bldc_hz,
         planning_->bldc_.get_target_hz());
  printf("test.suction_gain=%.1f -> ramp_rate=%.0f\n", sys_.test.suction_gain,
         planning_->bldc_.get_ramp_rate());
  printf("test.suction_amp_gain=%.3f test.suction_max_amp=%.3f "
         "-> amp_gain=%.3f max_amp=%.3f\n",
         sys_.test.suction_amp_gain, sys_.test.suction_max_amp,
         planning_->bldc_.get_amp_gain(), planning_->bldc_.get_max_amp());
  printf("test.suction_duty=%.2f  test.suction_duty_low=%.2f\n",
         sys_.test.suction_duty, sys_.test.suction_duty_low);

  // --- Step 1: ALIGN→RAMP→RUN を5秒間確認 ---
  printf("== BLDC test start (5sec) ==\n");
//   planning_->bldc_.test_direct(30.0f);
  printf("== BLDC test done ==\n");
  sleep_ms(200);

  // --- Step 2: ControlLaw 経由で運転 (RAMP 中は printf 抑制) ---
  planning_->suction_enable(sys_.test.suction_duty, sys_.test.suction_duty_low);

  while (!ui_->button_state()) {
    if (!planning_->bldc_.is_ramping()) {
      printf("batt=%.3fV duty=%.1f%%\n",
             se->ego.batt_kf,
             planning_->tgt_val->duty_suction);
    }
    sleep_ms(200);
  }

  planning_->suction_disable();
  printf("suction stopped\n");

  // disable()で1kHz tick自体が止まっているので、ここでのprintfは安全。
  planning_->bldc_.dump_trace();
}

void MainTask::dump1() {

  mp->reset_gyro_ref_with_check();
  sleep_ms(10);
  planning_->tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
  planning_->tgt_val->nmr.timstamp = planning_->tgt_val->nmr.timstamp + 1;
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

    if (ui_->button_state()) {
      planning_->tgt_val->ego_in.ang = planning_->tgt_val->ego_in.dist = 0;
    }

    sleep_ms(100);
  }
}

void MainTask::dump2() {
  const auto se = get_sensing_entity();
  mp->reset_gyro_ref_with_check();
  planning_->tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
  planning_->tgt_val->nmr.timstamp = planning_->tgt_val->nmr.timstamp + 1;
  planning_->send_command(*planning_->tgt_val);

  while (1) {
    printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", se->led_sen.left90.raw,
           se->led_sen.left45_3.raw, se->led_sen.left45_2.raw,
           se->led_sen.left45.raw, se->led_sen.front.raw,
           se->led_sen.right45.raw, se->led_sen.right45_2.raw,
           se->led_sen.right45_3.raw, se->led_sen.right90.raw);

    if (ui_->button_state()) {
      planning_->tgt_val->ego_in.ang = planning_->tgt_val->ego_in.dist = 0;
    }

    sleep_ms(100);
  }
}

void MainTask::encoder_test() {}

void MainTask::test_system_identification(bool para) {}
