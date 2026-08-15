#include "define.hpp"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "driver/am32_config.hpp"
#include "config_loader.hpp"
#include <stdio.h>

namespace {
void am32_printf_line(const char* line) { printf("%s\n", line); }

// 設定通信を終えた後、信号線をCore1(ControlLaw)が回し続けているPWMスライスへ
// 戻す。SuctionEscActuator::init()でPWMスライス自体は起動時から動作し続けて
// おり、config中もCore1は変わらずpwm_set_chan_level()を叩き続けている
// (物理ピンがPIOへ切り替わっているため単に信号がESCへ届いていないだけ)。
// そのためスライス自体の再初期化は不要で、GPIO機能をPWMへ戻すだけでよい。
void restore_suction_pwm_pin() {
  gpio_set_function(SUCTION_ESC_PWM, GPIO_FUNC_PWM);
}

// read_am32_param()/write_am32_param()の開始時点の生設定byte列を保存しておき、
// restore_am32_param()で書き戻せるようにする。人間単位への変換(motor_kv()等)
// を経由せず生byteのまま保存/復元することで、換算の往復誤差(例: timingの
// 旧フォーマット非対応)を回避する。
constexpr const char* kAm32BackupPath = "/am32_backup.bin";

void backup_am32_settings(const AM32Settings& s) {
  if (ConfigLoader::write_file(kAm32BackupPath, s.raw.buffer, AM32Settings::LAYOUT_SIZE)) {
    printf("am32: backup saved to %s (%u bytes)\n", kAm32BackupPath,
           (unsigned)AM32Settings::LAYOUT_SIZE);
  } else {
    printf("am32: backup save FAILED\n");
  }
}
}  // namespace

void MainTask::read_am32_param() {
  // Core1がconfig中も1kHzでsuction PWMを書き続けるため、意図しない残り
  // duty(旧テストの目標値)がESC復帰直後に反映されないよう先に止めておく。
  planning_->suction_disable();
  while (planning_->is_suction_ramping()) {
    sleep_ms(5);
  }

  // AM32(cached_/last_read_で192byte*2を持つ)をCore0のスタック(8KB)へ直接
  // 積まず、staticにして.bssへ逃がす(スタック圧迫によるフォルトの可能性を排除)。
  static AM32 esc;
  esc.init(SUCTION_ESC_PWM, pio0);

  printf("== AM32 read == (sizeof(AM32)=%u)\n", (unsigned)sizeof(AM32));
  const Am32ConfigStatus st = esc.enterConfigMode();
  if (st != Am32ConfigStatus::OK) {
    printf("enterConfigMode failed: %s (%s)\n",
           am32_config_status_to_string(st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
    restore_suction_pwm_pin();
    return;
  }
  printf("handshake OK. devinfo:");
  for (int i = 0; i < 9; i++) {
    printf(" %02X", esc.device_info()[i]);
  }
  printf("\n");

  am32_handle_cli(esc, "read", am32_printf_line);
  am32_handle_cli(esc, "dump", am32_printf_line);
  backup_am32_settings(esc.cached_settings());

  esc.exitConfigMode();
  restore_suction_pwm_pin();
  printf("== AM32 read done ==\n");
}

void MainTask::write_am32_param() {
  planning_->suction_disable();
  while (planning_->is_suction_ramping()) {
    sleep_ms(5);
  }

  static AM32 esc;
  esc.init(SUCTION_ESC_PWM, pio0);

  printf("== AM32 write ==\n");
  Am32ConfigStatus st = esc.enterConfigMode();
  if (st != Am32ConfigStatus::OK) {
    printf("enterConfigMode failed: %s (%s)\n",
           am32_config_status_to_string(st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
    restore_suction_pwm_pin();
    return;
  }

  AM32Settings settings;
  st = esc.readSettings(settings);
  if (st != Am32ConfigStatus::OK) {
    printf("readSettings failed: %s (%s)\n",
           am32_config_status_to_string(st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
    esc.exitConfigMode();
    restore_suction_pwm_pin();
    return;
  }
  printf("before: KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
         settings.motor_kv(), settings.motor_poles(), settings.timing(),
         settings.pwm_frequency_khz());
  backup_am32_settings(settings);  // restore_am32_param()で戻せるよう書き換え前の生byteを保存

  // TODO: 実際に書き換えたい値へ調整すること。以下はサンプル値。
  settings.set_motor_kv(8500);
  settings.set_motor_poles(12);
  settings.set_timing(25);

  st = esc.writeSettings(settings);  // 範囲チェック + RAM staging更新のみ
  if (st != Am32ConfigStatus::OK) {
    char verr[64] = {0};
    am32_validate_settings(settings, verr, sizeof(verr));  // 詳細理由を再取得して表示
    printf("writeSettings failed (validation): %s -- %s\n",
           am32_config_status_to_string(st), verr);
    esc.exitConfigMode();
    restore_suction_pwm_pin();
    return;
  }

  st = esc.saveSettings();  // ここで実際にESCのflashへ書き込む
  if (st != Am32ConfigStatus::OK) {
    printf("saveSettings failed: %s (%s) -- rolling back staged value\n",
           am32_config_status_to_string(st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
    esc.rollback();
  } else {
    printf("save OK\n");
  }

  AM32Settings verify;
  st = esc.readSettings(verify);
  if (st == Am32ConfigStatus::OK) {
    printf("after : KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
           verify.motor_kv(), verify.motor_poles(), verify.timing(),
           verify.pwm_frequency_khz());
  }

  esc.exitConfigMode();
  restore_suction_pwm_pin();
  printf("== AM32 write done ==\n");
}

// read_am32_param()/write_am32_param()が保存したバックアップ(/am32_backup.bin)
// をESCへ書き戻す。書き換え前の生byte列をそのまま使うため、motor_kv()等の
// 人間単位アクセサは経由しない(換算の往復誤差を避けるため)。
void MainTask::restore_am32_param() {
  printf("== AM32 restore ==\n");
  const int32_t sz = ConfigLoader::file_size(kAm32BackupPath);
  if (sz != (int32_t)AM32Settings::LAYOUT_SIZE) {
    printf("am32: no valid backup at %s (size=%ld, expected %u) -- run read/write first\n",
           kAm32BackupPath, (long)sz, (unsigned)AM32Settings::LAYOUT_SIZE);
    return;
  }

  AM32Settings settings{};
  size_t out_size = 0;
  if (!ConfigLoader::read_file_raw(kAm32BackupPath, settings.raw.buffer,
                                    sizeof(settings.raw.buffer), out_size) ||
      out_size != AM32Settings::LAYOUT_SIZE) {
    printf("am32: backup read failed\n");
    return;
  }
  printf("am32: backup loaded: KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
         settings.motor_kv(), settings.motor_poles(), settings.timing(),
         settings.pwm_frequency_khz());

  planning_->suction_disable();
  while (planning_->is_suction_ramping()) {
    sleep_ms(5);
  }

  static AM32 esc;
  esc.init(SUCTION_ESC_PWM, pio0);

  Am32ConfigStatus st = esc.enterConfigMode();
  if (st != Am32ConfigStatus::OK) {
    printf("enterConfigMode failed: %s (%s)\n",
           am32_config_status_to_string(st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
    restore_suction_pwm_pin();
    return;
  }

  st = esc.writeSettings(settings);  // 範囲チェック + RAM staging更新のみ
  if (st != Am32ConfigStatus::OK) {
    char verr[64] = {0};
    am32_validate_settings(settings, verr, sizeof(verr));
    printf("writeSettings failed (validation): %s -- %s\n",
           am32_config_status_to_string(st), verr);
    esc.exitConfigMode();
    restore_suction_pwm_pin();
    return;
  }

  st = esc.saveSettings();
  if (st != Am32ConfigStatus::OK) {
    printf("saveSettings failed: %s (%s)\n",
           am32_config_status_to_string(st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
  } else {
    printf("restore OK\n");
  }

  AM32Settings verify;
  st = esc.readSettings(verify);
  if (st == Am32ConfigStatus::OK) {
    printf("after restore: KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
           verify.motor_kv(), verify.motor_poles(), verify.timing(),
           verify.pwm_frequency_khz());
  }

  esc.exitConfigMode();
  restore_suction_pwm_pin();
  printf("== AM32 restore done ==\n");
}

void MainTask::test_suction() {
  const auto se = get_sensing_entity();
  const float target_v = sys_.test.suction_duty;

  mp->reset_gyro_ref_with_check();
  sleep_ms(250);

  // AM32 ESC移行によりBLDC V/Hzパラメータダンプ・ALIGN/RAMP/RUN直叩き
  // テスト(旧bldc_.test_direct())は廃止。ControlLaw経由でduty指令→ESCへの
  // PWM出力のみを確認する。
  printf("== suction ESC test ==\n");
  printf("test.suction_duty=%.2f  test.suction_duty_low=%.2f\n",
         sys_.test.suction_duty, sys_.test.suction_duty_low);

  const uint64_t t_suction_start = time_us_64();
  planning_->suction_enable(sys_.test.suction_duty, sys_.test.suction_duty_low);

  while (planning_->is_suction_ramping()) {
    sleep_ms(5);
  }
  const uint64_t t_ramp_done = time_us_64();
  printf("ramp done: %.1fms\n",
         (double)(t_ramp_done - t_suction_start) / 1000.0);

  while (!ui_->button_state()) {
    printf("batt=%.3fV duty=%.1f%%\n",
           se->ego.batt_kf,
           planning_->tgt_val->duty_suction);
    sleep_ms(200);
  }

  planning_->suction_disable();
  printf("suction stopped\n");
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
