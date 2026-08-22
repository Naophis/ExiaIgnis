#include "define.hpp"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "driver/am32_config.hpp"
#include "config_loader.hpp"
#include <cmath>
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

// tools/param_tuner/profile/am32.yaml から生成された /am32.txt を読み込む。
//
// [重要] ESCのbootloaderはreceiveBuffer()呼び出しから20ms以内に次フレームの
// 先頭byteが来ないと invalid_command=101 とみなし、その場でapplicationへ
// jumpしてしまう(出典: 調査レポート§2.2)。LittleFSのflash erase/program相当の
// アクセスはCLAUDE.mdにもある通り~100msかかり得るため、この読み込みは
// 必ずenterConfigMode()より前(=ESCとの通信区間の外側)で行うこと。
bool load_am32_target_doc(JsonDocument& doc) {
  return ConfigLoader::load_file("/am32.txt", doc);
}

// 読み込み済みdocの内容で、out (ESCから読み出した現在値) の該当フィールドだけ
// 上書きする。純粋にRAM上の操作でLittleFSへは触れない(config session中に
// 呼んでよい)。doc側に存在しないキーはoutの現在値がそのまま残る(部分上書き)。
// 各setterが内部で範囲クランプするため、ここでの追加チェックは不要。
void apply_am32_target_doc(const JsonDocument& doc, AM32Settings& out) {
  out.set_max_ramp_raw(doc["max_ramp_raw"] | out.max_ramp_raw());
  out.set_motor_kv(doc["motor_kv"] | out.motor_kv());
  out.set_motor_poles(doc["motor_poles"] | out.motor_poles());
  out.set_timing(doc["timing"] | out.timing());
  out.set_auto_advance_enabled(doc["auto_advance"] | out.auto_advance_enabled());
  out.set_pwm_frequency_khz(doc["pwm_frequency_khz"] | out.pwm_frequency_khz());
  // pwm_mode(0=Fixed/1=Variable/2=by RPM、fw>=2.18限定)が指定されていればそちらを
  // 優先する。無ければ従来通りbool方式のvariable_pwmを使う(常にraw=1を書く)。
  if (doc["pwm_mode"].is<int>()) {
    out.set_pwm_mode_raw(doc["pwm_mode"].as<uint8_t>());
  } else {
    out.set_variable_pwm_enabled(doc["variable_pwm"] | out.variable_pwm_enabled());
  }
  out.set_startup_power_percent(doc["startup_power_percent"] | out.startup_power_percent());
  out.set_sine_startup_enabled(doc["sine_startup"] | out.sine_startup_enabled());
  out.set_bidirectional_enabled(doc["bidirectional"] | out.bidirectional_enabled());
  out.set_complementary_pwm_enabled(doc["complementary_pwm"] | out.complementary_pwm_enabled());
  out.set_stuck_rotor_protection_enabled(
      doc["stuck_rotor_protection"] | out.stuck_rotor_protection_enabled());
  out.set_brake_on_stop(doc["brake_on_stop"] | out.brake_on_stop());
  out.set_drag_brake_strength(doc["drag_brake_strength"] | out.drag_brake_strength());
  out.set_driving_brake_strength(doc["driving_brake_strength"] | out.driving_brake_strength());
  if (doc["current_limit_disabled"] | false) {
    out.disable_current_limit();
  } else {
    out.set_current_limit_amps(doc["current_limit_amps"] | out.current_limit_amps());
  }
  if (doc["temp_limit_disabled"] | false) {
    out.disable_temperature_limit();
  } else {
    out.set_temperature_limit_c(doc["temp_limit_c"] | out.temperature_limit_c());
  }
  out.set_beep_volume(doc["beep_volume"] | out.beep_volume());
  out.set_low_voltage_cutoff_raw(
      doc["low_voltage_cutoff_raw"] | out.low_voltage_cutoff_raw());
  out.set_low_cell_volt_cutoff_v(
      doc["low_cell_volt_cutoff"] | out.low_cell_volt_cutoff_v());
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
  // [重要] handshake成功直後、次のESC通信(readSettings)を始めるまでの間に
  // printfを挟まないこと(saveSettings()のコメント参照: USB CDC経由のprintfは
  // 最大500msブロックし得るため、間に挟むとESC bootloaderの新フレーム待ち
  // 20msタイムアウトを超えてしまう)。devinfoの表示はreadSettings完了後
  // ("dump"はRAM上の操作でESC通信を伴わないため、その後段なら安全)まで遅らせる。
  am32_handle_cli(esc, "read", am32_printf_line);
  am32_handle_cli(esc, "dump", am32_printf_line);

  printf("handshake OK (attempt %d). devinfo:", esc.last_handshake_attempts());
  for (int i = 0; i < 9; i++) {
    printf(" %02X", esc.device_info()[i]);
  }
  printf("\n");

  esc.exitConfigMode();
  restore_suction_pwm_pin();

  // ESCとの通信区間の外側(config mode終了後)でLittleFSへ書き込む
  // (理由: apply_am32_target_doc()のコメント参照)。
  backup_am32_settings(esc.cached_settings());
  printf("== AM32 read done ==\n");
}

void MainTask::write_am32_param() {
  planning_->suction_disable();
  while (planning_->is_suction_ramping()) {
    sleep_ms(5);
  }

  // /am32.txt の読み込みはESCとのconfig session開始前に済ませる
  // (apply_am32_target_doc()のコメント参照: session中のLittleFSアクセスは
  // ESC bootloaderの20ms待受タイムアウトを超えて勝手にjumpさせてしまう)。
  JsonDocument target_doc;
  if (!load_am32_target_doc(target_doc)) {
    printf("am32: /am32.txt not found -- upload tools/param_tuner/profile/am32.yaml "
           "first (aborting, nothing written)\n");
    return;
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

  // [重要] ここから下、ESCと通信する一連の呼び出し(readSettings→writeSettings→
  // saveSettings→readSettings)の間には絶対にprintfを挟まない。USB CDC経由の
  // printf()はホストの受信状況次第で最大500msブロックし得る(pico_stdio_usb既定の
  // PICO_STDIO_USB_STDOUT_TIMEOUT_US)ため、間に1回でも挟むとESC bootloaderの
  // 新フレーム待ち20msタイムアウトを超えて勝手にapplicationへjumpしてしまう
  // (実際にこれが原因でsaveSettings()が恒常的にTIMEOUTしていた実例がある)。
  // 診断出力は全ESC通信が終わった後にまとめて行う。
  AM32Settings settings;  // ESCから読み出した"before"値。書き込み後のバックアップに使う
  Am32ConfigStatus read_st = esc.readSettings(settings);

  AM32Settings target = settings;
  Am32ConfigStatus write_st = Am32ConfigStatus::NOT_IN_CONFIG_MODE;  // "未試行"のsentinel
  Am32ConfigStatus save_st = Am32ConfigStatus::NOT_IN_CONFIG_MODE;
  Am32ConfigStatus verify_st = Am32ConfigStatus::NOT_IN_CONFIG_MODE;
  AM32Settings verify{};
  char verr[64] = {0};
  // saveSettings()失敗時の実プロトコルステータス。この直後にreadSettings(verify)を
  // 呼ぶとesc.last_protocol_status()がそちらの結果で上書きされてしまうため、
  // 表示用に先に退避しておく。
  Am32Protocol::Status save_proto_st = Am32Protocol::Status::OK;

  if (read_st == Am32ConfigStatus::OK) {
    // target_docの内容で上書きする(RAM上の操作のみ、LittleFSへは触れない)。
    // yaml側に無い項目はESCの現在値のまま。
    apply_am32_target_doc(target_doc, target);
    write_st = esc.writeSettings(target);  // 範囲チェック + RAM staging更新のみ
    if (write_st != Am32ConfigStatus::OK) {
      am32_validate_settings(target, verr, sizeof(verr));  // 詳細理由を後で表示
    } else {
      save_st = esc.saveSettings();  // ここで実際にESCのflashへ書き込む
      save_proto_st = esc.last_protocol_status();
      if (save_st != Am32ConfigStatus::OK) {
        esc.rollback();
      }
      verify_st = esc.readSettings(verify);
    }
  }

  // ここまででESCとの全通信が完了。以降はまとめて出力してよい。
  printf("handshake OK (attempt %d)\n", esc.last_handshake_attempts());
  if (read_st != Am32ConfigStatus::OK) {
    printf("readSettings failed: %s (%s)\n",
           am32_config_status_to_string(read_st),
           Am32Protocol::status_to_string(esc.last_protocol_status()));
  } else {
    printf("before: KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
           settings.motor_kv(), settings.motor_poles(), settings.timing(),
           settings.pwm_frequency_khz());
    printf("target: KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
           target.motor_kv(), target.motor_poles(), target.timing(),
           target.pwm_frequency_khz());
    if (write_st != Am32ConfigStatus::OK) {
      printf("writeSettings failed (validation): %s -- %s\n",
             am32_config_status_to_string(write_st), verr);
    } else if (save_st != Am32ConfigStatus::OK) {
      printf("saveSettings failed: %s (%s) -- rolling back staged value\n",
             am32_config_status_to_string(save_st), Am32Protocol::status_to_string(save_proto_st));
    } else {
      printf("save OK\n");
    }
    if (verify_st == Am32ConfigStatus::OK) {
      printf("after : KV=%u Poles=%u Timing=%u PWMfreq=%ukHz\n",
             verify.motor_kv(), verify.motor_poles(), verify.timing(),
             verify.pwm_frequency_khz());
    }
  }

  esc.exitConfigMode();
  restore_suction_pwm_pin();

  // ESCとの通信区間の外側(config mode終了後)でLittleFSへ書き込む。
  // "before"値(=書き換え前の生byte)をrestore_am32_param()用に保存する。
  if (read_st == Am32ConfigStatus::OK) {
    backup_am32_settings(settings);
  }
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

  // [重要] saveSettings()とreadSettings()の間にprintfを挟まない
  // (write_am32_param()のコメント参照)。診断出力はどちらも終わってからまとめて行う。
  Am32ConfigStatus save_st = esc.saveSettings();
  Am32Protocol::Status save_proto_st = esc.last_protocol_status();  // readSettingsで上書きされる前に退避
  if (save_st != Am32ConfigStatus::OK) {
    esc.rollback();
  }
  AM32Settings verify{};
  Am32ConfigStatus verify_st = esc.readSettings(verify);

  printf("handshake OK (attempt %d)\n", esc.last_handshake_attempts());
  if (save_st != Am32ConfigStatus::OK) {
    printf("saveSettings failed: %s (%s)\n",
           am32_config_status_to_string(save_st), Am32Protocol::status_to_string(save_proto_st));
  } else {
    printf("restore OK\n");
  }
  if (verify_st == Am32ConfigStatus::OK) {
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

  while (1) {
    mp->reset_gyro_ref_with_check();
    sleep_ms(250);

    // AM32 ESC移行によりBLDC V/Hzパラメータダンプ・ALIGN/RAMP/RUN直叩き
    // テスト(旧bldc_.test_direct())は廃止。ControlLaw経由でduty指令→ESCへの
    // PWM出力のみを確認する。
    printf("== suction ESC test ==\n");
    printf("test.suction_duty=%.2f  test.suction_duty_low=%.2f\n",
           sys_.test.suction_duty, sys_.test.suction_duty_low);

    const uint64_t t_suction_start = time_us_64();
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);

    while (planning_->is_suction_ramping()) {
      sleep_ms(5);
    }
    const uint64_t t_ramp_done = time_us_64();
    printf("ramp done: %.1fms\n",
           (double)(t_ramp_done - t_suction_start) / 1000.0);

    while (!ui_->button_state()) {
      printf("batt=%.3fV duty=%.1f%%\n", se->ego.batt_kf,
             planning_->tgt_val->duty_suction);
      sleep_ms(200);
    }

    planning_->suction_disable();
    printf("suction stopped\n");
  }
}

void MainTask::dump1() {

  mp->reset_gyro_ref_with_check();
  sleep_ms(10);
  planning_->tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
  planning_->tgt_val->nmr.timstamp = planning_->tgt_val->nmr.timstamp + 1;
  planning_->send_command(*planning_->tgt_val);
  const auto se = get_sensing_entity();

  // 加速度計オフセット/ゲイン校正用: dump1()を回している間、色々な姿勢に
  // 傾けて各軸の生値(センサー座標系、gyro_pos補正前)の最小・最大を追跡する。
  // 静止状態を保って重力(1g)がその軸の可動域いっぱいに載る姿勢を何通りか
  // 経由すれば、min/maxが実質±1gの読み値に収束する。
  // offset=(min+max)/2, gain=2g/(max-min) を都度計算して表示するので、
  // 値が安定したらhardware.yamlのaccel_?_param.offset/gainに書き写す。
  float accel_min_x = 1e9f, accel_max_x = -1e9f;
  float accel_min_y = 1e9f, accel_max_y = -1e9f;
  float accel_min_z = 1e9f, accel_max_z = -1e9f;

  const char ESC = '\033';
  while (1) {
    // 静止していれば向きに関わらず合成加速度の大きさは常に1gのはず。
    // ここから外れているサンプルは、姿勢変更中の動的な加速度(ジャーク)を
    // 拾ってしまっている可能性が高いのでmin/max更新から除外する。
    const float mag = sqrtf(se->accel_x.data * se->accel_x.data +
                            se->accel_y.data * se->accel_y.data +
                            se->accel_z.data * se->accel_z.data);
    constexpr float kG = 9806.65f;
    constexpr float kMagTolerance = 1000.0f; // ±1000mm/s^2 (約±10%)
    const bool is_still = fabsf(mag - kG) < kMagTolerance;

    if (is_still) {
      if (se->accel_x.data < accel_min_x) accel_min_x = se->accel_x.data;
      if (se->accel_x.data > accel_max_x) accel_max_x = se->accel_x.data;
      if (se->accel_y.data < accel_min_y) accel_min_y = se->accel_y.data;
      if (se->accel_y.data > accel_max_y) accel_max_y = se->accel_y.data;
      if (se->accel_z.data < accel_min_z) accel_min_z = se->accel_z.data;
      if (se->accel_z.data > accel_max_z) accel_max_z = se->accel_z.data;
    }

    printf("%c[2J", ESC);
    printf("%c[0;0H", ESC);

    printf("SW1 %d \n", gpio_get(BTN_PIN));
    printf("Temp %f \n", se->ego.temp);

    printf("gyro: %d\t(%0.3f)\n", se->gyro.raw,
           planning_->tgt_val->gyro_zero_p_offset);
    printf("gyro2: %d\t(%0.3f)\n", se->gyro2.raw,
           planning_->tgt_val->gyro2_zero_p_offset);
    printf("accel_x: %f\t(%f)\n", se->ego.accel_x_raw,
           (se->ego.accel_x_raw - param_->accel_x_param.offset) / 9806.65 *
               param_->accel_x_param.gain);
    printf("accel_y: %f\t(%f)\n", se->accel_y.data,
           (se->accel_y.data - param_->accel_y_param.offset) / 9806.65 *
               param_->accel_y_param.gain);
    printf("accel_z: %f\t(%f)\n", se->accel_z.data,
           (se->accel_z.data - param_->accel_z_param.offset) / 9806.65 *
               param_->accel_z_param.gain);
    printf("accel_corr(gyro_pos補正後): %f, %f, %f\n", se->ego.accel_x_corr,
           se->ego.accel_y_corr, se->ego.accel_z_corr);
    printf("accel_mag: %.1f (1g=%.1f) %s\n", mag, kG,
           is_still ? "static" : "moving(除外中)");
    {
      const float rx = accel_max_x - accel_min_x;
      const float ry = accel_max_y - accel_min_y;
      const float rz = accel_max_z - accel_min_z;
      const float gain_x = rx > 1000 ? 2 * kG / rx : 0.0f;
      const float gain_y = ry > 1000 ? 2 * kG / ry : 0.0f;
      const float gain_z = rz > 1000 ? 2 * kG / rz : 0.0f;
      printf("accel_calib_x: min=%.1f max=%.1f -> offset=%.1f gain=%.4f%s\n",
             accel_min_x, accel_max_x, (accel_min_x + accel_max_x) / 2, gain_x,
             rx > 1000 ? "" : " (要姿勢変更)");
      printf("accel_calib_y: min=%.1f max=%.1f -> offset=%.1f gain=%.4f%s\n",
             accel_min_y, accel_max_y, (accel_min_y + accel_max_y) / 2, gain_y,
             ry > 1000 ? "" : " (要姿勢変更)");
      printf("accel_calib_z: min=%.1f max=%.1f -> offset=%.1f gain=%.4f%s\n",
             accel_min_z, accel_max_z, (accel_min_z + accel_max_z) / 2, gain_z,
             rz > 1000 ? "" : " (要姿勢変更)");
    }
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

void MainTask::test_system_identification(bool para) {
  mp->reset_gyro_ref_with_check();

  // Resistは電流センサなしで摩擦モデル経由で推定するため、吸引による
  // 荷重・グリップ変化やモーター電流の変動要因を持ち込まず、常に非吸引で
  // 実施する(sys_.test.suction_activeには依存しない)。

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();
  req_error_reset();

  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }

  // para==false: 並進(直進)方向のduty step応答。para==true: 旋回(roll)
  // 方向のduty step応答(左右逆符号)。PID/FFを介さずduty_l/rを直接そのまま
  // sysid_time[ms]間出力し(control_law.cpp set_next_duty()参照)、その間の
  // エンコーダ速度・バッテリー電圧をログして、後でオフラインでResist等を
  // 解析する。
  const float duty = sys_.test.sysid_duty;
  if (!para) {
    mp->system_identification(MotionType::STRAIGHT, duty, duty,
                              sys_.test.sysid_time);
  } else {
    mp->system_identification(MotionType::PIVOT, -duty, duty,
                              sys_.test.sysid_time);
  }

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
