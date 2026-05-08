#include "main/main_task.hpp"
#include "config_dump.hpp"
#include "config_loader.hpp"
#include "config_mapping.hpp"
#include "define.hpp"
#include "hardware/pwm.h"
#include "pico/error.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <cstdlib>
#include <cstring>
#include <stdio.h>

std::shared_ptr<MainTask> MainTask::s_instance;

std::shared_ptr<MainTask>
MainTask::create(std::shared_ptr<SensingTask> sensing,
                 std::shared_ptr<PlanningTask> planning,
                 std::shared_ptr<input_param_t> param) {
  s_instance = std::shared_ptr<MainTask>(new MainTask());
  s_instance->sensing_ = sensing;
  s_instance->planning_ = planning;
  s_instance->param_ = param;
  return s_instance;
}

void MainTask::start() { s_instance->run(); }

std::shared_ptr<sensing_result_entity_t> MainTask::get_sensing_entity() {
  return sensing_->get_sensing_entity();
}

// ─── USB シリアル通信 (main_task_usb.cpp) ────────────────────────────────
int  usb_read_with_timeout(char *buf, size_t max_size, uint32_t idle_ms);
bool rx_usb_cmd(char *buf, int len);

bool MainTask::load_params() {
  bool any = false;
  // 各ファイルが存在しない場合は警告を出してスキップ (初回未転送でも動作継続)
  any |= ConfigLoader::load_as("/hardware.txt", *param_);
  any |= ConfigLoader::load_as("/sensor.hf", *param_);
  any |= ConfigLoader::load_as("/offset.hf", *param_);
  any |= ConfigLoader::load_as("/system.txt", sys_);
  return any;
}

void MainTask::run() {
  // ブザー PWM 設定 (GPIO はどのコアからでも設定可)
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  uint pwm_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
  uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
  pwm_set_enabled(pwm_slice, false);

  const auto se = get_sensing_entity();

  ui_.init(pwm_slice, pwm_channel, se);
  ui_.LED_headlight();
  // ui_.hello_exia();
  ui_.coin(60);
  ui_.coin(60);

  // ─── パラメータ読み込み ───────────────────────────────────────────────────
  printf("[main] loading params from LittleFS...\n");
  if (load_params()) {
    ui_.coin(80);
    printf("[main] params OK  mode=%d  maze=%d\n", sys_.user_mode,
           sys_.maze_size);
  } else {
    printf("[main] no param files found, using defaults.\n");
  }

  // ─── LittleFS ファイル一覧 + 残量 ──────────────────────────────────────
  printf("[main] LittleFS files:\n");
  ConfigLoader::list_files(
      [](void *, const char *name, int32_t size) {
        printf("  %-32s %d bytes\n", name, (int)size);
      },
      nullptr);
  {
    uint32_t used, total;
    ConfigLoader::storage_info(used, total);
    printf("[main] storage: %u / %u KB used (%u KB free, %u%%)\n",
           used / 1024, total / 1024, (total - used) / 1024,
           used * 100 / total);
  }

  // ─── ボタン待機ループ ────────────────────────────────────────────────────
  // ・"filename@json_content\n" 形式でファイルを受信・保存する
  // ・ファイル受信後は即座にパラメータを再ロードする
  // ・ボタンを押して離したら (button_state_hold) ループを抜ける
  printf("[main] ready. send files or press button to start.\n");
  static char rx_buf[16384];
  bool updated = false;

  // MainTask::dump_all_params();

  // print_system_params();
  // print_hardware_params();
  // print_offset_params();
  // print_sensor_params();

  while (true) {
    if (ui_.button_state_hold()) {
      ui_.coin(100);
      break;
    }
    int rlen = usb_read_with_timeout(rx_buf, sizeof(rx_buf), 50);
    if (rlen > 0 && rx_usb_cmd(rx_buf, rlen)) {
      updated = true;
      load_params();
      printf("[main] params reloaded  mode=%d  maze=%d\n", sys_.user_mode,
             sys_.maze_size);
    }
  }
  if (updated)
    load_params();
  printf("[main] starting.\n");

  sleep_ms(500);

  printf("[main] user_mode=%d\n", sys_.user_mode);
  if (sys_.user_mode != 0) {
    run_test_mode(sys_.user_mode);
  } else {
    run_main_mode();
  }
}
void MainTask::print_system_params() {
  printf("=== System Parameters ===\n");
  printf("User Mode: %d\n", sys_.user_mode);
  printf("Maze Size: %d\n", sys_.maze_size);
  printf("Circuit Mode: %d\n", sys_.circuit_mode);
  printf("HF CL: %d\n", sys_.hf_cl);
  printf("Goals:\n");
  for (const auto &goal : sys_.goals) {
    printf("  - (x: %d, y: %d)\n", goal.x, goal.y);
  }
  printf("test.v_max: %.2f\n", sys_.test.v_max);
  printf("test.accl: %.2f\n", sys_.test.accl);
  printf("test.decel: %.2f\n", sys_.test.decel);
  printf("test.dist: %.2f\n", sys_.test.dist);
}
void MainTask::print_hardware_params() {
  printf("=== Hardware Parameters ===\n");
  printf("tire_tread: %f\n", param_->tire_tread);
  printf("tire: %f\n", param_->tire);
  printf("battery_gain: %f\n", param_->battery_gain);
  printf("gear_a: %f\n", param_->gear_a);
  printf("gear_b: %f\n", param_->gear_b);
}
void MainTask::print_offset_params() {
  printf("=== Offset Parameters ===\n");
  printf("clear_angle: %f\n", param_->clear_angle);
}
void MainTask::print_sensor_params() {
  printf("=== Sensor Parameters ===\n");
  printf("sen_ref_p.normal.ref.left90: %d\n",
         param_->sen_ref_p.normal.ref.left90);
  printf("sen_ref_p.normal.ref.left45: %d\n",
         param_->sen_ref_p.normal.ref.left45);
  printf("sen_ref_p.normal.ref.right45: %d\n",
         param_->sen_ref_p.normal.ref.right45);
  printf("sen_ref_p.normal.ref.right90: %d\n",
         param_->sen_ref_p.normal.ref.right90);

  printf("sensor_gain.l90: %f, %f\n", param_->sensor_gain.l90.a,
         param_->sensor_gain.l90.b);
  printf("sensor_gain.l45_3: %f, %f\n", param_->sensor_gain.l45_3.a,
         param_->sensor_gain.l45_3.b);
  printf("sensor_gain.l45_2: %f, %f\n", param_->sensor_gain.l45_2.a,
         param_->sensor_gain.l45_2.b);
  printf("sensor_gain.l45: %f, %f\n", param_->sensor_gain.l45.a,
         param_->sensor_gain.l45.b);
  printf("sensor_gain.r45: %f, %f\n", param_->sensor_gain.r45.a,
         param_->sensor_gain.r45.b);
  printf("sensor_gain.r45_2: %f, %f\n", param_->sensor_gain.r45_2.a,
         param_->sensor_gain.r45_2.b);
  printf("sensor_gain.r45_3: %f, %f\n", param_->sensor_gain.r45_3.a,
         param_->sensor_gain.r45_3.b);
  printf("sensor_gain.r90: %f, %f\n", param_->sensor_gain.r90.a,
         param_->sensor_gain.r90.b);
}

// シリアルコマンド "DUMP" で呼ばれる。
// param_ / sys_ の全フィールドを JSON で stdout に出力する。
// USB CDC バッファ溢れ防止のため 256 バイトごとにスリープを挟む。
void MainTask::dump_all_params() {
  if (!s_instance)
    return;

  auto dump_json = [](const char *label, const auto &obj) {
    printf("=== DUMP:%s ===\n", label);
    fflush(stdout);

    JsonDocument doc;
    doc.set(obj);

    // measureJson で必要サイズを確認してからバッファ確保
    size_t need = measureJson(doc) + 1;
    char *buf = static_cast<char *>(malloc(need));
    if (!buf) {
      printf("ERR:no memory\n");
      fflush(stdout);
      return;
    }
    serializeJson(doc, buf, need);

    // 256 バイトずつ送信して USB CDC バッファ溢れを防ぐ
    size_t sent = 0;
    while (sent < need - 1) {
      size_t chunk = (need - 1 - sent) < 256 ? (need - 1 - sent) : 256;
      fwrite(buf + sent, 1, chunk, stdout);
      fflush(stdout);
      sleep_ms(10);
      sent += chunk;
    }
    printf("\n");
    fflush(stdout);
    free(buf);
  };

  dump_json("param", *s_instance->param_);
  dump_json("sys", s_instance->sys_);
}