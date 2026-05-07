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

// ─── USB シリアル受信 ─────────────────────────────────────────────────────
// idle_ms の無通信が続いたら返す。\n で終端された行を1つ読み込む。
static int usb_read_with_timeout(char *buf, size_t max_size, uint32_t idle_ms) {
  size_t len = 0;
  absolute_time_t deadline = make_timeout_time_ms(idle_ms);
  while (len < max_size - 1) {
    if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0)
      break;
    int c = getchar_timeout_us(1000);
    if (c == PICO_ERROR_TIMEOUT)
      continue;
    buf[len++] = (char)c;
    deadline = make_timeout_time_ms(idle_ms); // 文字受信ごとにリセット
    if ((char)c == '\n')
      break;
  }
  buf[len] = '\0';
  return (int)len;
}

// ─── USB コマンド処理 ─────────────────────────────────────────────────────
// "filename@content" → ファイルを保存して true を返す (再ロード要)。
// "LIST" / "DELETE:name" / "READ:name" も処理する (常に false を返す)。
static bool rx_usb_cmd(char *buf, int len) {
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
    buf[--len] = '\0';
  if (len == 0)
    return false;

  // ── filename@content ─────────────────────────────────────────────────
  char *at = strchr(buf, '@');
  if (at) {
    *at = '\0';
    const char *name = buf;
    const char *content = at + 1;

    char path[68];
    path[0] = '/';
    strncpy(path + 1, name, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';

    // flash_range_erase/prog は割り込みを ~100ms 禁止するため USB CDC
    // が切断される。 OK を先に送信し、USB が届けてから書き込む。
    printf("OK\n");
    fflush(stdout);
    sleep_ms(80); // USB が OK を転送するまで待機

    ConfigLoader::write_file(path, reinterpret_cast<const uint8_t *>(content),
                             strlen(content));
    return true;
  }

  // ── LIST ──────────────────────────────────────────────────────────────
  if (strcmp(buf, "LIST") == 0) {
    ConfigLoader::list_files(
        [](void *, const char *name, int32_t size) {
          printf("%s:%d\n", name, (int)size);
        },
        nullptr);
    printf("OK\n");
    fflush(stdout);
    return false;
  }

  // ── DELETE:name ───────────────────────────────────────────────────────
  if (strncmp(buf, "DELETE:", 7) == 0) {
    char path[68];
    path[0] = '/';
    strncpy(path + 1, buf + 7, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';
    printf(ConfigLoader::delete_file(path) ? "OK\n" : "ERR:not found\n");
    fflush(stdout);
    return false;
  }

  // ── READ:name ─────────────────────────────────────────────────────────
  if (strncmp(buf, "READ:", 5) == 0) {
    char path[68];
    path[0] = '/';
    strncpy(path + 1, buf + 5, sizeof(path) - 2);
    path[sizeof(path) - 1] = '\0';

    int32_t fsize = ConfigLoader::file_size(path);
    if (fsize < 0) {
      printf("ERR:not found\n");
      fflush(stdout);
      return false;
    }

    uint8_t *fbuf = (uint8_t *)malloc((size_t)fsize);
    if (!fbuf) {
      printf("ERR:no memory\n");
      fflush(stdout);
      return false;
    }

    size_t out_size = 0;
    if (ConfigLoader::read_file_raw(path, fbuf, (size_t)fsize, out_size)) {
      printf("%u\n", (unsigned)out_size);
      fflush(stdout);
      fwrite(fbuf, 1, out_size, stdout);
      fflush(stdout);
      printf("OK\n");
      fflush(stdout);
    } else {
      printf("ERR:read failed\n");
      fflush(stdout);
    }
    free(fbuf);
    return false;
  }

  return false;
}

bool MainTask::load_params() {
  bool any = false;
  // 各ファイルが存在しない場合は警告を出してスキップ (初回未転送でも動作継続)
  any |= ConfigLoader::load_as("/hardware.json", *param_);
  any |= ConfigLoader::load_as("/sensor.json", *param_);
  any |= ConfigLoader::load_as("/offset.json", *param_);
  any |= ConfigLoader::load_as("/system.json", sys_);
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

  // ─── ボタン待機ループ ────────────────────────────────────────────────────
  // ・"filename@json_content\n" 形式でファイルを受信・保存する
  // ・ファイル受信後は即座にパラメータを再ロードする
  // ・ボタンを押して離したら (button_state_hold) ループを抜ける
  printf("[main] ready. send files or press button to start.\n");
  static char rx_buf[16384];
  bool updated = false;

  MainTask::dump_all_params();

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