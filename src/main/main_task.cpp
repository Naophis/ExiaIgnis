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
  s_instance->mp = std::make_shared<MotionPlanning>();
  s_instance->lgc = std::make_shared<MazeSolverBaseLgc>();
  s_instance->pc = std::make_shared<PathCreator>();
  s_instance->search_ctrl = std::make_shared<SearchController>();
  s_instance->mp->set_path_creator(s_instance->pc);
  return s_instance;
}

void MainTask::start() { s_instance->run(); }

std::shared_ptr<sensing_result_entity_t> MainTask::get_sensing_entity() {
  return sensing_->get_sensing_entity();
}

// ─── USB シリアル通信 (main_task_usb.cpp) ────────────────────────────────
int usb_read_with_timeout(char *buf, size_t max_size, uint32_t idle_ms);
bool rx_usb_cmd(char *buf, int len);

void MainTask::load_param_after() {
  planning_->ctl_.set_suction_gain(sys_.test.suction_gain);
  printf("[param] suction_gain = %f\n", sys_.test.suction_gain);
}

bool MainTask::load_params() {
  bool any = false;
  // 各ファイルが存在しない場合は警告を出してスキップ (初回未転送でも動作継続)
  any |= ConfigLoader::load_as("/hardware.txt", *param_);
  any |= ConfigLoader::load_as("/sensor.hf", *param_);
  any |= ConfigLoader::load_as("/offset.hf", *param_);
  any |= ConfigLoader::load_as("/system.txt", sys_);
  load_param_after();
  return any;
}

void MainTask::run() {
  printf("[run] A: PWM init\n");
  // ブザー PWM 設定 (GPIO はどのコアからでも設定可)
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  uint pwm_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
  uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
  pwm_set_enabled(pwm_slice, false);

  const auto se = get_sensing_entity();

  printf("[run] B: ui init\n");
  ui_.init(pwm_slice, pwm_channel, se);
  ui_.set_tgt_val(tgt_val_);
  ui_.set_planning(planning_);
  ui_.LED_headlight();
  // ui_.hello_exia();
  printf("[run] C: coin\n");
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
    printf("[main] storage: %u / %u KB used (%u KB free, %u%%)\n", used / 1024,
           total / 1024, (total - used) / 1024, used * 100 / total);
  }

  // ─── ボタン待機ループ ────────────────────────────────────────────────────
  // ・"filename@json_content\n" 形式でファイルを受信・保存する
  // ・ファイル受信後は即座にパラメータを再ロードする
  // ・ボタンを押して離したら (button_state_hold) ループを抜ける
  printf("[main] ready. send files or press button to start.\n");
  bool updated = false;
  {
    constexpr size_t RX_BUF_SIZE = 16384;
    char *rx_buf = static_cast<char *>(malloc(RX_BUF_SIZE));
    if (!rx_buf) {
      printf("[main] ERR: failed to allocate rx_buf\n");
    } else {
      uint32_t hb_tick = 0;
      while (true) {
        if (ui_.button_state_hold()) {
          ui_.coin(100);
          break;
        }
        int rlen = usb_read_with_timeout(rx_buf, RX_BUF_SIZE, 50);
        if (rlen > 0 && rx_usb_cmd(rx_buf, rlen)) {
          updated = true;
          ui_.coin(25);
        }
        // 2秒ごとにハートビートを出力してシリアル接続を確認できるようにする
        // send_file.py の "[" スキップフィルタで無視されるので通信に影響なし
        if (++hb_tick >= 40) {
          hb_tick = 0;
          printf("[main] waiting... mode=%d maze=%d\n", sys_.user_mode,
                 sys_.maze_size);
        }
      }
      free(rx_buf);
    }
  }
  if (updated)
    load_params();
  printf("[main] starting.\n");

  sleep_ms(500);

  printf("[main] user_mode=%d\n", sys_.user_mode);
  setup_components();
  reset_tgt_data();
  reset_ego_data();
  if (sys_.user_mode != 0) {
    run_test_mode(sys_.user_mode);
  } else {
    run_main_mode();
  }
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