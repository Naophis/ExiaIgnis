#include "main/main_task.hpp"
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

    bool ok = ConfigLoader::write_file(
        path, reinterpret_cast<const uint8_t *>(content), strlen(content));
    printf(ok ? "OK\n" : "ERR:write failed\n");
    fflush(stdout);
    return ok;
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

  ui_.init(pwm_slice, pwm_channel);
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

  // ─── 吸引 PWM テスト ─────────────────────────────────────────────────────
  // ボタン押下から 3 秒後に duty_suction を直接駆動する。
  // ControlLaw・バッテリ補正・ランプを完全バイパスして motor_.apply()
  // を直叩き。
  static constexpr float SUCTION_TEST_DUTY = 15.0f; // [%] 要チューニング
  static constexpr uint32_t SUCTION_DELAY_MS = 3000; // 押下から起動までの遅延
  bool prev_btn = false;
  bool suction_test_on = false;
  bool btn_held = false;
  absolute_time_t btn_press_time = nil_time;

  while (true) {
    // ---- ボタン処理 (吸引 PWM 直接テスト) ----
    bool btn = !gpio_get(BTN_PIN); // active low

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

    // 押下中かつ未起動: 3 秒経過で吸引 ON
    if (btn_held && !suction_test_on) {
      if (absolute_time_diff_us(btn_press_time, get_absolute_time()) >=
          (int64_t)SUCTION_DELAY_MS * 1000LL) {
        planning_->set_suction_test(SUCTION_TEST_DUTY);
        suction_test_on = true;
        btn_held = false;
        ui_.play_tone(BUZZER_FREQ_HZ);
      }
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
             "  suction=%5.1f%%  tick=%lu  %s\n",
             ps.duty_l, ps.duty_r, ps.duty_suction, ps.tick,
             suction_test_on ? "[SUCTION TEST]" : "");

      printf("[sys]      mode=%d  maze=%d  goals=%u\n", sys_.user_mode,
             sys_.maze_size, (unsigned)sys_.goals.size());
    }

    sleep_ms(25);
  }
}
