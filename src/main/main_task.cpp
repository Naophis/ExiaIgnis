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
MainTask::create(std::shared_ptr<SensingTask>   sensing,
                 std::shared_ptr<PlanningTask>  planning,
                 std::shared_ptr<input_param_t> param) {
  s_instance = std::shared_ptr<MainTask>(new MainTask());
  s_instance->sensing_  = sensing;
  s_instance->planning_ = planning;
  s_instance->param_    = param;
  return s_instance;
}

void MainTask::start() { s_instance->run(); }

std::shared_ptr<sensing_result_entity_t> MainTask::get_sensing_entity() {
  return sensing_->get_sensing_entity();
}

// ─── シリアルファイル転送プロトコル処理 ──────────────────────────────────────
// send_file.py と対話し、ボタン待機中の WRITE/READ/LIST/DELETE コマンドを処理する。
// WRITE でファイルを受信・保存した場合は true を返す (呼び出し側で再ロード要)。
static bool process_serial_cmd() {
    static char line_buf[256];
    static int  line_len = 0;

    // 利用可能なバイトを非ブロッキングで読み込み、行が揃ったらコマンド処理
    while (true) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;
        if (c == '\r') continue;
        if (c != '\n') {
            if (line_len < (int)sizeof(line_buf) - 1)
                line_buf[line_len++] = (char)c;
            continue;
        }

        // \n 受信 → コマンド確定
        line_buf[line_len] = '\0';
        line_len = 0;
        const char* cmd = line_buf;

        // ── WRITE:<name>:<size> ─────────────────────────────────────────────
        if (strncmp(cmd, "WRITE:", 6) == 0) {
            const char* p     = cmd + 6;
            const char* colon = strrchr(p, ':');
            if (!colon) { printf("ERR:bad format\n"); fflush(stdout); continue; }

            // ファイル名をコピーして null 終端
            char name[64];
            size_t nlen = (size_t)(colon - p);
            if (nlen == 0 || nlen >= sizeof(name) - 2) {
                printf("ERR:name too long\n"); fflush(stdout); continue;
            }
            name[0] = '/';
            memcpy(name + 1, p, nlen);
            name[nlen + 1] = '\0';

            size_t size = (size_t)atoi(colon + 1);
            if (size == 0 || size > 128 * 1024) {
                printf("ERR:invalid size\n"); fflush(stdout); continue;
            }

            // ペイロード受信 (ブロッキング)
            uint8_t* buf = (uint8_t*)malloc(size);
            if (!buf) { printf("ERR:no memory\n"); fflush(stdout); continue; }

            size_t received = 0;
            while (received < size) {
                int b = getchar_timeout_us(100000); // 100 ms/byte
                if (b == PICO_ERROR_TIMEOUT) break;
                buf[received++] = (uint8_t)b;
            }

            if (received == size) {
                if (ConfigLoader::write_file(name, buf, size)) {
                    printf("OK\n");
                } else {
                    printf("ERR:write failed\n");
                }
            } else {
                printf("ERR:timeout (%u/%u bytes)\n",
                       (unsigned)received, (unsigned)size);
            }
            fflush(stdout);
            free(buf);

            if (received == size) return true; // 再ロード要求
            continue;
        }

        // ── READ:<name> ─────────────────────────────────────────────────────
        if (strncmp(cmd, "READ:", 5) == 0) {
            char name[64];
            const char* src = cmd + 5;
            name[0] = '/';
            strncpy(name + 1, src, sizeof(name) - 2);
            name[sizeof(name) - 1] = '\0';

            int32_t fsize = ConfigLoader::file_size(name);
            if (fsize < 0) {
                printf("ERR:not found\n"); fflush(stdout); continue;
            }

            uint8_t* buf = (uint8_t*)malloc((size_t)fsize);
            if (!buf) { printf("ERR:no memory\n"); fflush(stdout); continue; }

            size_t out_size = 0;
            if (ConfigLoader::read_file_raw(name, buf, (size_t)fsize, out_size)) {
                printf("%u\n", (unsigned)out_size);
                fflush(stdout);
                fwrite(buf, 1, out_size, stdout);
                fflush(stdout);
                printf("OK\n");
                fflush(stdout);
            } else {
                printf("ERR:read failed\n");
                fflush(stdout);
            }
            free(buf);
            continue;
        }

        // ── LIST ─────────────────────────────────────────────────────────────
        if (strcmp(cmd, "LIST") == 0) {
            ConfigLoader::list_files(
                [](void*, const char* name, int32_t size) {
                    printf("%s:%d\n", name, (int)size);
                },
                nullptr);
            printf("OK\n");
            fflush(stdout);
            continue;
        }

        // ── DELETE:<name> ────────────────────────────────────────────────────
        if (strncmp(cmd, "DELETE:", 7) == 0) {
            char name[64];
            const char* src = cmd + 7;
            name[0] = '/';
            strncpy(name + 1, src, sizeof(name) - 2);
            name[sizeof(name) - 1] = '\0';

            if (ConfigLoader::delete_file(name)) {
                printf("OK\n");
            } else {
                printf("ERR:not found\n");
            }
            fflush(stdout);
            continue;
        }

        // 未知コマンドは無視 (センサー出力の取りこぼしなど)
    }
    return false;
}

bool MainTask::load_params() {
  bool any = false;
  // 各ファイルが存在しない場合は警告を出してスキップ (初回未転送でも動作継続)
  any |= ConfigLoader::load_as("/hardware.json", *param_);
  any |= ConfigLoader::load_as("/sensor.json",   *param_);
  any |= ConfigLoader::load_as("/offset.json",   *param_);
  any |= ConfigLoader::load_as("/system.json",   sys_);
  return any;
}

void MainTask::run() {
  // ブザー PWM 設定 (GPIO はどのコアからでも設定可)
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  uint pwm_slice   = pwm_gpio_to_slice_num(BUZZER_PIN);
  uint pwm_channel = pwm_gpio_to_channel(BUZZER_PIN);
  pwm_set_enabled(pwm_slice, false);

  const auto se = get_sensing_entity();

  ui_.init(pwm_slice, pwm_channel);
  ui_.LED_headlight();
  ui_.hello_exia();

  // ─── パラメータ読み込み ───────────────────────────────────────────────────
  printf("[main] loading params from LittleFS...\n");
  if (load_params()) {
    ui_.coin(80);
    printf("[main] params OK  mode=%d  maze=%d\n",
           sys_.user_mode, sys_.maze_size);
  } else {
    printf("[main] no param files found, using defaults.\n");
  }

  // ─── ボタン待機ループ ────────────────────────────────────────────────────
  // ・シリアルコマンド (send_file.py) でファイル転送を受け付ける
  // ・WRITE 受信後は即座にパラメータを再ロードする
  // ・1 秒ごとにも再ロードを試みる (手動書き換え対応)
  // ・ボタンを押して離したら (button_state_hold) ループを抜ける
  printf("[main] ready. send files or press button to start.\n");
  absolute_time_t next_reload = make_timeout_time_ms(1000);
  while (true) {
    if (ui_.button_state_hold()) {
      ui_.coin(100);
      break;
    }

    // ファイル転送コマンドを処理; WRITE があれば即再ロード
    if (process_serial_cmd()) {
      load_params();
      printf("[main] params reloaded (file transfer)  mode=%d  maze=%d\n",
             sys_.user_mode, sys_.maze_size);
      next_reload = make_timeout_time_ms(1000);
    }

    // 1 秒ごとの定期再ロード
    if (absolute_time_diff_us(get_absolute_time(), next_reload) <= 0) {
      if (load_params()) {
        printf("[main] params reloaded (periodic)  mode=%d  maze=%d\n",
               sys_.user_mode, sys_.maze_size);
      }
      next_reload = make_timeout_time_ms(1000);
    }

    sleep_ms(25);
  }
  printf("[main] starting.\n");

  // ─── 吸引 PWM テスト ─────────────────────────────────────────────────────
  // ボタン押下から 3 秒後に duty_suction を直接駆動する。
  // ControlLaw・バッテリ補正・ランプを完全バイパスして motor_.apply() を直叩き。
  static constexpr float    SUCTION_TEST_DUTY = 15.0f;  // [%] 要チューニング
  static constexpr uint32_t SUCTION_DELAY_MS  = 3000;   // 押下から起動までの遅延
  bool            prev_btn        = false;
  bool            suction_test_on = false;
  bool            btn_held        = false;
  absolute_time_t btn_press_time  = nil_time;

  while (true) {
    // ---- ボタン処理 (吸引 PWM 直接テスト) ----
    bool btn = !gpio_get(BTN_PIN); // active low

    if (btn != prev_btn) {
      if (btn) {
        if (suction_test_on) {
          // 2 回目押下: 吸引 OFF
          planning_->set_suction_test(0.0f);
          suction_test_on = false;
          btn_held        = false;
          ui_.stop_tone();
          ui_.LED_headlight();
        } else {
          // 1 回目押下: 3 秒タイマー開始
          btn_held       = true;
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
        btn_held        = false;
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

      printf("[sys]      mode=%d  maze=%d  goals=%u\n",
             sys_.user_mode, sys_.maze_size,
             (unsigned)sys_.goals.size());
    }

    sleep_ms(25);
  }
}
