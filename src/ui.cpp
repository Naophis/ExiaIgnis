#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "ui.hpp"
#include "planning/planning_task.hpp"

// ============================================================
// 初期化
// ============================================================
void UserInterface::init(uint pwm_slice, uint pwm_channel,
                         std::shared_ptr<sensing_result_entity_t> sensing_result) {
    pwm_slice_   = pwm_slice;
    pwm_channel_ = pwm_channel;
    this->sensing_result = sensing_result;

    // I2C1: 400kHz, GPIO19=SCL / GPIO22=SDA
    i2c_init(I2C1_PORT, 400 * 1000);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SCL);
    gpio_pull_up(I2C1_SDA);
}


// ============================================================
// I2C1 低レベル書き込み
// LED ドライバのプロトコル: 1 バイト書き込み
// フォーマット: (channel[2:0] << 5) | brightness[4:0]
// ============================================================
void UserInterface::i2c_write_byte(uint8_t data) {
    // タイムアウト 5ms: デバイス未接続時の永久ブロックを防ぐ
    absolute_time_t timeout = make_timeout_time_ms(5);
    i2c_write_blocking_until(I2C1_PORT, LED_I2C_ADDR, &data, 1, false, timeout);
}

void UserInterface::led_write(uint8_t idx, bool state) {
    uint8_t blight = 0x00;
    if (state && idx < blight_level_list.size()) {
        // blight = blight_level_list[idx];
    }
    if(state) {
        blight = 0x0d;
    }
    i2c_write_byte((idx << 5) | blight);
}


// ============================================================
// LED 制御
// ============================================================
void UserInterface::LED_off_all() {
    for (uint8_t i = 1; i <= 6; ++i) led_write(i, false);
}

void UserInterface::LED_on_all() {
    for (uint8_t i = 1; i <= 6; ++i) led_write(i, true);
}

void UserInterface::LED_bit(int b0, int b1, int b2, int b3, int b4, int b5) {
    led_write(1, b0 == 1);
    led_write(2, b1 == 1);
    led_write(3, b2 == 1);
    led_write(4, b5 == 1);
    led_write(5, b4 == 1);
    led_write(6, b3 == 1);
}

void UserInterface::LED_headlight() {
    LED_bit(0, 0, 0, 0, 0, 0);
}


// ============================================================
// ボタン
// ============================================================
bool UserInterface::button_state() {
    return !gpio_get(BTN_PIN);  // active low
}

bool UserInterface::button_state_hold() {
    if (!gpio_get(BTN_PIN)) {
        while (!gpio_get(BTN_PIN)) tight_loop_contents();
        return true;
    }
    return false;
}


// ============================================================
// PWM ブザー
// ============================================================
void UserInterface::set_pwm_freq(int hz) {
    // 整数分周器固定(16) + wrap可変方式
    // 150MHz / (16 * 65536) ≈ 143Hz まで対応。音楽ノート全域をカバー。
    const uint8_t  div_int  = 16;
    const uint32_t sys_clk  = clock_get_hz(clk_sys);
    uint32_t       wrap     = sys_clk / (div_int * (uint32_t)hz) - 1;

    pwm_set_clkdiv_int_frac(pwm_slice_, div_int, 0);
    pwm_set_wrap(pwm_slice_, (uint16_t)wrap);
    pwm_set_chan_level(pwm_slice_, pwm_channel_, (wrap + 1) / 2);  // 50% duty
}

void UserInterface::play_tone(int hz) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);  // PWM出力に切り替え
    set_pwm_freq(hz);
    pwm_set_enabled(pwm_slice_, true);
}

void UserInterface::stop_tone() {
    pwm_set_enabled(pwm_slice_, false);
    // ピンをGPIO出力LOWに固定して電流を遮断
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);
}

void UserInterface::music_sync(MUSIC m, int time_ms) {
    play_tone((int)m);
    sleep_ms(time_ms);
    stop_tone();
}

// RTOS なし環境では async も sync と同じ動作
void UserInterface::music_async(MUSIC m, int time_ms) {
    music_sync(m, time_ms);
}


// ============================================================
// 楽曲シーケンス
// ============================================================
void UserInterface::coin(int time_ms) {
    music_sync(MUSIC::B5_, time_ms);
    music_sync(MUSIC::E6_, 2 * time_ms);
}

void UserInterface::hello_exia() {
    const int t = 120;
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::A6_, 3*t); sleep_ms(t);
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::A6_, t);   sleep_ms(10);
    music_sync(MUSIC::C7_, t + t/2); sleep_ms(t/3);
    music_sync(MUSIC::G6_, 2*t); sleep_ms(10);
    music_sync(MUSIC::F6_, 2*t); sleep_ms(10);
    music_sync(MUSIC::G6_, 2*t); sleep_ms(10);
}

void UserInterface::error() {
    const int t = 120;
    for (int i = 0; i < 4; ++i) music_sync(MUSIC::C4_, t);
}

int UserInterface::encoder_operation() {
  float v_r = sensing_result->ego.v_r;
  if (v_r > ENC_OPE_V_R_TH) {
    music_sync(MUSIC::G6_, 75);
    return 1;
  }
  if (v_r < -ENC_OPE_V_R_TH) {
    music_sync(MUSIC::C6_, 75);
    return -1;
  }
  return 0;
}

void UserInterface::motion_check() {
  int c = 0;
  printf("motion check start\n");
  if (tgt_val_ && pt_) {
    tgt_val_->nmr.motion_type = MotionType::READY;
    tgt_val_->nmr.timstamp = tgt_val_->nmr.timstamp + 1;
    pt_->send_command(*tgt_val_);
  }

  sleep_ms(1);
  while (1) {
    c++;
    if (c % 2 == 0) {
      LED_on_all();
    } else {
      LED_off_all();
    }
    if (button_state_hold()) {
      LED_off_all();
      break;
    }
    if (sensing_result->ego.left90_mid_dist < 90 &&
        sensing_result->ego.right90_mid_dist < 90 &&
        sensing_result->ego.left90_mid_dist > 10 &&
        sensing_result->ego.right90_mid_dist > 10) {
      LED_off_all();
      LED_off_all();
      for (int i = 0; i < 2; i++) {
        music_sync(MUSIC::C6_, 100);
        sleep_ms(50);
      }
      break;
    }
    sleep_ms(50);
  }
}

TurnDirection UserInterface::select_direction() {  
  TurnDirection td = TurnDirection::None;
  bool b = true;
  while (1) {
    if (sensing_result->ego.v_r > ENC_OPE_V_R_TH) {
      music_sync(MUSIC::G6_, 75);
      td = TurnDirection::Right;
      LED_bit(1, 1, 1, 0, 0, 0);
    }
    if (sensing_result->ego.v_l > ENC_OPE_V_R_TH) {
      music_sync(MUSIC::C6_, 75);
      td = TurnDirection::Left;
      LED_bit(0, 0, 0, 1, 1, 1);
    }

    if (td != TurnDirection::None) {
      if (button_state_hold()) {
        coin(80);
        LED_off_all();
        return td;
      }
    } else {
      LED_bit((int)b, (int)!b, (int)b, (int)b, (int)!b, (int)b);
      b = b ? false : true;
    }
    sleep_ms(25);
  }
}