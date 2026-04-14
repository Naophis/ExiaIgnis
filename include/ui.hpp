#pragma once
#include <stdint.h>
#include <vector>
#include "Music.hpp"
#include "define.hpp"

class UserInterface {
public:
    UserInterface() = default;

    // I2C1 + PWM buzzer を初期化。main の PWM setup より後に呼ぶこと。
    void init(uint pwm_slice, uint pwm_channel);

    // ---- LED (I2C LED driver @ 0x4D) ----
    void LED_on_all();
    void LED_off_all();
    void LED_bit(int b0, int b1, int b2, int b3, int b4, int b5);
    void LED_headlight();  // 前照灯 (b0, b5 のみ点灯)

    // ---- Button ----
    bool button_state();        // 押している間 true
    bool button_state_hold();   // 押して離した瞬間に true

    // ---- Buzzer ----
    void play_tone(int hz);     // 鳴らし続ける
    void stop_tone();           // 停止
    void music_sync(MUSIC m, int time_ms);   // 鳴らして time_ms 待ってから停止
    void music_async(MUSIC m, int time_ms);  // 鳴らして即リターン(time_ms は無視)

    // ---- 楽曲シーケンス ----
    void hello_exia();
    void coin(int time_ms);
    void error();

    std::vector<uint8_t> blight_level_list;  // インデックスごとの輝度値

private:
    void     led_write(uint8_t idx, bool state);
    void     i2c_write_byte(uint8_t data);
    void     set_pwm_freq(int hz);

    uint pwm_slice_   = 0;
    uint pwm_channel_ = 0;
};
