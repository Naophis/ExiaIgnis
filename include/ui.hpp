#pragma once
#include "Music.hpp"
#include "define.hpp"
#include "defines.hpp"
#include "structs.hpp"
#include <memory>
#include <stdint.h>
#include <vector>

// 前方宣言: planning_task.hpp の循環インクルードを避ける
class PlanningTask;

class UserInterface {
public:
  UserInterface() = default;

  // I2C1 + PWM buzzer を初期化。main の PWM setup より後に呼ぶこと。
  void init(uint pwm_slice, uint pwm_channel,
            std::shared_ptr<sensing_result_entity_t> sensing_result);

  // ---- LED (I2C LED driver @ 0x4D) ----
  void LED_on_all();
  void LED_off_all();
  void LED_bit(int b0, int b1, int b2, int b3, int b4, int b5);
  void LED_headlight(); // 前照灯 (b0, b5 のみ点灯)

  // ---- Button ----
  bool button_state();      // 押している間 true
  bool button_state_hold(); // 押して離した瞬間に true

  // ---- Buzzer ----
  void play_tone(int hz);                // 鳴らし続ける
  void stop_tone();                      // 停止
  void music_sync(MUSIC m, int time_ms); // 鳴らして time_ms 待ってから停止
  void music_async(MUSIC m, int time_ms); // 鳴らして即リターン(time_ms は無視)

  // ---- 楽曲シーケンス ----
  void hello_exia();
  void coin(int time_ms);
  void error();

  int encoder_operation();
  void motion_check();

  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> t) { tgt_val_ = t; }
  void set_planning(std::shared_ptr<PlanningTask> p) { pt_ = p; }
  // motion_check の壁距離表示が sen_ref_p.normal.ref.left45/right45 を読む。
  // 未設定なら MC_REF_DEFAULT を使う。
  void set_input_param(std::shared_ptr<input_param_t> p) { param_ = p; }

  TurnDirection select_direction();

  std::vector<uint8_t> blight_level_list; // インデックスごとの輝度値

private:
  void led_write(uint8_t idx, bool state);
  void i2c_write_byte(uint8_t data);
  void set_pwm_freq(int hz);

  // ---- motion_check の壁距離表示 (片側 LED 2 個) ----
  // err = dist - ref を 3 段階に量子化する。
  //   -1: 壁に近すぎ / 0: OK / +1: 壁から遠い
  struct WallGauge {
    bool wall = false; // 表示範囲内に壁がある
    bool ema_valid = false;
    float dist = 0; // EMA 後の距離 [mm]
    int level = 0;
  };
  void update_wall_gauge(WallGauge &g, float raw_dist, float ref);
  // level → 片側 2 個の点灯。blink は「近すぎ」の点滅位相。
  static void wall_gauge_led(const WallGauge &g, bool blink, int &first,
                             int &second);

  static constexpr float MC_REF_DEFAULT = 45.0f; // param 未設定時の基準 [mm]
  static constexpr float MC_OK_TH = 0.5f;        // |err| がこれ以下なら OK [mm]
  static constexpr float MC_HYST = 0.15f;        // 段階境界のヒステリシス [mm]
  static constexpr float MC_WALL_MIN = 20.0f;    // 壁ありとみなす距離範囲 [mm]
  static constexpr float MC_WALL_MAX = 70.0f;
  static constexpr float MC_EMA_ALPHA = 0.25f;   // 20ms 周期で時定数 約70ms
  static constexpr int MC_LOOP_MS = 20;
  static constexpr int MC_FAST_BLINK_LOOPS = 4;  // 80ms ごとに反転 (約6Hz)
  static constexpr int MC_SLOW_BLINK_LOOPS = 25; // 500ms ごとに反転 (1Hz)
  static constexpr int MC_REFRESH_LOOPS = 25;    // 変化が無くても 500ms ごとに再送

  uint pwm_slice_ = 0;
  uint pwm_channel_ = 0;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<motion_tgt_val_t> tgt_val_;
  std::shared_ptr<PlanningTask> pt_;
  std::shared_ptr<input_param_t> param_;
  static constexpr uint16_t ENC_OPE_V_R_TH = 90 * 1;
};
