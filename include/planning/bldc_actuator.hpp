#pragma once

#include "hardware/pwm.h"
#include "pico/types.h"
#include <stdint.h>

// MP6540H 3相ブラシレスモーター吸引用ドライバ
// GPIO8(PWM4A), GPIO9(PWM4B), GPIO10(PWM5A) に 120° 位相シフト PWM を出力
// GPIO11 (SUCTION_EN) で HW イネーブルを制御
//
// TODO: 現在は同一デューティを 3ch に設定するプレースホルダ実装。
//       120° 位相シフト生成はタイマー割り込みベースで別途実装予定。
class BldcActuator {
public:
  // GPIO ファンクション設定・PWM スライス初期化。Core0 の main から呼ぶ。
  void init();

  // デューティ比 [%] を 3ch PWM に反映する（enable 状態に依らず常時更新）
  void set_duty(float duty_percent);

  // SUCTION_EN を HIGH にして PWM スライスを有効化
  void enable();

  // SUCTION_EN を LOW にして PWM スライスを無効化
  void disable();

  bool is_enabled() const { return enabled_; }

private:
  uint     slice_s1_ = 0;   // PWM4: GPIO8(A) + GPIO9(B)
  uint     slice_s2_ = 0;   // PWM5: GPIO10(A)
  uint32_t wrap_     = 0;
  bool     enabled_  = false;
};
