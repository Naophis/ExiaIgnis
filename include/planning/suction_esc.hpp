#pragma once
#include "define.hpp"
#include "planning/suction_esc_actuator.hpp"
#include "planning/suction_esc_dshot_actuator.hpp"

// 吸引ESCへの指令経路の型エイリアス。
// SUCTION_ESC_USE_DSHOT(define.hpp)で実装を切り替える。両クラスは同じ
// 公開APIを持つため、PlanningTask/ControlLaw/MainTask側は型名だけ見ればよい
// (仮想関数を挟まないので1kHz IRQ経路のコストはゼロ)。
#if SUCTION_ESC_USE_DSHOT
using SuctionEsc = SuctionEscDshotActuator;  // ESCape32 / 標準DShot600
#else
using SuctionEsc = SuctionEscActuator;       // AM32 / RCサーボ標準PWM
#endif
