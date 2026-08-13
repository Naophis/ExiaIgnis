#pragma once

// ============================================================
// PSRAM (APS6404L 外付け, QSPI 共有)
// ============================================================
// GPIO0 = XIP_CS1n (QMI M1 CS1)
// XIP_CS1n に対応する GPIO: 0, 8, 19, 47 のみ (function 9)
#define PSRAM_CS_PIN 0

// ============================================================
// I2C1  (UI / LED driver)
// ============================================================
#define I2C1_PORT     i2c1
#define I2C1_SCL      23
#define I2C1_SDA      22
#define LED_I2C_ADDR  0x4D   // 7-bit (Pico SDK形式: 0x9A >> 1)

// ============================================================
// UART (dump_binary() 用。使用時はピンを別途 gpio_set_function すること)
// ============================================================
#define UART_ID    uart1
#define BAUD_RATE  115200

// ============================================================
// GPIO / Buttons
// ============================================================
// Tactile switch on GPIO2 (active low, pull-up)
#define BTN_PIN 2

// ============================================================
// PWM / Buzzer
// ============================================================
// Piezo buzzer on GPIO18 (PWM1 A)
#define BUZZER_PIN      18
#define BUZZER_FREQ_HZ  500
#define BUZZER_WRAP     999u  // PWM counter top (1000 steps)

// ============================================================
// PWM / Motor & Suction (50 kHz)
// ============================================================
#define M_PWM_L1      4   // PWM2 A
#define M_PWM_L2      5   // PWM2 B
#define M_PWM_R1      6   // PWM3 A
#define M_PWM_R2      7   // PWM3 B
#define SUCTION_EN    8   // GPIO output — HIGH = driver enabled (BldcActuator専用、現在未使用)
#define SUCTION_PWM1  9   // PWM4 B  (V phase, BldcActuator専用、現在未使用)
#define SUCTION_PWM2  10  // PWM5 A  (V phase, BldcActuator専用、現在未使用)
#define SUCTION_PWM3  11  // PWM5 B  (BldcActuator W相 / SuctionEscActuator兼用)

#define MOTOR_PWM_FREQ_HZ  37500u

// ============================================================
// PWM / Suction ESC (AM32, RCサーボ標準PWM 1000〜2000us)
// ============================================================
// AM32 ESCへのスロットル信号線。旧BldcActuatorのW相ピン(GPIO11)を流用。
#define SUCTION_ESC_PWM       SUCTION_PWM3
#define SUCTION_ESC_FREQ_HZ   250u    // 信号リフレッシュ周波数(AM32は自動判別、範囲内なら変更可)
#define SUCTION_ESC_PULSE_MIN_US  1000u  // 0%duty (アーム/停止)
#define SUCTION_ESC_PULSE_MAX_US  2000u  // 100%duty (フル)
