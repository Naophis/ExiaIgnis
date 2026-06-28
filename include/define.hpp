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
#define SUCTION_PWM1  8   // PWM4 A  (3-phase BLDC phase 1)
#define SUCTION_PWM2  9   // PWM4 B  (3-phase BLDC phase 2)
#define SUCTION_PWM3  10  // PWM5 A  (3-phase BLDC phase 3)
#define SUCTION_EN    11  // GPIO output — HIGH = driver enabled

#define MOTOR_PWM_FREQ_HZ  37500u
