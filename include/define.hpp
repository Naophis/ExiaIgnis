#pragma once

// ============================================================
// SPI
// ============================================================
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// ============================================================
// I2C0
// ============================================================
#define I2C_PORT i2c0
#define I2C_SDA  8
#define I2C_SCL  9

// ============================================================
// I2C1  (UI / LED driver)
// ============================================================
#define I2C1_PORT     i2c1
#define I2C1_SCL      19
#define I2C1_SDA      22
#define LED_I2C_ADDR  0x4D   // 7-bit (Pico SDK形式: 0x9A >> 1)

// ============================================================
// UART
// ============================================================
#define UART_ID      uart1
#define BAUD_RATE    115200
#define UART_TX_PIN  4
#define UART_RX_PIN  5

// ============================================================
// GPIO / Buttons
// ============================================================
// Tactile switch on GPIO4 (active low, pull-up)
#define BTN_PIN 4

// ============================================================
// PWM / Buzzer
// ============================================================
// Piezo buzzer on GPIO16 (PWM0 A)
#define BUZZER_PIN      17
#define BUZZER_FREQ_HZ  500
#define BUZZER_WRAP     999u  // PWM counter top (1000 steps)

// ============================================================
// PWM / Motor & Suction (50 kHz)
// ============================================================
#define M_PWM_L1      6   // PWM3 A
#define M_PWM_L2      7   // PWM3 B
#define M_PWM_R1      10  // PWM5 A
#define M_PWM_R2      11  // PWM5 B
#define SUCTION_PWM   8   // PWM4 A

#define MOTOR_PWM_FREQ_HZ  50000u
#define MOTOR_DUTY_PCT     5u     // duty (%) applied to active channel
