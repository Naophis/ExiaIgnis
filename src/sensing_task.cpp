#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "sensing_task.hpp"

// --- ADC / LED ---
#define LED0_PIN        20
#define LED1_PIN        21
#define LED_SETTLE_US   10

// --- ジャイロ (ASM330LHH) SPI0 ---
#define GYRO_SPI        spi0
#define GYRO_SCK_PIN    2
#define GYRO_MOSI_PIN   3
#define GYRO_MISO_PIN   4
#define GYRO_CS_PIN     5

#define ASM330_WHO_AM_I     0x0F
#define ASM330_CTRL2_G      0x11
#define ASM330_OUTX_L_G     0x22

// --- エンコーダ (AS5147P x2) SPI1 ---
#define ENC_SPI         spi1
#define ENC_SCK_PIN     10
#define ENC_MOSI_PIN    11
#define ENC_MISO_PIN    12
#define ENC0_CS_PIN     13
#define ENC1_CS_PIN     14

#define AS5147_READ_ANGLE   0xFFFF
#define AS5147_NOP_READ     0xC000


// static メンバの定義
std::shared_ptr<SensingTask> SensingTask::s_instance;


// ============================================================
// ファクトリ
// ============================================================
std::shared_ptr<SensingTask> SensingTask::create() {
    // private コンストラクタを呼ぶため new を使用
    s_instance = std::shared_ptr<SensingTask>(new SensingTask());
    return s_instance;
}


// ============================================================
// センサードライバ (private)
// ============================================================
void SensingTask::asm330_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {static_cast<uint8_t>(reg & 0x7F), val};
    gpio_put(GYRO_CS_PIN, 0);
    spi_write_blocking(GYRO_SPI, buf, 2);
    gpio_put(GYRO_CS_PIN, 1);
}

uint8_t SensingTask::asm330_read_reg(uint8_t reg) {
    uint8_t cmd = reg | 0x80;
    uint8_t val = 0;
    gpio_put(GYRO_CS_PIN, 0);
    spi_write_blocking(GYRO_SPI, &cmd, 1);
    spi_read_blocking(GYRO_SPI, 0x00, &val, 1);
    gpio_put(GYRO_CS_PIN, 1);
    return val;
}

void SensingTask::asm330_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t cmd = ASM330_OUTX_L_G | 0x80;
    uint8_t buf[6];
    gpio_put(GYRO_CS_PIN, 0);
    spi_write_blocking(GYRO_SPI, &cmd, 1);
    spi_read_blocking(GYRO_SPI, 0x00, buf, 6);
    gpio_put(GYRO_CS_PIN, 1);
    *gx = static_cast<int16_t>(buf[1] << 8 | buf[0]);
    *gy = static_cast<int16_t>(buf[3] << 8 | buf[2]);
    *gz = static_cast<int16_t>(buf[5] << 8 | buf[4]);
}

uint16_t SensingTask::as5147p_read_angle(uint gpio_cs) {
    uint16_t cmd, response;

    cmd = AS5147_READ_ANGLE;
    gpio_put(gpio_cs, 0);
    spi_write16_read16_blocking(ENC_SPI, &cmd, &response, 1);
    gpio_put(gpio_cs, 1);

    cmd = AS5147_NOP_READ;
    gpio_put(gpio_cs, 0);
    spi_write16_read16_blocking(ENC_SPI, &cmd, &response, 1);
    gpio_put(gpio_cs, 1);

    return response & 0x3FFF;
}


// ============================================================
// 直接ハードウェアアラーム IRQ ハンドラ
// alarm_pool を介さないため dispatch overhead なし (~12 cycle = 0.08us)
// ============================================================

// alarm 1: LED settle 後のセンサー読み取り (one-shot)
void SensingTask::timer_b_irq_handler() {
    timer_hw->intr = 1u << 1;  // alarm 1 割り込みフラグをクリア

    // auto *self = s_instance.get();

    // adc_select_input(0);
    // self->data.adc0 = adc_read();
    // adc_select_input(1);
    // self->data.adc1 = adc_read();

    // gpio_put(LED0_PIN, 0);
    // gpio_put(LED1_PIN, 0);

    // self->asm330_read_gyro(
    //     (int16_t *)&self->data.gx,
    //     (int16_t *)&self->data.gy,
    //     (int16_t *)&self->data.gz
    // );
    // self->data.enc0 = self->as5147p_read_angle(ENC0_CS_PIN);
    // self->data.enc1 = self->as5147p_read_angle(ENC1_CS_PIN);

    // self->data_ready = true;
}

// alarm 2: 250us 周期
void SensingTask::timer_a_irq_handler() {
    timer_hw->intr = 1u << 2;  // alarm 2 割り込みフラグをクリア

    // auto *self = s_instance.get();

    // // 次回アラームをすぐに設定 (絶対時刻 → ドリフトなし)
    // self->next_alarm_a_ += 250;
    // timer_hw->alarm[2] = self->next_alarm_a_;

    // // アラーム鳴動直後の時刻を記録 (NVIC latency ~0.08us のみ)
    // uint64_t now = time_us_64();
    // self->data.dt_us = self->prev_timestamp_ ? (uint32_t)(now - self->prev_timestamp_) : 0;
    // self->prev_timestamp_ = now;

    // gpio_put(LED0_PIN, 1);
    // gpio_put(LED1_PIN, 1);

    // // alarm 1 で LED_SETTLE_US 後にセンサー読み取りをスケジュール
    // timer_hw->alarm[1] = (uint32_t)now + LED_SETTLE_US;
}


// ============================================================
// コア1: 直接 IRQ 登録 → タイマー起動
// ============================================================
void SensingTask::run() {
    // alarm 1 (timer_b) と alarm 2 (timer_a) に直接ハンドラを登録
    irq_set_exclusive_handler(TIMER0_IRQ_1, timer_b_irq_handler);
    irq_set_exclusive_handler(TIMER0_IRQ_2, timer_a_irq_handler);

    // 両アラームを最高優先度に設定
    irq_set_priority(TIMER0_IRQ_1, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_priority(TIMER0_IRQ_2, PICO_HIGHEST_IRQ_PRIORITY);

    // NVIC でアラーム割り込みを有効化
    irq_set_enabled(TIMER0_IRQ_1, true);
    irq_set_enabled(TIMER0_IRQ_2, true);

    // タイマー周辺回路でアラーム割り込みを有効化
    hw_set_bits(&timer_hw->inte, (1u << 1) | (1u << 2));

    // 最初のアラームをスケジュール
    next_alarm_a_ = (uint32_t)time_us_64() + 250;
    timer_hw->alarm[2] = next_alarm_a_;

    while (true) {
        __wfi();
    }
}

void SensingTask::core1_entry() {
    flash_safe_execute_core_init();
    s_instance->run();
}


// ============================================================
// 初期化 (コア0の main から呼ぶ)
// ============================================================
void SensingTask::init() {
    // ADC + LED
    gpio_init(LED0_PIN);
    gpio_init(LED1_PIN);
    gpio_set_dir(LED0_PIN, GPIO_OUT);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);

    // SPI0: ジャイロ (ASM330LHH)
    spi_init(GYRO_SPI, 8000000);
    spi_set_format(GYRO_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(GYRO_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(GYRO_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(GYRO_MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(GYRO_CS_PIN);
    gpio_set_dir(GYRO_CS_PIN, GPIO_OUT);
    gpio_put(GYRO_CS_PIN, 1);
    sleep_ms(10);

    uint8_t who = asm330_read_reg(ASM330_WHO_AM_I);
    printf("ASM330LHH WHO_AM_I: 0x%02X (%s)\n", who, who == 0x44 ? "OK" : "NG");
    asm330_write(ASM330_CTRL2_G, 0x7C);  // ODR=833Hz, FS=±2000dps

    // SPI1: エンコーダ (AS5147P x2)
    spi_init(ENC_SPI, 8000000);
    spi_set_format(ENC_SPI, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(ENC_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(ENC_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ENC_MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(ENC0_CS_PIN);
    gpio_init(ENC1_CS_PIN);
    gpio_set_dir(ENC0_CS_PIN, GPIO_OUT);
    gpio_set_dir(ENC1_CS_PIN, GPIO_OUT);
    gpio_put(ENC0_CS_PIN, 1);
    gpio_put(ENC1_CS_PIN, 1);
}
