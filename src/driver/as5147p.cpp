#include "driver/as5147p.hpp"
#include "driver/spi_util.hpp"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <cstdio>

// Even parity over bits 14:0
static bool calc_parity(uint16_t val) {
    val &= 0x7FFF;
    val ^= val >> 8;
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return val & 1;
}

static uint16_t make_cmd(uint16_t frame) {
    return calc_parity(frame) ? (frame | 0x8000u) : frame;
}

// 16bit ワードを 8bit×2 で送受信 (sample/test.cpp と同じ方式)
static uint16_t spi_transfer16(spi_inst_t *spi, uint cs, uint16_t tx_word) {
    uint8_t tx[2] = {(uint8_t)(tx_word >> 8), (uint8_t)(tx_word & 0xFF)};
    uint8_t rx[2] = {0, 0};

    gpio_put(cs, 0);
    sleep_us(1);                              // CS setup time
    spi_write_read_blocking(spi, tx, rx, 2);
    sleep_us(1);                              // CS hold time
    gpio_put(cs, 1);

    return ((uint16_t)rx[0] << 8) | rx[1];
}

void AS5147P::init(spi_inst_t *spi, uint cs_pin) {
    spi_ = spi;
    cs_  = cs_pin;

    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);
}

// addr を読んで生の 16bit 値を返す (EF・パリティビット含む)
uint16_t AS5147P::read_reg(uint16_t addr) {
    // mode3 (CPOL=1, CPHA=1), 8-bit — SPI1共有時もSCK遷移不要
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    uint16_t cmd = make_cmd((1u << 14) | addr);
    uint16_t nop = make_cmd((1u << 14));  // read NOP (addr=0x0000)

    // Frame 1: コマンド送信 (レスポンスは破棄)
    spi_transfer16(spi_, cs_, cmd);

    busy_wait_us_32(1);  // t_CSn: CS HIGH 時間確保

    // Frame 2: NOP 送信、Frame1 へのレスポンス受信
    return spi_transfer16(spi_, cs_, nop);
}

void AS5147P::setup() {
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    // CS を HIGH のまま MISO を読む → 0x0000 なら MISO が L 固定
    uint8_t rx[2] = {0, 0};
    uint8_t tx[2] = {0xC0, 0x00};  // NOP コマンド (CS は上げたまま)
    spi_write_read_blocking(spi_, tx, rx, 2);
    uint16_t miso_rx = ((uint16_t)rx[0] << 8) | rx[1];

    // CS を LOW にした直後の MISO を SPI クロックで読む
    gpio_put(cs_, 0);
    sleep_us(1);
    uint8_t rx2[2] = {0, 0}, tx2[2] = {0xC0, 0x00};
    spi_write_read_blocking(spi_, tx2, rx2, 2);
    sleep_us(1);
    gpio_put(cs_, 1);
    uint16_t miso_cs_l = ((uint16_t)rx2[0] << 8) | rx2[1];

    uint16_t errfl  = read_reg(0x0001);
    uint16_t diaagc = read_reg(0x3FFC);
    uint16_t angle  = read_reg(0x3FFF);

    printf("[enc CS=%u] MISO(CS=H)=0x%04X  MISO(CS=L)=0x%04X\n"
           "           ERRFL=0x%04X(EF=%d err=%d%d%d)"
           "  DIAAGC=0x%04X(LF=%d AGC=%d)  angle=0x%04X(%d)\n",
           cs_,
           miso_rx, miso_cs_l,
           errfl,  (errfl >> 14) & 1, (errfl >> 2) & 1, (errfl >> 1) & 1, errfl & 1,
           diaagc, (diaagc >> 8) & 1, diaagc & 0xFF,
           angle,  angle & 0x3FFF);
}

uint16_t AS5147P::read_angle() {
    uint16_t raw = read_reg(0x3FFF);
    return raw & 0x3FFFu;
}
