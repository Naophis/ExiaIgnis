#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "driver/asm330lhh.hpp"
#include "driver/spi_util.hpp"

void ASM330LHH::init(spi_inst_t *spi, uint miso_pin, uint cs_pin, uint clk_pin, uint mosi_pin,
                     uint baud_rate) {
    spi_ = spi;
    cs_  = cs_pin;

    spi_init(spi_, baud_rate);
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);  // mode 3
    gpio_set_function(clk_pin,  GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);
    sleep_ms(10);
}

void ASM330LHH::setup() {
    uint8_t who = read_reg(ASM330_WHO_AM_I);
    printf("ASM330LHH WHO_AM_I: 0x%02X (%s)\n", who, who == 0x6B ? "OK" : "NG");

    write_reg(ASM330_CTRL3_C, 0x83);  // ソフトウェアリセット
    sleep_ms(200);
    while ((read_reg(ASM330_CTRL3_C) & 0x01) == 0x01) {}  // リセット完了待ち

    write_reg(ASM330_CTRL3_C, 0x44);  // BDU 有効
    sleep_ms(25);
    write_reg(ASM330_CTRL4_C, 0x04);  // I2C/I3C 無効
    sleep_ms(25);
    write_reg(ASM330_CTRL7_G, 0x00);  // ジャイロ HPF OFF
    sleep_ms(25);
    write_reg(ASM330_CTRL2_G, 0x91);  // ODR: 3333Hz, FS: ±4000dps
    sleep_ms(25);
    write_reg(ASM330_CTRL8_XL, 0x00); // XL LPF2 OFF
    sleep_ms(25);
}

int16_t ASM330LHH::read_gyro_z() {
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    uint8_t cmd = ASM330_OUTZ_L_G | 0x80;
    uint8_t buf[2];
    gpio_put(cs_, 0);
    spi_write_blocking(spi_, &cmd, 1);
    spi_read_blocking(spi_, 0x00, buf, 2);
    gpio_put(cs_, 1);
    return static_cast<int16_t>(buf[1] << 8 | buf[0]);
}

void ASM330LHH::write_reg(uint8_t reg, uint8_t val) {
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    uint8_t buf[2] = {static_cast<uint8_t>(reg & 0x7F), val};
    gpio_put(cs_, 0);
    spi_write_blocking(spi_, buf, 2);
    gpio_put(cs_, 1);
}

uint8_t ASM330LHH::read_reg(uint8_t reg) {
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    uint8_t cmd = reg | 0x80;
    uint8_t val = 0;
    gpio_put(cs_, 0);
    spi_write_blocking(spi_, &cmd, 1);
    spi_read_blocking(spi_, 0x00, &val, 1);
    gpio_put(cs_, 1);
    return val;
}
