#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "driver/as5147p.hpp"
#include "driver/spi_util.hpp"

void AS5147P::init(spi_inst_t *spi, uint cs_pin) {
    spi_ = spi;
    cs_  = cs_pin;

    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);
}

uint16_t AS5147P::read_angle() {
    // AS5147P: mode 1 (CPOL=0, CPHA=1), 16-bit transfer
    // SPI1 (ジャイロと共有) / SPI0 (バッテリADCと共有) どちらにも対応
    spi_set_format_safe(spi_, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    uint16_t cmd, response;

    // 1回目: READ_ANGLE コマンド送信
    cmd = 0xFFFF;
    gpio_put(cs_, 0);
    spi_write16_read16_blocking(spi_, &cmd, &response, 1);
    gpio_put(cs_, 1);

    // 2回目: NOP で前回のデータを取得
    cmd = 0xC000;
    gpio_put(cs_, 0);
    spi_write16_read16_blocking(spi_, &cmd, &response, 1);
    gpio_put(cs_, 1);

    return response & 0x3FFF;
}
