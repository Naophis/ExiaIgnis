#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "driver/ads7042.hpp"

void ADS7042::init(spi_inst_t *spi, uint cs_pin) {
    spi_ = spi;
    cs_  = cs_pin;

    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);
}

uint16_t ADS7042::read() {
    // ADS7042I: mode 0 (CPOL=0, CPHA=0), 16-bit transfer
    // フレーム: [15:14]=00, [13:2]=12bit結果, [1:0]=00
    spi_set_format(spi_, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    uint16_t tx = 0x0000;
    uint16_t rx = 0;
    gpio_put(cs_, 0);
    spi_write16_read16_blocking(spi_, &tx, &rx, 1);
    gpio_put(cs_, 1);

    return (rx >> 2) & 0x0FFF;
}
