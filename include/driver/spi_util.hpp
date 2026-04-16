#pragma once
#include "hardware/spi.h"
#include "hardware/structs/spi.h"

// PL022 SPI コントローラは CPOL/CPHA/DSS を変更する際に
// SSE (Synchronous Serial Port Enable) を 0 にする必要がある。
// Pico SDK の spi_set_format() はこれを行わないため、
// 複数デバイスで SPI バスを共有しモードを切り替える場合に使用すること。
static inline void spi_set_format_safe(spi_inst_t *spi,
                                       uint data_bits,
                                       spi_cpol_t cpol,
                                       spi_cpha_t cpha,
                                       spi_order_t order) {
    hw_clear_bits(&spi_get_hw(spi)->cr1, SPI_SSPCR1_SSE_BITS);  // SSE OFF
    spi_set_format(spi, data_bits, cpol, cpha, order);
    hw_set_bits(&spi_get_hw(spi)->cr1, SPI_SSPCR1_SSE_BITS);    // SSE ON
}
