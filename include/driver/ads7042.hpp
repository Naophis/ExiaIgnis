#pragma once
#include <stdint.h>
#include "hardware/spi.h"

// ADS7042I — 12-bit, single-channel SPI ADC
// SPI0 をバッテリ電圧測定に使用 (SPI バスは外部で初期化済みであること)
class ADS7042 {
public:
    void     init(spi_inst_t *spi, uint cs_pin);

    // 12-bit ADC 値を取得 [0–4095]
    uint16_t read();

private:
    spi_inst_t *spi_ = nullptr;
    uint        cs_  = 0;
};
