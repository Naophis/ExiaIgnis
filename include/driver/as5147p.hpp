#pragma once
#include <stdint.h>
#include "hardware/spi.h"

class AS5147P {
public:
    // SPI バスは外部で初期化済みであること
    // init() は CS ピンのみ設定する
    void init(spi_inst_t *spi, uint cs_pin);

    // 14bit 角度値を取得 [0–16383]
    uint16_t read_angle();

private:
    spi_inst_t *spi_ = nullptr;
    uint        cs_  = 0;
};
