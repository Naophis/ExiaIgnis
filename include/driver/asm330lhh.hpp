#pragma once
#include <stdint.h>
#include "hardware/spi.h"

// ASM330LHH レジスタアドレス
#define ASM330_WHO_AM_I   0x0F
#define ASM330_CTRL2_G    0x11
#define ASM330_CTRL3_C    0x12
#define ASM330_CTRL4_C    0x13
#define ASM330_CTRL7_G    0x16
#define ASM330_CTRL8_XL   0x17
#define ASM330_OUTZ_L_G   0x26

class ASM330LHH {
public:
    // init() で SPI バスの初期化からピン設定まで行う
    void init(spi_inst_t *spi, uint miso_pin, uint cs_pin, uint clk_pin, uint mosi_pin,
              uint baud_rate = 10'000'000);

    // レジスタ設定シーケンス (init() 完了後に呼ぶ)
    void setup();

    // ジャイロ Z 軸角速度を取得 [raw]
    int16_t read_gyro_z();

private:
    spi_inst_t *spi_  = nullptr;
    uint        cs_   = 0;

    void    write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
};
