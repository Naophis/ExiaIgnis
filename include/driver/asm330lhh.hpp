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
#define ASM330_OUTX_L_XL  0x28
#define ASM330_OUTY_L_XL  0x2A
#define ASM330_OUTZ_L_XL  0x2C

#define ASM330LHH_CTRL1_XL 0x10U
#define ASM330LHH_CTRL6_C 0x15U
#define ASM330LHH_CTRL9_XL 0x18U
// 実ODR(とタイムスタンプ周期)の公称値からのずれ。工場校正値・読み出し専用、
// int8、1LSB=0.15%。ODR_actual = (6667 + 0.0015*FREQ_FINE*6667) / ODRcoeff
// (ODRcoeff は 3333Hz 設定で 2)。ST AN5296 / datasheet の INTERNAL_FREQ_FINE。
#define ASM330LHH_INTERNAL_FREQ_FINE 0x63U

class ASM330LHH {
public:
    // init() で SPI バスの初期化からピン設定まで行う
    void init(spi_inst_t *spi, uint miso_pin, uint cs_pin, uint clk_pin, uint mosi_pin,
              uint baud_rate = 10'000'000);

    // レジスタ設定シーケンス (init() 完了後に呼ぶ)
    void setup();

    // ジャイロ Z 軸角速度を取得 [raw]
    int16_t read_gyro_z();

    // この個体の実 ODR (setup() で INTERNAL_FREQ_FINE から求める)。内部クロックの
    // 製造ばらつきでチップごとに公称値からずれる(±19% まで、本機は +5.25%)ので、
    // サンプル数×周期で積分する処理(FIFO 等)は公称値でなくこちらを使うこと。
    int8_t freq_fine() const { return freq_fine_; }
    float  gyro_odr_hz() const { return gyro_odr_hz_; }
    float  accel_odr_hz() const { return accel_odr_hz_; }
    float  gyro_sample_period_us() const {
        return gyro_odr_hz_ > 0.0f ? 1e6f / gyro_odr_hz_ : 0.0f;
    }
    // odr_code は CTRL1_XL / CTRL2_G の上位 4bit (1=12.5Hz … 9=3333Hz, 10=6667Hz)。
    // 0(power-down) と範囲外は 0 を返す。
    static float actual_odr_hz(uint8_t odr_code, int8_t freq_fine);

private:
    spi_inst_t *spi_  = nullptr;
    uint        cs_   = 0;
    int8_t      freq_fine_    = 0;
    float       gyro_odr_hz_  = 0.0f;
    float       accel_odr_hz_ = 0.0f;

    void    write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
};
