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

// FIFO (ジャイロのサンプルを取りこぼさず全部読むため、2026-09-29)。
// 1 ワード = TAG 1 バイト + X/Y/Z 各 2 バイトの 7 バイト。0x78 から連続読みすると
// 0x7E の次は 0x78 へ戻るので、N ワードを 1 回の転送で読める。
#define ASM330LHH_FIFO_CTRL3        0x09U  // [7:4] BDR_GY, [3:0] BDR_XL (コードは ODR と同じ、0=格納しない)
#define ASM330LHH_FIFO_CTRL4        0x0AU  // [2:0] FIFO_MODE (0=bypass, 6=continuous)
#define ASM330LHH_FIFO_STATUS1      0x3AU  // DIFF_FIFO[7:0] 未読ワード数
#define ASM330LHH_FIFO_STATUS2      0x3BU  // [6] FIFO_OVR_IA, [1:0] DIFF_FIFO[9:8]
#define ASM330LHH_FIFO_DATA_OUT_TAG 0x78U  // [7:3] TAG_SENSOR (0x01=ジャイロ)
#define ASM330LHH_FIFO_WORD_BYTES   7
#define ASM330LHH_FIFO_TAG_GYRO     0x01U
#define ASM330LHH_FIFO_MODE_BYPASS     0x00U
#define ASM330LHH_FIFO_MODE_CONTINUOUS 0x06U

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

    // FIFO を捨てて continuous モードで取り直す(溜まりすぎ・あふれ・タグ不一致時)。
    // Core1 の SensingTask から呼ぶ。SPI を mode 3 に切り替えてから書く。
    void fifo_flush();

private:
    spi_inst_t *spi_  = nullptr;
    uint        cs_   = 0;
    int8_t      freq_fine_    = 0;
    float       gyro_odr_hz_  = 0.0f;
    float       accel_odr_hz_ = 0.0f;

    void    write_reg(uint8_t reg, uint8_t val);
    uint8_t read_reg(uint8_t reg);
};
