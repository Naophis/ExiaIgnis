#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SPI_PORT spi0

// 好きなピンに合わせて変更
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#define AS5147P_REG_NOP      0x0000
#define AS5147P_REG_ERRFL    0x0001
#define AS5147P_REG_DIAAGC   0x3FFC
#define AS5147P_REG_MAG      0x3FFD
#define AS5147P_REG_ANGLEUNC 0x3FFE
#define AS5147P_REG_ANGLECOM 0x3FFF

static inline uint8_t even_parity_16(uint16_t x) {
    // 下位15bitに対して偶数パリティになるようにbit15へ入れる用途で使う
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1u;  // 1なら下位ビット群に1が奇数個
}

static inline uint16_t as5147p_make_read_cmd(uint16_t addr14) {
    uint16_t cmd = 0;
    cmd |= (1u << 14);          // R/W = 1 (read)
    cmd |= (addr14 & 0x3FFFu);  // 14bit address

    // bit15 = PARC (下位15bit全体が偶数個の1になるように設定)
    if (even_parity_16(cmd)) {
        cmd |= (1u << 15);
    }
    return cmd;
}

static uint16_t spi_transfer16(uint16_t tx_word) {
    uint8_t tx[2] = {
        (uint8_t)(tx_word >> 8),
        (uint8_t)(tx_word & 0xFF)
    };
    uint8_t rx[2] = {0, 0};

    gpio_put(PIN_CS, 0);
    // データシートではCSn立下りからCLK立上りまで最低350ns必要
    sleep_us(1);
    spi_write_read_blocking(SPI_PORT, tx, rx, 2);
    sleep_us(1);
    gpio_put(PIN_CS, 1);

    return ((uint16_t)rx[0] << 8) | rx[1];
}

static bool as5147p_check_data_parity(uint16_t frame) {
    // 読出しデータフレームも bit15 を含めて偶数パリティ
    return even_parity_16(frame) == 0;
}

static uint16_t as5147p_read_reg_raw(uint16_t addr14, bool *ef, bool *parity_ok) {
    uint16_t cmd = as5147p_make_read_cmd(addr14);

    // 1回目: 読みたいレジスタを指定
    spi_transfer16(cmd);

    // 2回目: NOPを送って前回指定したレジスタ内容を受け取る
    uint16_t rx = spi_transfer16(as5147p_make_read_cmd(AS5147P_REG_NOP));

    if (ef) {
        *ef = (rx >> 14) & 0x1u;   // EF bit
    }
    if (parity_ok) {
        *parity_ok = as5147p_check_data_parity(rx);
    }

    return rx & 0x3FFFu; // DATA[13:0]
}

static uint16_t as5147p_read_errfl(bool *ef, bool *parity_ok) {
    return as5147p_read_reg_raw(AS5147P_REG_ERRFL, ef, parity_ok);
}

static uint16_t as5147p_read_angle_raw(bool *ef, bool *parity_ok) {
    return as5147p_read_reg_raw(AS5147P_REG_ANGLECOM, ef, parity_ok);
}

static float as5147p_raw_to_deg(uint16_t raw14) {
    return (360.0f * (float)(raw14 & 0x3FFFu)) / 16384.0f;
}

static void as5147p_init(void) {
    spi_init(SPI_PORT, 1000 * 1000);  // まずは1MHzで安全に
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // AS5147Pは SPI mode 1
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    as5147p_init();

    while (true) {
        bool ef = false;
        bool parity_ok = false;

        uint16_t angle_raw = as5147p_read_angle_raw(&ef, &parity_ok);
        float angle_deg = as5147p_raw_to_deg(angle_raw);

        printf("ANGLECOM raw=%5u  deg=%8.3f  EF=%d  parity=%s\n",
               angle_raw, angle_deg, ef, parity_ok ? "OK" : "NG");

        if (ef) {
            bool err_ef = false;
            bool err_parity_ok = false;
            uint16_t errfl = as5147p_read_errfl(&err_ef, &err_parity_ok);
            printf("ERRFL=0x%04x (PARERR=%d INVCOMM=%d FRERR=%d)\n",
                   errfl,
                   (errfl >> 2) & 1,
                   (errfl >> 1) & 1,
                   (errfl >> 0) & 1);
        }

        sleep_ms(10);
    }
}