#include "driver/as5147p.hpp"
#include "driver/spi_util.hpp"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <cstdio>

// Even parity over bits 14:0
static bool calc_parity(uint16_t val) {
    val &= 0x7FFF;
    val ^= val >> 8;
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return val & 1;
}

static uint16_t make_cmd(uint16_t frame) {
    return calc_parity(frame) ? (frame | 0x8000u) : frame;
}

static bool _spiCalcEvenParity(uint16_t value) {
  bool even = 0;
  uint8_t i;
  for (i = 0; i < 16; i++) {
    if (value & (0x0001 << i)) {
      even = !even;
    }
  }
  return even;
}

void AS5147P::init(spi_inst_t *spi, uint cs_pin) {
    spi_ = spi;
    cs_  = cs_pin;

    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);

    dma_tx_ = dma_claim_unused_channel(true);
    dma_rx_ = dma_claim_unused_channel(true);

    dma_cfg_tx_ = dma_channel_get_default_config(dma_tx_);
    channel_config_set_transfer_data_size(&dma_cfg_tx_, DMA_SIZE_8);
    channel_config_set_dreq(&dma_cfg_tx_, spi_get_dreq(spi_, true));
    channel_config_set_read_increment(&dma_cfg_tx_, true);
    channel_config_set_write_increment(&dma_cfg_tx_, false);

    dma_cfg_rx_ = dma_channel_get_default_config(dma_rx_);
    channel_config_set_transfer_data_size(&dma_cfg_rx_, DMA_SIZE_8);
    channel_config_set_dreq(&dma_cfg_rx_, spi_get_dreq(spi_, false));
    channel_config_set_read_increment(&dma_cfg_rx_, false);
    channel_config_set_write_increment(&dma_cfg_rx_, true);
}

// 16bit ワードを 8bit×2 で DMA 送受信する。CS制御とt_CSn待ちは維持したまま、
// spi_write_read_blocking() のCPUポーリングをDMA転送に置き換える。
__attribute__((noinline, section(".time_critical.sensing.as5147p_read")))
int32_t AS5147P::spi_transfer16(uint16_t tx_word) {
    dma_tx_buf_[0] = (uint8_t)(tx_word >> 8);
    dma_tx_buf_[1] = (uint8_t)(tx_word & 0xFF);

    gpio_put(cs_, 0);
    sleep_us(2);
    dma_channel_configure(dma_tx_, &dma_cfg_tx_, &spi_get_hw(spi_)->dr,
                          dma_tx_buf_, 2, false);
    dma_channel_configure(dma_rx_, &dma_cfg_rx_, dma_rx_buf_,
                          &spi_get_hw(spi_)->dr, 2, false);
    dma_start_channel_mask((1u << dma_tx_) | (1u << dma_rx_));
    dma_channel_wait_for_finish_blocking(dma_rx_);
    sleep_us(1);
    gpio_put(cs_, 1);

    return ((int32_t)dma_rx_buf_[0] << 8) | dma_rx_buf_[1];
}

// addr を読んで生の 16bit 値を返す (EF・パリティビット含む)
__attribute__((noinline, section(".time_critical.sensing.as5147p_read_reg")))
int32_t AS5147P::read_reg(uint16_t addr) {
    spi_set_format_safe(spi_, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);  // mode 1

    uint16_t cmd = make_cmd((1u << 14) | addr);
    uint16_t nop = make_cmd(1u << 14);  // READ NOP (addr=0x0000, bit14=1=READ)

    // Frame 1: コマンド送信 (レスポンスは破棄)
    spi_transfer16(cmd);

    busy_wait_us_32(1);  // t_CSn: CS HIGH 時間確保

    // Frame 2: NOP 送信、Frame1 へのレスポンス受信
    return spi_transfer16(nop);
}

void AS5147P::setup() {
    spi_set_format_safe(spi_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    // CS を HIGH のまま MISO を読む → 0x0000 なら MISO が L 固定
    uint8_t rx[2] = {0, 0};
    uint8_t tx[2] = {0xC0, 0x00};  // NOP コマンド (CS は上げたまま)
    spi_write_read_blocking(spi_, tx, rx, 2);
    uint16_t miso_rx = ((uint16_t)rx[0] << 8) | rx[1];

    // CS を LOW にした直後の MISO を SPI クロックで読む
    gpio_put(cs_, 0);
    sleep_us(1);
    uint8_t rx2[2] = {0, 0}, tx2[2] = {0xC0, 0x00};
    spi_write_read_blocking(spi_, tx2, rx2, 2);
    sleep_us(1);
    gpio_put(cs_, 1);
    uint16_t miso_cs_l = ((uint16_t)rx2[0] << 8) | rx2[1];

    uint16_t errfl  = read_reg(0x0001);
    uint16_t diaagc = read_reg(0x3FFC);
    uint16_t angle  = read_reg(0x3FFF);

    printf("[enc CS=%u] MISO(CS=H)=0x%04X  MISO(CS=L)=0x%04X\n"
           "           ERRFL=0x%04X(EF=%d err=%d%d%d)"
           "  DIAAGC=0x%04X(LF=%d AGC=%d)  angle=0x%04X(%d)\n",
           cs_,
           miso_rx, miso_cs_l,
           errfl,  (errfl >> 14) & 1, (errfl >> 2) & 1, (errfl >> 1) & 1, errfl & 1,
           diaagc, (diaagc >> 8) & 1, diaagc & 0xFF,
           angle,  angle & 0x3FFF);
}

__attribute__((noinline, section(".time_critical.sensing.as5147p_read")))
int32_t AS5147P::read_angle() {
    for (int retry = 0; retry < 3; ++retry) {
        int32_t raw = read_reg(0x3FFF);
        // EF ビット (bit14) が立っていたらセンサーエラー → リトライ
        if (raw & (1 << 14)) continue;
        uint16_t w = (uint16_t)(raw & 0x7FFF);
        w ^= w >> 8; w ^= w >> 4; w ^= w >> 2; w ^= w >> 1;
        if ((w & 1u) == (uint16_t)((raw >> 15) & 1)) return raw & 0x3FFFu;
    }
    return -1;
}
