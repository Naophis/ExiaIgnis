#pragma once
#include <stdint.h>
#include "hardware/spi.h"
#include "hardware/dma.h"

class AS5147P {
public:
    // SPI バスは外部で初期化済みであること
    // init() は CS ピン設定 + 専用DMAチャンネル(TX/RX)確保を行う
    void init(spi_inst_t *spi, uint cs_pin);

    // ERRFL / DIAAGC を読んで SPI 疎通・磁石状態を printf 表示
    void setup();

    // 14bit 角度値を取得 [0–16383]
    int32_t read_angle();

private:
    spi_inst_t *spi_ = nullptr;
    uint        cs_  = 0;

    // 16bitフレーム(8bit x2)を DMA で送受信する。CS制御とフレーム間の
    // t_CSn待ち(sleep_us)はソフトウェアのまま維持し、SPIのバイトシフト
    // 部分だけをCPUポーリング(spi_write_read_blocking)からDMAへ置き換える
    // (gyro/batteryと同じ考え方。sensing_task.cppのDMA実装を参照)。
    int dma_tx_ = -1;
    int dma_rx_ = -1;
    dma_channel_config dma_cfg_tx_{};
    dma_channel_config dma_cfg_rx_{};
    uint8_t dma_tx_buf_[2]{};
    uint8_t dma_rx_buf_[2]{};

    int32_t spi_transfer16(uint16_t tx_word);
    int32_t read_reg(uint16_t addr);
};
