#pragma once
#include <stdint.h>
#include "defines.hpp"

#include <memory>
#include "pico/time.h"
#include "driver/asm330lhh.hpp"
#include "driver/as5147p.hpp"
#include "driver/ads7042.hpp"
#include "structs.hpp"
#include "planning/planning_task.hpp"
#include "hardware/dma.h"


class SensingTask {
public:
    static std::shared_ptr<SensingTask> create();

    void init();

    // Core0 から呼ぶ: TIMER0 IRQ を登録してタイマー起動 (ブロックしない)
    void start_irq();

    // 旧 Core1 エントリ (後方互換)
    static void core1_entry();

    // multicore_launch_core1 より前に呼ぶこと
    void configure(uint32_t led_settle_us, uint32_t interval_us);

    void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
    void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
    void set_planning_task(std::shared_ptr<PlanningTask> &_pt);
    void set_tgt_val(std::shared_ptr<motion_tgt_val_t> _tgt_val) { tgt_val = _tgt_val; }
    std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
private:
    SensingTask() = default;

    void run();
    void read_spi_sensors();

    std::shared_ptr<input_param_t> param;
    ASM330LHH gyro_;
    AS5147P   enc_r_;   // 右エンコーダ (SPI1, CS=GPIO17)
    AS5147P   enc_l_;   // 左エンコーダ (SPI0, CS=GPIO1)
    ADS7042   battery_; // バッテリADC  (SPI0, CS=GPIO5)

    // DMA channels: SPI1/SPI0 TX+RX for gyro/bat parallel transfer
    int dma_tx_spi1_ = -1;
    int dma_rx_spi1_ = -1;
    int dma_tx_spi0_ = -1;
    int dma_rx_spi0_ = -1;
    uint8_t  gyro_dma_tx_[3]{};
    uint8_t  gyro_dma_rx_[3]{};
    uint16_t bat_dma_tx_ = 0;
    uint16_t bat_dma_rx_ = 0;
    dma_channel_config dma_cfg_tx_spi1_{};
    dma_channel_config dma_cfg_rx_spi1_{};
    dma_channel_config dma_cfg_tx_spi0_{};
    dma_channel_config dma_cfg_rx_spi0_{};
    void init_dma();
    float w_old = 0;
    float vr_old = 0;
    float vl_old = 0;
    int64_t gyro_timestamp_old = 0;
    int64_t gyro_timestamp_now = 0;
    int64_t gyro2_timestamp_old = 0;
    int64_t gyro2_timestamp_now = 0;
    int64_t enc_r_timestamp_old = 0;
    int64_t enc_r_timestamp_now = 0;
    int64_t enc_l_timestamp_old = 0;
    int64_t enc_l_timestamp_now = 0;

    static void timer_b_irq_handler();  // alarm 1: 1kHz 定周期 + センサー読み取り
    void calc_vel(float gyro_dt, float enc_l_dt, float enc_r_dt);

    uint32_t led_settle_us_ = 18;    // LED 点灯後の安定待ち時間
    uint32_t interval_us_   = 1000;  // サンプリング周期

    uint32_t next_alarm_a_ = 0;
    uint64_t prev_timestamp_ = 0;
    uint64_t start_time_z = 0;

    static std::shared_ptr<SensingTask> s_instance;
    std::shared_ptr<sensing_result_entity_t> sensing_result;
    std::shared_ptr<motion_tgt_val_t> tgt_val;
    std::shared_ptr<PlanningTask> pt;
    float calc_enc_v(float now, float old, float dt);
};
