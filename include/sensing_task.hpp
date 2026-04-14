#pragma once
#include <stdint.h>
#include <memory>
#include "pico/time.h"

class SensingTask {
public:
    struct Data {
        uint32_t dt_us;      // 前回割り込みからの経過時間 [us]
        uint16_t adc0, adc1;
        int16_t  gx, gy, gz;
        uint16_t enc0, enc1;
    };

    // ファクトリ: インスタンスを生成して shared_ptr を返す
    // 内部に保持するため core1_entry からも参照できる
    static std::shared_ptr<SensingTask> create();

    void init();

    // multicore_launch_core1 に渡す静的エントリ
    static void core1_entry();

    volatile Data data{};
    volatile bool data_ready = false;

private:
    SensingTask() = default;

    void run();

    void     asm330_write(uint8_t reg, uint8_t val);
    uint8_t  asm330_read_reg(uint8_t reg);
    void     asm330_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);
    uint16_t as5147p_read_angle(uint gpio_cs);

    // 直接 IRQ ハンドラ (alarm_pool を介さない)
    static void timer_a_irq_handler();  // alarm 2: 250us 周期
    static void timer_b_irq_handler();  // alarm 1: LED settle 後のセンサー読み取り

    uint32_t next_alarm_a_ = 0;   // alarm 2 の次回ターゲット時刻 (32bit)
    uint64_t prev_timestamp_ = 0;

    // core1_entry が参照するための所有権付き保持
    static std::shared_ptr<SensingTask> s_instance;
};
