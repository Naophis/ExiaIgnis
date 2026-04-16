#pragma once
#include <stdint.h>
#include <memory>
#include "pico/time.h"
#include "driver/asm330lhh.hpp"
#include "driver/as5147p.hpp"
#include "driver/ads7042.hpp"

class SensingTask {
public:
    struct SensorData {
        uint16_t r90, r45, l45, l90;
    };

    struct Data {
        uint32_t   dt_us;     // 前回割り込みからの経過時間 [us]
        SensorData dark;      // 全LED消灯時のADC値
        SensorData lit;       // 各LED点灯時のADC値 (対応するLEDのみ点灯)
        int16_t    gz;        // ジャイロZ軸角速度
        uint16_t   enc_r, enc_l;
        uint16_t   battery;

        Data() = default;
        Data(const volatile Data &o)
            : dt_us(o.dt_us)
            , dark{o.dark.r90, o.dark.r45, o.dark.l45, o.dark.l90}
            , lit {o.lit.r90,  o.lit.r45,  o.lit.l45,  o.lit.l90}
            , gz(o.gz)
            , enc_r(o.enc_r), enc_l(o.enc_l)
            , battery(o.battery) {}
    };

    static std::shared_ptr<SensingTask> create();

    void init();

    static void core1_entry();

    volatile Data data{};
    volatile bool data_ready = false;

private:
    SensingTask() = default;

    void run();

    ASM330LHH gyro_;
    AS5147P   enc_r_;   // 右エンコーダ (SPI1, CS=GPIO9)
    AS5147P   enc_l_;   // 左エンコーダ (SPI0, CS=GPIO1)
    ADS7042   battery_; // バッテリADC  (SPI0, CS=GPIO5)

    static void timer_a_irq_handler();  // alarm 2: 1000us 周期 (1kHz)
    static void timer_b_irq_handler();  // alarm 1: LED settle 後のセンサー読み取り

    uint32_t next_alarm_a_ = 0;
    uint64_t prev_timestamp_ = 0;

    static std::shared_ptr<SensingTask> s_instance;
};
