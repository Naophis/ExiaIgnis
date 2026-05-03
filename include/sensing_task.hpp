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


class SensingTask {
public:
    // 全LED消灯時のADC値 (ambient)
    struct SensorDark {
        uint16_t r90, r45, l45, l90;
    };

    // 各LEDパターン点灯時のADC値
    struct SensorLit {
        uint16_t r90;
        uint16_t r45_1;     // R45_LED1 のみ
        uint16_t r45_both;  // R45_LED1 + R45_LED2
        uint16_t r45_2;     // R45_LED2 のみ
        uint16_t l45_1;     // L45_LED1 のみ
        uint16_t l45_both;  // L45_LED1 + L45_LED2
        uint16_t l45_2;     // L45_LED2 のみ
        uint16_t l90;
    };

    // lit - dark の差分 (負になる場合は 0)
    struct SensorDiff {
        uint16_t r90;
        uint16_t r45_1, r45_both, r45_2;
        uint16_t l45_1, l45_both, l45_2;
        uint16_t l90;
    };

    struct Data {
        uint32_t   dt_us;           // 前回割り込みからの経過時間 [us]
        uint32_t   sense_duration_us; // sensing シーケンス全体の処理時間 [us]
        SensorDark dark;
        SensorLit  lit;
        SensorDiff diff;
        int16_t    gz;         // ジャイロZ軸角速度
        uint64_t   gz_ts;      // gz 取得時刻 [us]
        uint64_t   gz_ts_z;      // gz 取得時刻_z [us]
        uint64_t   gz_dt;      // gz diff [us]
        
        uint16_t   enc_r;
        uint64_t   enc_r_ts;   // enc_r 取得時刻 [us]
        uint64_t   enc_r_ts_z;   // enc_r 取得時刻_z [us]
        uint64_t   enc_r_dt;   // enc_r diff [us]
        
        uint16_t   enc_l;
        uint64_t   enc_l_ts;   // enc_l 取得時刻 [us]
        uint64_t   enc_l_ts_z;   // enc_l 取得時刻_z [us]
        uint64_t   enc_l_dt;   // enc_l diff [us]
        uint16_t   battery;

        Data() = default;
        Data(const volatile Data &o)
            : dt_us(o.dt_us)
            , sense_duration_us(o.sense_duration_us)
            , dark{o.dark.r90, o.dark.r45, o.dark.l45, o.dark.l90}
            , lit {o.lit.r90,
                   o.lit.r45_1, o.lit.r45_both, o.lit.r45_2,
                   o.lit.l45_1, o.lit.l45_both, o.lit.l45_2,
                   o.lit.l90}
            , diff{o.diff.r90,
                   o.diff.r45_1, o.diff.r45_both, o.diff.r45_2,
                   o.diff.l45_1, o.diff.l45_both, o.diff.l45_2,
                   o.diff.l90}
            , gz(o.gz)
            , gz_ts(o.gz_ts), gz_ts_z(o.gz_ts_z), gz_dt(o.gz_dt)
            , enc_r(o.enc_r)
            , enc_r_ts(o.enc_r_ts), enc_r_ts_z(o.enc_r_ts_z), enc_r_dt(o.enc_r_dt)
            , enc_l(o.enc_l)
            , enc_l_ts(o.enc_l_ts), enc_l_ts_z(o.enc_l_ts_z), enc_l_dt(o.enc_l_dt)
            , battery(o.battery) {}
    };

    static std::shared_ptr<SensingTask> create();

    void init();

    // Core0 から呼ぶ: TIMER0 IRQ を登録してタイマー起動 (ブロックしない)
    void start_irq();

    // 旧 Core1 エントリ (後方互換)
    static void core1_entry();

    // multicore_launch_core1 より前に呼ぶこと
    void configure(uint32_t led_settle_us, uint32_t interval_us);

    volatile Data data{};
    volatile bool data_ready = false;

    void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
    void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
    void set_planning_task(std::shared_ptr<PlanningTask> &_pt);
private:
    SensingTask() = default;

    void run();
    void read_spi_sensors();

    std::shared_ptr<input_param_t> param;
    ASM330LHH gyro_;
    AS5147P   enc_r_;   // 右エンコーダ (SPI1, CS=GPIO9)
    AS5147P   enc_l_;   // 左エンコーダ (SPI0, CS=GPIO1)
    ADS7042   battery_; // バッテリADC  (SPI0, CS=GPIO5)
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

    static void timer_a_irq_handler();  // alarm 2: 周期タイマー
    static void timer_b_irq_handler();  // alarm 1: センサー読み取りシーケンス
    void calc_vel(float gyro_dt, float enc_l_dt, float enc_r_dt);

    uint32_t led_settle_us_ = 13;    // LED 点灯後の安定待ち時間
    uint32_t interval_us_   = 1000;  // サンプリング周期

    uint32_t next_alarm_a_ = 0;
    uint64_t prev_timestamp_ = 0;

    static std::shared_ptr<SensingTask> s_instance;
    std::shared_ptr<sensing_result_entity_t> sensing_result;
    std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
    std::shared_ptr<motion_tgt_val_t> tgt_val;
    std::shared_ptr<PlanningTask> pt;
    float calc_enc_v(float now, float old, float dt);
};
