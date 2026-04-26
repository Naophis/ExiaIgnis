#pragma once
#include <memory>
#include <vector>
#include <cstdint>
#include <cstddef>
#include "sensing_task.hpp"
#include "planning/planning_task.hpp"

// ============================================================
// PSRAM バンプアロケータ
// RP2350 QMI M1 にマップされた PSRAM 領域を管理する。
// dealloc は no-op: 確保した領域は LoggingTask の寿命全体で保持。
// ※ QMI M1 の初期化 (PSRAM チップ依存) は呼び出し元が行うこと。
// ============================================================
namespace psram_heap {
    void   init(void* base, size_t size);
    void*  alloc(size_t bytes, size_t align = alignof(std::max_align_t));
    size_t available();
}

template<typename T>
struct PsramAllocator {
    using value_type = T;

    T* allocate(std::size_t n) {
        void* p = psram_heap::alloc(n * sizeof(T), alignof(T));
        if (!p) panic("PSRAM OOM");
        return static_cast<T*>(p);
    }
    void deallocate(T*, std::size_t) {}

    template<typename U> struct rebind { using other = PsramAllocator<U>; };
    template<typename U> PsramAllocator(const PsramAllocator<U>&) noexcept {}
    PsramAllocator() noexcept = default;
    bool operator==(const PsramAllocator&) const noexcept { return true; }
    bool operator!=(const PsramAllocator&) const noexcept { return false; }
};

// ============================================================
// ログ 1 サンプル: sensing + planning の主要値
// ============================================================
struct LogEntry {
    // タイミング
    uint32_t timestamp_us;          // gz_ts 下位 32bit [us]
    uint32_t dt_us;                 // 前回 sensing からの経過時間 [us]

    // センサー差分 (SensorDiff: lit - dark)
    uint16_t l90, l45_1, l45_both, l45_2;
    uint16_t r45_1, r45_both, r45_2, r90;

    // モーション
    int16_t  gz;                    // ジャイロ Z [raw]
    uint16_t enc_l, enc_r;         // エンコーダ [0-16383]
    uint16_t battery;               // バッテリ ADC 値

    // Planning
    float    img_v,    img_w;       // 指令速度・角速度 [mm/s, rad/s]
    float    img_dist, img_ang;     // 累積距離・角度   [mm, rad]
    float    v_est,    w_est;       // 推定速度・角速度
    float    duty_l,   duty_r;      // モーター duty    [%]
    float    duty_suction;          // 吸引 duty        [%]
    uint8_t  mode;                  // MotionMode
    uint8_t  _pad[3];
};
static_assert(sizeof(LogEntry) % 4 == 0, "LogEntry must be 4-byte aligned");

// ============================================================
// LoggingTask
// Core0 (MainTask) から start/stop/dump を呼び、
// Core1 (planning IRQ) から append_from_irq() を呼ぶ。
// ============================================================
class LoggingTask {
public:
    static std::shared_ptr<LoggingTask> create();

    // Core0 の main() から呼ぶ (Core1 起動前)。
    // psram_base : PSRAM 先頭アドレス (例: 0x11000000)
    // psram_size : 利用可能バイト数   (例: 8 * 1024 * 1024)
    // max_entries: 最大サンプル数     (default: 60000 = 1kHz × 60s)
    void init(void* psram_base, size_t psram_size,
              size_t max_entries = 60000);

    // Core0 (MainTask) から呼ぶ
    void   start();
    void   stop();
    bool   is_logging() const { return active_; }
    size_t count()      const { return log_vec_.size(); }

    // Core1 (planning IRQ) から呼ぶ — active_ が false ならほぼ no-op
    static void append_from_irq(const SensingTask::Data&  d,
                                 const PlanningTask::State& ps);

    // Core0 (MainTask) から stop() 後に呼ぶ
    void dump_csv()    const;   // USB CDC に CSV 出力
    void dump_binary() const;   // UART にバイナリ出力

private:
    LoggingTask() = default;

    static std::shared_ptr<LoggingTask> s_instance;

    volatile bool active_  = false;
    size_t        log_cap_ = 0;

    using LogVec = std::vector<LogEntry, PsramAllocator<LogEntry>>;
    LogVec log_vec_;
};
