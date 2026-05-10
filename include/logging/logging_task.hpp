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
    static void append_from_irq(const sensing_result_entity_t& sr,
                                 const motion_tgt_val_t& tv);

    // Core0 (MainTask) から stop() 後に呼ぶ
    void dump_csv()      const;  // USB CDC にバイナリ出力 (rx_term.js バイナリプロトコル)
    void dump_csv_text() const;  // USB CDC にテキスト CSV 出力 (rx_term.js テキストプロトコル)
    void dump_binary()   const;  // UART にバイナリ出力

    // Astraea MotionPlanning 互換インターフェース
    void start_slalom_log()              { start(); }
    void stop_slalom_log()               { stop();  }
    void save(const std::string &)       {}
    void dump_log(const std::string &)   { dump_csv(); }
    void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
    void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
    void set_planning_task(std::shared_ptr<PlanningTask> &_pt);

private:
    LoggingTask() = default;

    static std::shared_ptr<LoggingTask> s_instance;

    volatile bool active_  = false;
    size_t        log_cap_ = 0;

    using LogVec = std::vector<log_data_t2, PsramAllocator<log_data_t2>>;
    LogVec log_vec_;
    std::shared_ptr<sensing_result_entity_t> sensing_result;
    std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
    std::shared_ptr<input_param_t> param;
    std::shared_ptr<PlanningTask> pt;
};
