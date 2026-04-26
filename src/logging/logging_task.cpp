#include "logging/logging_task.hpp"
#include "define.hpp"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include <stdio.h>

// ============================================================
// psram_heap: バンプアロケータ実装
// ============================================================
namespace psram_heap {
    static uint8_t* g_ptr = nullptr;
    static uint8_t* g_end = nullptr;

    void init(void* base, size_t size) {
        g_ptr = static_cast<uint8_t*>(base);
        g_end = g_ptr + size;
    }

    void* alloc(size_t bytes, size_t align) {
        uintptr_t p  = (reinterpret_cast<uintptr_t>(g_ptr) + align - 1) & ~(align - 1);
        uint8_t*  np = reinterpret_cast<uint8_t*>(p) + bytes;
        if (np > g_end) return nullptr;
        g_ptr = np;
        return reinterpret_cast<void*>(p);
    }

    size_t available() {
        return (g_ptr < g_end) ? static_cast<size_t>(g_end - g_ptr) : 0;
    }
}

// ============================================================
// LoggingTask
// ============================================================
std::shared_ptr<LoggingTask> LoggingTask::s_instance;

std::shared_ptr<LoggingTask> LoggingTask::create() {
    s_instance = std::shared_ptr<LoggingTask>(new LoggingTask());
    return s_instance;
}

void LoggingTask::init(void* psram_base, size_t psram_size, size_t max_entries) {
    psram_heap::init(psram_base, psram_size);
    log_cap_ = max_entries;
    log_vec_.reserve(max_entries);  // PSRAM に一括確保 (以降 realloc なし)
    printf("[LoggingTask] PSRAM %zu KB, cap=%zu entries (%zu KB), entry=%zu B\n",
           psram_size / 1024,
           max_entries,
           (max_entries * sizeof(LogEntry)) / 1024,
           sizeof(LogEntry));
}

void LoggingTask::start() {
    log_vec_.clear();   // size=0 に戻す (capacity・PSRAM 確保は維持)
    __dmb();
    active_ = true;
    printf("[LoggingTask] start\n");
}

void LoggingTask::stop() {
    active_ = false;
    __dmb();            // Core1 の最後の push_back が Core0 から見えるようにバリア
    printf("[LoggingTask] stop: %zu entries\n", log_vec_.size());
}

// ============================================================
// Core1 (planning IRQ) から呼ぶ高速パス
// ============================================================
void LoggingTask::append_from_irq(const SensingTask::Data&  d,
                                   const PlanningTask::State& ps)
{
    auto* self = s_instance.get();
    if (!self || !self->active_) return;

    if (self->log_vec_.size() >= self->log_cap_) {
        self->active_ = false;  // 満杯: 自動停止
        return;
    }

    LogEntry e;
    e.timestamp_us = static_cast<uint32_t>(d.gz_ts);
    e.dt_us        = d.dt_us;

    e.l90          = d.diff.l90;
    e.l45_1        = d.diff.l45_1;
    e.l45_both     = d.diff.l45_both;
    e.l45_2        = d.diff.l45_2;
    e.r45_1        = d.diff.r45_1;
    e.r45_both     = d.diff.r45_both;
    e.r45_2        = d.diff.r45_2;
    e.r90          = d.diff.r90;

    e.gz           = d.gz;
    e.enc_l        = d.enc_l;
    e.enc_r        = d.enc_r;
    e.battery      = d.battery;

    e.img_v        = ps.img_v;
    e.img_w        = ps.img_w;
    e.img_dist     = ps.img_dist;
    e.img_ang      = ps.img_ang;
    e.v_est        = ps.v_est;
    e.w_est        = ps.w_est;
    e.duty_l       = ps.duty_l;
    e.duty_r       = ps.duty_r;
    e.duty_suction = ps.duty_suction;
    e.mode         = static_cast<uint8_t>(ps.mode);
    e._pad[0] = e._pad[1] = e._pad[2] = 0;

    self->log_vec_.push_back(e);
}

// ============================================================
// USB CDC に CSV 出力 (Core0 から stop() 後に呼ぶ)
// ============================================================
void LoggingTask::dump_csv() const {
    const size_t n = log_vec_.size();
    printf("ready___:%zu\n", sizeof(LogEntry));
    sleep_ms(50);

    printf("index,timestamp_us,dt_us,"
           "l90,l45_1,l45_both,l45_2,r45_1,r45_both,r45_2,r90,"
           "gz,enc_l,enc_r,battery,"
           "img_v,img_w,img_dist,img_ang,"
           "v_est,w_est,duty_l,duty_r,duty_suction,mode\n");

    int flush_cnt = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& e = log_vec_[i];
        printf("%zu,%lu,%lu,"
               "%u,%u,%u,%u,%u,%u,%u,%u,"
               "%d,%u,%u,%u,"
               "%.3f,%.4f,%.2f,%.4f,"
               "%.3f,%.4f,%.2f,%.2f,%.2f,%u\n",
               i,
               static_cast<unsigned long>(e.timestamp_us),
               static_cast<unsigned long>(e.dt_us),
               e.l90, e.l45_1, e.l45_both, e.l45_2,
               e.r45_1, e.r45_both, e.r45_2, e.r90,
               static_cast<int>(e.gz), e.enc_l, e.enc_r, e.battery,
               e.img_v, e.img_w, e.img_dist, e.img_ang,
               e.v_est, e.w_est, e.duty_l, e.duty_r, e.duty_suction,
               static_cast<unsigned>(e.mode));

        // USB CDC バッファあふれ防止: 50行ごとに 1ms 待つ
        if (++flush_cnt >= 50) {
            flush_cnt = 0;
            sleep_ms(1);
        }
    }
    printf("end___\n");
}

// ============================================================
// UART にバイナリ出力 (Core0 から stop() 後に呼ぶ)
// プロトコル: [magic:4][count:4][entry_sz:4][data:count*entry_sz][magic:4]
// ※ uart_init() は呼び出し元で実施すること
// ============================================================
void LoggingTask::dump_binary() const {
    uart_inst_t* u = UART_ID;  // define.hpp: uart1

    const uint32_t magic    = 0x4C4F4700u;  // "LOG\0"
    const uint32_t count    = static_cast<uint32_t>(log_vec_.size());
    const uint32_t entry_sz = static_cast<uint32_t>(sizeof(LogEntry));

    uart_write_blocking(u, reinterpret_cast<const uint8_t*>(&magic),    4);
    uart_write_blocking(u, reinterpret_cast<const uint8_t*>(&count),    4);
    uart_write_blocking(u, reinterpret_cast<const uint8_t*>(&entry_sz), 4);
    uart_write_blocking(u, reinterpret_cast<const uint8_t*>(log_vec_.data()),
                        log_vec_.size() * sizeof(LogEntry));
    uart_write_blocking(u, reinterpret_cast<const uint8_t*>(&magic),    4);

    printf("[LoggingTask] binary dump done: %zu entries\n", log_vec_.size());
}
