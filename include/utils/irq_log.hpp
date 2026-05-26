#pragma once
#include <atomic>
#include <cstdint>

// SPSC lockfree ring buffer: Core1 IRQ (producer) → Core0 main (consumer)
//
// push() は IRQ 内から安全に呼べる（ノンブロッキング、バッファ満杯なら無視）
// drain() は Core0 のメインループから呼ぶ
struct IrqLog {
    static constexpr uint32_t SLOTS     = 32;   // バッファ段数（2の累乗）
    static constexpr uint32_t SLOT_SIZE = 48;   // 1メッセージの最大文字数（終端含む）
    static constexpr uint32_t MASK      = SLOTS - 1;

    char              buf[SLOTS][SLOT_SIZE];
    std::atomic<uint32_t> head{0};  // producer が管理 (Core1 IRQ)
    std::atomic<uint32_t> tail{0};  // consumer が管理 (Core0)

    // Core1 IRQ から呼ぶ。満杯なら無視（ブロックしない）
    void push(const char *s) noexcept {
        uint32_t h = head.load(std::memory_order_relaxed);
        if (h - tail.load(std::memory_order_acquire) >= SLOTS) return;
        char *dst = buf[h & MASK];
        uint32_t i = 0;
        while (i < SLOT_SIZE - 1 && s[i]) { dst[i] = s[i]; ++i; }
        dst[i] = '\0';
        head.store(h + 1, std::memory_order_release);
    }

    // Core0 から呼ぶ。溜まったメッセージを cb(msg) で全て処理する
    template<typename F>
    void drain(F &&cb) noexcept {
        uint32_t t = tail.load(std::memory_order_relaxed);
        uint32_t h = head.load(std::memory_order_acquire);
        while (t != h) {
            cb(buf[t & MASK]);
            tail.store(t + 1, std::memory_order_release);
            ++t;
        }
    }
};

extern IrqLog g_irq_log;

// ctl_.calc() 内サブチェックポイント用: planning_task.cpp のコマンド受信ブロックで
// 6 にセットし、tick 先頭でデクリメント。control_law.cpp がこれを読んでログ制御する。
extern volatile uint32_t g_ctl_debug_ticks;
