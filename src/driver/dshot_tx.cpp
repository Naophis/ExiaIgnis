#include "driver/dshot_tx.hpp"
#include "dshot_tx_std.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include <stdio.h>

namespace {
// 1ビットあたりのPIOサイクル数(pio/dshot_tx_std.pio の変調と一致させること)。
constexpr float kCyclesPerBit = 8.0f;

// DMAが無限に読み続けるアドレスはSRAMでなければならない。PSRAM(0x11xxxxxx)や
// flash(XIP)に置くとDMA読み出しがXIP経由になり、flash書き込み中(Core1停止中)に
// 停止しうる = 吸引ESCへの信号が途切れてフェイルセーフに入る。
// (dither_pwm.cpp の in_sram() と同じ検査)
bool in_sram(const void *p) {
    const uintptr_t x = (uintptr_t)p;
    return x >= 0x20000000u && x < 0x20082000u; // RAM 512k + SCRATCH
}
} // namespace

bool DshotTx::init(PIO pio, uint gpio, uint32_t bitrate_hz, uint32_t frame_hz) {
    if (initialized_) deinit();
    if (bitrate_hz == 0 || frame_hz == 0) return false;

    pio_  = pio;
    gpio_ = gpio;

    if (!in_sram((const void *)&frame_word_)) {
        printf("dshot_tx: frame_word_ is not in SRAM (%p) -- refusing to run\n",
               (const void *)&frame_word_);
        return false;
    }

    const int add = pio_add_program(pio_, &dshot_tx_std_program);
    if (add < 0) {
        printf("dshot_tx: pio_add_program failed\n");
        return false;
    }
    off_ = (uint)add;

    const int claim = pio_claim_unused_sm(pio_, false);
    if (claim < 0) {
        printf("dshot_tx: no free SM\n");
        pio_remove_program(pio_, &dshot_tx_std_program, off_);
        return false;
    }
    sm_ = (uint)claim;

    dma_ch_ = dma_claim_unused_channel(false);
    if (dma_ch_ < 0) {
        printf("dshot_tx: no free DMA channel\n");
        pio_sm_unclaim(pio_, sm_);
        pio_remove_program(pio_, &dshot_tx_std_program, off_);
        return false;
    }
    dma_timer_ = dma_claim_unused_timer(false);
    if (dma_timer_ < 0) {
        printf("dshot_tx: no free DMA pacing timer\n");
        dma_channel_unclaim((uint)dma_ch_);
        dma_ch_ = -1;
        pio_sm_unclaim(pio_, sm_);
        pio_remove_program(pio_, &dshot_tx_std_program, off_);
        return false;
    }

    // ---- PIO: idle-low で出力を確立してから SM を回す ----
    // pindirs を先に出力へ、pin の初期値を 0(idle-low)にしておく。
    // pio_gpio_init() でGPIOのファンクションをPIOへ向けた瞬間から
    // LOW が出力される(ESCが浮いた信号を見ないようにするため)。
    pio_sm_set_pins_with_mask(pio_, sm_, 0, 1u << gpio_);
    pio_sm_set_consecutive_pindirs(pio_, sm_, gpio_, 1, true);
    pio_gpio_init(pio_, gpio_);

    const float sys_clk = (float)clock_get_hz(clk_sys);
    const float clkdiv  = sys_clk / (kCyclesPerBit * (float)bitrate_hz);
    dshot_tx_std_program_init(pio_, sm_, off_, gpio_, clkdiv);

    // 起動直後から有効なフレームを送り続ける。frame_word_ の初期値 0 は
    // DSHOT_CMD_MOTOR_STOP そのもの(throttle=0, telemetry bit=0, CRC=0 →
    // dshot::build_command_frame(0,false,false)==0)であり、16bit全0でも
    // bit='0'は high 3/8 なので立ち上がりエッジは16回出る(=ESC側のレート
    // 自動判別・CRC検証をきちんと通る有効なフレームになる)。
    pio_sm_set_enabled(pio_, sm_, true);

    // ---- DMA: 固定アドレス(frame_word_) → PIO TX FIFO を無限に反復 ----
    // ペーシングタイマーの分数は clk_sys * num/den。num=1 固定で den を
    // clk_sys/frame_hz から求める(den は16bitなので上限でクランプする)。
    uint32_t den = (uint32_t)(clock_get_hz(clk_sys) / frame_hz);
    if (den > 0xFFFFu) den = 0xFFFFu;
    if (den < 1u) den = 1u;
    dma_timer_set_fraction((uint)dma_timer_, 1, (uint16_t)den);
    frame_hz_actual_ = clock_get_hz(clk_sys) / den;

    dma_channel_config c = dma_channel_get_default_config((uint)dma_ch_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);   // 常に frame_word_ を読む
    channel_config_set_write_increment(&c, false);  // 常に TX FIFO へ書く
    channel_config_set_dreq(&c, dma_get_timer_dreq((uint)dma_timer_));
    channel_config_set_irq_quiet(&c, true);
    channel_config_set_chain_to(&c, (uint)dma_ch_); // 自分自身 = chain なし
    dma_channel_configure((uint)dma_ch_, &c,
                          &pio_->txf[sm_],                   // write
                          &frame_word_,                      // read (固定アドレス)
                          dma_encode_endless_transfer_count(),
                          true);                             // 即開始

    initialized_ = true;
    printf("dshot_tx: gpio=%u pio_sm=%u dma_ch=%d timer=%d bitrate=%luHz "
           "frame=%luHz clkdiv=%.3f\n",
           (unsigned)gpio_, (unsigned)sm_, dma_ch_, dma_timer_,
           (unsigned long)bitrate_hz, (unsigned long)frame_hz_actual_,
           (double)clkdiv);
    return true;
}

void DshotTx::deinit() {
    if (!initialized_) return;
    dma_channel_abort((uint)dma_ch_);
    dma_channel_unclaim((uint)dma_ch_);
    dma_timer_unclaim((uint)dma_timer_);
    dma_ch_ = -1;
    dma_timer_ = -1;

    pio_sm_set_enabled(pio_, sm_, false);
    pio_sm_unclaim(pio_, sm_);
    pio_remove_program(pio_, &dshot_tx_std_program, off_);

    gpio_set_function(gpio_, GPIO_FUNC_SIO);
    gpio_set_dir(gpio_, GPIO_IN);

    initialized_ = false;
}

void DshotTx::attach_pin() {
    if (!initialized_) return;
    pio_gpio_init(pio_, gpio_);
}
