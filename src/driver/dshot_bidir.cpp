#include "driver/dshot_bidir.hpp"
#include "driver/dshot_gcr.hpp"
#include "dshot_bidir.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

namespace {
constexpr float kTxCyclesPerBit = 8.0f;
constexpr float kRxClkdivRatio  = 1.25f; // GCR 4bit->5bit展開分、テレメトリのbit periodはコマンド側の5/4倍
}

bool DshotBidir::init(PIO pio, uint gpio, uint32_t bitrate_hz) {
    if (initialized_) {
        deinit();
    }
    pio_  = pio;
    gpio_ = gpio;

    const int add_tx = pio_add_program(pio_, &dshot_tx_program);
    const int add_rx = pio_add_program(pio_, &dshot_rx_program);
    printf("dshot_bidir: pio_add_program tx=%d rx=%d\n", add_tx, add_rx);
    if (add_tx < 0 || add_rx < 0) {
        if (add_tx >= 0) pio_remove_program(pio_, &dshot_tx_program, (uint)add_tx);
        if (add_rx >= 0) pio_remove_program(pio_, &dshot_rx_program, (uint)add_rx);
        return false;
    }
    off_tx_ = (uint)add_tx;
    off_rx_ = (uint)add_rx;

    const int claim_tx = pio_claim_unused_sm(pio_, false);
    if (claim_tx < 0) {
        pio_remove_program(pio_, &dshot_tx_program, off_tx_);
        pio_remove_program(pio_, &dshot_rx_program, off_rx_);
        return false;
    }
    const int claim_rx = pio_claim_unused_sm(pio_, false);
    if (claim_rx < 0) {
        pio_sm_unclaim(pio_, (uint)claim_tx);
        pio_remove_program(pio_, &dshot_tx_program, off_tx_);
        pio_remove_program(pio_, &dshot_rx_program, off_rx_);
        return false;
    }
    sm_tx_ = (uint)claim_tx;
    sm_rx_ = (uint)claim_rx;
    printf("dshot_bidir: claimed sm_tx=%u sm_rx=%u\n", sm_tx_, sm_rx_);

    pio_gpio_init(pio_, gpio_);
    gpio_pull_up(gpio_); // idle-high(inverted DShot)。線が浮いてもESC非接続時に暴れないよう保険。
    gpio_set_input_enabled(gpio_, true);

    const float sys_clk = (float)clock_get_hz(clk_sys);
    const float tx_clkdiv = sys_clk / (kTxCyclesPerBit * (float)bitrate_hz);
    const float rx_clkdiv = tx_clkdiv * kRxClkdivRatio;

    dshot_tx_program_init(pio_, sm_tx_, off_tx_, gpio_, tx_clkdiv);
    dshot_rx_program_init(pio_, sm_rx_, off_rx_, gpio_, rx_clkdiv);

    tx_us_per_frame_ = 16.0f * kTxCyclesPerBit / (float)bitrate_hz * 1.0e6f;

    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_rx_, gpio_, 1, false);
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_set_enabled(pio_, sm_rx_, false);

    initialized_ = true;
    mode_ = BusMode::NONE;
    switch_to_rx();
    return true;
}

void DshotBidir::deinit() {
    if (!initialized_) return;
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_set_enabled(pio_, sm_rx_, false);
    pio_sm_unclaim(pio_, sm_tx_);
    pio_sm_unclaim(pio_, sm_rx_);
    pio_remove_program(pio_, &dshot_tx_program, off_tx_);
    pio_remove_program(pio_, &dshot_rx_program, off_rx_);

    gpio_set_function(gpio_, GPIO_FUNC_SIO);
    gpio_set_dir(gpio_, GPIO_IN);
    gpio_disable_pulls(gpio_);

    initialized_ = false;
    mode_ = BusMode::NONE;
}

void DshotBidir::switch_to_tx() {
    if (mode_ == BusMode::TX) return;
    pio_sm_set_enabled(pio_, sm_rx_, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, true);
    pio_sm_set_enabled(pio_, sm_tx_, true);
    mode_ = BusMode::TX;
}

void DshotBidir::switch_to_rx() {
    if (mode_ == BusMode::RX) return;
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_rx_, gpio_, 1, false);
    pio_sm_clear_fifos(pio_, sm_rx_);
    pio_sm_restart(pio_, sm_rx_);
    pio_sm_set_enabled(pio_, sm_rx_, true);
    mode_ = BusMode::RX;
}

bool DshotBidir::send_throttle(float throttle_pct, bool request_telemetry) {
    if (!initialized_) return false;

    const uint16_t val   = dshot::throttle_pct_to_value(throttle_pct);
    const uint16_t frame = dshot::build_command_frame(val, request_telemetry, /*bidirectional=*/true);

    switch_to_tx();
    // OSRは32bit、shift方向leftで上位16bitから消費されるため、frameを上位16bitへ詰める。
    pio_sm_put_blocking(pio_, sm_tx_, ((uint32_t)frame) << 16);

    // FIFO投入のみでは物理送出完了を保証しないため、フレーム送出に要する
    // 実時間分busy-waitしてから受信側へ切り替える(am32_protocol.cppの
    // wait_tx_drain()と同じ理由)。
    while (!pio_sm_is_tx_fifo_empty(pio_, sm_tx_)) {
        tight_loop_contents();
    }
    busy_wait_us((uint32_t)(tx_us_per_frame_ + 0.5f));

    switch_to_rx();
    return true;
}

bool DshotBidir::receive_telemetry(uint32_t *period_us_out, uint32_t timeout_us) {
    if (!initialized_) return false;
    if (mode_ != BusMode::RX) switch_to_rx();

    const uint64_t deadline = time_us_64() + timeout_us;
    while (pio_sm_is_rx_fifo_empty(pio_, sm_rx_)) {
        if (time_us_64() > deadline) {
            return false; // ESC未接続/未応答(bidirectional未対応ESC含む)
        }
    }
    const uint32_t raw = pio_sm_get(pio_, sm_rx_);
    // autopush閾値20bit、shift方向left: 20bit分がISRの下位20bitに収まった状態でpushされる。
    return dshot::decode_telemetry_gcr(raw & 0xFFFFFu, period_us_out);
}
