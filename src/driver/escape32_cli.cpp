#include "driver/escape32_cli.hpp"
#include "am32_halfduplex.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

namespace {
// am32_rx は shift_right で 8bit を ISR に入れて push するので、受信バイトは RX FIFO
// ワードの最上位バイト(am32_protocol.cpp と同じ取り出し方)。
inline uint8_t rx_byte(PIO pio, uint sm) {
  io_rw_8 *p = (io_rw_8 *)&pio->rxf[sm] + 3;
  return (uint8_t)*p;
}
}  // namespace

bool Escape32Cli::init(PIO pio, uint gpio) {
  if (initialized_) return true;
  pio_  = pio;
  gpio_ = gpio;
  if (!pio_can_add_program(pio_, &am32_tx_program) ||
      !pio_can_add_program(pio_, &am32_rx_program)) {
    printf("[esc_cli] no PIO instruction memory left\n");
    return false;
  }
  off_tx_ = pio_add_program(pio_, &am32_tx_program);
  off_rx_ = pio_add_program(pio_, &am32_rx_program);
  const int tx = pio_claim_unused_sm(pio_, false);
  const int rx = tx < 0 ? -1 : pio_claim_unused_sm(pio_, false);
  if (tx < 0 || rx < 0) {
    if (tx >= 0) pio_sm_unclaim(pio_, (uint)tx);
    pio_remove_program(pio_, &am32_tx_program, off_tx_);
    pio_remove_program(pio_, &am32_rx_program, off_rx_);
    printf("[esc_cli] no free PIO state machine\n");
    return false;
  }
  sm_tx_ = (uint)tx;
  sm_rx_ = (uint)rx;

  pio_gpio_init(pio_, gpio_);  // ピンを DShot の PIO から奪う(DShot 側の PIO/DMA は回り続けてよい)
  gpio_pull_up(gpio_);         // 受信中(Hi-Z)は idle-high。ESC 側にも PA2 の pull-up がある
  gpio_set_input_enabled(gpio_, true);

  const float clkdiv = (float)clock_get_hz(clk_sys) / (8.0f * (float)kBaud);
  am32_tx_program_init(pio_, sm_tx_, off_tx_, gpio_, clkdiv);
  am32_rx_program_init(pio_, sm_rx_, off_rx_, gpio_, clkdiv);
  pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, false);
  pio_sm_set_consecutive_pindirs(pio_, sm_rx_, gpio_, 1, false);
  pio_sm_set_enabled(pio_, sm_tx_, false);
  pio_sm_set_enabled(pio_, sm_rx_, false);

  initialized_ = true;
  mode_ = Bus::NONE;
  switch_to_tx();  // idle-high を駆動しておく: ESC が通電後 1 秒 High を見て CLI に入る条件
  return true;
}

void Escape32Cli::deinit() {
  if (!initialized_) return;
  pio_sm_set_enabled(pio_, sm_tx_, false);
  pio_sm_set_enabled(pio_, sm_rx_, false);
  pio_sm_unclaim(pio_, sm_tx_);
  pio_sm_unclaim(pio_, sm_rx_);
  pio_remove_program(pio_, &am32_tx_program, off_tx_);
  pio_remove_program(pio_, &am32_rx_program, off_rx_);
  // ピンは SIO 入力・プル無しで返す。呼び出し側が DShot 側へ付け直す(reattach_pin())。
  gpio_set_function(gpio_, GPIO_FUNC_SIO);
  gpio_set_dir(gpio_, GPIO_IN);
  gpio_disable_pulls(gpio_);
  initialized_ = false;
  mode_ = Bus::NONE;
}

void Escape32Cli::switch_to_tx() {
  if (mode_ == Bus::TX) return;
  pio_sm_set_enabled(pio_, sm_rx_, false);
  pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, true);
  pio_sm_set_enabled(pio_, sm_tx_, true);  // pull で待つ間 side-set が High を出す
  mode_ = Bus::TX;
}

void Escape32Cli::switch_to_rx() {
  if (mode_ == Bus::RX) return;
  pio_sm_set_enabled(pio_, sm_tx_, false);
  pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, false);  // 駆動を放す(Hi-Z + pull-up)
  pio_sm_set_consecutive_pindirs(pio_, sm_rx_, gpio_, 1, false);
  pio_sm_set_enabled(pio_, sm_rx_, true);
  mode_ = Bus::RX;
}

bool Escape32Cli::tx_bytes(const char *data, size_t len) {
  switch_to_tx();
  for (size_t i = 0; i < len; i++) {
    const uint64_t deadline = time_us_64() + 5000;
    while (pio_sm_is_tx_fifo_full(pio_, sm_tx_)) {
      if (time_us_64() > deadline) return false;
    }
    pio_sm_put(pio_, sm_tx_, (uint8_t)data[i]);
  }
  // FIFO が空になり、シフト中の最終バイト(start + 8bit + stop)が出終わるまで待つ。
  const uint64_t deadline = time_us_64() + (uint64_t)len * 10 * kBitTimeUs + 5000;
  while (!pio_sm_is_tx_fifo_empty(pio_, sm_tx_)) {
    if (time_us_64() > deadline) return false;
  }
  busy_wait_us(10 * kBitTimeUs);
  return true;
}

int Escape32Cli::command(const char *line, char *reply, size_t reply_size,
                         uint32_t timeout_ms) {
  if (!initialized_) return -1;
  if (reply && reply_size) reply[0] = '\0';

  char buf[256];
  const int n = snprintf(buf, sizeof buf, "%s\n", line);
  if (n <= 0 || (size_t)n >= sizeof buf) return -1;

  // 前回の残りを捨ててから送る。送り終えたらすぐ受信(Hi-Z)へ: ESC は '\n' を受けた
  // 直後に応答を出し始めるので、こちらが High を駆動したままだと衝突する。
  switch_to_rx();
  while (!pio_sm_is_rx_fifo_empty(pio_, sm_rx_)) (void)rx_byte(pio_, sm_rx_);
  if (!tx_bytes(buf, (size_t)n)) {
    switch_to_rx();
    return -1;
  }
  switch_to_rx();

  char   cur[320];
  size_t cur_len = 0, out_len = 0;
  const uint64_t deadline = time_us_64() + (uint64_t)timeout_ms * 1000;
  while (time_us_64() < deadline) {
    if (pio_sm_is_rx_fifo_empty(pio_, sm_rx_)) {
      tight_loop_contents();
      continue;
    }
    const char c = (char)rx_byte(pio_, sm_rx_);
    if (c == '\r') continue;
    if (c != '\n') {
      if (cur_len < sizeof cur - 1) cur[cur_len++] = c;
      continue;
    }
    cur[cur_len] = '\0';
    if (strcmp(cur, "OK") == 0) return 1;
    if (strcmp(cur, "ERROR") == 0) return 0;
    if (reply && out_len + cur_len + 1 < reply_size) {
      memcpy(reply + out_len, cur, cur_len);
      out_len += cur_len;
      reply[out_len++] = '\n';
      reply[out_len]   = '\0';
    }
    cur_len = 0;
  }
  return -1;
}
