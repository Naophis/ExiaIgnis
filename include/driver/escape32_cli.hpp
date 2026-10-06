#pragma once
#include "hardware/pio.h"
#include <stddef.h>
#include <stdint.h>

// ESCape32 の信号線 CLI クライアント(1 線半二重 UART、38400bps 8N1、idle-high)。
//
// ESCape32(sample/ESCape32 のフォーク、ターゲット MOUSEG431)は、通電後に信号線が
// 約 1 秒 High のままでパルス(DShot / サーボ PWM)が来ないと入力待ちをやめて CLI に
// 入る(src/io.c entryirq → cliirq、USART2 半二重 38400)。これは ESCape32 標準の機能で、
// AM32 時代に信号線 1 本で設定を読み書きしていたのと同じ位置づけ。コマンドは 1 行
// ('\n' 終端)、応答はテキスト数行のあとに必ず "OK" か "ERROR" の行(src/prog.c execcmd)。
// 受信文字のエコーは無い。
//   info / show / get <key> / set <key> <value> / save / reset / throt <v> / beep
//
// 物理層は AM32 の設定通信と同じ PIO プログラム(pio/am32_halfduplex.pio、8 cycles/bit の
// 1 ピン半二重 UART)を 38400 で動かす。マウスは ESC の電源(SUCTION_POWER_EN)も握って
// いるので、「DShot からピンを奪って High に保つ → 通電 → 約 2 秒待つ → コマンド」で
// CLI に入れる(MainTask::set_suction_esc_config() / show_suction_esc_config())。使い終わった
// ら deinit() し、DShot 側へピンを返す(SuctionEscDshotActuator::reattach_pin())。
//
// 注意: ESC 側は受信のフレーミングエラーで WWDG リセットをわざと掛ける(ノイズ対策)。
// 送るのは ASCII だけ。先頭の 2 バイトが 0x00 0xFF だとブートローダーへ再起動するので送らない。
class Escape32Cli {
public:
  // pio: 占有する PIO(am32_tx / am32_rx を載せ、SM を 2 つ使う。DShot は pio1 なので pio0)。
  // gpio: ESC の信号線(SUCTION_ESC_PWM)。成功すると信号線は High に駆動された状態になる
  // (ESC が CLI に入る条件)。
  bool init(PIO pio, uint gpio);
  void deinit();
  bool is_initialized() const { return initialized_; }

  // 1 行送って "OK" / "ERROR" まで応答を集める。reply には終端行を除いた応答('\n' 区切り、
  // reply_size - 1 で打ち切り)。戻り値 1 = OK、0 = ERROR、-1 = timeout_ms 以内に終端が来ない
  // (CLI に入っていない、線が違う、など)。
  int command(const char *line, char *reply, size_t reply_size, uint32_t timeout_ms);

private:
  void switch_to_tx();
  void switch_to_rx();
  bool tx_bytes(const char *data, size_t len);

  static constexpr uint32_t kBaud      = 38400;
  static constexpr uint32_t kBitTimeUs = 1000000 / kBaud;  // 26us

  PIO  pio_ = nullptr;
  uint gpio_ = 0, sm_tx_ = 0, sm_rx_ = 0, off_tx_ = 0, off_rx_ = 0;
  bool initialized_ = false;
  enum class Bus : uint8_t { NONE, TX, RX };
  Bus mode_ = Bus::NONE;
};
