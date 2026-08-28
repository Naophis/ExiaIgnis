#pragma once
#include <stdint.h>
#include "hardware/pio.h"

// Bidirectional(inverted) DShot600 の下位層(PIO半二重ドライバ)。
// ESCape32移行に向けた実装で、現状どこからも呼ばれていない(未結線)。
// 実機での波形検証(ロジックアナライザ)前提。pio/dshot_bidir.pio の
// コメント、include/driver/dshot_gcr.hpp のコメントも参照。
//
// am32_protocol.hpp の Am32Protocol と同様、1本のGPIOをTX/RX 2つのSMで
// 半二重共有する設計(排他的に有効化を切り替える)。
class DshotBidir {
public:
    // pio: 専有するPIOブロック(dshot_tx/dshot_rxの2プログラムをロードし、SMを2つ使用)。
    // gpio: ESCのS信号線につながるGPIO番号。
    // bitrate_hz: コマンド側のDShotビットレート(DShot600なら600000)。
    bool init(PIO pio, uint gpio, uint32_t bitrate_hz = 600000);
    void deinit();
    bool is_initialized() const { return initialized_; }

    // throttle_pct(0..100)をコマンドフレームへ変換し送出する(FIFO投入のみ、
    // 送出完了までブロックする)。request_telemetry=trueで次にESCからの
    // テレメトリ応答を要求する(bidirectional運用では通常常時true)。
    bool send_throttle(float throttle_pct, bool request_telemetry = true);

    // send_throttle()直後に呼び、ESCからのテレメトリ応答(GCR 20bit)を
    // 受信・デコードする。timeout_us以内に応答が来なければfalse。
    // 成功時 period_us_out に基準周期[us]を書く(0=モーター停止/無効値)。
    bool receive_telemetry(uint32_t *period_us_out, uint32_t timeout_us = 200);

private:
    enum class BusMode : uint8_t { NONE, TX, RX };

    void switch_to_tx();
    void switch_to_rx();

    PIO  pio_  = nullptr;
    uint gpio_ = 0;
    uint sm_tx_ = 0;
    uint sm_rx_ = 0;
    uint off_tx_ = 0;
    uint off_rx_ = 0;
    bool initialized_ = false;
    BusMode mode_ = BusMode::NONE;

    // 1bit=8 PIOサイクルで送出するのに要する実時間[us](tx_program_init()で
    // 使ったclkdivから逆算)。frame送出完了待ちに使う。
    float tx_us_per_frame_ = 0.0f;
};
