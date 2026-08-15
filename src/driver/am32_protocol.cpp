#include "driver/am32_protocol.hpp"
#include "am32_halfduplex.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

namespace {
// RX FIFOの32bit語のうち、上位8bit(MSB側)に受信byteが入る。
// (pico-examples pio/uart_rx と同じ取り出し方: shift_right=trueのISRを
//  8回shiftした後の値は最上位byteに収まる)
inline uint8_t rx_fifo_get_byte(PIO pio, uint sm) {
    io_rw_8* rxfifo_shift = (io_rw_8*)&pio->rxf[sm] + 3;
    return (uint8_t)*rxfifo_shift;
}
}  // namespace

bool Am32Protocol::init(PIO pio, uint gpio) {
    if (initialized_) {
        deinit();
    }
    pio_  = pio;
    gpio_ = gpio;

    const int add_tx = pio_add_program(pio_, &am32_tx_program);
    const int add_rx = pio_add_program(pio_, &am32_rx_program);
    printf("am32proto: pio_add_program tx=%d rx=%d\n", add_tx, add_rx);
    if (add_tx < 0 || add_rx < 0) {
        // PIO命令メモリ不足等。offsetが負のままpio_sm_init()へ渡ると不正なPCになるため
        // ここで確実に弾く(以前はこのチェックが無く、-1が巨大なuintへ暗黙変換されていた)。
        if (add_tx >= 0) {
            pio_remove_program(pio_, &am32_tx_program, (uint)add_tx);
        }
        if (add_rx >= 0) {
            pio_remove_program(pio_, &am32_rx_program, (uint)add_rx);
        }
        return false;
    }
    off_tx_ = (uint)add_tx;
    off_rx_ = (uint)add_rx;

    const int claim_tx = pio_claim_unused_sm(pio_, false);
    if (claim_tx < 0) {
        pio_remove_program(pio_, &am32_tx_program, off_tx_);
        pio_remove_program(pio_, &am32_rx_program, off_rx_);
        return false;
    }
    const int claim_rx = pio_claim_unused_sm(pio_, false);
    if (claim_rx < 0) {
        pio_sm_unclaim(pio_, (uint)claim_tx);
        pio_remove_program(pio_, &am32_tx_program, off_tx_);
        pio_remove_program(pio_, &am32_rx_program, off_rx_);
        return false;
    }
    sm_tx_ = (uint)claim_tx;
    sm_rx_ = (uint)claim_rx;
    printf("am32proto: claimed sm_tx=%u sm_rx=%u\n", sm_tx_, sm_rx_);

    pio_gpio_init(pio_, gpio_);
    gpio_pull_up(gpio_);  // idle-high。ESC bootloaderのsetReceive()と同じ想定(内部pull-up)。
    gpio_set_input_enabled(gpio_, true);

    const float clkdiv = (float)clock_get_hz(clk_sys) / (8.0f * (float)AM32_BAUD_RATE);
    am32_tx_program_init(pio_, sm_tx_, off_tx_, gpio_, clkdiv);
    am32_rx_program_init(pio_, sm_rx_, off_rx_, gpio_, clkdiv);

    // 両SMともpindir=inputで待機させる。以後の駆動権はswitch_to_tx()/switch_to_rx()が
    // 排他的に管理する(同一GPIOを2つのSMで共有する半二重構成のため)。
    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_rx_, gpio_, 1, false);
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_set_enabled(pio_, sm_rx_, false);

    initialized_ = true;
    mode_ = BusMode::NONE;
    switch_to_rx();  // 初期状態はidle-listening(ESCからの自発送信は無いが、安全な既定状態として)
    return true;
}

void Am32Protocol::deinit() {
    if (!initialized_) {
        return;
    }
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_set_enabled(pio_, sm_rx_, false);
    pio_sm_unclaim(pio_, sm_tx_);
    pio_sm_unclaim(pio_, sm_rx_);
    pio_remove_program(pio_, &am32_tx_program, off_tx_);
    pio_remove_program(pio_, &am32_rx_program, off_rx_);

    // GPIOを解放する。ESC電源OFF時に信号線経由で逆給電しないよう、
    // 呼び出し側がさらにHi-Zないし明示的なLow出力へ設定することを想定。
    gpio_set_function(gpio_, GPIO_FUNC_SIO);
    gpio_set_dir(gpio_, GPIO_IN);
    gpio_disable_pulls(gpio_);

    initialized_ = false;
    mode_ = BusMode::NONE;
}

void Am32Protocol::wait_powerup_silence() const {
    busy_wait_us(POWERUP_SILENCE_US);
}

void Am32Protocol::switch_to_tx() {
    if (mode_ == BusMode::TX) {
        return;
    }
    pio_sm_set_enabled(pio_, sm_rx_, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, true);
    pio_sm_set_enabled(pio_, sm_tx_, true);
    mode_ = BusMode::TX;
}

void Am32Protocol::switch_to_rx() {
    if (mode_ == BusMode::RX) {
        return;
    }
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, gpio_, 1, false);  // 駆動を解放(Hi-Z、pull-upでidle-high)
    pio_sm_set_consecutive_pindirs(pio_, sm_rx_, gpio_, 1, false);
    pio_sm_set_enabled(pio_, sm_rx_, true);
    mode_ = BusMode::RX;
}

Am32Protocol::Status Am32Protocol::enqueue_bytes(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        const uint64_t deadline = time_us_64() + 5000;
        while (pio_sm_is_tx_fifo_full(pio_, sm_tx_)) {
            if (time_us_64() > deadline) {
                return Status::TIMEOUT;
            }
        }
        pio_sm_put(pio_, sm_tx_, data[i]);
    }
    return Status::OK;
}

Am32Protocol::Status Am32Protocol::wait_tx_drain(size_t total_len_hint) {
    // TX FIFOおよびシフトレジスタが空になる(=最終byteの送出が完了する)まで待つ。
    const uint64_t drain_deadline = time_us_64() + (uint64_t)total_len_hint * 10 * BIT_TIME_US + 5000;
    while (!pio_sm_is_tx_fifo_empty(pio_, sm_tx_)) {
        if (time_us_64() > drain_deadline) {
            return Status::TIMEOUT;
        }
    }
    busy_wait_us(10 * BIT_TIME_US);  // 最終byteのstart+8bit+stopの送出完了を待つ
    return Status::OK;
}

Am32Protocol::Status Am32Protocol::tx_bytes(const uint8_t* data, size_t len) {
    Status st = enqueue_bytes(data, len);
    if (st != Status::OK) {
        return st;
    }
    return wait_tx_drain(len);
}

Am32Protocol::Status Am32Protocol::rx_bytes(uint8_t* out, size_t len, uint32_t timeout_us) {
    uint64_t deadline = time_us_64() + timeout_us;
    for (size_t i = 0; i < len; i++) {
        while (pio_sm_is_rx_fifo_empty(pio_, sm_rx_)) {
            if (time_us_64() > deadline) {
                return Status::TIMEOUT;
            }
        }
        out[i] = rx_fifo_get_byte(pio_, sm_rx_);
        deadline = time_us_64() + BYTE_TIMEOUT_US;  // 以降はbyte間タイムアウトへ切り替え
    }
    return Status::OK;
}

Am32Protocol::Status Am32Protocol::rx_ack(uint32_t timeout_us) {
    uint8_t ack = 0;
    Status st = rx_bytes(&ack, 1, timeout_us);
    if (st != Status::OK) {
        return st;
    }
    if (ack == ACK_OK) {
        return Status::OK;
    }
    if (ack == ACK_BAD_CRC) {
        return Status::BAD_CRC;
    }
    return Status::BAD_ACK;
}

uint16_t Am32Protocol::crc16(const uint8_t* data, size_t len) const {
    // 出典: bootloader/main.c crc16() を1:1で移植(poly 0xA001, init 0, LSB-first)
    uint16_t ret = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t xb = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (((xb & 0x01) ^ (ret & 0x0001)) != 0) {
                ret >>= 1;
                ret ^= 0xA001;
            } else {
                ret >>= 1;
            }
            xb = (uint8_t)(xb >> 1);
        }
    }
    return ret;
}

Am32Protocol::Status Am32Protocol::handshake(uint8_t devinfo_out[9], uint32_t timeout_us) {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    // 出典: am32-configurator/src/communication/direct.ts の `init` byte配列そのもの。
    // bootloader側 decodeInput() は rxBuffer[20]==0x7D かつ rxBuffer[12..13]=='\r','B' を
    // handshakeとして検出しsendDeviceInfo()を返す。
    static const uint8_t probe[21] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0x0D, 'B', 'L', 'H', 'e', 'l', 'i',
        0xF4, 0x7D,
    };
    switch_to_tx();
    Status st = tx_bytes(probe, sizeof(probe));
    if (st != Status::OK) {
        return st;
    }
    switch_to_rx();
    return rx_bytes(devinfo_out, 9, timeout_us);
}

Am32Protocol::Status Am32Protocol::set_address(uint16_t addr) {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    uint8_t frame[6];
    frame[0] = CMD_SET_ADDRESS;
    frame[1] = 0x00;
    frame[2] = (uint8_t)(addr >> 8);
    frame[3] = (uint8_t)(addr & 0xFF);
    const uint16_t crc = crc16(frame, 4);
    frame[4] = (uint8_t)(crc & 0xFF);
    frame[5] = (uint8_t)(crc >> 8);

    switch_to_tx();
    Status st = tx_bytes(frame, sizeof(frame));
    if (st != Status::OK) {
        return st;
    }
    switch_to_rx();
    return rx_ack();
}

Am32Protocol::Status Am32Protocol::write_buffer(const uint8_t* payload, uint16_t len) {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    if (len == 0 || len > MAX_PAYLOAD) {
        return Status::LENGTH_ERROR;
    }
    const uint8_t size_flag = (len == 256) ? 0x01 : 0x00;
    const uint8_t size_byte = (len == 256) ? 0x00 : (uint8_t)len;

    uint8_t frame[6];
    frame[0] = CMD_SET_BUFFER;
    frame[1] = 0x00;
    frame[2] = size_flag;
    frame[3] = size_byte;
    const uint16_t crc = crc16(frame, 4);
    frame[4] = (uint8_t)(crc & 0xFF);
    frame[5] = (uint8_t)(crc >> 8);

    const uint16_t pcrc = crc16(payload, len);
    const uint8_t crcbuf[2] = {(uint8_t)(pcrc & 0xFF), (uint8_t)(pcrc >> 8)};

    // 出典: bootloader/main.c decodeInput() — SET_BUFFERコマンド自体にACKは無く、
    // 直後に生payload(+CRC16 2byte)が続く前提でincoming_payload_no_commandへ遷移する。
    // ESC側はcommand+payload+crcを1つの連続フレームとして受信するため、
    // (bootloader/main.c serialreadChar()のbyte間タイムアウト5*BITTIME=260us)、
    // 3つのsegmentをtx_bytes()で個別に送るとsegment間に生じるsettle待ち(520us)が
    // 260usを超えてフレームが分断されてしまう。そのためenqueue_bytes()で継ぎ目
    // なく積み、最後に一度だけwait_tx_drain()する。
    switch_to_tx();
    Status st = enqueue_bytes(frame, sizeof(frame));
    if (st != Status::OK) {
        return st;
    }
    st = enqueue_bytes(payload, len);
    if (st != Status::OK) {
        return st;
    }
    st = enqueue_bytes(crcbuf, 2);
    if (st != Status::OK) {
        return st;
    }
    st = wait_tx_drain(sizeof(frame) + len + 2);
    if (st != Status::OK) {
        return st;
    }
    switch_to_rx();
    return rx_ack();
}

Am32Protocol::Status Am32Protocol::prog_flash() {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    uint8_t frame[4];
    frame[0] = CMD_PROG_FLASH;
    frame[1] = 0x00;
    const uint16_t crc = crc16(frame, 2);
    frame[2] = (uint8_t)(crc & 0xFF);
    frame[3] = (uint8_t)(crc >> 8);

    switch_to_tx();
    Status st = tx_bytes(frame, sizeof(frame));
    if (st != Status::OK) {
        return st;
    }
    switch_to_rx();
    return rx_ack();
}

Am32Protocol::Status Am32Protocol::erase_flash() {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    uint8_t frame[4];
    frame[0] = CMD_ERASE_FLASH;
    frame[1] = 0x00;
    const uint16_t crc = crc16(frame, 2);
    frame[2] = (uint8_t)(crc & 0xFF);
    frame[3] = (uint8_t)(crc >> 8);

    switch_to_tx();
    Status st = tx_bytes(frame, sizeof(frame));
    if (st != Status::OK) {
        return st;
    }
    switch_to_rx();
    return rx_ack();
}

Am32Protocol::Status Am32Protocol::read_flash(uint8_t* out, uint16_t len) {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    if (len == 0 || len > MAX_PAYLOAD) {
        return Status::LENGTH_ERROR;
    }
    const uint8_t size_byte = (len == 256) ? 0 : (uint8_t)len;

    uint8_t frame[4];
    frame[0] = CMD_READ_FLASH_SIL;
    frame[1] = size_byte;
    const uint16_t crc = crc16(frame, 2);
    frame[2] = (uint8_t)(crc & 0xFF);
    frame[3] = (uint8_t)(crc >> 8);

    switch_to_tx();
    Status st = tx_bytes(frame, sizeof(frame));
    if (st != Status::OK) {
        return st;
    }
    switch_to_rx();

    st = rx_bytes(out, len, ACK_TIMEOUT_US);
    if (st != Status::OK) {
        return st;
    }
    uint8_t crcbuf[2];
    st = rx_bytes(crcbuf, 2, BYTE_TIMEOUT_US);
    if (st != Status::OK) {
        return st;
    }
    uint8_t ack = 0;
    st = rx_bytes(&ack, 1, BYTE_TIMEOUT_US);
    if (st != Status::OK) {
        return st;
    }
    if (ack != ACK_OK) {
        return Status::BAD_ACK;
    }
    const uint16_t recv_crc = (uint16_t)(crcbuf[0] | (crcbuf[1] << 8));
    if (recv_crc != crc16(out, len)) {
        return Status::BAD_CRC;
    }
    return Status::OK;
}

Am32Protocol::Status Am32Protocol::cmd_run() {
    if (!initialized_) {
        return Status::NOT_INITIALIZED;
    }
    // 出典: bootloader/main.c — CMD_RUNはinvalid_command=101を設定し、outer loopが
    // 即座にjump()する。ESCは応答を返さない。
    const uint8_t frame[4] = {CMD_RUN, 0, 0, 0};
    switch_to_tx();
    Status st = tx_bytes(frame, sizeof(frame));
    switch_to_rx();  // バスを解放し、通常動作(PWM/DShot)へ戻す準備状態にする
    return st;
}

const char* Am32Protocol::status_to_string(Status s) {
    switch (s) {
        case Status::OK: return "OK";
        case Status::TIMEOUT: return "TIMEOUT";
        case Status::BAD_CRC: return "BAD_CRC";
        case Status::BAD_ACK: return "BAD_ACK";
        case Status::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case Status::LENGTH_ERROR: return "LENGTH_ERROR";
    }
    return "UNKNOWN";
}
