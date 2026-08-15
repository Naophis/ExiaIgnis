#pragma once
#include <stdint.h>
#include <stddef.h>
#include "hardware/pio.h"

// AM32 ESC bootloader 生プロトコル(protocol A)の下位層。
//
// 出典: AlkaMotors/AM32-bootloader の bootloader/main.c
//   - 物理層: 19200bps, 8N1, non-inverted, idle-high, half-duplex 1-wire bit-bang UART
//     (serialreadChar()直前のコメント "read one byte from the input pin,
//      19200, not inverted, one stop bit" より)
//   - コマンドID / ACK値 / CRC16(poly 0xA001, init 0, LSB-first) / handshake
//     プローブbyte列は同ファイルの decodeInput()/crc16()/send_ACK() 系、および
//     am32-configurator の src/communication/direct.ts と突き合わせて確認済み。
//
// 「4-way interface」(0x2F/0x2E, CRC-XMODEM を使う BLHeliSuite 由来の別プロトコル)
// とは別物であることに注意。AM32ファームウェア/ブートローダー自体はそちらを実装
// していない(FC側のパススルー変換でのみ使われる)。ESCと直接1本の信号線で
// 対話するRP2350ホストが使うべきは、ここに実装したprotocol Aのみ。
//
// 実装方式としてPIOによる half-duplex UART を選択した理由:
//   - baud=19200(1bit=52us)自体はCPUに対して十分低速だが、ESC側の
//     serialreadChar()はフレーム内バイト間ギャップを 5*BITTIME(=260us)
//     以内に制限しており、host(RP2350)側がCPU busy-waitでバイト送出すると
//     USB(TinyUSB)割り込み等のジッタでこの制約を破りやすい。
//     PIOのTX FIFO(4語)にバイトを積んでおけば、SMがCPUのスケジューリングと
//     無関係に一定間隔で送出し続けるため、この制約を確実に守れる。
//   - 受信側も同様に、ESCからのACK/データ待ちをPIO SMのハードウェアタイミングで
//     サンプリングすることで、CPU側のポーリング遅延によるビット化けを避けられる。
//   - 19200bpsという低速さと1本の信号線という制約から、素のUARTペリフェラル
//     (全二重・TX/RX別ピン前提)は半二重1線には非対応であり不採用。
class Am32Protocol {
public:
    // 名前は define.hpp のUART用マクロ BAUD_RATE(115200) との衝突を避けるため AM32_ 接頭辞にしている。
    static constexpr uint32_t AM32_BAUD_RATE = 19200;         // 出典: bootloader/main.c #define BAUDRATE
    static constexpr uint32_t BIT_TIME_US = 52;                // 1,000,000 / 19200 (bootloader #define BITTIME)

    // ESC側 checkForSignal() が application へ jump するか bootloader に留まるか
    // 判定し終えるまでの最大所要時間の見積り(500us+40ms+500us+5ms+500us+5ms)。
    // この間 host はラインを駆動してはならない(idle-high入力のまま待機する)。
    static constexpr uint32_t POWERUP_SILENCE_US = 60000;

    // ESC側 serialreadChar() の「フレーム先頭start-bit待ち」ハードタイムアウトは20ms
    // (これを超えると invalid_command が加算され bootloader が application へ jump する)。
    // host側はこれより早く送る前提だが、応答待ちにも同程度のマージンを見る。
    static constexpr uint32_t HANDSHAKE_TIMEOUT_US = 15000;

    static constexpr uint32_t ACK_TIMEOUT_US  = 8000;   // 通常コマンドのACK/応答待ちタイムアウト
    static constexpr uint32_t BYTE_TIMEOUT_US = 4000;   // 複数byte応答中、1byteごとのタイムアウト

    // ACK/NACK値 (出典: bootloader/main.c send_ACK()/send_BAD_ACK()/send_BAD_CRC_ACK())
    static constexpr uint8_t ACK_OK      = 0x30;
    static constexpr uint8_t ACK_BAD_CMD = 0xC1;
    static constexpr uint8_t ACK_BAD_CRC = 0xC2;

    // コマンドID (出典: bootloader/main.c #define CMD_*)
    static constexpr uint8_t CMD_RUN            = 0x00;
    static constexpr uint8_t CMD_PROG_FLASH     = 0x01;
    static constexpr uint8_t CMD_ERASE_FLASH    = 0x02;
    static constexpr uint8_t CMD_READ_FLASH_SIL = 0x03;
    static constexpr uint8_t CMD_SET_ADDRESS    = 0xFF;
    static constexpr uint8_t CMD_SET_BUFFER     = 0xFE;

    // アドレスmagic値 (出典: 同ファイル ADDRESS_MAGIC_*, BOOTLOADER_PROTOCOL_VERSION>=2で有効)
    static constexpr uint16_t ADDRESS_MAGIC_EEPROM    = 0x20;
    static constexpr uint16_t ADDRESS_MAGIC_FILE_NAME = 0x21;
    static constexpr uint16_t ADDRESS_MAGIC_CONTINUE  = 0x22;

    static constexpr size_t MAX_PAYLOAD = 256;

    enum class Status : uint8_t {
        OK = 0,
        TIMEOUT,          // ACK/データ待ちタイムアウト(ESC未接続/電源OFF/切断を含む)
        BAD_CRC,          // ESCが0xC2を返した、または受信データのCRC不一致
        BAD_ACK,          // ESCが0xC1、または想定外の値を返した
        NOT_INITIALIZED,
        LENGTH_ERROR,     // payload長が1〜256の範囲外
    };

    // pio: 専有するPIOブロック(am32_tx/am32_rxの2プログラムをロードし、SMを2つ使用)
    // gpio: ESCのS信号線につながるGPIO番号
    // 戻り値false: SM確保失敗など致命的な初期化エラー
    bool init(PIO pio, uint gpio);
    void deinit();
    bool is_initialized() const { return initialized_; }

    // 電源投入/リセット直後、ESCのcheckForSignal()完了を待つための静音区間。
    // 呼び出し中はラインをinit()時のidle-high入力のまま維持する(何も送信しない)。
    void wait_powerup_silence() const;

    // handshake: 固定21byteプローブを送信し、9byte devinfoを受信する。
    // プローブbyte列は am32-configurator の direct.ts init配列と同一(出典: 調査レポート参照)。
    // devinfo_out[9] = {'4','7','1', PIN_CODE, FLASH_SIZE_CODE, 0x06, 0x06, PROTOCOL_VER, 0x30}
    Status handshake(uint8_t devinfo_out[9], uint32_t timeout_us = HANDSHAKE_TIMEOUT_US);

    Status set_address(uint16_t addr);
    Status write_buffer(const uint8_t* payload, uint16_t len);   // SET_BUFFER + payload(+CRC) 送信
    Status prog_flash();                                          // 直前のset_address/write_bufferの内容をflashへ書き込む
    Status erase_flash();  // 出典: bootloader側ハンドラはaddress妥当性検証のみでflash消去自体は行わない(no-op)。
                             // page-alignedなwriteは自動消去されるため通常は不要(API対称性のために提供)。
    Status read_flash(uint8_t* out, uint16_t len);                // 直前のset_addressの位置からlen byte読み出す
    Status cmd_run();                                             // ESCをbootloaderからapplicationへjumpさせる(応答なし)

    uint16_t crc16(const uint8_t* data, size_t len) const;

    static const char* status_to_string(Status s);

private:
    enum class BusMode : uint8_t { NONE, TX, RX };

    void switch_to_tx();
    void switch_to_rx();
    Status tx_bytes(const uint8_t* data, size_t len);
    Status rx_bytes(uint8_t* out, size_t len, uint32_t timeout_us);
    Status rx_ack(uint32_t timeout_us = ACK_TIMEOUT_US);

    PIO  pio_  = nullptr;
    uint gpio_ = 0;
    uint sm_tx_ = 0;
    uint sm_rx_ = 0;
    uint off_tx_ = 0;
    uint off_rx_ = 0;
    bool initialized_ = false;
    BusMode mode_ = BusMode::NONE;
};
