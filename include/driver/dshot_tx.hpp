#pragma once
#include <stdint.h>
#include "hardware/pio.h"

// 標準(非反転)DShot の送信専用ドライバ。PIO 1本 + DMA 1本で、
// 「最後に指定したフレームを一定周期で送り続ける」自律出力を作る。
//
// [なぜ自律出力にするか]
// DShotはハードウェアPWMのような自律出力を持たないプロトコルで、フレームを
// 送り続けないとESC側がフェイルセーフ(停止)に入る。一方で吸引ESCは
//   - main()の冒頭(Core1起動やConfigLoader::init()より前)から
//   - Core1のplanning IRQが止まっている間(flash_safe_execute()中など)も
// 有効な信号が出続けていることを期待する(旧サーボPWM実装では
// ハードウェアPWMスライスがそれを担っていた。src/main.cpp冒頭の
// esc_.init()のコメント参照)。
// そこで「DMAが固定アドレス(frame_word_)を無限に読み続けてPIOのTX FIFOへ
// 流し込む」構成にし、CPUはそのアドレスを1回32bitストアするだけにした。
// CPUが止まってもPIOとDMAが同じフレームを送り続けるので、サーボPWM時代と
// 同じ「常時出力」セマンティクスが保たれる。
//
// [フレーム周期]
// DMAの転送はDMAペーシングタイマーのDREQで律速する(PIOのTX DREQではなく)。
// PIOは1フレーム(16bit)を約27us(DShot600)で吐き出し、その後 pull block で
// idle-lowのまま次フレームを待つため、フレーム周期 = タイマー周期になる。
// TX FIFO(深さ4)は常にほぼ空で、フルによる取りこぼしは起こらない
// (PIOの消費27us << タイマー周期400us)。
//
// [ESCape32との対応]
// ESCape32はidleレベルから極性を、ビット時間からレート(DShot300/600/1200)を
// 自動判別するため、ESC側の設定変更は不要(input_mode=0のまま。
// 1フレーム内に十分な数の立ち上がりエッジがあればロックする)。
// 非反転と判定された場合ESCは信号線を駆動しないので、こちらが常時駆動しても
// 出力衝突は起きない(pio/dshot_tx_std.pio のコメント参照)。
class DshotTx {
public:
    // pio        : 専有するPIOブロック(SM 1本、9命令を使用)
    // gpio       : ESCのS信号線につながるGPIO
    // bitrate_hz : DShotビットレート(DShot600なら600000)
    // frame_hz   : フレーム送出周期[Hz](ESCape32のフレーム境界判定に十分な
    //              idleギャップが空く範囲で選ぶ。2500Hz推奨)
    bool init(PIO pio, uint gpio, uint32_t bitrate_hz = 600000,
              uint32_t frame_hz = 2500);
    void deinit();
    bool is_initialized() const { return initialized_; }

    // 送出するフレームを差し替える(次のフレームから反映)。
    // 32bit単一ストアなのでCore1のIRQから呼んでも破れない
    // (DMAが読むのも32bit単位)。反映までの遅れは最大1フレーム周期。
    void set_frame(uint16_t frame) {
        // PIOはOSRをleft shiftで消費するため、フレームを上位16bitへ詰める。
        frame_word_ = ((uint32_t)frame) << 16;
    }
    uint16_t frame() const { return (uint16_t)(frame_word_ >> 16); }

    // 他の機能(AM32設定通信など)がGPIOを奪った後、信号線をこのPIOへ戻す。
    // PIO/DMAは動き続けているので、GPIOのファンクション再割り当てだけでよい。
    void attach_pin();

    uint32_t frame_hz() const { return frame_hz_actual_; }

private:
    volatile uint32_t frame_word_ = 0;  // DMAが無限に読み続ける実体(SRAM上)

    PIO  pio_   = nullptr;
    uint gpio_  = 0;
    uint sm_    = 0;
    uint off_   = 0;
    int  dma_ch_    = -1;
    int  dma_timer_ = -1;
    uint32_t frame_hz_actual_ = 0;
    bool initialized_ = false;
};
