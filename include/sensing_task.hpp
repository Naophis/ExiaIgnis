#pragma once
#include <stdint.h>
#include "defines.hpp"

#include <memory>
#include "pico/time.h"
#include "driver/asm330lhh.hpp"
#include "driver/as5147p.hpp"
#include "driver/ads7042.hpp"
#include "structs.hpp"
#include "planning/planning_task.hpp"
#include "hardware/dma.h"


class SensingTask {
public:
    static std::shared_ptr<SensingTask> create();

    void init();

    // Core0 から呼ぶ: TIMER0 IRQ を登録してタイマー起動 (ブロックしない)
    void start_irq();

    // 旧 Core1 エントリ (後方互換)
    static void core1_entry();

    // multicore_launch_core1 より前に呼ぶこと
    void configure(uint32_t interval_us);

    void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
    void set_input_param_entity(std::shared_ptr<input_param_t> &_param);
    void set_planning_task(std::shared_ptr<PlanningTask> &_pt);
    void set_tgt_val(std::shared_ptr<motion_tgt_val_t> _tgt_val) { tgt_val = _tgt_val; }
    std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
private:
    SensingTask() = default;

    void run();
    // ============================================================
    // 1ms を 4 つの枠に分けて読む(2026-09-29)。枠の開始はアラーム 1 で予約する。
    //   S0 (+0us)  : 45 系(環境光、R45・L45)
    //   S1 (+220us): 90 系(環境光、R90・L90) → 差分の計算
    //   S2 (+440us): IMU(1 点読み + FIFO)と角速度
    //   S3 (+630us): エンコーダー・バッテリー、速度、距離・角度の積分
    //   planning は +720us(PlanningTask::kPhaseAfterSensingUs)で、最大 247us
    //   (09-26〜29 のログ)なので次の S0(+1000us)までに終わる。
    // planning が動く時間帯にセンシングの処理を置かないので、同じ優先度のまま
    // 重ならない(受け渡しは今までどおり sensing_result を直接書く)。
    // ============================================================
    static constexpr uint32_t kSlotOffsetUs[4] = {0, 220, 440, 630};
    void slot_s0(uint64_t tick_start); // 45 系
    void slot_s1();                    // 90 系
    void read_imu();                   // S2
    void read_enc_bat();               // S3

    std::shared_ptr<input_param_t> param;
    ASM330LHH gyro_;
    AS5147P   enc_r_;   // 右エンコーダ (SPI1, CS=GPIO17)
    AS5147P   enc_l_;   // 左エンコーダ (SPI1, CS=GPIO1)
    ADS7042   battery_; // バッテリADC  (SPI1, CS=GPIO3)

    // DMA channels: gyro (SPI1, 8-bit) / battery (SPI1, 16-bit)
    int dma_tx_spi1_ = -1;
    int dma_rx_spi1_ = -1;
    int dma_tx_bat_  = -1;
    int dma_rx_bat_  = -1;
    // 1(アドレス) + ジャイロZ軸2 + 加速度X軸2 + Y軸2 + Z軸2バイト。
    // OUTZ_L_G(0x26)起点でIF_INCにより加速度計のOUTX_L_XL(0x28)〜
    // OUTZ_H_XL(0x2D)まで連続読み出しする(ASM330LHHのレジスタ配置が
    // OUTZ_H_G(0x27)の直後からOUTX/Y/Z_XLへ連続しているため)。
    // 加速度Zはピッチング等で重力成分がX/Y軸に漏れ込む影響の把握用。
    uint8_t  gyro_dma_tx_[9]{};
    uint8_t  gyro_dma_rx_[9]{};
    uint16_t bat_dma_tx_ = 0;
    uint16_t bat_dma_rx_ = 0;
    dma_channel_config dma_cfg_tx_spi1_{};
    dma_channel_config dma_cfg_rx_spi1_{};
    dma_channel_config dma_cfg_tx_bat_{};
    dma_channel_config dma_cfg_rx_bat_{};
    void init_dma();

    // ============================================================
    // ジャイロ FIFO (2026-09-29)
    // 1 点読みでは実 ODR(個体ごと、本機 3508.5Hz)のサンプルの 7 割を捨てて
    // おり、約 1.19kHz の振動が 195/313Hz に折り返していた。FIFO から全サンプル
    // を取り、角度は Σw·T_odr で積分する。使い方は gyro_param.fifo_mode。
    // ============================================================
    // 1 tick に読む上限。通常は 3〜4 個で、これを超えたら Core1 停止後などで
    // 溜まりすぎとみなして flush する。
    static constexpr int kGyroFifoMaxWords = 8;
    static constexpr int kGyroFifoBufBytes = 1 + ASM330LHH_FIFO_WORD_BYTES * kGyroFifoMaxWords;
    uint8_t  gyro_fifo_st_tx_[3]{};   // STATUS1/2 読み出し(アドレス + 2 バイト)
    uint8_t  gyro_fifo_st_rx_[3]{};
    uint8_t  gyro_fifo_tx_[kGyroFifoBufBytes]{};
    uint8_t  gyro_fifo_rx_[kGyroFifoBufBytes]{};
    int16_t  gyro_fifo_raw_[kGyroFifoMaxWords]{}; // この tick のジャイロ Z 生値(古い順)
    int      gyro_fifo_n_ = 0;          // 読めたサンプル数。-1: 溜まりすぎ/あふれ、-2: タグ不一致(どちらも flush 済み)
    // 直近のサンプルの w [rad/s]。末尾が最新。3 サンプル平均と、角加速度の
    // 直線あてはめ(fifo_alpha_win 個、最大 kGyroFifoHist)に使う
    static constexpr int kGyroFifoHist = 10;
    float    gyro_fifo_hist_[kGyroFifoHist]{};
    int      gyro_fifo_hist_n_ = 0;     // 有効な履歴数(flush で 0)
    bool     gyro_fifo_ang_valid_ = false; // この tick の gyro_fifo.d_ang が FIFO 由来か
    uint64_t gyro_fifo_t_read_ = 0;        // この tick に FIFO_STATUS を読んだ時刻(ログ用)
    // 実 ODR の実測: flush 後最初の読み出しを起点に、以後読んだサンプル数を数える
    bool     gyro_fifo_t0_set_ = false;
    uint64_t gyro_fifo_t0_ = 0;
    uint64_t gyro_fifo_t_last_ = 0;
    uint32_t gyro_fifo_cnt_ = 0;
    void  read_gyro_fifo();                           // SPI(mode 3 のまま Phase A の直後)
    void  update_gyro_fifo(float w_snap, float gyro_dt); // 生値 → w / 角度増分
    float gyro_raw_to_w(float raw) const;             // バイアスを引いて左右別ゲイン
    // t_read に読んだ値を planning が使うまでの時間 [s](次の planning tick の
    // アラーム時刻 − t_read、0〜1000us にクランプ)。planning は tick + 720us
    float plan_age_s(uint64_t t_read) const;
    float w_old = 0;
    int64_t gyro_timestamp_old = 0;
    int64_t gyro_timestamp_now = 0;
    int64_t gyro2_timestamp_old = 0;
    int64_t gyro2_timestamp_now = 0;
    int64_t accel_timestamp_old = 0;
    int64_t accel_timestamp_now = 0;
    int64_t enc_r_timestamp_old = 0;
    int64_t enc_r_timestamp_now = 0;
    int64_t enc_l_timestamp_old = 0;
    int64_t enc_l_timestamp_now = 0;

    static void timer_b_irq_handler();  // alarm 1: 枠の予約と振り分け(S0〜S3)
    void calc_vel(float gyro_dt, float enc_l_dt, float enc_r_dt);

    // ============================================================
    // LED点灯シーケンス 非同期ステートマシン (alarm 2)
    // busy_wait_us_32()でCPUをスピンさせる代わりに、
    // 「LED ON→アラームを待ち時間後にセット→ISR return」を1ステップ
    // ずつ繰り返す。待ち時間中にCPUが解放されるため、同一優先度で
    // プリエンプトされないPlanningTaskのIRQ(TIMER0_IRQ_0)が足止めされない。
    // ============================================================
    enum class LedStep : uint8_t { R90, L90, R45_1, R45_2, R45_3, L45_1, L45_2, L45_3 };
    static void led_seq_irq_handler();  // alarm 2: LEDシーケンスの非同期継続
    void led_seq_start_45();            // S0: R45 → L45
    void led_seq_start_90();            // S1: R90 → L90 → finalize_sensing
    void led_seq_end_45();              // 45 系のシーケンス終了
    void led_seq_advance();             // alarm 2 発火時: 直前ステップの読み取り+次の準備
    void led_abort_if_busy();           // 前の枠の LED が残っていたら消して打ち切る(重なりの検知)
    void finalize_sensing(bool led_on); // diff計算・battery計算・calc_time2 (元timer_b_irq_handler末尾)

    // try_start_*: 該当フラグがtrueならLED ON+ADCチャンネル選択+アラームセットして
    // true(=呼び出し元はreturnして待つ)を返す。falseならraw値を0埋めして直ちにfalseを返す
    // (=呼び出し元は待たずに次のステップへ進んでよい)。
    bool try_start_r90();
    bool try_start_l90();
    bool try_start_r45();
    bool try_start_l45();

    // 待ち時間[us] = カウント値(param->led_light_delay_cnt(_2)) *
    // param->led_light_delay_us_per_cnt (hardware.yaml、USB経由でホット
    // リロード可能)。
    // single  = R90/L90/R45(LED1のみ)/L45(LED1のみ)
    // extended= R45/L45のLED1+LED2拡張ステップ(WALL_OFF等でのみ発生)
    uint32_t wait_us_single() const;
    uint32_t wait_us_extended() const;

    LedStep  led_step_{};
    bool     seq_r90_ = false, seq_l90_ = false, seq_r45_ = false, seq_l45_ = false;
    bool     seq_extended_ = false;   // R45/L45でLED1+LED2の3ステップ読み取りが必要か
    uint64_t seq_sense_start_ = 0;    // t_r90等のタイムスタンプ基準 (=timer_b_irq_handlerのsense_start)

    uint32_t interval_us_   = 1000;  // サンプリング周期

    bool     skip_sensing_ = false;  // R90/L90とR45/L45のambient ADCを交互取得
    uint32_t next_alarm_a_ = 0;       // 次の枠の予約時刻
    uint8_t  slot_ = 0;               // 次に動く枠(0〜3)
    uint32_t tick_base_ = 0;          // この tick の S0 の予約時刻
    bool     led_busy_ = false;       // LED シーケンスの途中(alarm 2 待ち)
    bool     seq_led_on_ = true;      // この tick で LED を点けるか(S0 で決め S1 で使う)
    float    gyro_dt_ = 0.0f;         // S2 で求め S3 の角度積分で使う
    int32_t  slot_late_max_ = 0;      // この tick の枠の開始の遅れの最大 [us]
    uint64_t prev_timestamp_ = 0;
    uint64_t start_time_z = 0;

    static std::shared_ptr<SensingTask> s_instance;
    std::shared_ptr<sensing_result_entity_t> sensing_result;
    std::shared_ptr<motion_tgt_val_t> tgt_val;
    std::shared_ptr<PlanningTask> pt;
    float calc_enc_v(float now, float old, float dt);
    float correct_enc(int32_t raw, const float *table);
};
