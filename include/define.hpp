#pragma once

// ============================================================
// PSRAM (APS6404L 外付け, QSPI 共有)
// ============================================================
// GPIO0 = XIP_CS1n (QMI M1 CS1)
// XIP_CS1n に対応する GPIO: 0, 8, 19, 47 のみ (function 9)
#define PSRAM_CS_PIN 0

// ============================================================
// I2C1  (UI / LED driver)
// ============================================================
#define I2C1_PORT     i2c1
#define I2C1_SCL      23
#define I2C1_SDA      22
#define LED_I2C_ADDR  0x4D   // 7-bit (Pico SDK形式: 0x9A >> 1)

// ============================================================
// UART (dump_binary() 用。使用時はピンを別途 gpio_set_function すること)
// ============================================================
#define UART_ID    uart1
#define BAUD_RATE  115200

// ============================================================
// GPIO / Buttons
// ============================================================
// Tactile switch on GPIO2 (active low, pull-up)
#define BTN_PIN 2

// ============================================================
// PWM / Buzzer
// ============================================================
// Piezo buzzer on GPIO18 (PWM1 A)
#define BUZZER_PIN      18
#define BUZZER_FREQ_HZ  500
#define BUZZER_WRAP     999u  // PWM counter top (1000 steps)

// ============================================================
// PWM / Motor & Suction (50 kHz)
// ============================================================
#define M_PWM_L1      4   // PWM2 A
#define M_PWM_L2      5   // PWM2 B
#define M_PWM_R1      6   // PWM3 A
#define M_PWM_R2      7   // PWM3 B
#define SUCTION_EN    8   // GPIO output — HIGH = driver enabled (BldcActuator専用、現在未使用)
#define SUCTION_PWM1  9   // PWM4 B  (V phase, BldcActuator専用、現在未使用)
// GPIO10: ESC電源用ロジックゲートICのイネーブル入力。HIGH = ESC通電。
// SuctionEscActuatorのenable()/disable()と連動して駆動する。
#define SUCTION_POWER_EN  10
#define SUCTION_PWM3  11  // PWM5 B  (BldcActuator W相 / SuctionEscActuator兼用)

#define MOTOR_PWM_FREQ_HZ  100000u

// ============================================================
// Suction ESC スロットル信号 (パルス幅 1000〜2000us 相当のスケール)
// ============================================================
// 吸引ESCへのスロットル信号線。旧BldcActuatorのW相ピン(GPIO11)を流用。
// 実際の変調方式(DShot / サーボPWM)は下の SUCTION_ESC_USE_DSHOT で選ぶ。
#define SUCTION_ESC_PWM       SUCTION_PWM3
// 信号リフレッシュ周波数(サーボPWM経路のみ。DShot経路では
// SUCTION_ESC_DSHOT_FRAME_HZ が対応する)。
// [重要・実機検証済み] 元は250Hzだったが原因不明の低速異常回転(ゴリゴリ)が
// 頻発していた。50Hzへ変更を試みた際、kClkDivInt=16のままだと計算上の
// wrap値(187499)がpwm_set_wrap()の引数型uint16_tへ暗黙変換で切り詰められ、
// 実際には約166Hz(wrap=56427相当)が出力されていた。ところが「正しく」
// 分周比を上げてちょうど50Hzになるよう修正した版(wrap=62499、ただし
// duty分解能が9.375→3.125 ticks/usへ悪化)は改善せず、逆に「切り詰められて
// 偶然出ていた約166Hz(細かい分解能のまま)」の方で異常回転が解消することが
// 実機で確認された。原因(分解能か166Hz付近という値そのものか)は特定できて
// いないが、この166という値は「たまたま動いた」ものを明示的な意図に
// 変えただけで、実測結果は変えていない。
#define SUCTION_ESC_PULSE_MIN_US  1000u  // 0%duty (アーム/停止)
#define SUCTION_ESC_PULSE_MAX_US  2000u  // 100%duty (フル)

// ============================================================
// Suction ESC プロトコル選択 (ESCape32: DShot / AM32: サーボPWM)
// ============================================================
// 1 = 標準(非反転)DShot600 で指令する (SuctionEscDshotActuator)
// 0 = 従来のRCサーボ標準PWM 1000〜2000us で指令する (SuctionEscActuator)
//
// ESC側のファームウェアをAM32からESCape32へ載せ替えたため、既定はDShot。
// ESCape32はidleレベルから極性を、ビット時間からレート(DShot300/600/1200)を
// 自動判別するので、ESC側の設定(input_mode=0)は変更不要。
// パルス幅(us)→スロットルの対応はESCape32のサーボPWM入力と一致させてある
// ため、system.yamlのsuction_duty系(us単位)はそのまま流用できる
// (suction_esc_dshot_actuator.cpp の esc32_value_from_pulse_us() 参照)。
//
// 0に戻せばサーボPWM実装(AM32時代の経路)へそのまま切り替わる。
#define SUCTION_ESC_USE_DSHOT        1
// DShot送信に専有するPIOブロック。AM32設定通信(am32_config、pio0)と
// 衝突しないよう別ブロックにする。
#define SUCTION_ESC_DSHOT_PIO        pio1
#define SUCTION_ESC_DSHOT_BITRATE_HZ 600000u  // DShot600
// フレーム送出周期。フレーム自体は約27us(16bit)で終わり、残りはidle-low。
// ESCape32は「2ビット時間エッジなし」でフレーム境界を判定するため、
// この周期なら十分なギャップが空く。Core0/Core1のどちらの負荷にも依存しない
// (PIO + DMAが自律反復する。DshotTx参照)。
#define SUCTION_ESC_DSHOT_FRAME_HZ   2500u
