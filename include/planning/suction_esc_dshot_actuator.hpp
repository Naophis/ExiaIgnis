#pragma once
#include "driver/dshot_tx.hpp"
#include <stdint.h>

// 吸引モーター用ESC(ESCape32)への 標準(非反転)DShot600 出力。
// SuctionEscActuator(サーボ標準PWM 1000〜2000us)と同じ公開APIを持つ差し替え
// 実装で、どちらを使うかは include/planning/suction_esc.hpp の
// SUCTION_ESC_USE_DSHOT で切り替える(define.hpp)。
//
// [PWM版との違いは「線の上の信号」だけ]
// enable/disable/power_on/power_off のセマンティクス、および apply_us() が
// 受け取るパルス幅(us)の意味は PWM 版と完全に同じ。ControlLaw 側のランプ処理
// (suction_ramp_rate_us_x/y、suction_batt_boost_us_table 等、us単位で
// チューニング済みのテーブル群)はそのまま使える。
// us → DShotスロットル値の変換は、ESCape32 がサーボPWM入力に対して行う
// 変換式そのもの(src/io.c setthrot(): throt_min+50 のデッドバンド付き線形、
// 0〜2000)を再現しているため、同じ system.yaml の値が同じ実スロットルになる
// (=乗り換えで吸引力が変わらない)。
//
// [常時出力]
// DShotはフレームを送り続けないとESCがフェイルセーフに入るプロトコルだが、
// 送出は PIO + DMA が CPU非関与で自律的に反復する(DshotTx参照)。
// そのため init() 直後から、Core1のplanning IRQが動いていない期間も、
// 最後に指定したフレームが出続ける(PWM版のハードウェアPWMと同じ挙動)。
// apply_us() は32bitストア1回だけなのでIRQから呼んで安全。
class SuctionEscDshotActuator {
public:
  // GPIO設定・PIO/DMA初期化。Core0のmainから呼ぶ。
  // 呼び出し直後から DSHOT_CMD_MOTOR_STOP を送出し始める(ESCのアーム待ち状態)。
  void init();

  // duty[%] (0〜100) をパルス幅(1000〜2000us)に線形変換して出力する。
  void apply(float duty_pct);

  // パルス幅を us で直接指定する(1000〜2000usへクランプ)。
  // ControlLawはこちらを使う(system.yamlのsuction_duty系はus直接指定)。
  void apply_us(float pulse_us);

  // フラグ管理のみ(PWM版と同じ)。実際の出力遷移はControlLaw側のランプで行う。
  void enable();
  void disable();
  bool is_enabled() const { return enabled_; }

  // SUCTION_POWER_EN(ESC電源ゲート)のHIGH/LOWのみを切り替える。
  // ESC起動レイテンシを隠すため、enable()より前に単独で呼べる。
  void power_on();
  void power_off();

  // 他の機能(AM32設定通信など)がGPIOを奪った後に信号線を戻す。
  void reattach_pin();

  // ------------------------------------------------------------------
  // 回転方向の設定(DShot特殊コマンド経由)
  // ------------------------------------------------------------------
  // reversed=false → DSHOT_CMD_SPIN_DIRECTION_1(7) = ESCape32 の cfg.revdir=0
  // reversed=true  → DSHOT_CMD_SPIN_DIRECTION_2(8) = ESCape32 の cfg.revdir=1
  // save_to_esc=true なら続けて DSHOT_CMD_SAVE_SETTINGS(12) を送り、ESCの
  // フラッシュへ永続化する(以降の電源投入でも保持される)。
  //
  // ESCape32側の受け付け条件(src/io.c iotim_dma_isr()):
  //   - フレームのtelemetry要求bitが立っていること
  //   - モーターが停止していること(ertm==0)
  //   - 同一コマンドが6フレーム連続で届くこと(6フレーム目で1回だけ適用)
  // これらを満たすため、この関数は
  //   停止コマンド送出 → ESC通電 → 起動/ロック待ち → コマンドを数十ms保持
  // の順で進める。Core0から(走行前に)呼ぶこと。数百ms〜1秒台ブロックする。
  // 戻り値false: DShot未初期化。
  bool set_spin_direction(bool reversed, bool save_to_esc);

private:
  // throttle値(0 または 48〜2047)を1フレームとして送出し続ける。
  void send_throttle_value(uint16_t value);
  // DShot特殊コマンド(1〜47)を hold_ms の間送り続ける。
  void send_command(uint16_t cmd, uint32_t hold_ms);

  DshotTx dshot_;
  bool    enabled_ = false;
  // 特殊コマンド送出中フラグ。ControlLaw(Core1の1kHz IRQ)が毎tick
  // apply_us()でフレームを上書きしてしまうと、6フレーム連続という
  // コマンド成立条件を満たせないため、その間だけapply_us()を無効化する。
  // Core0が書き・Core1が読むためvolatile + __dmb()で公開する。
  volatile bool cmd_mode_ = false;
};
