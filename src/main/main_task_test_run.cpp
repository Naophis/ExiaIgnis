#include "define.hpp"
#include "hardware/gpio.h"
#include "main/main_task.hpp"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

// ─── USB シリアル通信 (main_task_usb.cpp) ────────────────────────────────
// test_front_sensor_sweep() の待機中に sensor.yaml 等を受けて即時反映する
// (dump1() と同じ)。
int usb_read_with_timeout(char *buf, size_t max_size, uint32_t idle_ms);
bool rx_usb_cmd(char *buf, int len);

void MainTask::test_run() {
  // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
  if (sys_.test.suction_active != 0) {
    planning_->suction_power_on();
  }
  mp->reset_gyro_ref_with_check();
  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  // 2026-09-20: 従来はsleep_ms(2500)固定(is_suction_ramping()待ちは
  // suction_enable()直後にfalseを返して素通りするため、ランプ開始から2.5秒)。
  // 本走行(exec_path_running)/test_slaと同じくhold()〜hold_settle_wait()〜
  // unhold()へ揃え、ランプ完了+向きの収束(hardware.yaml hold_settle、
  // total_max_msで絶対打ち切り)で走り出す。
  if (sys_.test.suction_active == 1 || sys_.test.suction_active == 2) {
    mp->hold();
    if (sys_.test.suction_active == 1) {
      planning_->suction_enable(sys_.test.suction_duty,
                                sys_.test.suction_duty_low);
    } else {
      planning_->suction_enable(sys_.test.suction_duty_burst,
                                sys_.test.suction_duty_burst_low);
    }
    mp->hold_settle_wait();
    mp->unhold();
  }
  if (param_->test_log_enable > 0) {
    lt_->start();
    // sleep_ms(5000);
  }
  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();
  req_error_reset();

  planning_->set_search_mode(test_search_mode > 0);

  // testモード用の速度→加速度LUTに切り替える。非吸引時はグリップ不足を
  // 想定し、LUTを使わず固定accl(sys_.test.accl)にフォールバックする。
  if (sys_.test.suction_active != 0) {
    param_->accl_v_x = sys_.test.accl_v_x;
    param_->accl_v_y = sys_.test.accl_v_y;
  } else {
    param_->accl_v_x.clear();
    param_->accl_v_y.clear();
  }

  // v_max→decel絶対値LUTもaccl_v_x/yと同じくsystem.yaml(sys_.test)側で
  // 管理し、testモード開始時にinput_param_tへコピーする(hardware.yamlには
  // 値を置かない、MainTask::apply_decel_v_max_lut()参照)。非吸引時は
  // accl_v_x/yと同じ理由でLUTを使わない。
  if (sys_.test.suction_active != 0) {
    param_->decel_v_max_enable = sys_.test.decel_v_max_enable;
    param_->decel_v_max_x = sys_.test.decel_v_max_x;
    param_->decel_v_max_y = sys_.test.decel_v_max_y;
  } else {
    param_->decel_v_max_enable = 0;
    param_->decel_v_max_x.clear();
    param_->decel_v_max_y.clear();
  }

  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  ps.dist = sys_.test.dist - 5;
  ps.accl = sys_.test.accl;
  ps.decel = apply_decel_v_max_lut(ps.v_max, sys_.test.decel);
  ps.sct = SensorCtrlType::Straight;
  if (sys_.test.dia == 1) {
    ps.sct = SensorCtrlType::Dia;
  }
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = apply_decel_v_max_lut(ps.v_max, sys_.test.decel);
  mp->go_straight(ps);
  planning_->motor_disable();
  lt_->stop();

  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  ui_->coin(120);
  planning_->set_search_mode(false);
  // 1回目: バイナリ dump (rx_term.js バイナリプロトコル)
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_csv();
  ui_->coin(120);
  // 2回目: テキスト dump (rx_term.js テキストプロトコル)
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  ui_->coin(120);
}

void MainTask::test_back() {
  // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
  if (sys_.test.suction_active != 0) {
    planning_->suction_power_on();
  }
  mp->reset_gyro_ref_with_check();

  if (sys_.test.suction_active == 1) {
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    sleep_ms(800);
  } else if (sys_.test.suction_active == 2) {
    planning_->suction_enable(sys_.test.suction_duty_burst,
                              sys_.test.suction_duty_burst_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    sleep_ms(800);
  }

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  req_error_reset();
  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }
  ps.v_max = sys_.test.v_max;
  ps.v_end = -20;
  ps.dist = sys_.test.dist - 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.motion_type = MotionType::STRAIGHT;
  ps.dia_mode = false;

  mp->go_straight(ps);
  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  mp->go_straight(ps);
  sleep_ms(100);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  lt_->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  mp->coin();
  lt_->save(slalom_log_file);
  ui_->coin(120);

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}

// センサー距離換算の校正用(テストモード28)。param console のセンサ校正タブと
// 組で使う。横壁は置いて記録し、前壁は機体が自分で走って測る(スイープ)。
//
// 待機中: dump2() と同じ 9 列の生値を 10Hz で出す(横壁の記録用)。yaml も
//   受信する(dump1() と同じ rx_usb_cmd)。1 秒ごとに "sweep: ready" も出す。
// スイープ: 走行は必ずケーブルなしで行う。人がするのは次だけ。
//   ケーブルを抜く → 機体のボタン → スタート位置に置く → 通常走行と同じ
//   開始手順(横位置の LED ゲージ → 前に手をかざす) → 機体が前壁へ向かって
//   走って止まる → ケーブルをつなぐ(ログは自動で送る)。
//   - ケーブルがつながっている間は開始手順へ入らない(つないだまま走らせない)。
//   - ケーブルなしでは画面が見えないので、走れない理由はブザー(error 音)で
//     知らせる。開始手順へ入る前ならボタンで待機へ戻れる。
//   - ログは、走ったあと最初につながって 1 秒後に送る。受信に失敗したら
//     ケーブルを 3 秒以上抜いて挿し直すと送り直す(次のスイープを走るまで
//     ログは残る)。yaml 受信時の flash 書き込みで USB が約 100ms 切れるが、
//     3 秒未満なので送り直しにはならない。
//   "sweep: ..." の各行は param console の案内表示が読むので、文言を変えたら
//   lib/sensor-calib-shared.ts の parseSweepStateLine も合わせること。
//
// 距離の基準は迷路の寸法で決める(2026-09-28、ユーザー案):
//   探索の走り出しと同じく offset_start_dist_search で区画中央へ出て、そこから
//   sensor_sweep_cells 区画進む。止まるのは区画中央 = 前壁まで cell/2 − 壁の
//   半分(ハーフで 42mm)。つまり「スタート区画 + cells 区画」の直線の突き当たりに
//   前壁を置く。開始位置の前壁距離 d0 = 走行距離 + 42 で、人が測る値が無い。
//   測りたい範囲(42〜138mm)に入る前に 1 区画ぶん壁制御で姿勢を整えられる。
//   実機 2 本(20260928_222909/222947、このときは 85mm 走行)で、寸法で決めた
//   開始位置での読みの差は 0.2〜0.7mm、a の差は 1% 以内だった。
//
// 壁制御は走行距離で決め打ちにする(2026-09-28、ユーザー指示):
//   走り出しから sensor_sweep_wall_ctrl_dist(100mm)だけ壁制御(sct=Straight)で
//   姿勢を整え、そこから先は壁制御を切って(sct=NONE)直前の向きをジャイロで
//   保つ。45 度センサー(right45_d 等)は前壁へ近づくと前壁を見てしまうが
//   (20260927_095027.csv では前壁まで約 74mm から)、その手前で切れている。
//   前センサーの読みで切る既存の判定(exist.front)は、校正中の係数に依るので
//   使わない。cells=2 なら前壁まで 137mm の所で切れ、測る範囲(42〜138mm)は
//   ほぼすべて壁制御なしの区間に入る。
//   go_straight を 2 本つなぐので、ログの dist は 2 本目の頭で 0 に戻る
//   (param console 側 parseSweepLog がつなぎ直す)。
//
// 前壁が近すぎると当たるので、いまの far 係数で読んだ前壁距離が近いあいだは
// 開始手順へ入らない(sensor_sweep_guard: 0 で無効。前センサーを一度も校正して
// いない機体用)。距離の読みは sensor_range_max(180mm)で頭打ちになるので、
// しきい値は「走行距離 − 15」と「sensor_range_max − 10」の小さい方にする
// (2026-09-28: 前者だけにしていて 180 > 180 が通らず、スタート位置で永久に
// 待っていた)。2 区画の直線(開始位置 147mm)の置き間違いはこれで止まる。
//
// ケーブルなしでこのモードへ入ったときは、ほかの走行テストと同じく、そのまま
// 開始手順へ進む(起動のボタン → 置く → 手をかざす)。つないで入ったときと、
// 1 本走ったあとは待機し、次のスイープは機体のボタンで始める(走り終えて
// 持ち上げた手を、手かざしと見なして走り出さないため)。
void MainTask::test_front_sensor_sweep() {
  const auto se = get_sensing_entity();
  // 速度・加減速・吸引は直進テスト(test_run)と同じく system.yaml の test から
  // 読む(v_max / accl / decel / suction_active)。yaml を受信し直したら次の
  // スイープから反映されるよう、値は走る直前に読む。
  const int cells = std::clamp(sys_.test.sensor_sweep_cells, 1, 4);
  const float dist = param_->offset_start_dist_search + param_->cell2 * cells;
  // 壁制御ありで走る距離(決め打ち)。止まるための距離は必ず残す
  const float ctrl_dist = std::clamp(sys_.test.sensor_sweep_wall_ctrl_dist,
                                     10.0f, dist - 60.0f);
  // 壁の厚みは区画の 1/15(ハーフ 6mm、クラシック 12mm)
  const float end_dist = param_->cell2 / 2 - param_->cell2 / 30;
  const float d0 = dist + end_dist;
  const bool guard = sys_.test.sensor_sweep_guard != 0;
  constexpr int kLoopMs = 10;
  constexpr float kClearDist = 95; // 前が空いたとみなす距離 [mm](手かざし判定は 90)
  constexpr int kDumpDelayMs = 1000;  // つながってからログを送るまで
  constexpr int kReplugMs = 3000;     // これ以上抜いてから挿すと送り直す

  constexpr size_t kRxBufSize = 16384;
  char *rx_buf = static_cast<char *>(malloc(kRxBufSize));

  bool has_log = false;      // 送れるログがある(次のスイープを走るまで)
  bool pending_dump = false; // まだ送っていない/送り直す
  int wired_ms = 0;
  int unplugged_ms = 0;
  // ケーブルなしで入ったら、待機を飛ばして開始手順へ
  bool armed = !stdio_usb_connected();
  // 前壁までの読みがこれ以上なら走ってよい
  const float far_ok =
      std::min(dist - 15.0f, param_->sensor_range_max - 10.0f);

  while (1) {
    planning_->tgt_val->nmr.motion_type = MotionType::SENSING_DUMP;
    planning_->tgt_val->nmr.timstamp = planning_->tgt_val->nmr.timstamp + 1;
    planning_->send_command(*planning_->tgt_val);

    // 待機: 生値を出しながらボタンを待つ。yaml が来たら受けて反映する。
    // ログがあれば、つながったところで送る。
    for (int tick = 0; !armed && !ui_->button_state_hold(); tick++) {
      if (stdio_usb_connected()) {
        if (has_log && unplugged_ms >= kReplugMs) {
          pending_dump = true;
        }
        unplugged_ms = 0;
        wired_ms += kLoopMs;
      } else {
        wired_ms = 0;
        unplugged_ms += kLoopMs;
      }
      if (pending_dump && wired_ms >= kDumpDelayMs) {
        printf("sweep: sending log (d0=%.1f)\n", d0);
        lt_->dump_log(slalom_log_file);
        printf("sweep: dumped\n");
        pending_dump = false;
        tick = 0;
      }
      if (tick % 100 == 0) {
        printf("sweep: ready (v=%.0f accl=%.0f decel=%.0f suction=%d "
               "dist=%.1f d0=%.1f ctrl=%.1f)\n",
               sys_.test.v_max, sys_.test.accl, sys_.test.decel,
               sys_.test.suction_active, dist, d0, ctrl_dist);
      }
      if (tick % 10 == 0) {
        printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", se->led_sen.left90.raw,
               se->led_sen.left45_3.raw, se->led_sen.left45_2.raw,
               se->led_sen.left45.raw, se->led_sen.front.raw,
               se->led_sen.right45.raw, se->led_sen.right45_2.raw,
               se->led_sen.right45_3.raw, se->led_sen.right90.raw);
      }
      if (rx_buf) {
        int rlen = usb_read_with_timeout(rx_buf, kRxBufSize, kLoopMs);
        if (rlen > 0 && rx_usb_cmd(rx_buf, rlen)) {
          load_params();
        }
      } else {
        sleep_ms(kLoopMs);
      }
    }
    armed = false;
    ui_->coin(40);

    // ケーブルが抜かれ、スタート位置に置かれる(前が 0.5 秒空く)のを待つ。
    // ここでボタンを押すと待機へ戻る(押し間違い用)。
    bool cancelled = false;
    for (int ok_ms = 0, tick = 0; ok_ms < 500; tick++) {
      sleep_ms(kLoopMs);
      if (ui_->button_state_hold()) {
        cancelled = true;
        break;
      }
      const float l = se->ego.left90_far_dist;
      const float r = se->ego.right90_far_dist;
      const bool wired = stdio_usb_connected();
      const bool clear = se->ego.left90_mid_dist > kClearDist &&
                         se->ego.right90_mid_dist > kClearDist;
      const bool room = !guard || (l >= far_ok && r >= far_ok);
      if (tick % 100 == 0) {
        if (wired) {
          printf("sweep: unplug the cable\n");
        } else if (clear && !room) {
          printf("sweep: front wall too close (L=%.0f R=%.0f, run=%.0f)\n", l,
                 r, dist);
        } else {
          printf("sweep: place at the start position\n");
        }
      }
      // 走れない理由を 2 秒ごとに音で知らせる(ケーブルなしでは画面が見えない)
      //   短い音 1 回 = ケーブルがつながっていると判定している
      //   低い音 4 回 = 前壁が近すぎる
      if (tick % 200 == 199) {
        if (wired) {
          ui_->coin(40);
        } else if (clear && !room) {
          ui_->error();
        }
      }
      ok_ms = (!wired && clear && room) ? ok_ms + kLoopMs : 0;
    }
    if (cancelled) {
      ui_->coin(40);
      printf("sweep: cancelled\n");
      continue;
    }

    // ここから停止まで、吸引・加減速テーブルの扱いと止まり方は test_run() と
    // 同じ。違いは、走行距離と、壁制御を途中で切ることだけ。
    const float v = std::max(10.0f, std::abs(sys_.test.v_max));
    const int suction = sys_.test.suction_active;

    printf("sweep: wave a hand in front to start\n");
    // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
    if (suction != 0) {
      planning_->suction_power_on();
    }
    mp->reset_gyro_ref_with_check();
    printf("sweep: running\n");

    reset_tgt_data();
    reset_ego_data();
    planning_->motor_enable();
    if (suction == 1 || suction == 2) {
      mp->hold();
      if (suction == 1) {
        planning_->suction_enable(sys_.test.suction_duty,
                                  sys_.test.suction_duty_low);
      } else {
        planning_->suction_enable(sys_.test.suction_duty_burst,
                                  sys_.test.suction_duty_burst_low);
      }
      mp->hold_settle_wait();
      mp->unhold();
    }

    lt_->start();
    reset_tgt_data();
    reset_ego_data();
    planning_->motor_enable();
    req_error_reset();
    planning_->set_search_mode(test_search_mode > 0);

    // 吸引時だけ速度→加速度 / v_max→減速度のテーブルを使う(test_run と同じ)
    if (suction != 0) {
      param_->accl_v_x = sys_.test.accl_v_x;
      param_->accl_v_y = sys_.test.accl_v_y;
      param_->decel_v_max_enable = sys_.test.decel_v_max_enable;
      param_->decel_v_max_x = sys_.test.decel_v_max_x;
      param_->decel_v_max_y = sys_.test.decel_v_max_y;
    } else {
      param_->accl_v_x.clear();
      param_->accl_v_y.clear();
      param_->decel_v_max_enable = 0;
      param_->decel_v_max_x.clear();
      param_->decel_v_max_y.clear();
    }

    // 壁制御を切った区間でも生値を取る(探索モードの sct=NONE は LED を消す)
    planning_->tgt_val->sensing_force_led = true;

    // 1 本目: 壁制御あり(姿勢を整える)
    ps.v_max = v;
    ps.v_end = v;
    ps.accl = sys_.test.accl;
    ps.decel = apply_decel_v_max_lut(ps.v_max, sys_.test.decel);
    ps.dist = ctrl_dist;
    ps.motion_type = MotionType::STRAIGHT;
    ps.sct = SensorCtrlType::Straight;
    ps.wall_off_req = WallOffReq::NONE;
    ps.dia_mode = false;
    mp->go_straight(ps);
    // 2 本目: 壁制御なし(測る範囲)
    ps.v_max = v;
    ps.v_end = 20;
    ps.accl = sys_.test.accl;
    ps.decel = apply_decel_v_max_lut(ps.v_max, sys_.test.decel);
    ps.dist = dist - ctrl_dist - 5;
    ps.sct = SensorCtrlType::NONE;
    mp->go_straight(ps);
    // 3 本目: 最後の 5mm。区画中央で止まる
    ps.v_max = 20;
    ps.v_end = sys_.test.end_v;
    ps.dist = 5;
    ps.accl = sys_.test.accl;
    ps.decel = apply_decel_v_max_lut(ps.v_max, sys_.test.decel);
    mp->go_straight(ps);
    planning_->motor_disable();
    // 停止処理の reset で dist が 0 に戻る前にログを止める
    lt_->stop();
    planning_->tgt_val->sensing_force_led = false;

    reset_tgt_data();
    reset_ego_data();
    req_error_reset();
    planning_->suction_disable();
    planning_->set_search_mode(false);
    ui_->coin(120);

    // ログは待機へ戻ってから、つながったところで送る
    has_log = true;
    pending_dump = true;
    wired_ms = 0;
    unplugged_ms = 0;
  }
}

void MainTask::test_front_wall_offset() {
  const auto se = get_sensing_entity();
  printf("search_walloff_offset= %f, %f\n",
         param_->sen_ref_p.search_exist.offset_l,
         param_->sen_ref_p.search_exist.offset_r);

  // ESC起動レイテンシをreset_gyro_ref_with_check()の待ち時間と重ねて隠す。
  if (sys_.test.suction_active != 0) {
    planning_->suction_power_on();
  }
  mp->reset_gyro_ref_with_check();

  if (sys_.test.suction_active == 1) {
    planning_->suction_enable(sys_.test.suction_duty,
                              sys_.test.suction_duty_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    sleep_ms(800);
  } else if (sys_.test.suction_active == 2) {
    planning_->suction_enable(sys_.test.suction_duty_burst,
                              sys_.test.suction_duty_burst_low);
    while (planning_->is_suction_ramping()) {
      sleep_ms(10);
    }
    sleep_ms(800);
  }

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();

  req_error_reset();
  if (param_->test_log_enable > 0) {
    lt_->start_slalom_log();
  }
  // planning_->active_logging(_f);

  ps.v_max = sys_.test.v_max;
  ps.v_end = sys_.test.v_max;
  ps.dist = param_->cell + param_->cell2 / 2;
  ps.accl = sys_.test.accl;
  ps.decel = sys_.test.decel;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_off_req = WallOffReq::NONE;
  ps.wall_off_dist_r = 0;
  ps.wall_off_dist_l = 0;
  ps.dia_mode = false;
  mp->go_straight(ps);

  ps.dist = param_->cell2 / 2 - 5;
  if (se->ego.left90_mid_dist < param_->sensor_range_mid_max &&
      se->ego.right90_mid_dist < param_->sensor_range_mid_max) {
    ps.dist -= (param_->front_dist_offset2 - se->ego.front_mid_dist);
  }

  ps.v_max = sys_.test.v_max;
  ps.v_end = 20;
  mp->go_straight(ps);

  ps.v_max = 20;
  ps.v_end = sys_.test.end_v;
  ps.dist = 5;
  mp->go_straight(ps);

  sleep_ms(100);
  planning_->motor_disable();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  planning_->suction_disable();

  lt_->stop_slalom_log();
  reset_tgt_data();
  reset_ego_data();
  req_error_reset();
  lt_->save(slalom_log_file);
  ui_->coin(120);

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}

void MainTask::test_front_ctrl(bool mode) {
  file_idx = sys_.test.file_idx;

  if (file_idx >= tpp.file_list_size) {
    ui_->error();
    return;
  }

  mp->reset_gyro_ref_with_check();

  reset_tgt_data();
  reset_ego_data();
  planning_->motor_enable();
  req_error_reset();

  mp->front_ctrl(mode);
  planning_->motor_disable();
  ui_->coin(120);

  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
  lt_->dump_log(slalom_log_file);
  while (1) {
    if (ui_->button_state_hold())
      break;
    sleep_ms(10);
  }
}
