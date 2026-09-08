#include "planning/control_law.hpp"
#include "define.hpp"
#include "utils/irq_log.hpp"
#include <algorithm>
#include <cmath>

void ControlLaw::init(MotorActuator *motor, SuctionEscActuator *esc,
                      SensorProcessor *sensor,
                      TrajectoryGenerator *trj, EgoEstimator *ego,
                      std::shared_ptr<motion_tgt_val_t> tgt_val,
                      std::shared_ptr<sensing_result_entity_t> sensing_result,
                      std::shared_ptr<input_param_t> param) {
  motor_ = motor;
  esc_   = esc;
  sensor_ = sensor;
  trj_ = trj;
  ego_ = ego;
  ee = std::make_shared<pid_error_entity_t>();
  tgt_val_ = tgt_val;
  sensing_result_ = sensing_result;
  param_ = param;
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc(bool motor_en, bool suction_en, bool search_mode,
                 float last_tgt_angle, float dt) {
  motor_en_ = motor_en;
  suction_en_ = suction_en;
  search_mode_ = search_mode;
  w_reset_ = 1;
  last_tgt_angle_ = last_tgt_angle;
  dt_ = dt;

  // 2026-09-06: 走行開始(motor_enの立ち上がり)で走り出し姿勢リセットを武装
  // する(update_start_align()参照)。path_run()はhold()の前と後で
  // motor_enable()を呼ぶが、Core1側のmotor_enは最初の呼び出しでtrueになった
  // まま維持されるため立ち上がりは走行あたり一度だけ。
  if (motor_en_ && !motor_en_prev_) {
    start_align_pending_ = true;
    start_align_cnt_ = 0;
    start_align_fire_dist_ = -1.0e9f;
    start_align_anchor_valid_ = false;
  } else if (!motor_en_) {
    start_align_pending_ = false;
    start_align_cnt_ = 0;
  } else if (!start_align_pending_ && param_->start_align.repeat_dist > 0 &&
             tgt_val_->global_pos.dist - start_align_fire_dist_ >=
                 param_->start_align.repeat_dist) {
    // 前回の発火からrepeat_dist走ったら再武装(structs.hpp start_align_t::
    // repeat_dist参照)。発火自体は次に壁追従が収束した時点。
    start_align_pending_ = true;
    start_align_cnt_ = 0;
  }
  motor_en_prev_ = motor_en_;

  const bool ctl_log = (g_ctl_debug_ticks > 0);
  if (ctl_log)
    g_irq_log.push("Ca"); // calc() start, before axel_degenerate block

  // axel degenerate gain (pre-calc before sensor PID, mirrors tick() logic)
  float axel_degenerate_gain = 1.0f;
  diff_old = diff;
  // 速度→加速度LUT: 探索/タイムアタック/testモードで別々のテーブルを走行
  // 開始時にparam_->accl_v_x/yへコピーして使う(main_task_run.cpp,
  // main_task_test_run.cpp参照)。axel_degenerateと違い探索モードでも適用する。
  if (tgt_val_->motion_type == MotionType::STRAIGHT &&
      param_->accl_v_x.size() >= 2) {
    tgt_val_->tgt_in.accl = sensor_->interp1d(param_->accl_v_x, param_->accl_v_y,
                                              tgt_val_->ego_in.v, false);
  }
  if (!search_mode_ && tgt_val_->motion_type == MotionType::STRAIGHT) {
    if (param_->axel_degenerate_x.size() >= 2 &&
        tgt_val_->nmr.sct == SensorCtrlType::Straight) {
      SensingControlType type = SensingControlType::None;
      diff = ABS(check_sen_error(type));
      if (diff == 0)
        diff = diff_old;
      axel_degenerate_gain = sensor_->interp1d(
          param_->axel_degenerate_x, param_->axel_degenerate_y, diff, false);
      tgt_val_->tgt_in.axel_degenerate_gain =
          (1 - param_->sensor_gain.front2.b) *
              tgt_val_->tgt_in.axel_degenerate_gain +
          param_->sensor_gain.front2.b * axel_degenerate_gain;
    } else if (param_->axel_degenerate_dia_x.size() >= 2 &&
               tgt_val_->nmr.sct == SensorCtrlType::Dia) {
      SensingControlType type = SensingControlType::None;
      diff = ABS(check_sen_error_dia(type));
      if (diff == 0)
        diff = diff_old;
      axel_degenerate_gain =
          sensor_->interp1d(param_->axel_degenerate_dia_x,
                            param_->axel_degenerate_dia_y, diff, false);
      if (axel_degenerate_gain < 0 &&
          tgt_val_->tgt_in.end_v > tgt_val_->ego_in.v) {
        tgt_val_->tgt_in.axel_degenerate_gain = 0.01f;
      }
      tgt_val_->tgt_in.axel_degenerate_gain =
          (1 - param_->sensor_gain.front2.b) *
              tgt_val_->tgt_in.axel_degenerate_gain +
          param_->sensor_gain.front2.b * axel_degenerate_gain;
    }
  } else {
    diff = diff_old = 0;
    tgt_val_->tgt_in.axel_degenerate_gain = axel_degenerate_gain;
  }

  if (ctl_log)
    g_irq_log.push("Cb"); // after axel_degenerate block, before calc_tgt_duty
  calc_tgt_duty();
  if (ctl_log)
    g_irq_log.push("Cc"); // after calc_tgt_duty, before check_fail_safe
  check_fail_safe();
  if (ctl_log)
    g_irq_log.push("Cd"); // after check_fail_safe, before set_next_duty
  set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);
}

// ============================================================

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc_tgt_duty() {
  const bool dlog = (g_ctl_debug_ticks > 0);
  if (dlog)
    g_irq_log.push("D0"); // calc_tgt_duty start

  const unsigned char reset_req = motor_en_ ? 1 : 0;
  const unsigned char enable = 1;
  duty_sen = 0;
  sen_ang = 0;

  ee->s_val.p = ee->s_val.i = ee->s_val.d = 0;
  ee->s_val.p_val = ee->s_val.i_val = ee->s_val.d_val = 0;
  ee->s_val.z = ee->s_val.zz = 0;

  if (tgt_val_->nmr.sct == SensorCtrlType::Straight) {
    duty_sen = calc_sensor_pid();
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else if (tgt_val_->nmr.sct == SensorCtrlType::Dia) {
    // sen_kanayama_dwはcalc_sensor_pid_dia()内で毎回設定される(kanayama_dia
    // 有効時のみ非0、無効時/柱ロスト時は同関数内で0にする)。2026-08-23の
    // SLALOM/SLA_BACK_STR跨ぎ凍結バグ(20260823_072147.csv)と同じ理由で、
    // ここで呼ばれない他sct(Straight/NONE)遷移時にも凍結が残らないよう、
    // 呼び出し元(calc_sensor_pid_dia)側で毎tick明示的に更新する。
    duty_sen = calc_sensor_pid_dia();
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
  } else if (tgt_val_->nmr.sct == SensorCtrlType::NONE) {
    duty_sen = sen_ang = 0;
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
    sen_kanayama_dw = 0; // 同上
  }
  if (dlog)
    g_irq_log.push("D1"); // after sct sensor_pid block

  sensing_result_->ego.duty.sen = duty_sen;
  sensing_result_->ego.duty.sen_ang = sen_ang;

  calc_pid_val();
  calc_pid_val_ang();
  calc_pid_val_ang_vel();
  calc_pid_val_front_ctrl();
  if (dlog)
    g_irq_log.push("D2"); // after calc_pid_val*

  duty_c = 0;
  duty_roll = 0;
  duty_front_ctrl_roll_keep = 0;
  duty_roll_ang = 0;
  duty_front_ctrl_trans = 0;
  duty_front_ctrl_roll = 0;
  reset_pid_val();

  if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    calc_front_ctrl_duty();
  } else {
    calc_translational_ctrl();
    calc_angle_velocity_ctrl();
  }
  if (dlog)
    g_irq_log.push("D3"); // after translational/angle ctrl
  sensing_result_->ego.duty.sen = duty_sen;

  summation_duty();
  apply_duty_limitter();

  if (tgt_val_->motion_type == MotionType::NONE) {
    tgt_duty.duty_l = tgt_duty.duty_r = 0;
  }
  if (!motor_en_) {
    clear_ctrl_val();
  }

  sensing_result_->ego.duty.duty_r = tgt_duty.duty_r;
  sensing_result_->ego.duty.duty_l = tgt_duty.duty_l;

  sensing_result_->ego.duty.ff_duty_front = trj_->mpc_next_ego.ff_duty_front;
  sensing_result_->ego.duty.ff_duty_roll = trj_->mpc_next_ego.ff_duty_roll;
  sensing_result_->ego.duty.ff_duty_rpm_r = trj_->mpc_next_ego.ff_duty_rpm_r;
  sensing_result_->ego.duty.ff_duty_rpm_l = trj_->mpc_next_ego.ff_duty_rpm_l;
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc_translational_ctrl() {
  const float dt = param_->dt;
  if (!motor_en_) {
    const unsigned char reset = 0;
    vel_pid.step(&ee->v.error_p, &param_->motor_pid.p, &param_->motor_pid.i,
                 &param_->motor_pid.d, &reset, &dt, &duty_c);
    set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
                 param_->motor_pid.p * ee->v.error_p,
                 vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0,
                 0);
  } else {
    if (tgt_val_->motion_type == MotionType::STRAIGHT ||
        tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
      if (last_accl > 0 && tgt_val_->ego_in.accl < 0) {
        ee->v.error_i *= param_->ff_front_gain_decel;
      }
    }

    auto v_error_i = ee->v.error_i;
    if (param_->motor_pid2.antiwindup) {
      if ((v_error_i * ee->v.error_p) < 0 &&
          ABS(ee->v.error_p) > param_->motor_pid2.windup_dead_bind) {
        v_error_i *= param_->motor_pid2.windup_gain;
      }
    }

    const auto diff_dist =
        tgt_val_->ego_in.img_dist - sensing_result_->ego.dist_kf;
    auto kp_gain = param_->motor_pid2.p * ee->v.error_p;
    auto ki_gain = param_->motor_pid2.i * v_error_i;
    auto kb_gain = param_->motor_pid2.b * diff_dist;
    auto kd_gain = param_->motor_pid2.d * ee->v_kf.error_d;
    duty_c = kp_gain + ki_gain + kb_gain + kd_gain;

    // 2026-09-05: kd_gain は v_kf.error_d から作っているのに、ログ(CSVの
    // m_pid_d 列)には v.error_d を渡していたため m_pid_d と m_pid_d_v が
    // 対応せず、ログから motor_pid2.d の実効値だけ逆算できなかった
    // (p/i は逆算できていた)。kd_gain と同じ量を渡す。制御出力は不変。
    set_ctrl_val(ee->v_val, ee->v.error_p, v_error_i, diff_dist,
                 ee->v_kf.error_d, kp_gain, ki_gain, kb_gain, kd_gain,
                 ee->v_log.gain_zz, ee->v_log.gain_z);
  }
  if (w_reset_ == 0 || !motor_en_) {
    ee->w.error_i = ee->w.error_d = 0;
    ee->w_log.gain_z = ee->w_log.gain_zz = 0;
  }
  last_accl = tgt_val_->ego_in.accl;
}

__attribute__((noinline, section(".time_critical.control_law"))) float
ControlLaw::calc_sensor_pid() {
  float duty = 0;
  SensingControlType type = SensingControlType::None;
  // conditional integration: 高速(非探索)モードでは|error_p|が
  // windup_dead_bindを超えている間(壁ロスト直後の再捕捉時など、誤差が
  // 一方向に張り付きうる場面)はI項の積算そのものを止める。ヨーレート
  // ループの「逆符号のときだけ減衰」方式は同方向の持続的な誤差に対して
  // 無力(2026-08-22, str_ang_pidへの初回I項導入で発散・リバート済み)
  // だったため、より単純で安全なこの方式を採用する。
  //
  // 2026-09-05: これに加えて「前tickで壁を見ていなかった(type == None)」場合も
  // 積算を止める。従来は壁の有無に関係なく毎tick積算しており、壁なし区間で
  // check_sen_error()が返す0や、壁あり/なし判定の境界で出るノイズ性の微小
  // バイアスまで貯め込んでいた(20260905_050639.csv: 45°距離が44.0〜45.5mmの
  // 狭い帯=exist閾値44.75/ref45と同じ帯に張り付き、s_pid_pの平均はわずか
  // -0.11mmなのにsen.error_iはwindup_i_max(-0.09)へ張り付いたまま)。
  const bool freeze_sen_i =
      (!search_mode_ && param_->str_ang_pid_fast.antiwindup &&
       ABS(ee->sen.error_p) > param_->str_ang_pid_fast.windup_dead_bind) ||
      (!search_mode_ && !sen_ctrl_active_prev_);
  if (!freeze_sen_i) {
    // 2026-08-30: dtスケーリングなしで生の誤差(mm)をそのまま積算していた
    // ため、1kHzでは実質1000倍の強さで積分され、わずかな定常偏差でも
    // 1秒未満でwindup_i_maxクランプに張り付いたまま常時飽和/チャタリング
    // していた(20260830_162335.csv、s_pid_pは終始2.5mm未満なのにs_pid_iは
    // idx120から常時クランプ近辺)。dt_を掛けて本来の時間積分に修正
    // (windup_i_max/kanayama_straight.kiは1/dt倍・dt倍で再スケール済み、
    // hardware.yaml参照)。
    ee->sen.error_i += ee->sen.error_p * dt_;
  }
  if (param_->str_ang_pid_fast.windup_i_max > 0) {
    ee->sen.error_i =
        std::clamp(ee->sen.error_i, -param_->str_ang_pid_fast.windup_i_max,
                   param_->str_ang_pid_fast.windup_i_max);
  }
  ee->sen.error_d = ee->sen.error_p;
  ee->sen.error_p = check_sen_error(type);

  // ego_in.ang再アンカー(2026-08-23): ego_in.angはSensingTask::calc_vel()
  // (sensing_task.cpp)で毎tick生ジャイロ(w_raw/w_kf)を積分しているだけの
  // 実測ヘディングで、壁が見えない間は無補正でドリフトする(実測: STRAIGHT
  // 400tickで2.7°→3.7°の単調ドリフトを確認)。壁を新規に検出した瞬間
  // (壁と正対しているはず、という前提)にego_in.ang/global_pos.angを
  // ゼロへスナップし、それまでのドリフト蓄積をリセットする。
  const bool wall_found_now = (type == SensingControlType::Wall);
  // 2026-09-06: 走り出し直後はこのスナップを止める(structs.hpp
  // start_align_t::snap_skip_distのコメント参照)。走行1tick目で必ず壁を
  // 新規検出するため、hold中に残った向き(unhold時のang)をここで0に上書き
  // してしまい、kim.thetaだけが残差を持ち越す食い違いになっていた
  // (20260906_0439xx.csv idx2451→2452)。start_align未発火かつ走行開始から
  // snap_skip_dist以内のときだけ抑止し、それ以外は従来通り。
  const bool skip_snap =
      param_->start_align.enable > 0 && start_align_pending_ &&
      !search_mode_ &&
      tgt_val_->global_pos.dist < param_->start_align.snap_skip_dist;
  if (wall_found_now && !wall_found_prev_ && !skip_snap) {
    tgt_val_->ego_in.ang = 0;
    tgt_val_->global_pos.ang = 0;
  }
  wall_found_prev_ = wall_found_now;
  update_start_align(type);

  if (search_mode_) {
    if (ee->sen.error_p > param_->search_sen_ctrl_limitter) {
      ee->sen.error_p = param_->search_sen_ctrl_limitter;
    } else if (ee->sen.error_p < -param_->search_sen_ctrl_limitter) {
      ee->sen.error_p = -param_->search_sen_ctrl_limitter;
    }
  }
  ee->sen.error_d = ee->sen.error_p - ee->sen.error_d;

  if (search_mode_) {
    sen_kanayama_dw = 0;
    if (ee->sen.error_p != 0) {
      duty = param_->str_ang_pid.p * ee->sen.error_p -
             param_->str_ang_pid.i * ee->sen.error_d;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param_->str_ang_pid.p * ee->sen.error_p, 0, 0,
                   -param_->str_ang_pid.i * ee->sen.error_d,
                   ee->sen_log.gain_zz, ee->sen_log.gain_z);
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  } else {
    // いいとこ取り構成(2026-08-23): str_ang_pid_fastのP+D(速い直接duty
    // 注入)とKanayamaカスケード(ky/ki/k_theta、ヨーレートループ経由の
    // 遅い/滑らかな補正)を排他にせず並行して常時実行する。速い成分は
    // P+Dが、定常偏差の解消・向き調整は主にKanayamaのki/k_thetaが担当
    // する想定。二重積分を避けるため、str_ang_pid_fast.iはhardware.yaml
    // 側で0にして運用する(コード上はどちらも独立に有効)。
    if (ee->sen.error_p != 0) {
      const float i_gain = param_->str_ang_pid_fast.i * ee->sen.error_i;
      duty = param_->str_ang_pid_fast.p * ee->sen.error_p + i_gain -
             param_->str_ang_pid_fast.d * ee->sen.error_d;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, ee->sen.error_p, ee->sen.error_i, 0,
                   ee->sen.error_d, param_->str_ang_pid_fast.p * ee->sen.error_p,
                   i_gain, 0, -param_->str_ang_pid_fast.d * ee->sen.error_d,
                   ee->sen_log.gain_zz, ee->sen_log.gain_z);
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, ee->sen_log.gain_zz,
                   ee->sen_log.gain_z);
    }

    // Kanayamaカスケード: str_ang_pid_fastのduty注入と並行して、Δwを
    // calc_pid_val_ang_vel()のoffsetに加算する(sen_kanayama_dw経由、
    // ego_in.w自体は書き換えない、2026-08-22の事故コメント参照)。
    // [ログ注意] enable時はここでee->s_valをKanayamaのey/e_theta/Δw内訳で
    // 上書きするため、s_pid_*列は上のstr_ang_pid_fast内訳ではなくこちらが
    // 表示される(duty_l/r・ang_kf等の実効値には影響しない)。
    if (param_->kanayama_straight.enable) {
      const float ey = ee->sen.error_p;
      // e_thetaにはang_kfではなくkim_thetaを使う。enable_kalman_gyro=0のとき
      // ang_kf/ang_kf2は単にego_in.ang(目標値そのもの)が代入されるだけで
      // 実測と独立していないため、e_thetaが常にゼロになってしまう
      // (EgoEstimator::update()参照)。kim_thetaは実測ジャイロKF(w_kf)を
      // 積分したTrajectoryGenerator::calc_kanayama()内の値で、
      // kanayama.enableに関わらず毎tick更新される独立した実測角度。
      const float e_theta =
          tgt_val_->ego_in.ang - sensing_result_->ego.kim_theta;
      // I項はstr_ang_pid_fast用に条件付き積分・絶対値クランプ済みの
      // ee->sen.error_iを共用(antiwindup/windup_dead_bind/windup_i_maxも
      // str_ang_pid_fast側のものをそのまま使う)。
      const float ki_gain = param_->kanayama_straight.ki * ee->sen.error_i;
      const float delta_w = param_->kanayama_straight.ky * ey + ki_gain +
                            param_->kanayama_straight.k_theta * sinf(e_theta);
      sen_kanayama_dw = delta_w;
      set_ctrl_val(ee->s_val, ey, ee->sen.error_i, 0, e_theta,
                   param_->kanayama_straight.ky * ey, ki_gain, 0,
                   param_->kanayama_straight.k_theta * sinf(e_theta),
                   delta_w, delta_w);
    } else {
      sen_kanayama_dw = 0;
    }
  }

  sen_ctrl_active_prev_ = (type != SensingControlType::None);

  float limit = 0;
  if (type == SensingControlType::None) {
    // 2026-09-05: ここの早期リターンはduty(str_ang_pid_fastの直接duty注入)しか
    // 止めておらず、その手前で計算済みのsen_kanayama_dwはそのまま
    // calc_pid_val_ang_vel()のoffsetへ乗り続けていた。壁を見失っている間も、
    // 直前にクランプまで貯まったI項(ki*error_i)が一定のヨー指令を出し続ける
    // ことになる(20260905_050639.csv: 壁なし区間を含む720mm直進で
    // ki*error_i = -0.231 rad/s が出っぱなし、姿勢P(+0.076)を打ち消して
    // heading が -1.2°に張り付き、y が -16.8mm ずれた)。Δw側も止める。
    sen_kanayama_dw = 0;
    return 0;
  } else if (type == SensingControlType::Wall) {
    limit = sensor_->interp1d(param_->sensor_deg_limitter_v,
                              param_->sensor_deg_limitter_str,
                              tgt_val_->ego_in.v, false);
  } else if (type == SensingControlType::Piller) {
    limit = sensor_->interp1d(param_->sensor_deg_limitter_v,
                              param_->sensor_deg_limitter_piller,
                              tgt_val_->ego_in.v, false);
  }
  limit = limit / 180.0f * M_PI;
  duty = std::clamp(duty, -limit, limit);
  if (!search_mode_) {
    if (tgt_val_->motion_type == MotionType::WALL_OFF ||
        tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
      limit = param_->angle_pid.b / 180.0f * M_PI;
      duty = std::clamp(duty, -limit, limit);
    }
  }
  return duty;
}

// 走り出し姿勢リセット(start_align、2026-09-06)。structs.hpp start_align_t
// のコメント参照。calc_sensor_pid()からtype確定後に毎tick呼ばれる。
//
// 「壁追従が収束した」= 壁を見ている状態で、ヘディング(ego_in.ang)の変化
// と壁誤差(sen.error_p)の変化がどちらもしきい値以内でticks回連続。壁との
// 距離が一定でヘディングも一定なら機体は壁と平行(=迷路座標で向き0)と
// みなせるので、その瞬間にジャイロ積分系の基準をゼロへ揃える。
// ・ego_in.ang/ang_kf: angle_pid(img_ang+duty_sen-ang_kf)の実測側。ここに
//   残っていた偽の角度(20260906_034820.csvでは-1.9°)が、angle_pidと壁PDが
//   互いに打ち消し合う均衡(横に約1mmずれて走る)の原因だった。
// ・kim.theta: 2D Kanayama(SLALOM/SLA_BACK_STR)とang.i_bias(kc_gain,
//   turn_angle_fb.w_gain)の実測側。既存の壁検出スナップ(上のwall_found)
//   はang側しか触らないため、両者が食い違うのを避ける意味でも揃える。
// ・積分(w.error_i/ang.error_i/sen.error_i等): 偽の角度を打ち消すために
//   育っていた分をclear_dist再アンカー(check_sen_error())と同様に捨てる。
// 発火は走行あたり一度(motor_enの立ち上がりで再武装)。hold中はsct=NONEで
// ここへ来ないため対象外。探索走行は既存のclear_dist再アンカーに任せる。
void ControlLaw::update_start_align(SensingControlType type) {
  if (!start_align_pending_ || search_mode_ ||
      param_->start_align.enable <= 0) {
    return;
  }
  const bool judging = tgt_val_->motion_type == MotionType::STRAIGHT &&
                       type == SensingControlType::Wall &&
                       !tgt_val_->hold_active && tgt_val_->ego_in.v > 10.0f;
  if (!judging) {
    start_align_cnt_ = 0;
    start_align_dist_ = 0;
    return;
  }
  const float ang = tgt_val_->ego_in.ang;
  const float err = ee->sen.error_p;
  // 壁から見た横位置[mm、左が正]。両方の45°が有効範囲なら(l45-r45)/2の
  // 符号反転(両壁の平均で共通モードの較正差・姿勢の影響を打ち消せる)、
  // 片壁ならsen.error_p(=2*(ref-d)、右寄りで正)の-1/2。片壁式はref(45)を
  // 中心とみなすため実際の中心((l+r)/2≒43.9)と1mm前後ずれるので、
  // アンカーと発火で方式が違うときは横位置の補正を行わない(向きだけ)。
  // 武装後に最初に壁を見たtickでアンカーを取り、発火時に「アンカーからの
  // 壁基準の横移動」でpos_y/kim.yを合わせる。
  const auto se = sensing_result_;
  const bool two_wall = (30.0f < se->ego.left45_dist && se->ego.left45_dist < 60.0f &&
                         30.0f < se->ego.right45_dist && se->ego.right45_dist < 60.0f);
  const float lat_now =
      two_wall ? -0.5f * (se->ego.left45_dist - se->ego.right45_dist)
               : -0.5f * err;
  if (!start_align_anchor_valid_) {
    start_align_anchor_valid_ = true;
    start_align_anchor_two_ = two_wall;
    start_align_anchor_lat_ = lat_now;
    start_align_anchor_pos_y_ = ego_->pos.get_state()[1];
    start_align_anchor_kim_x_ = ego_->kim.x;
    start_align_anchor_kim_y_ = ego_->kim.y;
  }
  const bool use_fit = param_->start_align.slope_th > 0;
  if (start_align_cnt_ == 0) {
    start_align_ang0_ = ang;
    start_align_err0_ = err;
    start_align_dist_ = 0;
    start_align_fit_sx_ = start_align_fit_sy_ = start_align_fit_sxx_ =
        start_align_fit_sxy_ = start_align_fit_syy_ = start_align_fit_sa_ = 0;
    start_align_fit_two_ = two_wall;
  }
  const float ang_th = param_->start_align.ang_th / 180.0f * M_PI;
  // err_abs_th: 壁PDがまだ大きく操舵している間(|err|大)は窓を進めない
  // (structs.hpp start_align_t::err_abs_thのコメント参照。20260908_031634.csv
  // idx177の極値発火対策)。
  const bool err_large = param_->start_align.err_abs_th > 0 &&
                         ABS(err) > param_->start_align.err_abs_th;
  // 回帰方式では壁誤差の「変化」(err_th)は勾配で評価するので見ない。代わりに
  // 窓内で両壁/片壁モードが変わったらやり直す(latの定義が変わるため)。
  const bool restart =
      err_large || ABS(ang - start_align_ang0_) > ang_th ||
      (use_fit ? (two_wall != start_align_fit_two_)
               : (ABS(err - start_align_err0_) > param_->start_align.err_th));
  if (restart) {
    // 窓をやり直す(次tickで現在値を窓の基準に取り直す)
    start_align_cnt_ = 0;
    start_align_dist_ = 0;
    return;
  }
  if (use_fit) {
    // x: 窓内走行距離、y: 回転による見かけの横移動(lat_k·Δang)を除いた壁横位置
    const float k = param_->start_align.lat_k * 180.0f / M_PI; // mm/rad
    const float x = start_align_dist_;
    const float y = lat_now - k * (ang - start_align_ang0_);
    start_align_fit_sx_ += x;
    start_align_fit_sy_ += y;
    start_align_fit_sxx_ += x * x;
    start_align_fit_sxy_ += x * y;
    start_align_fit_syy_ += y * y;
    start_align_fit_sa_ += (ang - start_align_ang0_);
  }
  start_align_cnt_++;
  start_align_dist_ += ABS(tgt_val_->ego_in.v) * dt_;
  if (start_align_cnt_ < param_->start_align.ticks ||
      start_align_dist_ < param_->start_align.dist_mm) {
    return;
  }

  // 発火時に置くヘディング[rad]。従来方式は0(壁と平行とみなす)、回帰方式は
  // 窓の勾配から求めた壁基準の実ヘディング。
  float h = 0.0f;
  if (use_fit) {
    const float n = (float)start_align_cnt_;
    const float sxx = start_align_fit_sxx_ - start_align_fit_sx_ * start_align_fit_sx_ / n;
    const float sxy = start_align_fit_sxy_ - start_align_fit_sx_ * start_align_fit_sy_ / n;
    const float syy = start_align_fit_syy_ - start_align_fit_sy_ * start_align_fit_sy_ / n;
    bool ok = false;
    if (sxx > 1.0f) {
      const float m = sxy / sxx;
      const float sse = syy - m * sxy;
      const float resid = sqrtf(std::max(sse, 0.0f) / n);
      const float slope = atanf(m); // 窓内の平均ヘディング(並進のみ)[rad]
      ok = ABS(slope) < param_->start_align.slope_th / 180.0f * M_PI &&
           resid < param_->start_align.resid_th;
      // ジャイロ融合: 現在の向き = 壁基準の窓平均 + ジャイロの窓平均からの
      // 偏差。壁PDのリミットサイクル(±0.5°、周期120mm前後@400mm/s)の途中で
      // 発火しても、その瞬間の向きを窓平均で置き換えてしまわない
      // (20260908_035048.csv idx233: 窓平均−0.48°に対し実際は+0.3°付近)。
      h = slope + (ang - start_align_ang0_) - start_align_fit_sa_ / n;
    }
    if (!ok) {
      // まだ横に動いている/直線に乗っていない。窓を伸ばして毎tick再評価し、
      // 2*dist_mmを超えたら取り直す(古い過渡を引きずらないため)。
      if (start_align_dist_ > 2.0f * param_->start_align.dist_mm) {
        start_align_cnt_ = 0;
        start_align_dist_ = 0;
      }
      return;
    }
  }

  // 再アンカー: 現在の向きを迷路座標のh(従来方式では0)とする。
  // 不感帯(structs.hpp start_align_t::apply_th): ジャイロとの差が小さければ
  // 向きはジャイロのまま(壁推定の雑音を持ち込まない)。横位置の合わせ込み・
  // アンカー更新・再武装は常に行う。
  const float ang_before = tgt_val_->ego_in.ang;
  const bool apply_heading =
      !(param_->start_align.apply_th > 0 &&
        ABS(h - ang_before) < param_->start_align.apply_th / 180.0f * M_PI);
  if (apply_heading) {
    tgt_val_->ego_in.ang = h;
    tgt_val_->global_pos.ang = h;
    sensing_result_->ego.ang_kf = h;
    sensing_result_->ego.ang_kf2 = h;
    ego_->kf_ang.offset(h - ang_before);
  }
  // 2026-09-08: 世界座標(pos)とkim.yの横位置を壁基準で合わせ直す。
  // 従来はang/kim.thetaだけ0に戻していたため、置いた向きのズレ(0.2〜0.7°)
  // がposのx軸の向きとして最後まで残り、壁PDで実機がまっすぐ走っていても
  // pos_yが右へ流れ続けていた(20260908_024346.csv: 707mmで実機の横移動
  // +1.5mmに対しpos_y -4.1mm、023014.csvは-14.7mm)。
  // 当初は誤差角で軌跡ごと原点まわりに回したが、誤差角はアンカーから徐々に
  // 育つ(ドリフト)ため「一定オフセット」を仮定する回転では過補正になり、
  // 再アンカーのたびにpos_yが+5mm等飛んだ(20260908_030521.csv idx1335)。
  // 横位置は壁センサーで直接測れるので、アンカー(前回発火/最初に壁を見た
  // tick)からの壁基準の横移動量にpos_y/kim.yを合わせ、向きは誤差角
  // (kim.theta: スナップされず旋回リベースも公称角なのでposと同じジャイロ
  // 誤差を持つ)だけ捨てる。x(進行方向)は触らない。制御にはposを使って
  // いないので走りは変わらない。
  const float d = apply_heading ? (h - ego_->kim.theta) : 0.0f;
  {
    const bool same_mode = (two_wall == start_align_anchor_two_);
    const auto st0 = ego_->pos.get_state();
    float dy_pos = 0.0f;
    if (same_mode) {
      const float dy_wall = lat_now - start_align_anchor_lat_;
      dy_pos = (start_align_anchor_pos_y_ + dy_wall) - st0[1];
      // kim.yはセグメント開始でリセットされる。アンカー以降にリセットが
      // 入っていなければ(kim.xが減っていなければ)同じ壁基準で合わせる。
      if (ego_->kim.x >= start_align_anchor_kim_x_) {
        ego_->kim.y = start_align_anchor_kim_y_ + dy_wall;
      }
    }
    ego_->pos.shift(0.0f, dy_pos, d);
    tgt_val_->ego_in.pos_y += dy_pos;
    const auto st = ego_->pos.get_state();
    sensing_result_->ego.pos_x = st[0];
    sensing_result_->ego.pos_y = st[1];
    sensing_result_->ego.pos_ang = st[2];
    start_align_anchor_two_ = two_wall;
    start_align_anchor_lat_ = lat_now;
    start_align_anchor_pos_y_ = st[1];
    start_align_anchor_kim_x_ = ego_->kim.x;
    start_align_anchor_kim_y_ = ego_->kim.y;
  }
  if (apply_heading) {
    ego_->kim.theta = h;
    ee->ang.error_i = ee->ang.error_d = ee->ang.error_dd = 0;
    ee->ang.i_slow = ee->ang.i_bias = 0;
    ee->w.error_i = ee->w.error_d = ee->w.error_dd = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = ee->w_kf.error_dd = 0;
    ee->sen.error_i = 0;
  }
  tgt_val_->start_align_count = tgt_val_->start_align_count + 1;
  tgt_val_->start_align_ang = ang_before;
  start_align_fire_dist_ = tgt_val_->global_pos.dist;

  start_align_pending_ = false;
  start_align_cnt_ = 0;
  start_align_dist_ = 0;
}

__attribute__((noinline, section(".time_critical.control_law"))) float
ControlLaw::calc_sensor_pid_dia() {
  float duty = 0;
  SensingControlType type = SensingControlType::None;
  ee->sen_dia.error_d = ee->sen_dia.error_p;
  ee->sen_dia.error_p = check_sen_error_dia(type);
  // 90度センサー(left90_mid/right90_mid)は斜め走行中はほぼ正面(柱の角)を
  // 向いており、真横を向く45度センサーのような線形な横偏差計測にはなって
  // いない(2026-08-24、ユーザー指摘)。柱近傍で幾何的に歪んだ値が出ても
  // 暴走しないよう誤差の絶対値をth(mm)でクランプする。th<=0なら無効
  // (要実機チューニング、まずは控えめな値から)。
  if (param_->sensor_pid_dia.th > 0) {
    ee->sen_dia.error_p = std::clamp(
        ee->sen_dia.error_p, -param_->sensor_pid_dia.th, param_->sensor_pid_dia.th);
  }
  ee->sen_dia.error_d = ee->sen_dia.error_p - ee->sen_dia.error_d;

  if (type == SensingControlType::DiaPiller && ee->sen_dia.error_p != 0) {
    // I項は使わない(2026-08-24、ユーザー判断): 斜めは壁/柱までの距離が
    // 進行状況に応じて凸凹し定常値に収束しないため積分と相性が悪い。
    // D項もerror_d(距離差分)ではなくw_kf基準の残差角速度を使う: 同じ理由
    // でerror_dもノイズだらけになる。raw w_kfだとtrj_->w_cmd由来の計画的な
    // 旋回成分まで抑制対象に入ってしまうため、w_cmdを差し引いた残差
    // (=計画外の回転)だけを減衰対象にする。
    const float w_residual = sensing_result_->ego.w_kf - trj_->w_cmd;
    const float p_gain = param_->sensor_pid_dia.p * ee->sen_dia.error_p;
    const float d_gain = -param_->sensor_pid_dia.d * w_residual;
    duty = p_gain + d_gain;
    const float gain = 0.1f;
    set_ctrl_val(ee->s_val, ee->sen_dia.error_p * gain, 0, 0,
                 w_residual * gain, p_gain * gain, 0, 0, d_gain * gain, 0, 0);
  } else {
    duty = 0;
    set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  // kanayama_dia(2026-08-24): kanayama_straightとは別パラメータにする
  // (ユーザー方針)。ki(積分)はey(柱距離誤差)が進行状況で非単調に変化する
  // ため未実装のまま様子見。kyは積分と違い今の誤差にその場で比例するだけで
  // 蓄積して発散する経路がないため、クランプ済みのeyに対して追加(2026-08-24)。
  // ただしsensor_pid_dia.pも同じeyに反応する比例項(経路A、角度ループ経由)
  // のため、kyを足すと同じ誤差に対する二重のP相当になる。kanayama_straight.ky
  // (0.03)よりだいぶ小さい値から始めてduty_senの効きを削らない範囲で
  // 調整すること。
  //
  // 出力クランプ(2026-08-24追加): duty_senはsensor_deg_limitter_diaで
  // ±3°等に抑えられるが、sen_kanayama_dwは角速度ループへ直結しておりこの
  // クランプを経由しない。ey(柱距離誤差)は幾何的に大きく歪みth(mm)でしか
  // 抑えていないため、ky×eyがthに張り付いた状態が何十tickも続くと
  // 無制限のΔwオフセットが乗り続け、中心に収束せずbang-bang的に符号反転する
  // 不具合を実機で確認した(20260824_030238.csv、ky=0.01でs_pid_pが±15mm
  // (th上限)に60tick以上張り付き続けていた)。windup_deg(deg/s相当、未使用
  // フィールドを流用)でsen_kanayama_dw自体もクランプする。
  if (type == SensingControlType::DiaPiller && param_->kanayama_dia.enable) {
    const float ey = ee->sen_dia.error_p;
    const float e_theta =
        tgt_val_->ego_in.ang - sensing_result_->ego.kim_theta;
    const float ky_gain = param_->kanayama_dia.ky * ey;
    const float ktheta_gain = param_->kanayama_dia.k_theta * sinf(e_theta);
    sen_kanayama_dw = ky_gain + ktheta_gain;
    if (param_->kanayama_dia.windup_deg > 0) {
      const float dw_limit = param_->kanayama_dia.windup_deg / 180.0f * M_PI;
      sen_kanayama_dw = std::clamp(sen_kanayama_dw, -dw_limit, dw_limit);
    }
    // [ログ注意] kanayama_straightと同様、enable時はここでee->s_valを
    // ky/e_theta/Δw内訳で上書きする(duty_l/r等の実効値には影響しない)。
    set_ctrl_val(ee->s_val, ey, 0, 0, e_theta, ky_gain, 0, 0, ktheta_gain,
                 sen_kanayama_dw, sen_kanayama_dw);
  } else {
    sen_kanayama_dw = 0;
  }

  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::DiaPiller) {
    limit = sensor_->interp1d(param_->sensor_deg_limitter_v,
                              param_->sensor_deg_limitter_dia,
                              tgt_val_->ego_in.v, false);
  }
  // 直進側calc_sensor_pid()と同じ度→rad変換が抜けていたバグ修正(2026-08-24)。
  // sensor_deg_limitter_diaは度で設定されているが、この変換がないと事実上
  // ノーリミットになる(1.5〜6.5度のつもりが1.5〜6.5radになっていた、
  // 20260824_005119.csv index1208-1253でduty_sen=0.42まで到達して発覚)。
  limit = limit / 180.0f * M_PI;
  duty = std::clamp(duty, -limit, limit);
  return duty;
}

float ControlLaw::check_sen_error(SensingControlType &type) {
  const auto se = sensing_result_;
  const auto prm = param_;
  float error = 0;
  int check = 0;
  float dist_mod = (int)(tgt_val_->ego_in.dist / param_->dist_mod_num);
  float tmp_dist = tgt_val_->ego_in.dist - param_->dist_mod_num * dist_mod;

  bool expand_right = false;
  bool expand_left = false;

  auto exist_right45 = prm->sen_ref_p.normal.exist.right45;
  auto exist_left45 = prm->sen_ref_p.normal.exist.left45;

  auto wall_th =
      search_mode_
          ? sensor_->interp1d(param_->clear_dist_ragne_dist_list,
                              param_->clear_dist_ragne_th_list, tmp_dist, false)
          : std::min(exist_left45, exist_right45);

  auto exist_right45_expand = wall_th;
  auto exist_left45_expand = wall_th;

  float val_left = 1000;
  float val_right = 1000;

  bool range_check_right =
      (1 < se->ego.right45_dist) && (se->ego.right45_dist < exist_right45);
  bool range_check_left =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45);

  bool dist_check_right = ABS(tgt_val_->global_pos.dist -
                              right_keep.star_dist) > prm->right_keep_dist_th;
  bool dist_check_left = ABS(tgt_val_->global_pos.dist - left_keep.star_dist) >
                         prm->left_keep_dist_th;

  bool check_diff_right =
      ABS(se->ego.right45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_r;
  bool check_diff_left =
      ABS(se->ego.left45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_l;

  if (!search_mode_) {
    // 2026-09-05: 非探索側は速度が大きく変わるため、生の1tick差分ではなく
    // kireme_diff_v_ref で速度正規化した差分で判定する
    // (structs.hpp input_param_t::kireme_diff_v_ref のコメント参照)。
    // kireme_diff_v_ref==0 なら _norm は生の差分と同値なので従来と同じ挙動。
    // 探索側(上の kireme_r/l)は速度がほぼ一定で問題が起きず、しきい値も
    // 探索速度で調整済みのため生の差分のままにする。
    const float r45_diff = se->ego.right45_dist_diff_norm;
    const float l45_diff = se->ego.left45_dist_diff_norm;
    // 2026-09-06: 正規化しきい値にノイズ下限(kireme_diff_noise_th)を設ける。
    // 低速では正規化ゲイン(kireme_diff_v_ref/v、v=400で5.5倍)がセンサー
    // ノイズや通常操舵によるみかけの距離変化まで増幅し、kireme_*_wall_off
    // =0.25 が生の0.045mm/tick(量子化1LSB相当)まで締まっていた。その結果
    // 唯一の壁で check_diff が落ちると check_*_sensor_error() は check++ だけ
    // して誤差を足さないため error_p=0 → duty_sen/Δw が1tick抜け、角度目標が
    // 5°分跳んで角速度目標に約1rad/sのスパイクが数tickおきに入っていた
    // (20260906_025821.csv idx432-550)。|生差分| < min(しきい値, noise_th)
    // なら速度によらず壁ありとみなす。min()で挟むため v>=kireme_diff_v_ref
    // では従来と同一(structs.hpp kireme_diff_noise_th のコメント参照)。
    const float noise_th = prm->kireme_diff_noise_th;
    const float r45_raw = se->ego.right45_dist_diff;
    const float l45_raw = se->ego.left45_dist_diff;
    const auto kireme_ok = [noise_th](float raw, float norm, float th) {
      return ABS(raw) < std::min(th, noise_th) || ABS(norm) < th;
    };
    if (tgt_val_->motion_type == MotionType::WALL_OFF ||
        tgt_val_->motion_type == MotionType::SLA_FRONT_STR) {
      check_diff_right = kireme_ok(
          r45_raw, r45_diff,
          (r45_diff < 0) ? prm->sen_ref_p.normal.ref.kireme_r_wall_off2
                         : prm->sen_ref_p.normal.ref.kireme_r_wall_off);
      check_diff_left = kireme_ok(
          l45_raw, l45_diff,
          (l45_diff < 0) ? prm->sen_ref_p.normal.ref.kireme_l_wall_off2
                         : prm->sen_ref_p.normal.ref.kireme_l_wall_off);
    } else {
      check_diff_right = kireme_ok(r45_raw, r45_diff,
                                   prm->sen_ref_p.normal.ref.kireme_r_fast);
      check_diff_left = kireme_ok(l45_raw, l45_diff,
                                  prm->sen_ref_p.normal.ref.kireme_l_fast);
    }
  }

  bool check_front_left =
      (10 < se->ego.left90_mid_dist) &&
      (se->ego.left90_mid_dist < prm->sen_ref_p.normal.exist.front);
  bool check_front_right =
      (10 < se->ego.right90_mid_dist) &&
      (se->ego.right90_mid_dist < prm->sen_ref_p.normal.exist.front);

  if (!check_diff_right)
    enable_expand_right = false;
  if (!check_diff_left)
    enable_expand_left = false;

  if (search_mode_ && tgt_val_->tgt_in.tgt_dist > 80 &&
      tgt_val_->tgt_in.tgt_dist < 100 &&
      tgt_val_->motion_type == MotionType::STRAIGHT) {
    expand_right = (10 < se->ego.right45_dist) &&
                   (se->ego.right45_dist < prm->sen_ref_p.search_exist.right45);
    expand_left = (10 < se->ego.left45_dist) &&
                  (se->ego.left45_dist < prm->sen_ref_p.search_exist.left45);
  } else {
    if (enable_expand_right) {
      exist_right45_expand = wall_th + 1.5f;
      expand_right = (10 < se->ego.right45_dist) &&
                     (se->ego.right45_dist < exist_right45_expand);
    } else {
      exist_right45_expand = 0;
    }
    if (enable_expand_left) {
      exist_left45_expand = wall_th + 1.5f;
      expand_left = (10 < se->ego.left45_dist) &&
                    (se->ego.left45_dist < exist_left45_expand);
    } else {
      exist_left45_expand = 0;
    }
  }

  bool range_check_right_expand = (1 < se->ego.right45_dist) &&
                                  (se->ego.right45_dist < exist_right45_expand);
  bool range_check_left_expand =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45_expand);

  if (!(check_front_left && check_front_right)) {
    const bool is_wall_off_mode =
        (tgt_val_->motion_type == MotionType::WALL_OFF);
    if (!is_wall_off_mode) {
      check_left_sensor_error(error, check, range_check_left, dist_check_left,
                              check_diff_left, expand_left,
                              range_check_left_expand);
      check_right_sensor_error(error, check, range_check_right,
                               dist_check_right, check_diff_right, expand_right,
                               range_check_right_expand);
    } else {
      if (tgt_val_->motion_dir == MotionDirection::LEFT) {
        check_right_sensor_error(error, check, range_check_right,
                                 dist_check_right, check_diff_right,
                                 expand_right, range_check_right_expand);
        check_left_sensor_error(error, check, range_check_left, dist_check_left,
                                check_diff_left, expand_left,
                                range_check_left_expand);
      } else {
        check_left_sensor_error(error, check, range_check_left, dist_check_left,
                                check_diff_left, expand_left,
                                range_check_left_expand);
        check_right_sensor_error(error, check, range_check_right,
                                 dist_check_right, check_diff_right,
                                 expand_right, range_check_right_expand);
      }
    }
    if (check != 0) {
      type = SensingControlType::Wall;
    }
  }

  if (enable_expand_left && !enable_expand_right && check_diff_right &&
      ((1 < se->ego.right45_dist) &&
       (se->ego.right45_dist < wall_th + 0.75f))) {
    enable_expand_right = true;
  } else if (!enable_expand_left && enable_expand_right && check_diff_left &&
             ((1 < se->ego.left45_dist) &&
              (se->ego.left45_dist < wall_th + 0.75f))) {
    enable_expand_left = true;
  }

  if (check == 0) {
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;

    bool right_check = false;
    bool left_check = false;

    const bool range_check_passed_right =
        (prm->sen_ref_p.normal2.ref.kireme_r < se->sen.r45.sensor_dist) &&
        (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.exist.right45) &&
        (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;
    const bool range_check_passed_left =
        (prm->sen_ref_p.normal2.ref.kireme_l < se->sen.l45.sensor_dist) &&
        (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.exist.left45) &&
        (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;

    const bool exist_right45_b =
        se->ego.right45_dist < prm->sen_ref_p.search_exist.right45;
    const bool exist_left45_b =
        se->ego.left45_dist < prm->sen_ref_p.search_exist.left45;

    if (!(check_front_left && check_front_right)) {
      if (range_check_passed_right && !exist_left45_b) {
        error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
        check++;
        right_check = true;
      }
      if (range_check_passed_left && !exist_right45_b) {
        error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
        check++;
        left_check = true;
      }

      if (check == 0) {
        const bool range_check_passed_right_near =
            (prm->sen_ref_p.normal2.ref.right90 < se->sen.r45.sensor_dist) &&
            (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.ref.kireme_r) &&
            (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;
        const bool range_check_passed_left_near =
            (prm->sen_ref_p.normal2.ref.left90 < se->sen.l45.sensor_dist) &&
            (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.ref.kireme_l) &&
            (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;
        if (!range_check_passed_left_near && range_check_passed_right_near &&
            !exist_left45_b) {
          error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
          check++;
        } else if (range_check_passed_left_near &&
                   !range_check_passed_right_near && !exist_right45_b) {
          error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
          check++;
        }
      }
      if (check != 0 && !(tgt_val_->motion_type == MotionType::SLA_FRONT_STR)) {
        type = SensingControlType::Piller;
      }
    }
  } else {
    if (tgt_val_->tgt_in.tgt_dist >= prm->clear_dist_order) {
      if (!(prm->clear_dist_ragne_from <= tmp_dist &&
            tmp_dist <= prm->clear_dist_ragne_to)) {
        if (std::abs(tgt_val_->ego_in.ang - tgt_val_->ego_in.img_ang) <
            prm->clear_angle) {
          if ((tgt_val_->tgt_in.tgt_dist - tgt_val_->ego_in.dist) >
              (prm->cell / 2)) {
            tgt_val_->global_pos.ang = tgt_val_->global_pos.img_ang;
            ee->w.error_i = ee->w.error_d = ee->w.error_dd = 0;
            ee->w_kf.error_i = ee->w_kf.error_d = ee->w_kf.error_dd = 0;
            ee->ang.error_i = ee->ang.error_d = ee->ang.error_dd = 0;
            ee->ang.i_slow = ee->ang.i_bias = 0;
            w_reset_ = 0;
          }
        }
      }
    }
  }

  if (check == 2)
    return error;
  if (check == 1)
    return error * 2;
  ee->sen.error_i = 0;
  ee->sen_log.gain_zz = 0;
  ee->sen_log.gain_z = 0;
  return 0;
}

float ControlLaw::check_sen_error_dia(SensingControlType &type) {
  float error = 0;
  int check = 0;
  const auto se = sensing_result_;

  if (tgt_val_->tgt_in.tgt_dist > param_->sen_ctrl_front_th &&
      (tgt_val_->tgt_in.tgt_dist - tgt_val_->ego_in.dist) >
          param_->sen_ctrl_front_diff_th) {
    const bool valid_right90 =
        1 < se->ego.right90_mid_dist &&
        se->ego.right90_mid_dist < param_->sen_ref_p.dia.exist.right90;
    const bool valid_left90 =
        1 < se->ego.left90_mid_dist &&
        se->ego.left90_mid_dist < param_->sen_ref_p.dia.exist.left90;
    const bool valid_right45 =
        1 < se->sen.r45.sensor_dist &&
        se->sen.r45.sensor_dist < param_->sen_ref_p.dia.exist.right45 &&
        se->sen.r45.sensor_dist < se->ego.right45_dist;
    const bool valid_left45 =
        1 < se->sen.l45.sensor_dist &&
        se->sen.l45.sensor_dist < param_->sen_ref_p.dia.exist.left45 &&
        se->sen.l45.sensor_dist < se->ego.left45_dist;

    if (valid_right90) {
      error += param_->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val_->dia_state.right_old =
          param_->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val_->dia_state.right_save = true;
      tgt_val_->dia_state.left_save = false;
      check++;
    }
    if (valid_left90) {
      error -= param_->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val_->dia_state.left_old =
          param_->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val_->dia_state.left_save = true;
      tgt_val_->dia_state.right_save = false;
      check++;
    }
    if (!valid_left90 && !valid_right90 && param_->sensor_gain.front4.a != 0) {
      if (valid_right45) {
        error += param_->sen_ref_p.dia.ref.right45 - se->sen.r45.sensor_dist;
        check++;
      }
      if (valid_left45) {
        error -= param_->sen_ref_p.dia.ref.left45 - se->sen.l45.sensor_dist;
        check++;
      }
      if (!(valid_left45 && valid_right45)) {
        if (tgt_val_->dia_state.right_save) {
          error += tgt_val_->dia_state.right_old;
          check++;
        }
        if (tgt_val_->dia_state.left_save) {
          error -= tgt_val_->dia_state.left_old;
          check++;
        }
      }
    }
  }

  if (check == 0) {
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else {
    type = SensingControlType::DiaPiller;
    if (tgt_val_->tgt_in.tgt_dist >= param_->clear_dist_order) {
      if (std::abs(tgt_val_->ego_in.ang - tgt_val_->ego_in.img_ang) <
          param_->clear_angle) {
        // reserved
      }
    }
  }

  if (check == 2)
    return error;
  if (check == 1)
    return error * 2;
  return 0;
}

void ControlLaw::check_fail_safe() {
  if (!motor_en_) {
    tgt_val_->fss.error = 0;
    return;
  }
  if (ABS(ee->ang.error_p) > param_->fail_check_ang_th) {
    fail_check_ang++;
  } else {
    fail_check_ang = 0;
  }
  if (tgt_val_->motion_type == MotionType::WALL_OFF ||
      tgt_val_->motion_type == MotionType::WALL_OFF_DIA) {
    keep_wall_off_cnt++;
  } else {
    keep_wall_off_cnt = 0;
  }
  if (ABS(ee->v.error_i) > param_->fail_check.v)
    tgt_val_->fss.error = 1;
  if (ABS(ee->w.error_i) > param_->fail_check.w)
    tgt_val_->fss.error = 1;
  if (ABS(ee->ang.error_i) > param_->fail_check.ang)
    tgt_val_->fss.error = 1;
  if (keep_wall_off_cnt > param_->fail_check.wall_off)
    tgt_val_->fss.error = 1;
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc_pid_val() {
  ee->v.error_dd = ee->v.error_d;
  ee->v_kf.error_dd = ee->v_kf.error_d;
  ee->dist.error_dd = ee->dist.error_d;

  ee->v.error_d = ee->v.error_p;
  ee->v_r.error_d = ee->v_r.error_p;
  ee->v_l.error_d = ee->v_l.error_p;
  ee->v_kf.error_d = ee->v_kf.error_p;
  ee->dist.error_d = ee->dist.error_p;

  ee->v.error_p = trj_->v_cmd - sensing_result_->ego.v_c;
  ee->v_r.error_p = trj_->ideal_v_r - sensing_result_->ego.v_r;
  ee->v_l.error_p = trj_->ideal_v_l - sensing_result_->ego.v_l;
  ee->v_kf.error_p = trj_->v_cmd - sensing_result_->ego.v_kf;
  ee->w_kf.error_p = trj_->w_cmd - sensing_result_->ego.w_kf;

  ee->dist.error_p = tgt_val_->global_pos.img_dist - tgt_val_->global_pos.dist;
  if (ee->dist.error_p > param_->front_ctrl_error_th) {
    ee->dist.error_p = param_->front_ctrl_error_th;
  } else if (ee->dist.error_p < -param_->front_ctrl_error_th) {
    ee->dist.error_p = -param_->front_ctrl_error_th;
  }

  ee->v_kf.error_d = ee->v_kf.error_p - ee->v_kf.error_d;
  ee->v.error_d = ee->v.error_p - ee->v.error_d;
  ee->dist.error_d = ee->dist.error_p - ee->dist.error_d;
  ee->v_l.error_d = ee->v_l.error_p - ee->v_l.error_d;
  ee->v_r.error_d = ee->v_r.error_p - ee->v_r.error_d;

  ee->v_kf.error_dd = ee->v_kf.error_d - ee->v_kf.error_dd;
  ee->v.error_dd = ee->v.error_d - ee->v.error_dd;
  ee->dist.error_dd = ee->dist.error_d - ee->dist.error_dd;

  // 2026-09-05: hold_active(MotionPlanning::hold())中はv.error_iを積まない。
  // motor_pid2.iは高速走行の速度追従用にチューニングされたゲインで、hold中
  // (v_cmd=0で静止保持するだけ)には本来不要。この積分はdtスケーリング無しで
  // 毎tick無条件加算され、motion_type==NONE/FRONT_CTRL以外ではreset_pid_val()
  // でも一切リセットされないため、STRAIGHT→WALL_OFF→SLA_FRONT_STR→SLALOM→
  // SLA_BACK_STRと一連の走行内では区間をまたいでずっと生き続ける
  // (実測: hold中だけで-92まで、旋回中は-1527まで到達、20260905_225826.csv)。
  // motor_pid2.antiwindupのヒステリシス条件(符号反転+しきい値超え)は、hold中
  // のように小さい誤差が同符号で単調に積み続けるケースを検知できず無力
  // だった。hold_active中はここで積算自体を止め、hold終了時にv.error_iが
  // ほぼゼロの状態で実走行へ引き継がれるようにする。
  if (tgt_val_->hold_active) {
    ee->v.error_i = 0;
  } else {
    ee->v.error_i += ee->v.error_p;
  }
  if (tgt_val_->motion_type != MotionType::FRONT_CTRL) {
    ee->dist.error_i += ee->dist.error_p;
  }
  ee->v_l.error_i += ee->v_l.error_p;
  ee->v_r.error_i += ee->v_r.error_p;

  tgt_val_->v_error = ee->v.error_i;
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc_pid_val_ang() {
  const auto tgt = tgt_val_;
  const auto se = sensing_result_;

  ee->ang.error_dd = ee->ang.error_d;
  ee->ang.error_d = ee->ang.error_p;

  float offset = 0;
  if (tgt->motion_type != MotionType::FRONT_CTRL) {
    offset += duty_sen;
  }

  ee->ang.error_p = (tgt->ego_in.img_ang + offset) - se->ego.ang_kf;
  ee->ang.error_d = ee->ang.error_p - ee->ang.error_d;
  ee->ang.error_dd = ee->ang.error_d - ee->ang.error_dd;
  ee->ang.error_i += ee->ang.error_p;

  if (!(tgt->motion_type == MotionType::STRAIGHT ||
      tgt->motion_type == MotionType::SLA_FRONT_STR ||
      tgt->motion_type == MotionType::SLA_BACK_STR ||
      tgt->motion_type == MotionType::WALL_OFF ||
      tgt->motion_type == MotionType::WALL_OFF_DIA)) {
    ee->ang.error_d = ee->ang.error_dd = ee->ang.error_i = 0;
  }

  float ang_error_i = ee->ang.error_i;
  if (param_->angle_pid.antiwindup) {
    if (ang_error_i * ee->ang.error_p < 0 &&
        ABS(ee->ang.error_p) > param_->angle_pid.windup_dead_bind) {
      ang_error_i *= param_->angle_pid.windup_gain;
    }
  }

  duty_roll_ang = param_->angle_pid.p * ee->ang.error_p +
                  param_->angle_pid.i * ang_error_i +
                  param_->angle_pid.d * ee->ang.error_d;

  calc_angle_i_bias();

  set_ctrl_val(ee->ang_val, ee->ang.error_p, ee->ang.error_i, duty_sen,
               ee->ang.error_d, param_->angle_pid.p * ee->ang.error_p,
               param_->angle_pid.i * ang_error_i,
               param_->gyro_pid.c * ee->ang.i_bias,
               param_->angle_pid.d * ee->ang.error_d, 0, 0);
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc_pid_val_ang_vel() {
  const auto tgt = tgt_val_;
  const auto se = sensing_result_;

  ee->w.error_dd = ee->w.error_d;
  ee->w_kf.error_dd = ee->w_kf.error_d;
  ee->w.error_d = ee->w.error_p;
  ee->w_kf.error_d = ee->w_kf.error_p;

  float offset = 0;
  ee->aw_log.dbg_off_ang = 0;
  if (param_->torque_mode == 2) {
    // 2026-09-05: hold_active(MotionPlanning::hold())中はangle_pid由来の
    // duty_roll_angをw_cmdへ足さない。calc_pid_val_ang()のang.error_iリセット
    // 条件を括弧修正した結果、STRAIGHT中(=hold含む)はang.error_iがリセット
    // されず持続するようになったため、holdの約2.45秒間ずっと積分され続けて
    // しまう。holdには専用のkc_gain(hold_ang_gain)+hold_kc_i_gain
    // (hold_ang_i_gain)という別経路の角度補正が既にあり、angle_pid由来の
    // オフセットが同時に効くと二重に競合する(実機で悪化を確認)。
    if (!(tgt->motion_type == MotionType::PIVOT ||
          tgt->motion_type == MotionType::FRONT_CTRL) &&
        !tgt_val_->hold_active) {
      offset += duty_roll_ang;
      ee->aw_log.dbg_off_ang = duty_roll_ang; // デバッグ用一時フィールド
    }
  }
  offset += sen_kanayama_dw;
  ee->aw_log.dbg_off_kny = sen_kanayama_dw; // デバッグ用一時フィールド

  // turn_angle_fb.w_gain(2026-08-23追加): ee->ang.i_biasをw目標offsetへ
  // 直接加算する(sen_kanayama_dwと同じ経路)。duty_rollへ直接足すgain/
  // gain_i/gain_dと違い、既存gyro_pid.bの積分(w_error_i)と戦わない
  // (calc_angle_velocity_ctrl()側のturn_angle_fb_gainコメント参照)。
  ee->aw_log.dbg_off_wgain = 0;
  if (param_->turn_angle_fb.enable &&
      angle_i_bias_active(tgt->motion_type)) {
    ee->aw_log.dbg_off_wgain = // デバッグ用一時フィールド
        param_->turn_angle_fb.w_gain * ee->ang.i_bias;
    offset += ee->aw_log.dbg_off_wgain;
  }
  // trj_->w_cmd(2026-08-23、常時Kanayama化): kanayama.enable時はego_in.wの
  // 代わりにcalc_kanayama()が2D姿勢誤差(dx/dy/e_theta)から補正したw_cmdを
  // 使う。kanayama無効時はcalc_kanayama()内でw_cmd=ego_in.wにフォール
  // バックされるため、この置き換えは無効時の挙動を一切変えない
  // (trajectory_generator.cpp calc_kanayama()参照)。
  ee->aw_log.duty_roll_before = (trj_->w_cmd + offset);

  ee->w.error_p = (trj_->w_cmd + offset) - se->ego.w_lp;
  ee->w_kf.error_p = (trj_->w_cmd + offset) - se->ego.w_kf;

  ee->w.error_d = ee->w.error_p - ee->w.error_d;
  ee->w_kf.error_d = ee->w_kf.error_p - ee->w_kf.error_d;

  ee->w.error_dd = ee->w.error_d - ee->w.error_dd;
  ee->w_kf.error_dd = ee->w_kf.error_d - ee->w_kf.error_dd;

  // conditional integration は v=2200 検証で有意な悪化(振動増大)が確認されたため無効化。
  // sat_roll_dirはduty_r/duty_lどちらが張り付いたかだけを見ており、並進(duty_c)由来か
  // ヨー(duty_roll)由来かを区別できていないため、並進飽和時にもヨーI項の必要な積算まで
  // 止めてしまっていた可能性が高い(control_law.cpp:apply_duty_limitter()参照)。
  // sat_roll_dir自体の計算・ログは残し、I項への反映のみ止める。
  ee->w.error_i += ee->w.error_p;
  ee->w_kf.error_i += ee->w_kf.error_p;

  tgt_val_->w_error = ee->w.error_i;
}

void ControlLaw::calc_pid_val_front_ctrl() {
  const auto se = sensing_result_;
  if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->w.error_i = ee->w.error_d = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    if (se->ego.front_dist < param_->cell) {
      ee->dist.error_p =
          se->ego.front_dist - param_->sen_ref_p.search_exist.front_ctrl;
      ee->ang.error_p = (se->ego.right90_dist - se->ego.left90_dist) / 2 -
                        param_->sen_ref_p.search_exist.kireme_r;
      ee->dist.error_i += ee->dist.error_p;
    } else {
      ee->dist.error_p = ee->dist.error_i = ee->dist.error_d = 0;
      ee->ang.error_p = ee->ang.error_i = ee->ang.error_d = 0;
    }
  }
}

void ControlLaw::reset_pid_val() {
  if (tgt_val_->motion_type == MotionType::FRONT_CTRL || !motor_en_ ||
      tgt_val_->motion_type == MotionType::NONE) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->v_kf.error_i = ee->v_kf.error_d = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = 0;
    ee->v_log.gain_z = ee->v_log.gain_zz = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l_log.gain_z = ee->v_l_log.gain_zz = 0;
    ee->v_r_log.gain_z = ee->v_r_log.gain_zz = 0;
    ee->sen.error_i = ee->sen.error_d = 0;
    ee->sen_log.gain_zz = ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = ee->sen_dia.error_d = 0;
    ee->sen_log_dia.gain_zz = ee->sen_log_dia.gain_z = 0;
    ee->aw_log.duty_roll_before = ee->aw_log.duty_roll = 0;
    mpc_u_prev = mpc_d_estimated = 0;
    ee->aw_log.mpc_d_estimated = 0;
  }
  ee->v_val.p = ee->v_val.i = ee->v_val.d = 0;
  ee->w_val.p = ee->w_val.i = ee->w_val.d = 0;
  ee->v_val.p_val = ee->v_val.i_val = ee->v_val.d_val = 0;
  ee->w_val.p_val = ee->w_val.i_val = ee->w_val.d_val = 0;
  ee->v_val.z = ee->v_val.zz = 0;
  ee->w_val.z = ee->w_val.zz = 0;
}

// ee->ang.i_bias(=img_ang-kim.theta、実測基準の姿勢誤差)を計算する対象
// motion_typeか。PIVOT系/BACK_STRAIGHT/READY/FRONT_CTRLは基準となる
// img_ang自体の意味が異なる(またはこの区間で姿勢保持が不要)ため除外。
bool ControlLaw::angle_i_bias_active(MotionType mt) const {
  if (search_mode_) return false;
  return !(mt == MotionType::NONE || mt == MotionType::PIVOT ||
           mt == MotionType::PIVOT_PRE || mt == MotionType::PIVOT_PRE2 ||
           mt == MotionType::PIVOT_AFTER || mt == MotionType::PIVOT_OFFSET ||
           mt == MotionType::BACK_STRAIGHT || mt == MotionType::READY ||
           mt == MotionType::FRONT_CTRL);
}

void ControlLaw::calc_angle_i_bias() {
  if (angle_i_bias_active(tgt_val_->motion_type)) {
    ee->ang.i_bias = tgt_val_->ego_in.img_ang - ego_->kim.theta;
  } else {
    ee->ang.i_bias = 0;
  }
}

void ControlLaw::check_left_sensor_error(float &error, int &check,
                                         bool range_check_left,
                                         bool dist_check_left,
                                         bool check_diff_left, bool expand_left,
                                         bool range_check_left_expand) {
  const auto se = sensing_result_;
  const auto prm = param_;
  const bool is_wall_off_mode = (tgt_val_->motion_type == MotionType::WALL_OFF);

  if (is_wall_off_mode && error != 0) {
    expand_left = range_check_left_expand = false;
  }

  if (range_check_left) {
    if (dist_check_left && check_diff_left) {
      enable_expand_left = true;
      error -= prm->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    } else if (expand_left && range_check_left_expand && dist_check_left &&
               check_diff_left) {
      enable_expand_left = true;
      error -= param_->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    }
    check++;
  } else if (expand_left && range_check_left_expand) {
    if (dist_check_left && check_diff_left) {
      enable_expand_left = true;
      error -= param_->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
    }
    check++;
  } else {
    left_keep.star_dist = tgt_val_->global_pos.dist;
  }
}

void ControlLaw::check_right_sensor_error(
    float &error, int &check, bool range_check_right, bool dist_check_right,
    bool check_diff_right, bool expand_right, bool range_check_right_expand) {
  const auto se = sensing_result_;
  const auto prm = param_;
  const bool is_wall_off_mode = (tgt_val_->motion_type == MotionType::WALL_OFF);

  if (is_wall_off_mode && error != 0) {
    expand_right = range_check_right_expand = false;
  }

  if (range_check_right) {
    if (dist_check_right && check_diff_right) {
      enable_expand_right = true;
      error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
    } else if (expand_right && range_check_right_expand && dist_check_right &&
               check_diff_right) {
      enable_expand_right = true;
      error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
    }
    check++;
  } else if (expand_right && range_check_right_expand) {
    if (dist_check_right && check_diff_right) {
      error += param_->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      enable_expand_right = true;
    }
    check++;
  } else {
    right_keep.star_dist = tgt_val_->global_pos.dist;
    enable_expand_right = false;
  }
}

void ControlLaw::set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                              float error_i2, float error_d, float val_p,
                              float val_i, float val_i2, float val_d, float zz,
                              float z) {
  val.p = error_p;
  val.i = error_i;
  val.i2 = error_i2;
  val.d = error_d;
  val.p_val = val_p;
  val.i_val = val_i;
  val.i2_val = val_i2;
  val.d_val = val_d;
  val.zz = zz;
  val.z = z;
}

void ControlLaw::calc_front_ctrl_duty() {
  const unsigned char reset = 0;
  param_->motor_pid.i = param_->motor_pid.d = 0;
  vel_pid.step(&ee->v.error_p, &param_->motor_pid.p, &param_->motor_pid.i,
               &param_->motor_pid.d, &reset, &dt_, &duty_c);
  set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
               param_->motor_pid.p * ee->v.error_p,
               vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0, 0);

  ee->w.error_i = ee->w.error_d = 0;
  ee->w_kf.error_i = ee->w_kf.error_d = 0;
  ee->w_log.gain_z = ee->w_log.gain_zz = 0;

  auto diff_ang = 0.0f;
  auto kp_gain = param_->front_ctrl_roll_pid.p * ee->w.error_p;
  auto ki_gain = 0.0f;
  auto kb_gain = 0.0f;
  auto kc_gain = 0.0f;
  auto kd_gain = param_->front_ctrl_roll_pid.d * ee->w_kf.error_d;

  limitter(kp_gain, ki_gain, kb_gain, kd_gain, param_->gyro_pid_gain_limitter);
  duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
              (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt_;

  ee->ang_log.gain_zz = ee->ang_log.gain_z;
  ee->ang_log.gain_z = duty_roll;

  set_ctrl_val(
      ee->w_val, ee->w.error_p, diff_ang, ee->w.error_i, ee->w_kf.error_d,
      param_->gyro_pid.p * ee->w.error_p, param_->gyro_pid.i * diff_ang,
      param_->gyro_pid.b * ee->w.error_i, param_->gyro_pid.d * ee->w_kf.error_d,
      ee->ang_log.gain_zz, ee->ang_log.gain_z);

  sensing_result_->ego.duty.sen = 0;

  duty_front_ctrl_trans =
      param_->front_ctrl_dist_pid.p * ee->dist.error_p +
      param_->front_ctrl_dist_pid.i * ee->dist.error_i +
      param_->front_ctrl_dist_pid.d * sensing_result_->ego.v_c;
  duty_front_ctrl_roll = param_->front_ctrl_angle_pid.p * ee->ang.error_p +
                         param_->front_ctrl_angle_pid.i * ee->ang.error_i +
                         param_->front_ctrl_angle_pid.d * ee->w_kf.error_p;
  duty_front_ctrl_roll_keep =
      param_->front_ctrl_keep_angle_pid.p * ee->ang.error_p +
      param_->front_ctrl_keep_angle_pid.i * ee->ang.error_i +
      param_->front_ctrl_keep_angle_pid.d * ee->w_kf.error_p;
  gyro_pid_windup_histerisis = false;
  gyro_pid_histerisis_i = 0.0f;
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::calc_angle_velocity_ctrl() {
  const auto se = sensing_result_;
  if (tgt_val_->motion_type != ee->ang_log.prev_motion_type) {
    const MotionType prev_mt = ee->ang_log.prev_motion_type;
    const bool was_turn = (prev_mt == MotionType::SLALOM ||
                          prev_mt == MotionType::SLA_BACK_STR);
    const bool now_turn = (tgt_val_->motion_type == MotionType::SLALOM ||
                          tgt_val_->motion_type == MotionType::SLA_BACK_STR);
    // turn_angle_fb直接duty注入(gain/gain_i/gain_d)はSLALOM/SLA_BACK_STR
    // 限定(2026-08-23、STRAIGHT等の定常保持はw_gain/offset経路に一本化した
    // ため、直接注入側の積分・D項状態はturn区間の出入りだけを見ればよい)。
    if (was_turn != now_turn) {
      // 旋回への突入、および旋回(SLALOM/SLA_BACK_STR)からの離脱で積分を
      // ゼロクリアする。離脱時にクリアしないと旋回中に貯めた「追いつく
      // ための蓄積値」がそのままSTRAIGHTへ持ち越され、既に旋回は終わって
      // いるのに古い積分値が余計な補正をかけ続けて発振の引き金になる
      // (2026-08-23、20260823_054151.csv: 旋回直後のSTRAIGHTでw_lp±6rad/s
      // 級の振動、idx820付近まで約250tick持続してから収束)。
      turn_angle_fb_integral_ = 0.0f;
    }
    if (now_turn) {
      // 旋回区間内でのmotion_type切り替え(SLALOM<->SLA_BACK_STR境界含む)
      // でD項の前回値を現在値に同期する。img_angはmotion_type境界で新
      // セグメントの基準に切り替わりi_bias(=img_ang-kim.theta)が不連続に
      // ジャンプするため、そのまま差分を取ると1tickだけ巨大な偽の変化量を
      // 拾ってしまう(2026-08-23、20260823_052617.csvでduty急変・大振動を
      // 確認して発覚)。
      turn_angle_fb_i_bias_prev_ = ee->ang.i_bias;
    }
    if (was_turn && !now_turn) {
      // 旋回終了: ヨーレートI項を0クリアする。v=2200でフェアなベースライン
      // (n=4, angOsc平均12.00°)比 angOsc平均8.22°への改善を確認済み
      // (2026-08-20)。ヒステリシス脱出側(下記)と同様にee->ang.error_p/dt_
      // による再着火も試したが(turn_log/12, n=4)、angOsc平均9.52°・
      // stdev3.23へ悪化(turn_log/11の平均7.49°・stdev1.13比)し、実機でも
      // 走行軌跡の不安定化が確認された(2026-08-21)ため0クリアに差し戻し。
      // ヒステリシス脱出はw.error_pがデッドバンドを下回った瞬間にのみ発火
      // するため再着火時のerror_pは元々小さいことが保証されるが、旋回終了
      // はmotion_type遷移で無条件に発火し残差の大きさに歯止めがない。
      // error_p/dt_はdt_=0.001で1000倍されるため、わずかな残差でも
      // ヨーレートI項に大きなステップを注入してしまい不安定化した。
      ee->w.error_i = 0.0f;
      gyro_pid_windup_histerisis = false;
      gyro_pid_histerisis_i = 0.0f;
    }
    ee->ang_log.omega_ref_prev = tgt_val_->ego_in.w;
    ee->ang_log.prev_motion_type = tgt_val_->motion_type;
  }

  if (tgt_val_->motion_type == MotionType::NONE) {
    duty_roll = param_->gyro_pid.p * ee->w.error_p +
                param_->gyro_pid.b * ee->w.error_i +
                param_->gyro_pid.c * ee->w.error_d;
    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;
    set_ctrl_val(ee->w_val, ee->w.error_p, ee->w.error_i, 0, ee->w.error_d,
                 param_->gyro_pid.p * ee->w.error_p,
                 param_->gyro_pid.b * ee->w.error_i, param_->gyro_pid.b * 0,
                 param_->gyro_pid.c * ee->w.error_d, ee->ang_log.gain_zz,
                 ee->ang_log.gain_z);
    gyro_pid_windup_histerisis = false;
    gyro_pid_histerisis_i = 0;
  } else {
    auto diff_ang = (tgt_val_->ego_in.img_ang - sensing_result_->ego.ang_kf);
    auto ang_sum = ee->ang.error_i;
    if (tgt_val_->motion_type == MotionType::SLALOM) {
      diff_ang = 0;
      ang_sum = 0;
    }
    auto w_error_i = ee->w.error_i;
    auto w_error_d = ee->w_kf.error_d;

    ee->aw_log.was_aw = (float)gyro_pid_windup_histerisis;
    ee->aw_log.w_i_base = w_error_i;

    if (param_->gyro_pid.antiwindup) {
      float db = param_->gyro_pid.windup_dead_bind;
      if (duty_sen != 0) {
        db *= param_->gyro_pid.windup_gain;
      }
      if (tgt_val_->motion_type == MotionType::SLALOM &&
          tgt_val_->tgt_in.v_max < 500) {
        db *= param_->gyro_pid.windup_gain;
      }
      // ヒステリシスの突入/脱出判定をそのまま毎tick反映すると、
      // (w_error_i*error_p<0)の判定がdeadband境界付近でノイズにより毎tick
      // 反転し、脱出のたびに再点火(下記、実質1000倍)が発火してw_error_iが
      // 巨大値と小さい値を交互に繰り返すチャタリングを起こす
      // (20260830_215101.csv解析、旋回直後のSTRAIGHTで毎tick g_i2が
      // -180台/+数十を往復し角度収束を乱していた)。判定が
      // kGyroPidWindupDebounceTicks回連続で一致するまで確定状態を更新せず、
      // 再点火も「確定状態がtrue→falseへ実際に変わったtick」のみで行う
      // (確定falseが続く間は毎tick再点火しない)ようエッジ検出する。
      constexpr int kGyroPidWindupDebounceTicks = 3;
      const bool want_histerisis =
          (w_error_i * ee->w.error_p < 0) &&
          ((ABS(ee->w.error_p) > db) ||
           (gyro_pid_windup_histerisis && ABS(ee->w.error_p) > db * 0.75f));
      if (want_histerisis == gyro_pid_windup_histerisis) {
        gyro_pid_windup_debounce_cnt_ = 0;
      } else {
        gyro_pid_windup_debounce_cnt_++;
      }

      const bool was_histerisis = gyro_pid_windup_histerisis;
      bool just_exited = false;
      if (gyro_pid_windup_debounce_cnt_ >= kGyroPidWindupDebounceTicks) {
        gyro_pid_windup_debounce_cnt_ = 0;
        gyro_pid_windup_histerisis = want_histerisis;
        just_exited = was_histerisis && !gyro_pid_windup_histerisis;
      }

      if (gyro_pid_windup_histerisis) {
        gyro_pid_histerisis_i += ee->w.error_p;
        w_error_i = gyro_pid_histerisis_i;
      } else if (just_exited) {
        // 2026-08-30: 当初のerror_p版を一旦元に戻す。同日中にi_bias版
        // ([[project-turn-control-tuning]]の前作踏襲)・kim.theta直接版も
        // 試したが、旋回終了時点の残差自体(SLA_BACK_STR突入時のkim.theta)
        // がrunごとに7.5〜24.4°と大きくばらつき、n=1のログ比較では
        // どの再点火元が優れているか切り分けられなかった。チャタリング
        // debounceは有効と確認済みなのでそれだけ残し、再点火の式自体は
        // 変更前のerror_pに戻して、n≥4の同一条件反復でまず旋回残差自体の
        // ばらつきとSTRAIGHT収束のばらつきを定量化してから再検討する。
        w_error_i = ee->w.error_i = ee->ang.error_p / dt_;
        gyro_pid_histerisis_i = 0;
      } else {
        gyro_pid_histerisis_i = 0;
      }

      ee->aw_log.w_error_i_raw = w_error_i;
      ee->aw_log.gyro_pid_histerisis_i = gyro_pid_histerisis_i;

      if (tgt_val_->motion_type == MotionType::SLALOM) {
        w_error_i =
            std::clamp(w_error_i * dt_, -ABS(tgt_val_->tgt_in.tgt_angle),
                       ABS(tgt_val_->tgt_in.tgt_angle)) /
            dt_;
      } else if (tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
        w_error_i = std::clamp(w_error_i * dt_, -ABS(last_tgt_angle_),
                               ABS(last_tgt_angle_)) /
                    dt_;
      }

      // 2026-09-04: 上のtgt_angle基準クランプは旋回角度分(例: 90°→dt_で
      // 割り戻すと約90000)まで許容する非常に緩い安全弁で、実際の暴走
      // (SLA_BACK_STR中にg_i2が20tick程度で-477→+517まで膨れ上がる)には
      // 全く効いていなかった。しかもこのケースはw_error_iとerror_pが
      // 同符号(=w_lpが同じ方向へ長時間乗り続ける単調な残差)で、上の
      // アンチワインド突入条件(w_error_i*error_p<0、符号反転=オーバー
      // シュート検知)ではそもそも捕捉できない種類の暴走だった。積算値
      // そのものへの実用的な絶対値クランプ(gyro_pid.windup_i_max、
      // 既存str_ang_pid_fastと同じ仕組み)を追加する。0なら無効。
      if (param_->gyro_pid.windup_i_max > 0) {
        w_error_i = std::clamp(w_error_i, -param_->gyro_pid.windup_i_max,
                               param_->gyro_pid.windup_i_max);
      }
      ee->aw_log.w_error_i_clamped = w_error_i;
    }

    if (!(tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
          tgt_val_->motion_type == MotionType::SLA_BACK_STR ||
          tgt_val_->motion_type == MotionType::PIVOT)) {
      diff_ang = 0;
      ang_sum = 0;
    }

    // 旋回終端ブレーキ(2026-08-23): SLALOM/SLA_BACK_STRでff_duty_rollが
    // ideal_wと共にゼロへ落ちた後、実測角速度(w_lp)が慣性で収束しきらず
    // 残留する問題への対策(20260823_032339.csv/20260823_032239.csvで確認、
    // SLA_BACK_STR突入後もw_lpが収束せず増大するケースあり)。gyro_pid.p/dは
    // FF主導設計を保つため極小(p=0.000325等)のままにし、計画角速度が
    // ほぼゼロ(=FFがもう仕事をしていない)かつ実残差が大きい間だけ、
    // 専用ゲインturn_end_brake.p/dに切り替えて能動的に残留回転を止める。
    //
    // [2026-08-23 修正] 当初SLALOMも対象に含めていたが、|ideal_w|<w_thは
    // 旋回終盤だけでなく旋回"開始"直後(idealwがまだ0から立ち上がる途中)にも
    // 成立してしまい、旋回入り口で誤爆して過大なduty(飽和→発振)を起こした
    // (20260823_033928.csvで確認)。SLA_BACK_STRは常にideal_w=0で立ち上がり
    // 局面が存在しないため、SLA_BACK_STRのみを対象にして誤爆を構造的に排除する。
    //
    // [単位に関する注意] duty_roll(=kp_gain+ki_gain+...の合計)はduty%では
    // なくトルクとして扱われ、summation_duty()で
    // torque*Resist/(Km*gear_a/gear_b)/battery*100 (≈4650倍、実測値から算出)
    // という変換を経てduty%になる(control_law.cpp:1305-1313)。通常の
    // gyro_pid.p=0.000325が極小なのはこの増幅を見込んだ値であり、
    // turn_end_brake.p/dも同じ増幅を受けることに注意(小さい値で十分効く)。
    const bool turn_end_brake_active =
        param_->turn_end_brake.enable &&
        tgt_val_->motion_type == MotionType::SLA_BACK_STR &&
        ABS(tgt_val_->ego_in.w) < param_->turn_end_brake.w_th;
    // [2026-08-23夜 修正] err_thによるON/OFFゲートを撤去。ゼロ交差の瞬間に
    // |error_p|が一瞬err_th未満へ落ちてbrakeがOFFになり、その間に誤差が
    // 育った状態で再点火すると強いゲインがフル飽和を起こし、二段目の
    // オーバーシュートを生んでいた(20260823_212145.csvで確認、idx142で
    // 一瞬OFF→idx143で誤差3.08まで育った状態にp=0.0025が掛かりduty即飽和
    // →w_lpが-34まで急落)。turn_angle_fb.i_w_gateで既に踏んだのと同じ
    // 「ハードゲート再点火」の踏み間違い([[project-kanayama-2d-bugfix-2026-08-23]]
    // 参照)。SLA_BACK_STRは短い過渡区間でerr_thが想定していた定常チャタ
    // リング対策は不要なため、motion_type+w_thのみでゲートする。

    auto kp_gain = (turn_end_brake_active ? param_->turn_end_brake.p
                                           : param_->gyro_pid.p) *
                   ee->w.error_p;
    auto ki_gain = param_->gyro_pid.i * diff_ang;
    auto kb_gain = param_->gyro_pid.b * w_error_i;
    // auto kb_gain = param_->gyro_pid.b * ee->w.error_i;
    auto kc_gain = param_->gyro_pid.c * ee->ang.i_bias;

    // 2026-09-05: hold_active(MotionPlanning::hold()実行中)限定の角度積分I項。
    // 通常のkc_gain(P)は吸引ファンの反動トルクのような定常外乱を完全には
    // 打ち消せず、kim_thetaが±0.2〜0.6°程度の定常偏差で頭打ちになることを
    // 実機で確認した(20260905_222607.csv: 吸引duty(duty_suction)が完全に
    // プラトーした後もkim_thetaが収束しなかった)。hold_active中だけ積分し、
    // それ以外(実走行中の通常STRAIGHT等)では毎tickゼロクリアして持ち越さない
    // (STRAIGHT区間はduty_roll側の他ループとの相互作用を避けるため対象外の
    // まま)。
    if (tgt_val_->hold_active) {
      // 2026-09-06: reset-on-move + duty上限(structs.hpp
      // hold_ang_i_reset_ang_thのコメント参照)。吸引プラトー後は吸引スカート
      // の摩擦で機体が固着し、差動duty±6〜8%では動かない(20260905_22xx-23xx
      // .csv、P=0.06でも4.0でも残留0.2〜0.8°)ため、I項は「固着を破るまで
      // ランプ→動いたら捨てる」スティクション破りとして働かせる。
      // 「動いた」は角度で判定する。当初|w_lp|>0.1rad/sで判定したが、固着中
      // でもファン振動でw_lpが±0.25rad/s揺れ、プラトーの78〜97%のtickで
      // リセットが掛かり積分が育たなかった(20260906_0439xx-0441xx.csv)。
      const float kim_theta_raw = ego_->kim.theta;
      if (!hold_active_prev_) {
        hold_kim_lp_ = kim_theta_raw;
        hold_i_ang_ref_ = kim_theta_raw;
        hold_ang_integral_ = 0.0f;
      }
      // 判定用LPF(structs.hpp hold_ang_i_reset_lp_msのコメント参照)。
      // 角度ジッタの大きいrunで生値判定が数tickおきにリセットを掛けて
      // I項が育たなかったため、揺れを均してから動き量を見る。
      if (param_->hold_ang_i_reset_lp_ms > 0) {
        const float alpha = std::min(
            1.0f, dt_ * 1000.0f / param_->hold_ang_i_reset_lp_ms);
        hold_kim_lp_ += (kim_theta_raw - hold_kim_lp_) * alpha;
      } else {
        hold_kim_lp_ = kim_theta_raw;
      }
      const float reset_th =
          param_->hold_ang_i_reset_ang_th / 180.0f * M_PI;
      if (reset_th > 0 && ABS(hold_kim_lp_ - hold_i_ang_ref_) > reset_th) {
        hold_ang_integral_ = 0.0f;
        hold_i_ang_ref_ = hold_kim_lp_;
      } else {
        hold_ang_integral_ += ee->ang.i_bias * dt_;
      }
      if (param_->hold_ang_i_max_duty > 0 && param_->hold_ang_i_gain > 0 &&
          param_->Resist > 0) {
        // duty%→トルク単位(summation_duty()のtorque_mode==2変換の逆)
        const float km_gear = param_->Km * (param_->gear_a / param_->gear_b);
        const float i_max_torque = param_->hold_ang_i_max_duty / 100.0f *
                                   se->ego.battery_lp * km_gear /
                                   param_->Resist;
        const float i_max = i_max_torque / param_->hold_ang_i_gain;
        hold_ang_integral_ = std::clamp(hold_ang_integral_, -i_max, i_max);
      }
    } else {
      hold_ang_integral_ = 0.0f;
    }
    hold_active_prev_ = tgt_val_->hold_active;
    const float hold_kc_i_gain = param_->hold_ang_i_gain * hold_ang_integral_;
    kc_gain += hold_kc_i_gain;

    // 2026-09-05: hold中はkc_gain(P、hold_ang_gain使用)とhold_kc_i_gain(I)が
    // duty_rollの主成分になるが、これまでどちらもログに出ておらず
    // (g_pid_*_vは速度ループ4項kp/ki/kb/kdのみを記録)、hold中にduty_l/rが
    // 大きく振れているのにg_pid_*_vが常にほぼ0に見える(ユーザー指摘)原因に
    // なっていた。ang_pid_p_v/i_vはSTRAIGHT区間ではturn_angle_fb系が使わず
    // 死んでいるため、ここへhold専用に配線して可視化する。
    if (tgt_val_->hold_active) {
      set_ctrl_val(ee->ang_val, ee->ang.i_bias, hold_ang_integral_, 0, 0,
                   kc_gain - hold_kc_i_gain, hold_kc_i_gain, 0, 0, 0, 0);
    }

    auto kd_gain = (turn_end_brake_active ? param_->turn_end_brake.d
                                           : param_->gyro_pid.d) *
                   w_error_d;
    limitter(kp_gain, ki_gain, kb_gain, kd_gain,
             param_->gyro_pid_gain_limitter);

    // 姿勢保持フィードバック(2026-08-23): ee->ang.i_bias(=img_ang-kim.theta)
    // 自体は正しい符号・大きさで実測とのズレを検出できている(20260823_040833.
    // csv解析: 旋回終盤で-2.7deg相当、目標-45degに対し実測-42.3degの不足と
    // 一致)が、既存kc_gain(=gyro_pid.c*i_bias、他モーションと共用)だけでは
    // 力不足。
    // [2026-08-23 修正] duty_rollへの直接加算(このgain/gain_i/gain_d)は
    // STRAIGHTを含む全angle_i_bias_active()区間で有効にしていたが、
    // duty_rollへ一定バイアスを足し続ける形は既存gyro_pid.bの積分
    // (w_error_i)から見ると「外乱」でしかなく、長時間続くSTRAIGHTでは
    // kb_gainがそれを正確に打ち消してしまい実質無効化していた
    // (20260823_062913.csv/062821.csv、kb_gainが逆算したturn_angle_fb出力と
    // ほぼ完全に相殺)。この直接加算はSLALOM/SLA_BACK_STRの速い過渡(kb_gainが
    // 反応する前の短時間)にのみ効かせ、STRAIGHT等の定常保持はw_gain
    // (calc_pid_val_ang_vel()のoffset経由、gyro_pid.bと戦わない経路)に
    // 一本化する(structs.hpp turn_angle_fb_t参照)。
    const bool turn_transient =
        tgt_val_->motion_type == MotionType::SLALOM ||
        tgt_val_->motion_type == MotionType::SLA_BACK_STR;
    float turn_angle_fb_gain = 0.0f;
    if (param_->turn_angle_fb.enable && turn_transient) {
      turn_angle_fb_gain = param_->turn_angle_fb.gain * ee->ang.i_bias;

      // ang.i_bias専用の積分(2026-08-23): 既存w_error_i(アンチワインド
      // ヒステリシス付き)を再利用したturn_w_pidは実機で発散した
      // (20260823_050137.csv、duty全飽和・kim_thetaオーバーシュート)。
      // ヒステリシスの離散的な切り替えが原因と見て、これとは無関係な
      // 単純クランプ付き積分をi_bias専用に新設する。
      // I蓄積ウェイト(2026-08-23): |実測w_lp|が大きいほど積分の蓄積速度を
      // 連続的に絞る(w_lp=0でweight=1、|w_lp|>=i_w_gateでweight=0への線形
      // ランプ)。旋回終端の残留角速度は同一config・同一FFでも実機ばらつきで
      // 2〜3倍変わり(20260823_054917.csv vs 054824.csv)、無条件蓄積だと
      // その過渡回転そのものに巻き込まれて育つ量がrun毎に違ってしまい、
      // オシレーション有無の分岐点になっていた。
      // [2026-08-23 修正] 当初on/offのハードゲートで実装したが
      // (20260823_060428.csv)、ゲートが開いた瞬間に凍結中に持ち越された
      // i_biasが一気にフル蓄積を再開してしまい、それ自体がステップ的な
      // 再点火となって2段目の発振を生んだ(アンチワインドヒステリシスの
      // 離散切り替えで発散したturn_w_pidと同じ落とし穴)。on/offではなく
      // 連続的な重み付けにすることで、この再点火時のステップを無くす。
      // i_w_gate=0なら従来通り常時フル蓄積。
      float i_weight = 1.0f;
      if (param_->turn_angle_fb.i_w_gate > 0) {
        i_weight = std::clamp(
            1.0f - ABS(se->ego.w_lp) / param_->turn_angle_fb.i_w_gate, 0.0f,
            1.0f);
      }
      turn_angle_fb_integral_ += ee->ang.i_bias * dt_ * i_weight;
      if (param_->turn_angle_fb.i_max > 0) {
        turn_angle_fb_integral_ = std::clamp(turn_angle_fb_integral_,
                                             -param_->turn_angle_fb.i_max,
                                             param_->turn_angle_fb.i_max);
      }
      turn_angle_fb_gain +=
          param_->turn_angle_fb.gain_i * turn_angle_fb_integral_;

      // D: ang.i_biasの1tick差分に掛けてduty_rollへ追加。angle_pid本来の
      // d=4.5相当の減衰役をここで担う(2026-08-23、angle_pidはgyro_pid.p
      // 経由の希釈により実質無効だったため、この直接加算経路に統合)。
      const float i_bias_d = ee->ang.i_bias - turn_angle_fb_i_bias_prev_;
      turn_angle_fb_i_bias_prev_ = ee->ang.i_bias;
      turn_angle_fb_gain += param_->turn_angle_fb.gain_d * i_bias_d;
    }

    duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
                turn_angle_fb_gain +
                (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt_;

    // turn_end_brakeスルーレート制限(2026-08-23夜): p/dだけではゼロ交差
    // 直後に数tickでduty_rollがフル反転し、グリップ音・kim_thetaの発散気味
    // な収束の原因になっていた(structs.hpp turn_end_brake_t参照)。
    // turn_end_brake_active中のみ、前tickからの変化量をslewでクランプする。
    if (turn_end_brake_active && param_->turn_end_brake.slew > 0) {
      const float max_step = param_->turn_end_brake.slew;
      duty_roll = std::clamp(duty_roll,
                              turn_end_brake_duty_prev_ - max_step,
                              turn_end_brake_duty_prev_ + max_step);
    }
    turn_end_brake_duty_prev_ = duty_roll;

    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;

    float dt = param_->dt;
    float b = param_->gyro_pid.mpc_b;
    float w_meas = -ee->w.error_p + tgt_val_->ego_in.w;

    float w_pred = mpc_w_prev + (mpc_u_prev + mpc_d_estimated) * b * dt;
    float observer_k = param_->gyro_pid.mpc_observer_k;
    mpc_d_estimated += observer_k * (w_meas - w_pred);
    mpc_w_prev = w_meas;
    ee->aw_log.mpc_d_estimated = mpc_d_estimated; // ログ確認用。制御出力へはまだ未結線

    if (param_->enable_mpc > 0 && tgt_val_->motion_type == MotionType::SLALOM) {
      // MPC override reserved
    }
    mpc_u_prev = duty_roll;

    set_ctrl_val(ee->w_val,     //
                 ee->w.error_p, //
                 diff_ang,      //
                 w_error_i,     //
                 w_error_d,     //
                 kp_gain,       //
                 ki_gain,       //
                 kb_gain,       //
                 kd_gain,       //
                 ee->ang_log.gain_zz, ee->ang_log.gain_z);
  }
}

__attribute__((noinline, section(".time_critical.control_law"))) void
ControlLaw::summation_duty() {
  auto ff_front = trj_->mpc_next_ego.ff_duty_front;
  auto ff_roll = trj_->mpc_next_ego.ff_duty_roll;
  const auto se = sensing_result_;

  // duty_rollは calc_angle_velocity_ctrl()/calc_front_ctrl_duty() で今tick分が
  // 既に確定している。ee->aw_log.duty_roll は従来 reset_pid_val() で 0 に
  // されるだけで実値が書き込まれない死んだログフィールドだったため、ここで
  // 実際に使われる値をミラーする。
  ee->aw_log.duty_roll = duty_roll;

  // torque_mode!=2 のときは実際に出力へ使われないため既定でゼロにしておく
  // (torque_mode==2 分岐でのみ実値を書き込む)
  se->ego.duty.ff_front_torque = 0;
  se->ego.duty.ff_roll_torque = 0;
  se->ego.duty.ff_friction_torque_r = 0;
  se->ego.duty.ff_friction_torque_l = 0;

  if (tgt_val_->motion_type == MotionType::WALL_OFF ||
      tgt_val_->motion_type == MotionType::WALL_OFF_DIA) {
    ff_front = param_->ff_roll_gain_before * ff_front;
    trj_->mpc_next_ego.ff_duty_front = ff_front;
  }
  if (tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
    ff_front = param_->ff_front_gain_14 * ff_front;
    trj_->mpc_next_ego.ff_duty_front = ff_front;
  }
  // 2026-08-30: ff_roll_gain_before/after/entryはこのff_roll(=mpc_next_ego.
  // ff_duty_roll)に掛けていたが、hardware.yamlのtorque_mode=2運用下では
  // ff_roll_torque(下のtorque_mode==2分岐)が実際の出力に使われ、ff_roll/
  // ff_duty_rollはFRONT_CTRL専用の未使用経路だった。そのためff_roll_gain_
  // entry/afterを0.75/0.5と振っても実機挙動が一切変わらなかった
  // (20260830_231723/232416/233252.csv、[[project_slalom_entry_overshoot_2026-08-30]]
  // 参照)。ゲイン係数を一度だけ計算し、torque_mode問わず両方の経路(ff_roll
  // とff_roll_torque由来のff_roll2)に適用するよう修正する。
  float ff_roll_gain_factor = 1.0f;
  if (tgt_val_->motion_type == MotionType::SLALOM) {
    if (tgt_val_->ego_in.sla_param.base_alpha > 0) {
      ff_roll_gain_factor = (tgt_val_->ego_in.alpha < 0)
                                 ? param_->ff_roll_gain_after
                                 : param_->ff_roll_gain_entry;
    } else if (tgt_val_->ego_in.sla_param.base_alpha < 0) {
      ff_roll_gain_factor = (tgt_val_->ego_in.alpha > 0)
                                 ? param_->ff_roll_gain_after
                                 : param_->ff_roll_gain_entry;
    }
  }
  ff_roll *= ff_roll_gain_factor;
  se->ego.duty.ff_duty_roll = trj_->mpc_next_ego.ff_duty_roll = ff_roll;
  auto ff_duty_r = ff_front + ff_roll + trj_->mpc_next_ego.ff_duty_rpm_r;
  auto ff_duty_l = ff_front - ff_roll + trj_->mpc_next_ego.ff_duty_rpm_l;

  if (param_->FF_keV == 0) {
    ff_duty_l = ff_duty_r = 0;
  }

  if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    tgt_duty.duty_r =
        (duty_c + duty_front_ctrl_trans + duty_roll + duty_front_ctrl_roll +
         duty_front_ctrl_roll_keep + ff_duty_r) /
        se->ego.battery_lp * 100;
    tgt_duty.duty_l =
        (duty_c + duty_front_ctrl_trans - duty_roll - duty_front_ctrl_roll -
         duty_front_ctrl_roll_keep + ff_duty_l) /
        se->ego.battery_lp * 100;
  } else if (param_->torque_mode == 2) {
    auto ff_front2 = trj_->mpc_next_ego.ff_front_torque;
    // ff_roll_torqueはmpc_tgt_calc.cpp(Simulinkコード生成、
    // ff_roll_torque = 0.5*Lm*alpha2*Resist)側で既にResistが掛かっている。
    // 下でtorque_r/torque_lごとResist/km_gearを掛けるため、ここで割らないと
    // ff_roll2だけResistが二重適用(実効Resist^2)になり、ff_front_torque
    // (Resist抜きで生成される)や他の項とスケールが揃わない(2026-08-23発見、
    // 旋回終端ブレーキの検討中に判明)。ここで一度割って二重適用を相殺する。
    auto ff_roll2 =
        ff_roll_gain_factor * trj_->mpc_next_ego.ff_roll_torque / param_->Resist;
    auto ff_duty_r2 = trj_->mpc_next_ego.ff_duty_rpm_r;
    auto ff_duty_l2 = trj_->mpc_next_ego.ff_duty_rpm_l;
    auto ff_friction_r = trj_->mpc_next_ego.ff_friction_torque_r;
    auto ff_friction_l = trj_->mpc_next_ego.ff_friction_torque_l;

    if (param_->FF_keV == 0) {
      ff_front2 = ff_roll2 = ff_duty_r2 = ff_duty_l2 = ff_friction_r =
          ff_friction_l = 0;
    } else {
      // mpc_tgt_calc.cpp(Simulink自動生成)のsign()実装が、入力がちょうど
      // 0.0fを跨ぐ瞬間だけ0を返す仕様のため、走行中でも数tickおきに
      // ff_friction_torque_r/lが瞬間的に0へ落ちるチャタリングを確認
      // (20260904_171508.csv)。直前値が非ゼロだったのに今回だけ厳密に
      // 0.0fになった場合は直前値を保持してこの瞬間的な落ち込みを吸収する。
      //
      // 2026-09-05: ただし「本当に0が正しい」ケースまで保持してしまうと、
      // 直前の非ゼロ値を永久にラッチしたままになる。実際の式は
      //   ff_front_torque      = Mass * accl * (tire/2)      (sign()を含まない)
      //   ff_friction_torque_x = sign(v_x)*coulomb + v_x*viscous
      // なので、それぞれ accl / ideal_v_x がゼロなら 0 が正解。
      // 保持対象を「元の入力が非ゼロなのに出力だけ0になった」場合に限定する。
      // 元の 20260904_171508.csv でも 0 に落ちていたのは ff_friction_torque_r
      // だけ(idx53)で、ff_front_torque が 0 だったのは accl==0 の定速巡航中
      // だったため。この誤保持で v=400mm/s 定速中に ff_front_torque=0.000527
      // (=+0.27V, duty約2%)が1627tick張り付き、I項が-0.34Vでそれを打ち消して
      // いた(20260905_045305.csv)。
      //
      // 2026-09-06: 上のガード全体が「並進速度ego_in.vが1.0を超える間だけ」
      // 有効という外側条件を持っていたが、SLALOM/PIVOTの旋回中は並進速度が
      // 一時的に1.0以下へ落ち込む一方、左右輪の個別速度(ideal_v_r/l)は
      // 依然として1.0を超えて回っている区間がある。この外側条件のせいで
      // 旋回のたびに保護が丸ごと無効化され、「周期的に(=旋回のたびに)
      // ff_torque/friction_torqueが0に落ちる」症状として現れていた
      // (t_1900.yamlでのfast-run計測で確認)。各項目は元々それぞれの
      // 入力(accl / ideal_v_r / ideal_v_l)自身の大きさで判定しているため、
      // 外側のvゲートを撤廃し各項目のガードだけで十分。
      if (ABS(tgt_val_->ego_in.accl) > 1.0f && ff_front2 == 0.0f &&
          ff_front_torque_prev_ != 0.0f) {
        ff_front2 = ff_front_torque_prev_;
      }
      if (ABS(trj_->ideal_v_r) > 1.0f && ff_friction_r == 0.0f &&
          ff_friction_torque_r_prev_ != 0.0f) {
        ff_friction_r = ff_friction_torque_r_prev_;
      }
      if (ABS(trj_->ideal_v_l) > 1.0f && ff_friction_l == 0.0f &&
          ff_friction_torque_l_prev_ != 0.0f) {
        ff_friction_l = ff_friction_torque_l_prev_;
      }
      // ff_roll_torque(=ff_roll2)はalpha2(mpc_tgt_calc.cpp内のMerge1[0]、
      // exp/pow系のプロファイル式)に比例する。2026-09-06、実機ログ
      // (t_1900.yaml実測、latest.csv)でSLALOM中にalphaが-2716〜-754等
      // 明確に非ゼロの区間でもff_roll_torqueが数tickおきに0へ落ちるのを
      // 確認。当初alpha2自身をゲート判定に使っていたが、alpha2は
      // ff_roll_torqueの入力そのものなので0に落ちた瞬間はゲートも同時に
      // 0になり判定が機能しない(自己参照)。front(accl)/friction(ideal_v)
      // と同様に「別系統の関連量」でゲートする必要があるため、同じ
      // sla_param由来だが別配列(Merge、time_step基準)で計算されalphaでは
      // このチャタリングが再現しないegoin.alphaをゲートに使う。
      if (ABS(tgt_val_->ego_in.alpha) > 1.0f && ff_roll2 == 0.0f &&
          ff_roll_torque_prev_ != 0.0f) {
        ff_roll2 = ff_roll_torque_prev_;
      }
    }
    ff_front_torque_prev_ = ff_front2;
    ff_friction_torque_r_prev_ = ff_friction_r;
    ff_friction_torque_l_prev_ = ff_friction_l;
    ff_roll_torque_prev_ = ff_roll2;
    se->ego.duty.ff_front_torque = ff_front2;
    se->ego.duty.ff_roll_torque = ff_roll2;
    se->ego.duty.ff_friction_torque_r = ff_friction_r;
    se->ego.duty.ff_friction_torque_l = ff_friction_l;

    float torque_r = ff_front2 + ff_roll2 + duty_c + duty_roll + ff_friction_r;
    float torque_l = ff_front2 - ff_roll2 + duty_c - duty_roll + ff_friction_l;

    const float km_gear = param_->Km * (param_->gear_a / param_->gear_b);
    float req_v_r = torque_r * param_->Resist / km_gear + ff_duty_r2;
    float req_v_l = torque_l * param_->Resist / km_gear + ff_duty_l2;

    tgt_duty.duty_r = req_v_r / se->ego.battery_lp * 100;
    tgt_duty.duty_l = req_v_l / se->ego.battery_lp * 100;
  }
}

void ControlLaw::apply_duty_limitter() {
  if (tgt_val_->motion_type == MotionType::STRAIGHT ||
      tgt_val_->motion_type == MotionType::SLALOM ||
      tgt_val_->motion_type == MotionType::SLA_BACK_STR ||
      tgt_val_->motion_type == MotionType::SLA_FRONT_STR ||
      tgt_val_->motion_type == MotionType::PIVOT) {
    const auto min_duty = param_->min_duty;
    if (0 <= tgt_duty.duty_r && tgt_duty.duty_r < min_duty)
      tgt_duty.duty_r = min_duty;
    else if (-min_duty < tgt_duty.duty_r && tgt_duty.duty_r <= 0)
      tgt_duty.duty_r = -min_duty;
    if (0 <= tgt_duty.duty_l && tgt_duty.duty_l < min_duty)
      tgt_duty.duty_l = min_duty;
    else if (-min_duty < tgt_duty.duty_l && tgt_duty.duty_l <= 0)
      tgt_duty.duty_l = -min_duty;

    // 2026-09-04: SLALOM/SLA_BACK_STR中、ff_rollがff_frontを上回ると内側
    // 車輪のduty指令が負(=逆回転)になり実測v_l/v_rが大きくマイナスに振れて
    // スリップする現象を確認(t_2200)。PIVOT(その場旋回、両輪逆符号が正常)は
    // 対象外。下限クランプ(turn_duty_floor)だけだと「落ちきってから頭打ち」
    // にしかならず、落ちる速度自体が速いと間に合わずスリップするため、
    // duty変化速度自体もスルーレート制限(turn_duty_slew)する。
    if (tgt_val_->motion_type == MotionType::SLALOM ||
        tgt_val_->motion_type == MotionType::SLA_BACK_STR) {
      if (param_->turn_duty_slew > 0) {
        const float max_step = param_->turn_duty_slew;
        tgt_duty.duty_r = std::clamp(tgt_duty.duty_r,
                                     turn_duty_r_prev_ - max_step,
                                     turn_duty_r_prev_ + max_step);
        tgt_duty.duty_l = std::clamp(tgt_duty.duty_l,
                                     turn_duty_l_prev_ - max_step,
                                     turn_duty_l_prev_ + max_step);
      }
      if (param_->turn_duty_floor > 0) {
        tgt_duty.duty_r = std::max(tgt_duty.duty_r, param_->turn_duty_floor);
        tgt_duty.duty_l = std::max(tgt_duty.duty_l, param_->turn_duty_floor);
      }
    }
    turn_duty_r_prev_ = tgt_duty.duty_r;
    turn_duty_l_prev_ = tgt_duty.duty_l;
  } else if (tgt_val_->motion_type == MotionType::FRONT_CTRL) {
    const auto max_duty = param_->sen_ref_p.search_exist.offset_l;
    tgt_duty.duty_r = std::clamp(tgt_duty.duty_r, -max_duty, max_duty);
    tgt_duty.duty_l = std::clamp(tgt_duty.duty_l, -max_duty, max_duty);
  }

  const auto max_duty = param_->max_duty;
  if (!isfinite(tgt_duty.duty_r))
    tgt_duty.duty_r = 0;
  if (!isfinite(tgt_duty.duty_l))
    tgt_duty.duty_l = 0;

  const float prev_r = tgt_duty.duty_r;
  const float prev_l = tgt_duty.duty_l;
  tgt_duty.duty_r = std::clamp(tgt_duty.duty_r, -max_duty, max_duty);
  tgt_duty.duty_l = std::clamp(tgt_duty.duty_l, -max_duty, max_duty);

  if (prev_r != tgt_duty.duty_r || prev_l != tgt_duty.duty_l) {
    ee->aw_log.sat_flag = 1.0f;
  } else {
    ee->aw_log.sat_flag = 0.0f;
  }

  // duty_roll(ヨートルク)は summation_duty() で duty_r に+、duty_l に-で乗る。
  // duty_r が上限で頭打ち、または duty_l が下限で頭打ちなら「duty_rollを+方向に
  // これ以上振っても効かない」ことを意味する(並進側飽和との厳密な切り分けはしない近似)。
  const bool push_positive_blocked =
      (prev_r > tgt_duty.duty_r) || (prev_l < tgt_duty.duty_l);
  const bool push_negative_blocked =
      (prev_r < tgt_duty.duty_r) || (prev_l > tgt_duty.duty_l);
  if (push_positive_blocked && !push_negative_blocked) {
    ee->aw_log.sat_roll_dir = 1.0f;
  } else if (push_negative_blocked && !push_positive_blocked) {
    ee->aw_log.sat_roll_dir = -1.0f;
  } else {
    ee->aw_log.sat_roll_dir = 0.0f;
  }
}

// 走行開始時に keep_dist ヒステリシスを一度だけ外す(2026-09-08、structs.hpp
// input_param_t::keep_dist_th_start_skip のコメント参照)。check_sen_error()の
// dist_check_* は |global_pos.dist - star_dist| > *_keep_dist_th で判定するので、
// star_dist をしきい値+1mm だけ手前へ置けば1tick目から真になる。壁が範囲外に
// なった時点で check_*_sensor_error() が star_dist を現在距離へ更新するため、
// 以降は従来のヒステリシスに戻る(=効果は一回限り)。
void ControlLaw::skip_keep_dist_once() {
  const float th =
      std::max(param_->left_keep_dist_th, param_->right_keep_dist_th);
  left_keep.star_dist = right_keep.star_dist =
      tgt_val_->global_pos.dist - th - 1.0f;
}

void ControlLaw::clear_ctrl_val() {
  duty_c = duty_roll = duty_front_ctrl_roll_keep = duty_roll_ang = 0;
  sen_kanayama_dw = 0;
  turn_angle_fb_integral_ = 0;
  turn_angle_fb_i_bias_prev_ = 0;
  wall_found_prev_ = false;
  sen_ctrl_active_prev_ = false;
  ee->v.error_i = ee->v.error_d = ee->v.error_dd = 0;
  ee->dist.error_i = ee->dist.error_d = ee->dist.error_dd = 0;
  ee->w.error_i = ee->w.error_d = ee->w.error_dd = 0;
  ee->ang.error_i = ee->ang.error_d = ee->ang.error_dd = 0;
  ee->ang.i_slow = ee->ang.i_bias = 0;
  ee->sen.error_i = ee->sen.error_d = ee->sen.error_dd = 0;
  ee->sen_dia.error_i = ee->sen_dia.error_d = ee->sen_dia.error_dd = 0;
  tgt_duty.duty_r = tgt_duty.duty_l = 0;
  ee->v_log.gain_zz = ee->v_log.gain_z = 0;
  ee->dist_log.gain_zz = ee->dist_log.gain_z = 0;
  ee->w_log.gain_zz = ee->w_log.gain_z = 0;
  ee->ang_log.gain_zz = ee->ang_log.gain_z = 0;
  ee->sen_log.gain_z = ee->sen_log.gain_zz = 0;
  ee->v_l_log.gain_zz = ee->v_l_log.gain_z = 0;
  ee->v_r_log.gain_zz = ee->v_r_log.gain_z = 0;
  tgt_val_->global_pos.ang = 0;
  tgt_val_->global_pos.img_ang = 0;
  tgt_val_->global_pos.dist = 0;
  tgt_val_->global_pos.img_dist = 0;
  ee->v_val.p_val = 0;
}

void ControlLaw::limitter(float &kp, float &ki, float &kb, float &kd,
                          pid_param_t &lim) {
  if (lim.mode == 0)
    return;
  if (kp > lim.p)
    kp = lim.p;
  else if (kp < -lim.p)
    kp = -lim.p;
  if (ki > lim.i)
    ki = lim.i;
  else if (ki < -lim.i)
    ki = -lim.i;
  if (kb > lim.b)
    kb = lim.b;
  else if (kb < -lim.b)
    kb = -lim.b;
  if (kd > lim.d)
    kd = lim.d;
  else if (kd < -lim.d)
    kd = -lim.d;
}

void ControlLaw::set_next_duty(float duty_l, float duty_r, float duty_suction) {
  float duty_suction_in = 0.0f;

  if (motor_en_) {
    if (param_->motor_debug_mode > 0) {
      duty_l = param_->motor_debug_mode_duty_l;
      duty_r = param_->motor_debug_mode_duty_r;
    } else if (tgt_val_->nmr.sys_id.enable) {
      // システム同定(test_system_identification): PID/FFを一切介さず
      // duty_l/rを直接そのまま出力する。motor_debug_modeと同じ最終段
      // オーバーライド方式(motion_planning.cpp::system_identification()参照)。
      duty_l = tgt_val_->nmr.sys_id.left_v;
      duty_r = tgt_val_->nmr.sys_id.right_v;
    }
    // 上記の最終段オーバーライドはローカル変数のみを差し替えるため、
    // これより前(control_law.cpp:180)でログ済みのsensing_result_->ego.duty
    // は無関係なPID計算値のままになってしまう。実際にモーターへ送る値で
    // 上書きし、ログが実duty(motor_->apply()への入力)と一致するようにする。
    sensing_result_->ego.duty.duty_l = duty_l;
    sensing_result_->ego.duty.duty_r = duty_r;
  } else {
    duty_l = 0.0f;
    duty_r = 0.0f;
  }
  // suction_duty/duty_low/duty_burst/duty_burst_lowはESCへの目標パルス幅
  // (us、1000〜2000)をそのまま指定する値(structs.hppのコメント参照)。
  // BLDC時代のバッテリー電圧duty%補正(*100/batt_kf)はus直接指定の空間には
  // そのままは対応しないため廃止したが、電圧低下時の始動失敗傾向を受け、
  // 下方のsuction_batt_boost_v/us_tableによるus加算方式で補正を再導入した。
  //
  // suction_en_==falseの間も目標を最小パルス(1000us=停止)にしたまま同じ
  // ランプ処理へ流し込む。即座にMINへ叩き落とすと、まだ高速回転している
  // 吸引モーターに対して急ブレーキ相当のコマンドを送ることになり、大きな
  // 逆起電力/回生電流が生じ得るため、停止時もsuction_ramp_us_per_sec_の
  // 速度で徐々に緩める(=有効/無効どちらの遷移でも同じ滑らかなランプに
  // 統一する)。
  suction_target_us_ = (float)SUCTION_ESC_PULSE_MIN_US;
  if (suction_en_) {
    const bool high_suction =
        tgt_val_->tgt_in.tgt_dist > 60 &&
        (tgt_val_->ego_in.state == 0 || tgt_val_->ego_in.state == 1) &&
        tgt_val_->motion_type == MotionType::STRAIGHT;
    suction_target_us_ =
        high_suction ? tgt_duty.duty_suction_low : tgt_duty.duty_suction;
    // 電圧モジュレーション方式のESCは印加電圧(≒バッテリー電圧×duty)が
    // 下がるほど同じduty指令でも実際のRPMが下がる(閉ループRPM制御では
    // ないため)。低電圧ほど始動失敗が顕著という実測傾向に対し、電圧が
    // 下がった分だけduty指令(パルス幅)を上乗せして補う。
    if (suction_batt_boost_v_table_.size() >= 2 &&
        suction_batt_boost_us_table_.size() >= 2) {
      const auto se = sensing_result_;
      suction_target_us_ += sensor_->interp1d(suction_batt_boost_v_table_,
                                              suction_batt_boost_us_table_,
                                              se->ego.batt_kf, false);
    }
    if (suction_target_us_ < (float)SUCTION_ESC_PULSE_MIN_US)
      suction_target_us_ = (float)SUCTION_ESC_PULSE_MIN_US;
    if (suction_target_us_ > (float)SUCTION_ESC_PULSE_MAX_US)
      suction_target_us_ = (float)SUCTION_ESC_PULSE_MAX_US;
  }

  // 現在のパルス幅に応じたランプ速度で目標パルス幅へ線形にランプする
  // (AM32側の自前ソフトスタートと併用)。1999付近(高domain)で序盤から脱調が
  // 頻発し、一律にランプを遅くしたら改善した実測を受け、旧BLDCの
  // battery_v/elec_hz依存gainテーブルと同じ発想で、パルス幅域ごとに
  // 個別のランプ速度を指定できるLUTを導入(suction_ramp_rate_us_x/y、
  // 空ならsuction_ramp_us_per_sec_の固定レートのまま)。安全マージンを
  // 見て目標到達直前の速度(現在値側)を使う。
  float ramp_rate = suction_ramp_us_per_sec_;
  if (suction_ramp_rate_us_x_.size() >= 2 && suction_ramp_rate_us_y_.size() >= 2) {
    ramp_rate = sensor_->interp1d(suction_ramp_rate_us_x_, suction_ramp_rate_us_y_,
                                  suction_pulse_us_, false);
  }
  if (suction_pulse_us_ < suction_target_us_) {
    suction_pulse_us_ += ramp_rate * dt_;
    if (suction_pulse_us_ > suction_target_us_)
      suction_pulse_us_ = suction_target_us_;
  } else if (suction_pulse_us_ > suction_target_us_) {
    suction_pulse_us_ -= ramp_rate * dt_;
    if (suction_pulse_us_ < suction_target_us_)
      suction_pulse_us_ = suction_target_us_;
  }
  duty_suction_in = suction_pulse_us_;
  if (!isfinite(duty_suction_in))
    duty_suction_in = (float)SUCTION_ESC_PULSE_MIN_US;

  tgt_val_->duty_suction = duty_suction_in;
  motor_->apply(duty_l, duty_r);
  esc_->apply_us(duty_suction_in);
}

void ControlLaw::pl_req_activate(const planning_req_t &pl_req) {
  if (pl_req.error_gyro_reset == 1) {
    ee->v.error_i = 0;
  }
  if (pl_req.error_vel_reset == 1) {
    ee->dist.error_i = 0;
  }
  if (pl_req.error_dist_reset == 1) {
    ee->w.error_i = 0;
    ee->w_kf.error_i = 0;
  }
  if (pl_req.error_ang_reset == 1) {
    ee->ang.error_i = 0;
    ee->ang.i_slow = 0;
    ee->ang.i_bias = 0;
  }
  if (pl_req.error_led_reset == 1) {
    // ee->led.error_i = 0;
  }
  // if (tgt_val->pl_req.log_start == 1) {
  //   log_active = true;
  // }
  // if (tgt_val->pl_req.log_end == 1) {
  //   log_active = false;
  // }
}