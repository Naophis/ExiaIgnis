#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "gen_code_mpc/bus.h"
#include "include/defines.hpp"
#include "include/enums.hpp"
#include "include/maze_solver.hpp"

#include "gen_code_conv_single2half/half_type.h"
#include "gen_code_conv_single2half/rtwtypes.h"

#include <cmath>
#include <deque>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

union LED_bit {
  struct {
    unsigned int b0 : 1;
    unsigned int b1 : 1;
    unsigned int b2 : 1;
    unsigned int b3 : 1;
    unsigned int b4 : 1;
    unsigned int b5 : 3;
  };
  uint8_t byte;
};

typedef struct {
  float x = 0;
  float y = 0;
  float theta = 0;

  float v = 0;
  float vx = 0;
  float vy = 0;
  float w = 0;

  float accl = 0;
  float alpha = 0;
} t_kinematics_state;

typedef struct {
  float right = 0;
  float left = 0;
  float right_old = 0;
  float left_old = 0;
} encoder_data_t;

typedef struct {
  int raw = 0;
  float data = 0;
} sensing_data_t;

typedef struct {
  sensing_data_t right90;
  sensing_data_t right45;
  sensing_data_t right45_2;
  sensing_data_t right45_3;
  sensing_data_t front;
  sensing_data_t left45;
  sensing_data_t left45_2;
  sensing_data_t left45_3;
  sensing_data_t left90;
} led_sensor_t;

typedef struct {
  float right = 0;
  float left = 0;
} rpm_t;

typedef struct {
  float duty_l = 0;
  float duty_r = 0;
  float duty_suction = 0;
  float duty_suction_low = 0;
  float sen = 0;
  float sen_ang = 0;

  float ff_duty_front;
  float ff_duty_roll;
  float ff_duty_rpm_r;
  float ff_duty_rpm_l;

  // torque_mode==2 で実際にsummation_duty()に使われるトルク系FF
  // (ff_duty_front/roll はduty系の並行計算値で、torque_mode==2時は出力に使われない)
  float ff_front_torque = 0;
  float ff_roll_torque = 0;
  float ff_friction_torque_r = 0;
  float ff_friction_torque_l = 0;
} duty_t;

typedef struct {
  float front;
  float roll;
} ff_duty_t;

typedef struct {
  float v_r = 0;
  float v_l = 0;
  float v_r_old = 0;
  float v_l_old = 0;
  float v_c = 0;
  float filter_v = 0;

  float main_v = 0;

  float w_raw = 0;
  float w_raw2 = 0;
  float w_lp = 0;
  float w_lp2 = 0;
  float w_kf = 0;
  float w_kf2 = 0;
  float v_kf = 0;
  float v_kf_l = 0; // kf_v_l.get_state() (v_kfへの入力の可視化用、EgoEstimator::update)
  float v_kf_r = 0; // kf_v_r.get_state()
  float dist_kf = 0;
  float ang_kf = 0;
  float ang_kf2 = 0;
  float batt_kf = 0;
  float accel_x_raw = 0;

  // gyro_pos(取り付け位置・向き)補正後、車体基準点での加速度[mm/s^2]。
  // EgoEstimator::update()で算出(センサー座標系→車体座標系の回転 +
  // レバーアーム補正 a_ref = a_body - alpha×r - w×(w×r)、roll/pitch角速度は無視)。
  float accel_x_corr = 0;
  float accel_y_corr = 0;
  float accel_z_corr = 0;

  float v_ave = 0;
  float v_lp = 0;
  float integrate_accl_x_ave = 0;

  float sum_v_ave = 0;
  float sum_integrate_accl_x_ave = 0;

  float w_kalman = 0;
  float ang_kalman = 0;
  float battery_raw = 0;
  float battery_lp = 0;

  float right90_raw = 0;
  float right90_lp = 0;
  float right45_raw = 0;
  float right45_lp = 0;
  float front_raw = 0;

  float front_lp = 0;
  float left45_raw = 0;
  float left45_lp = 0;
  float left90_raw = 0;
  float left90_lp = 0;

  float right45_2_raw = 0;
  float right45_2_lp = 0;
  float left45_2_raw = 0;
  float left45_2_lp = 0;
  float right45_3_raw = 0;
  float right45_3_lp = 0;
  float left45_3_raw = 0;
  float left45_3_lp = 0;

  float front_lp_old = 0;
  float left45_lp_old = 0;
  float left90_lp_old = 0;
  float right45_lp_old = 0;
  float right90_lp_old = 0;
  float left45_2_lp_old = 0;
  float right45_2_lp_old = 0;
  float left45_3_lp_old = 0;
  float right45_3_lp_old = 0;

  volatile float front_dist = 0;
  volatile float left45_dist = 0;
  volatile float left45_2_dist = 0;
  volatile float left45_3_dist = 0;
  volatile float left90_dist = 0;
  volatile float right45_dist = 0;
  volatile float right45_2_dist = 0;
  volatile float right45_3_dist = 0;
  volatile float right90_dist = 0;
  volatile float front_far_dist = 0;
  volatile float left90_far_dist = 0;
  volatile float right90_far_dist = 0;
  volatile float left90_mid_dist = 0;
  volatile float right90_mid_dist = 0;
  volatile float front_mid_dist = 0;

  volatile float left45_dist_diff = 0;
  volatile float left45_2_dist_diff = 0;
  volatile float left45_3_dist_diff = 0;
  volatile float right45_dist_diff = 0;
  // 2026-09-05: 上の *_dist_diff は「1tickあたりの生の差分」なので、同じ壁
  // エッジでも速度が半分になれば差分も半分になる。切れ目(kireme)判定は
  // 走行距離あたりの変化率で見るべきなので、input_param_t::kireme_diff_v_ref
  // を基準速度として正規化した値を別フィールドで持つ(生の差分は
  // WallOffController が別途チューニングしたしきい値で使っているため変更
  // しない)。kireme_diff_v_ref==0 のときは生の差分と同値。
  volatile float left45_dist_diff_norm  = 0;
  volatile float right45_dist_diff_norm = 0;
  volatile float right45_2_dist_diff = 0;
  volatile float right45_3_dist_diff = 0;
  volatile float left90_dist_diff = 0;
  volatile float right90_dist_diff = 0;

  volatile float temp = 0;

  float front_dist_old = 0;
  float left45_dist_old = 0;
  float left45_2_dist_old = 0;
  float left45_3_dist_old = 0;
  float left90_dist_old = 0;
  float right45_dist_old = 0;
  float right45_2_dist_old = 0;
  float right45_3_dist_old = 0;
  float right90_dist_old = 0;
  bool exist_r_wall = false;
  bool exist_l_wall = false;

  rpm_t rpm;
  duty_t duty;
  ff_duty_t ff_duty;
  char motion_type = 0;

  float pos_x;
  float pos_y;
  float pos_ang;

  float knym_v;
  float knym_w;
  float odm_x;
  float odm_y;
  float odm_theta;
  float kim_x;
  float kim_y;
  float kim_theta;
} ego_entity_t;

typedef struct {
  float x;
  float y;
  float theta;
  float v;
  float w;
} kinematics_t;

typedef struct {
  float sensor_dist = 300;
  float global_run_dist = 0;
  float angle = 0;
} sen_log_t;

typedef struct {
  float r45_dist = 0;
  float l45_dist = 0;
  float global_run_dist = 0;
} sen_log2_t;

typedef struct {
  sen_log_t l90;
  sen_log_t l45;
  sen_log_t l45_2;
  sen_log_t l45_3;
  // sen_log_t front;
  sen_log_t r45;
  sen_log_t r45_2;
  sen_log_t r45_3;
  sen_log_t r90;
} sen_logs_t;

typedef struct {
  std::deque<sen_log2_t> list;
} sen_dist_log_t;

typedef struct {
  led_sensor_t led_sen;
  led_sensor_t led_sen_after;
  led_sensor_t led_sen_before;
  sensing_data_t gyro;
  sensing_data_t gyro2;
  sensing_data_t accel_x;
  sensing_data_t accel_y;
  sensing_data_t accel_z;
  int gyro_list[5];
  deque<int> enc_r_list;
  deque<int> enc_l_list;
  sensing_data_t battery;
  encoder_data_t encoder_raw;
  encoder_data_t encoder;
  ego_entity_t ego;
  sen_logs_t sen;
  sen_dist_log_t sen_dist_log;
  int16_t calc_time;
  int16_t calc_time2;
  int16_t t_spi;     // sense_start からの累積 [us]: read_spi_sensors 終了
  int16_t t_ambient; // ambient ADC 終了
  int16_t t_r90;     // R90 シーケンス終了
  int16_t t_r45;     // R45 シーケンス終了
  int16_t t_l45;     // L45 シーケンス終了
  int16_t t_l90;     // L90 終了 (diff 前)
  int16_t t_gyro;    // read_spi_sensors 先頭からの累積: gyro 終了
  int16_t t_encr;    // enc_r 終了
  int16_t t_encl;    // enc_l 終了
  int16_t t_bat;     // battery + calc_vel 終了 (≈SPI総計)
  int64_t sensing_timestamp;
  float ang_kf_sum = 0;
  float img_ang_sum = 0;
  float img_ang_z = 0;
} sensing_result_entity_t;

typedef struct {
  float vel = 0;
  float speed = 0;
  float accl = 0;
} xva_t;

typedef struct {
  float p = 0;
  float i = 0;
  float d = 0;
  float b = 0;
  float c = 0;
  char mode = 0;
  char antiwindup = 0;
  float windup_gain = 0;
  float windup_dead_bind = 0;
  float windup_i_max = 0; // I項積算値そのものの絶対値クランプ。0なら無効
  float i_theta_tau = 0;
  float theta_gate = 0;
  float omega_gate = 0;
  float i_theta_slew = 0;
  float i_theta_max = 0;
  float alpha_stop = 0;
  float alpha_rate = 0;
  float theta_damp_th = 0;
  float omega_damp = 0;
  float th = 1;
  float theta_gate_on = 0;
  float theta_gate_full = 0;
  float theta_kp = 0;
  float theta_kd = 0;
  float omega_add_max = 0;
  float alpha_rate_end = 0;
  float k_stop = 0;
  float theta_eps = 0;
  float s_gate = 0;
  float mpc_q_ang = 0;
  float mpc_q_vel = 0;
  float mpc_b = 0;
  float mpc_r = 0;
  int mpc_horizon = 0;
  int mpc_max_iter = 5;
  float mpc_max_torque = 0;
  float mpc_observer_k = 0.05f; // Disturbance observer gain
} pid_param_t;

typedef struct {
  float gyro_w_gain_right = 0;
  float gyro_w_gain_left = 0;
  float retry_min_th = 0;
  float retry_max_th = 0;
  float robust_th = 0;
  float lp_delay = 0;
  int list_size = 256;
  int loop_size = 10;
} gyro_param_t;

typedef struct {
  float gain = 0;
  float offset = 0; // 重力ベースのbias校正値[mm/s^2]。true=(raw-offset)*gain
} accel_param_t;

typedef struct {
  float lp_delay = 1.0;
} sen_param_t;

typedef struct {
  float right45;
  float left45;
  float right90;
  float left90;
  float front;
  float kireme_r;
  float kireme_l;
  float kireme_r_fast;
  float kireme_l_fast;
  float kireme_r_wall_off;
  float kireme_l_wall_off;
  float kireme_r_wall_off2;
  float kireme_l_wall_off2;
} sen_ref_param3_t;

typedef struct {
  float front;
  float right45;
  float left45;
  float right90;
  float left90;
  float kireme_r;
  float kireme_l;
  float offset_r;
  float offset_l;
  float front_ctrl;
  float front_ctrl_th;
} sen_search_param_t;

typedef struct {
  float dist;
  float right45;
  float left45;
  float right45_2;
  float left45_2;
} sen_expand_param_t;

typedef struct {
  sen_ref_param3_t ref;
  sen_ref_param3_t ref_search;
  sen_ref_param3_t exist;
  sen_expand_param_t expand;
} sen_ref_param2_t;

typedef struct {
  sen_ref_param2_t normal;
  sen_ref_param2_t normal2;
  sen_ref_param2_t dia;
  sen_search_param_t search_exist;
  sen_search_param_t search_ref;
} sen_ref_param_t;

typedef struct {
  float a;
  float b;
} sensor_gain_param_t;

typedef struct {
  sensor_gain_param_t l90;
  sensor_gain_param_t l45;
  sensor_gain_param_t front;
  sensor_gain_param_t front2;
  sensor_gain_param_t front3;
  sensor_gain_param_t front4;
  sensor_gain_param_t front_ctrl_th;
  sensor_gain_param_t r45;
  sensor_gain_param_t l45_2;
  sensor_gain_param_t r45_2;
  sensor_gain_param_t l45_3;
  sensor_gain_param_t r45_3;
  sensor_gain_param_t r90;
  sensor_gain_param_t l90_far;
  sensor_gain_param_t r90_far;
  sensor_gain_param_t l90_mid;
  sensor_gain_param_t r90_mid;
} sensor_gain_t;

// typedef struct{

// } wall_off_p

typedef struct {
  float left_str;
  float right_str;
  float left_diff_th;
  float right_diff_th;
  float left_str_exist;
  float right_str_exist;
  float left_dia;
  float right_dia;
  float left_dia_noexit;
  float right_dia_noexit;
  float left_dia_oppo;
  float right_dia_oppo;

  float left_dia2;
  float right_dia2;
  float exist_dist_l;
  float exist_dist_r;
  float exist_dist_l2;
  float exist_dist_r2;
  float noexist_th_l;
  float noexist_th_r;
  float noexist_th_l2;
  float noexist_th_r2;
  float div_th_l;
  float div_th_r;
  float div_th_l2;
  float div_th_r2;
  float div_th_l3;
  float div_th_r3;
  float div_th_dia_l;
  float div_th_dia_r;

  float exist_dia_th_l;
  float exist_dia_th_r;
  float exist_dia_th_l2;
  float exist_dia_th_r2;
  float noexist_dia_th_l;
  float noexist_dia_th_r;
  float noexist_dia_th_l2;
  float noexist_dia_th_r2;

  float wall_off_exist_wall_th_l;
  float wall_off_exist_wall_th_r;
  float wall_off_exist_dia_wall_th_l;
  float wall_off_exist_dia_wall_th_r;

  bool search_wall_off_enable;
  float search_wall_off_l_dist_offset;
  float search_wall_off_r_dist_offset;
  float search_wall_off_offset_dist;

  float ctrl_exist_wall_th_l;
  float ctrl_exist_wall_th_r;
  float go_straight_wide_ctrl_th;

  float diff_check_dist = 20;
  float diff_dist_th_l = 20;
  float diff_dist_th_r = 20;
  float diff_check_dist_dia = 15;
  float diff_check_dist_dia_2 = 5;

} wall_off_hold_dist_t;

typedef struct {
  int duty;
  int v;
  int w;
  int ang;
  int wall_off;
} fail_check_cnt_t;

typedef struct {
  float v_lp_gain = 0;
  float accl_x_hp_gain = 0;
  float gain = 0;
  int enable = 0;
} comp_param_t;

typedef struct {
  float kx = 0;
  float ky = 0;
  float k_theta = 0;
  float ki = 0; // STRAIGHT用カスケードのI項ゲイン(calc_sensor_pid()参照)。
                // 積分値自体はstr_ang_pid_fast用のee->sen.error_iを共用
                // (条件付き積分・絶対値クランプもstr_ang_pid_fast側の
                // antiwindup/windup_dead_bind/windup_i_maxをそのまま使う)。
  char enable = 0;
  char windup = 0;
  float windup_deg = 0;
} kanayama_t;

// 旋回終端(SLALOM/SLA_BACK_STR)でff_duty_rollがideal_wと共にゼロへ落ちた後、
// 実測角速度(w_lp)が慣性で収束しきらず残ってしまう問題への対策
// (2026-08-23、20260823_032339.csv/20260823_032239.csvで解析)。
// |ego_in.w|(計画角速度)がw_th未満まで小さくなり、かつ|w.error_p|(残差)が
// err_thを超えている間だけ、通常のgyro_pid.p/dの代わりにこのp/dを使う。
// gyro_pid.p/d自体はFF主導設計を維持するため極小のまま変更しない
// (control_law.cpp calc_angle_velocity_ctrl()参照)。
typedef struct {
  int enable = 0;
  float w_th = 0.5f;   // |ideal_w|がこれ未満で「終端」とみなす(rad/s)
  float err_th = 0.3f; // 2026-08-23夜: ON/OFFゲートとしては未使用(control_law.cpp参照)
  float p = 0.0f;
  float d = 0.0f;
  // duty_rollスルーレート制限(2026-08-23夜追加): p/dだけでは残留角速度の
  // ゼロ交差直後に数tickでdutyがフル反転し(グリップ音・kim_thetaの遅い
  // 収束の原因、20260823_215751.csv等)、run毎に収束したり発振したりと
  // 不安定だった。turn_end_brake作動中だけduty_rollの1tickあたりの変化量に
  // 上限を掛け、急反転を防ぐ。0なら無効(制限なし)。
  float slew = 0.0f;
} turn_end_brake_t;

// 旋回角度不足(SLALOM/SLA_BACK_STR)への追加角度フィードバック(2026-08-23)。
// ee->ang.i_bias(=img_ang-kim.theta、calc_angle_i_bias()参照)は実測との
// ズレを正しい符号・大きさで検出できている(20260823_040833.csv解析:
// 旋回終盤で-2.7deg相当、目標-45degに対し実測-42.3degの不足と一致)が、
// 既存のkc_gain(=gyro_pid.c*i_bias、他モーションと共用)だけではduty%換算で
// 最大1.7〜3.5%程度にしかならず力不足で埋めきれない。gyro_pid.cを直接上げると
// STRAIGHT等他のモーションにも影響するため、SLALOM/SLA_BACK_STR専用の
// 追加ゲインとして分離する(control_law.cpp calc_angle_velocity_ctrl()参照)。
// SLALOM/SLA_BACK_STR用、実測kim_thetaを基準にした本物のPID(2026-08-23)。
// angle_pid(p=4.5,d=4.5)は本来この役割を担うはずだったが、(1)基準が
// ang_kf(=ego_in.ang、enable_kalman_gyro=0では計画値そのもの)で実測と
// 無関係、(2)出力duty_roll_angがw目標へのオフセットとしてしか作用せず
// gyro_pid.p(0.000325)経由で1/1000以下に希釈される、という二重の理由で
// 実質何も収束させていなかった(2026-08-23判明)。ee->ang.i_bias
// (=img_ang-kim.theta、calc_angle_i_bias()参照)は実測基準の正しい信号なので、
// これに対してduty_rollへ直接加算する(希釈経路を通らない)本物のP+I+D
// として作り直す。
typedef struct {
  int enable = 0;
  float gain = 0.0f; // P: ee->ang.i_biasに掛けてduty_rollへ追加(gyro_pid.cとは別枠)
  // I: ang.i_bias専用の積分項。既存w_error_i(アンチワインドヒステリシス付き)
  // の流用は実機で発散したため、単純なクランプ付き積分を別途新設
  // (turn_angle_fb_integral_参照)。旋回開始でゼロクリアされ、
  // SLALOM/SLA_BACK_STR中のみ積算。
  float gain_i = 0.0f;
  float i_max = 0.0f; // 積分状態(rad*s)の絶対値クランプ。0なら無効
  // I項の蓄積ウェイト閾値(2026-08-23追加): |実測w_lp|=0でweight=1、
  // |w_lp|>=i_w_gateでweight=0となる線形ランプで積分の蓄積速度を連続的に
  // 絞る(control_law.cpp calc_angle_velocity_ctrl()参照)。同一config・
  // 同一FFでも旋回終端の残留角速度が実機ばらつきで2〜3倍変わり
  // (20260823_054917.csv vs 054824.csv、w_lp突入後2.6〜3.2rad/s vs
  // 0.9〜1.0rad/s)、積分が過渡回転そのものに巻き込まれて育つ量がrun毎に
  // 違うことがオシレーション有無の分岐点になっていた。on/offのハード
  // ゲートも試したが(20260823_060428.csv)、ゲートが開く瞬間に凍結中の
  // i_biasが一気にフル蓄積を再開すること自体がステップ的な再点火となり
  // 2段目の発振を生んだため、連続的な重み付けに変更した。0なら無効
  // (常時フル蓄積、従来動作)。
  float i_w_gate = 0.0f;
  // D: ang.i_biasの1tick差分(turn_angle_fb_i_bias_prev_参照)に掛けて
  // duty_rollへ追加。積分(gain_i)による位相遅れ・発振を減衰させる狙い。
  float gain_d = 0.0f;
  // w目標offsetへのΔw(2026-08-23追加、rad/s per rad): ee->ang.i_biasに
  // 掛けてcalc_pid_val_ang_vel()のoffsetへ加算する(sen_kanayama_dwと同じ
  // 経路)。gain/gain_i/gain_dはduty_rollへ直接加算するため、i_biasが長時間
  // 一定値を保つ(STRAIGHT走行中の定常オフセット)と既存gyro_pid.bの積分
  // (w_error_i)が「外乱」とみなして正確に打ち消してしまい、正味トルクが
  // ゼロに収束して向きが直らないことが判明した(20260823_062913.csv/
  // 062821.csv、kim_thetaが2.4°付近で数百tick停滞。gyro_pid.bのkb_gainが
  // 逆算したturn_angle_fb出力とほぼ完全に相殺していた)。w_gainはw目標
  // 自体をずらす経路なので、gyro_pid.bはこれと戦わずΔwへ実測wを追従させる
  // 側に回る(既存kanayama_straightのk_theta*sin(e_theta)と同じ発想だが、
  // k_theta=0.0005は壁追従用のゲインで弱すぎるため、旋回後の残差解消専用
  // に独立させる)。gain/gain_i/gain_dは旋回直後の速い過渡分の減衰用として
  // 残す(gyro_pid.bの反応は遅いため短時間なら相殺されない)。0なら無効。
  float w_gain = 0.0f;
} turn_angle_fb_t;

// SLALOM/SLA_BACK_STR限定の角速度PID(積分b+減衰d)ブースト(2026-08-23)。
// gyro_pid.b(角速度誤差の積分)を直接上げると収束は改善するが、積分特有の
// 位相遅れにより発振しやすくなる(実機確認済み)。bを増やすときは同時にd
// (減衰)も増やして位相余裕を確保するのがセオリー。gyro_pid.b/dはSTRAIGHT等
// 他モーションと共用のため変更せず、SLALOM/SLA_BACK_STR限定の追加項として
// 分離する(control_law.cpp calc_angle_velocity_ctrl()参照)。
typedef struct {
  int enable = 0;
  float b = 0.0f; // w_error_iに掛けてduty_rollへ追加(gyro_pid.bとは別枠)
  float d = 0.0f; // w_error_dに掛けてduty_rollへ追加(gyro_pid.dとは別枠、bとペアで減衰を確保)
} turn_w_pid_t;

// ASM330LHHの取り付け位置(車体基準点=v/wの基準点からのオフセット、mm)・
// 向き(センサー座標系→車体座標系への回転角、deg)。EgoEstimatorで
// 加速度のレバーアーム補正・座標変換に使う。全ゼロなら補正なし(単位行列
// ・オフセットなし)。
typedef struct {
  float x = 0;       // mm
  float y = 0;       // mm
  float z = 0;       // mm
  float x_theta = 0; // deg
  float y_theta = 0; // deg
  float z_theta = 0; // deg
} gyro_pos_t;

typedef struct {
  float dt = 0.001;
  int trj_length = 1;
  float tire = 12;
  float tire2 = 12;
  int log_size = 1300;
  float gear_a = 37;
  float gear_b = 8;
  float max_duty = 99;
  float min_duty = 8;
  float battery_gain = 3.3;
  float Ke = 0;
  float Km = 0;
  float Resist = 0;
  float Mass = 0;
  float Lm = 0;
  float coulomb_friction = 0;
  float viscous_friction = 0;
  // 吸引ON時用の摩擦FF(2026-08-23追加、要実機チューニング)。coulomb_friction/
  // viscous_frictionは吸引OFF状態でチューニングされた値だが、吸引ONだと
  // 荷重(押し付け力)が約250g増加し摩擦も増えるため、OFF基準の値のままだと
  // 加速フェーズでFFが摩擦分を過小評価し、vel_pid(FB)がその穴埋めを
  // 背負って過大反応する(latest.csv解析: ff_duty_frontがほぼ一定・
  // ff_duty_rpmも最大2.3%止まりなのに対しduty_lが最大99.9%まで張り付き、
  // v_cが目標の2倍近く跳ねる"羽"状のオーバーシュートを確認)。
  // TrajectoryGenerator::copy_tgt()でtgt_val->duty_suction(実際の吸引
  // パルス幅)を見てON/OFFを二値判定し、切り替えて使う。初期値は
  // coulomb_friction/viscous_frictionと同じ(未チューニング、要実機調整)。
  float coulomb_friction_suction = 0;
  float viscous_friction_suction = 0;
  int MotorHz = 37500; // 駆動モーターPWM周波数(MotorActuator::init())

  float battery_init_cov = 0.95;
  float battery_p_noise = 0.05;
  float battery_m_noise = 0.35f;
  float encoder_init_cov = 0.95;
  float encoder_p_noise = 0.05;
  float encoder_m_noise = 0.035;
  float w_init_cov = 0.95;
  float w_p_noise = 0.05;
  float w_m_noise = 0.035;
  float v_init_cov = 0.95;
  float v_p_noise = 0.05;
  float v_m_noise = 0.035;
  float ang_init_cov = 0.95;
  float ang_p_noise = 0.05;
  float ang_m_noise = 0.035;
  float dist_init_cov = 0.95;
  float dist_p_noise = 0.05;
  float dist_m_noise = 0.035;

  float pos_init_cov = 0.95;
  float pos_p_noise = 0.05;
  float pos_m_noise = 0.035;

  float tread = 38;
  int FF_front = 0;
  int FF_roll = 0;
  int FF_keV = 0;
  float offset_start_dist = 0;
  float offset_start_dist_search = 0;
  float long_run_offset_dist = 5;
  float pivot_back_offset = 0;
  float cell = 90;
  float cell2 = 90;
  float pivot_angle_180 = 180;
  float pivot_angle_90 = 90;
  float wall_off_front_move_dist_th = 90;
  float wall_off_front_move_dia_dist_th = 90;
  float ff_front_gain_14 = 1;
  float ff_roll_gain_before = 1;
  float ff_roll_gain_after = 1;
  float ff_front_gain_decel = 1;
  // 2026-08-30: SLALOM旋回入り口(角加速度がbase_alphaと同符号=立ち上がり)の
  // ff_roll減衰ゲイン。ff_roll_gain_beforeは既にWALL_OFFのff_front用に
  // チューニング済みで流用すると干渉するため別フィールドにする
  // (control_law.cpp calc_translational_ctrl()参照)。
  float ff_roll_gain_entry = 1;
  // 2026-09-04: SLALOM/SLA_BACK_STR中、ff_roll(横系差動)がff_front(前進系)を
  // 上回ると内側車輪のduty指令が負(=逆回転)になり、実測v_l/v_rが大きく
  // マイナスに振れてスリップする現象を確認(t_2200、SLA_BACK_STR突入直後
  // v_rが-313〜-833まで振れる)。この2モーションに限り、車輪dutyがこの値を
  // 下回らないようフロアを掛ける(apply_duty_limitter()参照)。0なら無効。
  float turn_duty_floor = 0;
  // 2026-09-04: turn_duty_floorだけでは変化が速すぎる場合に間に合わないため、
  // SLALOM/SLA_BACK_STR中のduty変化量を1tickあたりこの値までに制限する
  // (apply_duty_limitter()参照)。0なら無効(スルーレート制限なし)。
  float turn_duty_slew = 0;
  pid_param_t front_ctrl_roll_pid;
  pid_param_t motor_pid;
  pid_param_t motor_pid_gain_limitter;
  pid_param_t motor_pid2;
  pid_param_t motor2_pid_gain_limitter;
  pid_param_t motor_pid3;
  pid_param_t gyro_pid;
  pid_param_t gyro_pid_gain_limitter;
  turn_end_brake_t turn_end_brake;
  turn_angle_fb_t turn_angle_fb;
  turn_w_pid_t turn_w_pid;
  pid_param_t str_ang_pid;
  // 高速走行(非探索)時の壁PD専用ゲイン。str_ang_pidは.p/.iを探索モードの
  // P/D、.b/.dを高速モードのP/Dとして兼用する紛らわしい構成だったため、
  // I項導入を機に高速側だけ独立させた(p/i/d本来の意味で使う)。
  pid_param_t str_ang_pid_fast;
  pid_param_t str_ang_dia_pid;
  pid_param_t angle_pid;
  pid_param_t front_ctrl_angle_pid;
  pid_param_t front_ctrl_dist_pid;
  pid_param_t front_ctrl_keep_angle_pid;
  // pid_param_t sensor_pid;
  pid_param_t sensor_pid_dia;
  gyro_param_t gyro_param;
  gyro_param_t gyro2_param;
  accel_param_t accel_x_param;
  accel_param_t accel_y_param;
  accel_param_t accel_z_param;
  gyro_pos_t gyro_pos;
  comp_param_t comp_param;
  sen_param_t battery_param;
  sen_param_t led_param;
  MotionDirection motion_dir;
  sen_ref_param_t sen_ref_p;
  sensor_gain_t sensor_gain;
  float sakiyomi_time = 1;
  float search_sen_ctrl_limitter = 1;
  // v > accl_param.limit(5500固定, motion_planning.cpp/planning_task.cpp)
  // 域での加減速ソフトスタート用パラメータ。mpc_tgt_calc.cppのdecel/accl
  // 分岐で (1 - pow(1 - min(counter/decel_delay_cnt, 1), decel_delay_n)) を
  // 掛けて減速指令を滑らかに立ち上げる。0のままだと counter/0 -> Inf ->
  // pow(0,0)=1 -> 係数0 となり、v>5500で減速指令が完全に消える(実機で確認)。
  int decel_delay_cnt = 5;
  float decel_delay_n = 4;
  float clear_angle = 0;
  float clear_dist_order = 0;
  float front_dist_offset = 0;
  float front_dist_offset0 = 0;
  float front_dist_offset2 = 0;
  float front_dist_offset3 = 0;
  float front_dist_offset4 = 0;
  float front_dist_offset_dia_front = 0;
  float front_dist_offset_dia_45_th = 0;
  float front_dist_offset_dia_right45 = 0;
  float front_dist_offset_dia_left45 = 0;

  float sla_wall_ref_l = 45;
  float sla_wall_ref_r = 45;
  float sla_max_offset_dist = 45;
  bool large_offset_enable = false;
  bool dia45_offset_enable = false;
  bool dia135_offset_enable = false;
  bool orval_offset_enable = false;
  float large_offset_max_dist = 5;
  float dia45_offset_max_dist = 0;
  float dia135_offset_max_dist = 0;
  float orval_offset_max_dist = 0;
  float dia45_2_offset_max_dist = 0;
  float dia135_2_offset_max_dist = 0;
  float dia90_offset_max_dist = 0;
  float lim_angle = 0;

  float front_ctrl_error_th = 4;

  float clear_dist_ragne_from = 0;
  float clear_dist_ragne_to = 0;
  float clear_dist_ragne_to2 = 0;

  std::vector<float> clear_dist_ragne_dist_list;
  std::vector<float> clear_dist_ragne_th_list;
  std::vector<float> clear_dist_ragne_dist_list_fast;

  float wall_off_hold_dist;
  wall_off_hold_dist_t wall_off_dist;
  float wall_off_diff_ref_th = 5;
  float wall_off_diff_ref_front_th = 10;
  float wall_off_wait_dist = 40;
  float wall_off_wait_dist_dia = 40;
  int search_log_enable = 0;
  int seach_timer = 60 * 3;
  int test_log_enable = 0;
  int fast_log_enable = 0;
  float front_dist_offset_pivot_th = 0;
  float front_dist_offset_pivot = 0;
  float pivot_back_dist0 = 0;
  float pivot_back_dist1 = 0;
  int sen_log_size = 100;
  int sen_log_size2 = 100;
  // LED点灯後の安定待ち時間(us) = カウント値 × led_light_delay_us_per_cnt。
  // led_light_delay_cnt: R90/L90/R45(LED1のみ)/L45(LED1のみ)の単発点灯ステップ
  // led_light_delay_cnt2: R45/L45のLED1+LED2拡張ステップ(WALL_OFF等でのみ発生)
  int led_light_delay_cnt = 1000;
  int led_light_delay_cnt2 = 1000;
  float led_light_delay_us_per_cnt = 1.0f;
  bool set_param = false;
  float logging_time = 4.0;
  float offset_after_turn_l2 = 0.0;
  float offset_after_turn_r2 = 0.0;
  float offset_after_turn_l = 0.0;
  float offset_after_turn_r = 0.0;
  float offset_after_turn_dia_l = 0.0;
  float offset_after_turn_dia_r = 0.0;

  float dia_turn_exist_th_l = 0.0;
  float dia_turn_exist_th_r = 0.0;
  float dia_turn_th_l = 0.0;
  float dia_turn_th_r = 0.0;
  float dia_turn_ref_l = 0.0;
  float dia_turn_ref_r = 0.0;
  float dia_turn_max_dist_l = 0.0;
  float dia_turn_max_dist_r = 0.0;
  float wall_off_pass_dist = 10;

  float dia_wall_off_ref_l = 0;
  float dia_wall_off_ref_r = 0;
  float dia_wall_off_ref_l_wall = 0;
  float dia_wall_off_ref_r_wall = 0;
  float dia_wall_off_ref_l_wall2 = 0;
  float dia_wall_off_ref_r_wall2 = 0;
  float dia_wall_off_ref_l_piller = 0;
  float dia_wall_off_ref_r_piller = 0;
  float dia_offset_max_dist = 0;

  float slip_param_K = 0;
  float slip_param_k2 = 0;

  fail_check_cnt_t fail_check;
  float fail_check_ang_th = 30.0 / 180 * M_PI;

  float normal_sla_offset = 4;
  float normal_sla_offset_front = 4;
  float normal_sla_offset_back = 4;
  float front_diff_th = 3;
  float ff_v_th = 3;
  float ff_front_dury = 3;

  MotorDriveType motor_driver_type = MotorDriveType::EN1_PH1;
  uint8_t motor_debug_mode = 0;
  uint8_t motor_r_cw_ccw_type = 0;
  uint8_t motor_l_cw_ccw_type = 0;
  float motor_debug_mode_duty_r = 0;
  float motor_debug_mode_duty_l = 0;

  // hl or cl
  float pivot_straight = 43;
  float pivot_back_enable_front_th = 100;
  float search_front_ctrl_th = 60;
  float judge_pivot = 110;
  float sensor_range_min = 5;
  float sensor_range_max = 180;
  float sensor_range_mid_max = 150;
  float sensor_range_far_max = 150;
  float dist_mod_num = 90;
  // 壁の切れ目(kireme)判定に使う *_dist_diff の速度正規化の基準速度[mm/s]。
  // *_dist_diff は1tickあたりの生の差分なので、同じ壁エッジでも低速では
  // 小さく出る(v=400mm/sなら1tickで0.4mmしか進まない)。kireme_*_fast /
  // kireme_*_wall_off* は最短走行の速度域で合わせてあるため、低速直進では
  // 実質無効になっていた(20260905_144848.csv: 切れ目で実測0.18mm/tick
  // (最大0.42)に対し kireme_r_fast=1.25 が一度も発火せず、後退中の右壁が
  // 有効なまま左右差が-5.5mmまで育ち duty_sen が±5°のレールに34tick
  // 張り付いて発振した)。この速度で走っていたら差分がいくつになるかへ
  // 換算する = 走行距離あたりの変化率で判定するのと等価。
  // 探索走行は速度がほぼ一定でこの問題が起きないうえ kireme_r/l が
  // 探索速度で調整済みのため、正規化は非探索(fast/wall_off)側にのみ効かせる。
  // 0 なら無効(従来通り生の差分)。
  float kireme_diff_v_ref = 0;
  float sen_ctrl_front_th = 45;
  float sen_ctrl_front_diff_th = 40;
  float th_offset_dist = 58;
  float sla_front_ctrl_th = 110;
  float orval_front_ctrl_min = 40;
  float orval_front_ctrl_max = 130;
  float wall_off_front_ctrl_min = 40;
  float dia_turn_offset_calc_th = 52;
  float go_straight_wide_ctrl_th = 60;
  float wall_off_pass_through_offset_r = 8;
  float wall_off_pass_through_offset_l = 8;
  float tire_tread = 38;
  float right_keep_dist_th = 0;
  float left_keep_dist_th = 0;
  float normal_sla_l_wall_off_th_in = 100;
  float normal_sla_r_wall_off_th_in = 100;
  float normal_sla_l_wall_off_th_out = 100;
  float normal_sla_r_wall_off_th_out = 100;
  float normal_sla_l_wall_off_ref_cnt = 100;
  float normal_sla_r_wall_off_ref_cnt = 100;
  float normal_sla_l_wall_off_dist = 5;
  float normal_sla_r_wall_off_dist = 5;
  float normal_sla_l_wall_off_margin = 10;
  float normal_sla_r_wall_off_margin = 10;
  char torque_mode = 0;
  char enable_kalman_gyro = 0;
  char enable_kalman_encoder = 0;
  char enable_mpc = 0;
  float dia90_offset = 0;
  kanayama_t kanayama;
  // STRAIGHT壁追従用Kanayamaカスケード(calc_sensor_pid()参照)。kxは未使用。
  // str_ang_pid_fastのP+D直接duty注入と排他ではなく常時並行実行し、
  // ey(壁センサー横偏差)・e_theta(kim_theta基準の向き誤差)からのΔwを
  // 角速度ループへ追加offsetとして加算する(2026-08-23の並行構成)。
  kanayama_t kanayama_straight;
  // Dia壁/柱追従用Kanayamaカスケード(calc_sensor_pid_dia()参照、2026-08-24
  // 追加)。kanayama_straightとは別パラメータにする: 斜めのey(柱距離誤差)は
  // 進行状況で非単調に変化し、ky/kiをそのまま流用すると遅い積分が暴れる
  // リスクが高いため、kxとkyとkiは未使用のまま0固定運用とし、kim_theta基準の
  // e_theta(実測ヘディング)を使うk_thetaのみ使う想定。
  kanayama_t kanayama_dia;

  // 軸退化ゲインテーブル (control_law で interp1d に渡す)
  std::vector<float> axel_degenerate_x;
  std::vector<float> axel_degenerate_y;
  std::vector<float> axel_degenerate_dia_x;
  std::vector<float> axel_degenerate_dia_y;

  // 速度→加速度テーブル (control_law で tgt_in.accl 書き換えに使用)
  std::vector<float> accl_v_x;
  std::vector<float> accl_v_y;

  // v_max→decel絶対値のLUT(2026-08-23追加)。
  // [2026-08-23 修正] 当初「空配列(size<2)なら無効」としていたが、
  // from_json_vector()はJSONキーが存在しない場合dst.clear()まで到達せず
  // 前回ロードされた値が残ってしまう(yamlから行を削除/コメントアウトして
  // pushしても無効化されない実害を確認)。配列の空/非空に頼るのをやめ、
  // 明示的なenableフラグで確実にON/OFFする。
  // なお本機能自体、mpc_tgt_calc側が残り距離から実際の減速度を毎tick
  // 再計算するため、この`decel`入力は「いつ減速フェーズに切り替えるか」
  // の閾値にしか効かず、片輪スリップ対策としては効果が無いことが判明
  // 済み(20260823_155443.csv等)。enable=0がデフォルト。
  int decel_v_max_enable = 0;
  std::vector<float> decel_v_max_x;
  std::vector<float> decel_v_max_y;

  // センサー角速度リミッタテーブル (control_law で interp1d に渡す)
  std::vector<float> sensor_deg_limitter_v;
  std::vector<float> sensor_deg_limitter_str;
  std::vector<float> sensor_deg_limitter_dia;
  std::vector<float> sensor_deg_limitter_piller;

  // 軌道インデックステーブル (trajectory_generator で interp1d に渡す)
  std::vector<int> trj_idx_v;
  std::vector<int> trj_idx_val;
} input_param_t;

typedef struct {
  float error_p;
  float error_i;
  float error_i_keep;
  float error_d;
  float error_dd;
  float i_slow;
  float i_bias;
} pid_error_t;

typedef struct {
  float p;
  float i;
  float i2;
  float d;
  float p_val;
  float i_val;
  float i2_val;
  float d_val;
  float zz;
  float z;
} pid_error2_t;

typedef struct {
  float gain_z;
  float gain_zz;
  float omega_ref_prev;
  MotionType prev_motion_type;
} gain_log_t;

typedef struct {
  float was_aw;
  float enter_aw;
  float keep_aw;
  float w_i_base;
  float w_error_i_raw; // clamp前
  float w_error_i_clamped;
  float gyro_pid_histerisis_i;
  float sat_flag;
  float duty_roll;
  float duty_roll_before;
  float mpc_d_estimated; // calc_angle_velocity_ctrl() の外乱オブザーバ推定値(現状は未結線、ログ確認用)
  // apply_duty_limitter() が前tickで判定したduty飽和方向。
  // +1: duty_roll を+方向にこれ以上振っても効かない(duty_r+側 or duty_l-側で頭打ち)
  // -1: duty_roll を-方向にこれ以上振っても効かない(duty_r-側 or duty_l+側で頭打ち)
  //  0: 余裕あり
  float sat_roll_dir;
  // デバッグ用(2026-08-23): calc_pid_val_ang_vel()のoffset内訳を直接確認する
  // ための一時フィールド。offset=dbg_off_ang+dbg_off_kny+dbg_off_wgain
  // (duty_roll_before=ego_in.w+offsetとの整合性確認用)。
  float dbg_off_ang;
  float dbg_off_wgain;
  float dbg_off_kny;
} aw_log_t;

typedef struct {
  pid_error_t v;
  pid_error_t v_kf;
  pid_error_t dist;
  pid_error_t w;
  pid_error_t v_r;
  pid_error_t v_l;
  pid_error_t w_kf;
  pid_error_t ang;

  gain_log_t v_log;
  gain_log_t dist_log;
  gain_log_t w_log;

  gain_log_t v_r_log;
  gain_log_t v_l_log;

  gain_log_t ang_log;
  gain_log_t sen_log;
  gain_log_t sen_log_dia;
  pid_error_t sen;
  pid_error_t sen_dia;

  pid_error2_t v_val;
  pid_error2_t w_val;
  pid_error2_t ang_val;
  pid_error2_t s_val;

  aw_log_t aw_log;

} pid_error_entity_t;

// 指示速度
typedef struct {
  float v_max = 0;
  float accl = 0;
  float w_max = 0;
  float alpha = 0;
} motion_tgt_t;

typedef struct {
  int hz = 0;
  int time = 0;
  int timstamp = 0;
} buzzer_t;

typedef struct {
  int time_stamp = 0;
  int error_gyro_reset = 0;
  int error_vel_reset = 0;
  int error_led_reset = 0;
  int error_ang_reset = 0;
  int error_dist_reset = 0;
  // int log_start = 0;
  // int log_end = 0;
} planning_req_t;

typedef struct {
  int error;
} fail_safe_state_t;

typedef struct {
  volatile float right_v;
  volatile float left_v;
  volatile bool enable = false;
} sys_id_t;

typedef struct {
  volatile float v_max;
  volatile float v_end;
  volatile float accl;
  volatile float decel;
  volatile float dist;
  volatile float w_max;
  volatile float w_end;
  volatile float alpha;
  volatile float ang;
  volatile float sla_alpha;
  volatile float sla_time;
  volatile float sla_pow_n;
  volatile float sla_rad;
  volatile float dia90_offset;
  volatile TurnDirection td;
  volatile TurnType tt;
  volatile RUN_MODE2 motion_mode;
  volatile MotionType motion_type;

  volatile int timstamp = 0;
  MotionDirection motion_dir;
  volatile bool dia_mode = false;
  SensorCtrlType sct;
  sys_id_t sys_id;
  volatile bool tgt_reset_req = false;
  volatile bool ego_reset_req = false;
} new_motion_req_t;

typedef struct {
  volatile float img_dist;
  volatile float img_ang;
  volatile float dist;
  volatile float ang;
} global_ego_pos_t;

typedef struct {
  float x = 0;
  float y = 0;
} pos_t;

typedef struct {
  float right_old;
  float left_old;
  bool right_save = false;
  bool left_save = false;
  float dia90_offset = 0;
} dia_state_t;
typedef struct {
  t_tgt tgt_in;
  t_ego ego_in;
  volatile int16_t calc_time;
  volatile int16_t calc_time2;
  volatile int16_t calc_time_diff;
  volatile int16_t pln_t_ego;      // ego.update() 終了時点の累積 [us]
  volatile int16_t pln_t_sensor;   // sensor_.calc_dist() 終了
  volatile int16_t pln_t_trj;      // trj_.generate() 終了 (first_req 時のみ更新)
  volatile int16_t pln_t_kanayama; // trj_.calc_kanayama() 終了
  volatile int16_t pln_t_copy;     // trj_.copy_tgt() 終了
  volatile int16_t pln_t_ctl;      // ctl_.calc() 終了 (≈ total)
  volatile global_ego_pos_t global_pos;
  volatile int32_t motion_mode;
  MotionType motion_type;
  MotionDirection motion_dir;
  volatile bool dia_mode = false;
  planning_req_t pl_req;
  fail_safe_state_t fss;
  volatile float gyro_zero_p_offset = 0;
  volatile float var_unbiased_dps2 = 0;
  volatile float var_robust_dps2 = 0;
  volatile int gyro_retry = 0;
  volatile CalibrationMode calibration_mode = CalibrationMode::NONE;
  volatile float gyro2_zero_p_offset = 0;
  volatile float accel_x_zero_p_offset = 0;
  volatile float accel_y_zero_p_offset = 0;
  volatile float temp_zero_p_offset = 0;
  buzzer_t buzzer;
  new_motion_req_t nmr;
  pos_t p;
  dia_state_t dia_state;
  float v_error;
  float w_error;
  TurnDirection td;
  TurnType tt;
  float duty_suction = 0;
  // WallOffController::continuous_turn_flag のミラー。ログ観点のみで使用
  // (実際の閾値切替はwall_off_controller.cpp側の同名フラグで行われる)
  bool continuous_turn = false;
} motion_tgt_val_t;

typedef struct {
  volatile float v_max = 0;
  volatile float v_end = 0;
  volatile float accl = 0;
  volatile float decel = 0;
  volatile float dist = 0;
  MotionType motion_type = MotionType::NONE;
  SensorCtrlType sct = SensorCtrlType::NONE;
  WallOffReq wall_off_req = WallOffReq::NONE;
  WallCtrlMode wall_ctrl_mode = WallCtrlMode::NONE;
  volatile float wall_off_dist_r = 0;
  volatile float wall_off_dist_l = 0;
  volatile bool dia_mode = false;
  volatile bool skil_wall_off = false;
  volatile bool search_str_wide_ctrl_r = false;
  volatile bool search_str_wide_ctrl_l = false;
  volatile float dia90_offset = 0;
} param_straight_t;

typedef struct {
  volatile float w_max = 0;
  volatile float w_end = 0;
  volatile float alpha = 0;
  volatile float ang = 0;
  TurnDirection RorL = TurnDirection::None;
} param_roll_t;

typedef struct {
  float radius = 0;
  float v_max = 0;
  float v_end = 0;
  float ang = 0;
  TurnDirection RorL = TurnDirection::None;
} param_normal_slalom_t;

typedef struct {
  float v_max = 0;
  float end_v = 0;
  float accl = 0;
  float decel = 0;
  // 速度→加速度LUT(要素数2未満なら無効、上のaccl固定値のまま)。
  // testモード開始時にinput_param_t.accl_v_x/yへコピーして使う
  // (ControlLaw::calc()参照、main_task_test_run.cpp)。
  std::vector<float> accl_v_x;
  std::vector<float> accl_v_y;
  // v_max→decel絶対値LUT(2026-08-23追加)。accl_v_x/yと同じくtestモード
  // 開始時にinput_param_t.decel_v_max_*へコピーして使う
  // (MainTask::apply_decel_v_max_lut()、main_task_test_run.cpp参照)。
  // decel_v_max_enable=0がデフォルト(mpc_tgt_calc側が残り距離から実際の
  // 減速度を毎tick再計算するため、この`decel`入力は閾値にしか効かず片輪
  // スリップ対策としては効果が無いことが判明済み、20260823_155443.csv)。
  int decel_v_max_enable = 0;
  std::vector<float> decel_v_max_x;
  std::vector<float> decel_v_max_y;
  float dia_accl = 0;
  float dia_decel = 0;
  float dist = 0;
  float w_max = 0;
  float w_end = 0;
  float alpha = 0;
  float ang = 0;
  int suction_active = 0;
  // AM32 ESC移行後、suction_duty/duty_low/duty_burst/duty_burst_lowは
  // 全てESCへの目標パルス幅を「us(1000〜2000)」で直接指定する値として
  // 扱う(0〜100%のduty%ではない。ControlLaw::set_next_duty()参照)。
  float suction_duty = 0;
  float suction_duty_low = 0;
  float suction_duty_burst = 0;
  float suction_duty_burst_low = 0;
  // 目標パルス幅へのランプ速度(us/sec)。例: 2000なら1000us分(0%→100%
  // 相当)を0.5秒でランプする。ControlLaw::set_suction_ramp_rate()参照。
  float suction_esc_ramp_us_per_sec = 2000.0f;
  // バッテリー電圧が低いほど始動失敗(脱調/ロック未確立)が顕著になる実測
  // 傾向を受けての補正。suction_target_us_(ControlLaw::set_next_duty())に
  // 電圧依存で上乗せするus値を、電圧(昇順)→上乗せusの区分線形LUTで指定する
  // (batt_kf基準。sensor_->interp1d()使用)。suction_batt_boost_v_table・
  // suction_batt_boost_us_tableは同じ長さで指定すること。空(未指定)なら
  // 補正なし(従来通りduty_suction/duty_suction_lowをそのまま使う)。
  // 値は実機未検証の初期値なので要調整。
  std::vector<float> suction_batt_boost_v_table;
  std::vector<float> suction_batt_boost_us_table;
  // 現在のパルス幅(us、昇順)ごとにランプ速度(us/sec)を変える区分線形LUT。
  // 1999付近(高域)で序盤から脱調する症状が、一律にsuction_esc_ramp_us_per_sec
  // を下げたら改善した実測を受けての機能。低域は元の速さを保ちつつ、高域
  // だけさらに遅くする、といった調整に使う(ControlLaw::set_next_duty()
  // 参照)。suction_ramp_rate_us_x・suction_ramp_rate_us_yは同じ長さで
  // 指定すること。空(未指定)ならsuction_esc_ramp_us_per_secの固定レートの
  // まま。値は実機未検証の初期値なので要調整。
  std::vector<float> suction_ramp_rate_us_x;
  std::vector<float> suction_ramp_rate_us_y;
  float suction_bldc_hz = 0;
  // BldcActuatorのbattery_v→{gain, max_amp} LUT(可変長、区分線形補間)。
  // amp = AMP_BASE*(hz/AMP_BASE_HZ)*gain のV/Hz比例式に使うgainと、その
  // クランプ上限max_ampを、バッテリー電圧ごとに個別調整する。
  // suction_batt_v_table(電圧、昇順)・suction_batt_gain_table・
  // suction_batt_max_amp_table は同じ長さで指定すること。空(未指定)なら
  // BldcActuator側のデフォルト3点のまま。
  std::vector<float> suction_batt_v_table;
  std::vector<float> suction_batt_gain_table;
  std::vector<float> suction_batt_max_amp_table;
  // BldcActuatorの起動ランプ速度(elec_hz/sec、ControlLawのデューティランプ
  // tick数にも兼用)は、バッテリー電圧ではなく現在の回転数推定値(elec_hz)を
  // X軸とするLUT(可変長、区分線形補間)から都度決める。回転数が上がるほど
  // 脱調しやすくなるため、hzに応じてランプ速度を徐々に減衰させる用途。
  // suction_batt_ramp_gain_table_hz(elec_hz、昇順)・
  // suction_batt_ramp_gain_table_val は同じ長さで指定すること。
  std::vector<float> suction_batt_ramp_gain_table_hz;
  std::vector<float> suction_batt_ramp_gain_table_val;
  float sla_dist = 0;
  int file_idx = 0;
  int sla_type = 0;
  int sla_return = 0;
  int sla_type2 = 0;
  int turn_times = 0;
  int ignore_opp_sen = 0;
  int dia = 0;
  int sysid_test_mode = 0;
  float sysid_duty = 0;
  float sysid_time = 0;
  int start_turn = 0;
  int search_mode = 0;
} test_mode_t;

typedef struct {
  std::vector<point_t> goals;
  int maze_size = 0;
  int user_mode = 0;
  int circuit_mode = 0;
  test_mode_t test;
  int hf_cl = 0;
  float suction_bldc_hz = 6000.0f;
} system_t;

typedef struct {
  int normal = 0;
  int large = 0;
  int orval = 0;
  int dia45 = 0;
  int dia45_2 = 0;
  int dia135 = 0;
  int dia135_2 = 0;
  int dia90 = 0;
} profile_idx_t;

typedef struct {
  std::vector<std::string> file_list;
  int file_list_size = 0;
  int profile_idx_size = 0;
  std::vector<std::unordered_map<TurnType, int>> profile_list;
  std::unordered_map<int, std::unordered_map<TurnType, int>> profile_map;
} turn_param_profile_t;

typedef struct {
  bool enable = 0;
  int timestamp = 0;
} motor_req_t;

typedef struct {
  float right = 0;
  float left = 0;
} slalom_offset_t;

typedef struct {
  float v = 0;
  float end_v = 0;
  float ang = 0;
  float ref_ang = 0;
  float rad = 0;
  float rad2 = 0;
  slalom_offset_t front;
  slalom_offset_t back;
  int pow_n = 0;
  float time = 0;
  float time2 = 0;
  TurnType type = TurnType::None;
} slalom_param2_t;

typedef struct {
  float v_max = 0;
  float accl = 0;
  float decel = 0;
  float w_max = 0;
  float w_end = 0;
  float alpha = 0;
  // 速度→加速度LUT(要素数2未満なら無効、上のaccl固定値のまま)。
  // 走行モード開始時にinput_param_t.accl_v_x/yへコピーして使う
  // (ControlLaw::calc()参照、main_task_run.cpp)。
  std::vector<float> accl_v_x;
  std::vector<float> accl_v_y;
} straight_param_t;

typedef struct {
  std::unordered_map<TurnType, slalom_param2_t> map;
  std::unordered_map<TurnType, slalom_param2_t> map_slow;
  std::unordered_map<TurnType, slalom_param2_t> map_fast;
  std::unordered_map<StraightType, straight_param_t> str_map;
  char suction = 0;
  float suction_duty = 0;
  float suction_duty_low = 0;
  float cell_size = 90;
  float start_offset = 16;
} param_set_t;

typedef struct {
  std::vector<float> path_s;
  std::vector<unsigned char> path_t;
  float time = 10000;
  bool result = false;
  char type = 0;
} path_set_t;

typedef struct {
  float time;
} path_req_t;

typedef struct {
  float time;
} create_path_result_t;

typedef struct {
  bool is_turn = false;
  TurnType next_turn_type = TurnType::None;
  float v_max = 0;
  float v_end = 0;
  float accl = 0;
  float decel = 0;
  bool skip_wall_off = false;
  float carry_over_dist = 0;
} next_motion_t;

typedef struct {
  // int idx;
  float img_v;
  float v_l;
  float v_c;
  float v_r;
  float accl;
  float img_w;
  float w_lp;
  float alpha;

  float img_dist;
  float dist;
  float img_ang;
  float ang;

  float duty_l;
  float duty_r;

  float left90_lp;
  float left45_lp;
  float front_lp;
  float right45_lp;
  float right90_lp;
  float battery_lp;

  char motion_type;

  float duty_ff_front;
  float duty_ff_roll;
  float duty_sensor_ctrl;
  float pos_x;
  float pos_y;
} log_data_t;

union float16_bitmap {
  struct {
    unsigned int s : 1;
    unsigned int e : 8;
    unsigned int m : 23;
  };
  float data;
};

union uint16_bitmap {
  struct {
    unsigned int s : 1;
    unsigned int e : 5;
    unsigned int m : 10;
  };
  int16_t data;
};

typedef struct {
  char fast_idx;
  char normal_idx;
  char slow_idx;
} exec_pram_t;

typedef struct {
  // int idx;
  real16_T img_v;
  real16_T v_l;
  real16_T v_c;
  real16_T v_c2;
  real16_T v_r;
  int16_t v_r_enc;
  int16_t v_l_enc;
  real16_T accl;
  real16_T accl_x;
  real16_T dist_kf;

  real16_T img_w;
  real16_T w_lp;
  real16_T alpha;

  real16_T img_dist;
  real16_T dist;
  real16_T img_ang;
  real16_T ang;
  real16_T ang_kf;

  real16_T duty_l;
  real16_T duty_r;

  int16_t left90_lp;
  int16_t left45_lp;
  // real16_T front_lp;
  int16_t right45_lp;
  int16_t right90_lp;
  real16_T battery_lp;

  int16_t left45_2_lp;
  int16_t right45_2_lp;
  int16_t left45_3_lp;
  int16_t right45_3_lp;

  uint8_t motion_type;
  int16_t motion_timestamp;
  uint8_t continuous_turn; // WallOffController::continuous_turn_flag のミラー

  // real16_T duty_ff_front;
  // real16_T duty_ff_roll;
  real16_T duty_sensor_ctrl;
  real16_T sen_log_l45;
  real16_T sen_log_r45;
  real16_T sen_log_l45_2;
  real16_T sen_log_r45_2;
  real16_T sen_log_l45_3;
  real16_T sen_log_r45_3;
  int16_t sen_calc_time;
  int16_t sen_calc_time2;
  int16_t pln_calc_time;
  // int16_t pln_calc_time2;
  int16_t pln_time_diff;

  real16_T m_pid_p;
  real16_T m_pid_i;
  real16_T m_pid_i2;
  real16_T m_pid_d;
  real16_T m_pid_p_v;
  real16_T m_pid_i_v;
  real16_T m_pid_i2_v;
  real16_T m_pid_d_v;

  real16_T g_pid_p;
  real16_T g_pid_i;
  real16_T g_pid_i2;
  real16_T g_pid_d;
  real16_T g_pid_p_v;
  real16_T g_pid_i_v;
  real16_T g_pid_i2_v;
  real16_T g_pid_d_v;

  real16_T ang_pid_p;
  real16_T ang_pid_i;
  real16_T ang_pid_d;
  real16_T ang_pid_p_v;
  real16_T ang_pid_i_v;
  real16_T ang_pid_d_v;

  real16_T s_pid_p;
  real16_T s_pid_i;
  real16_T s_pid_i2;
  real16_T s_pid_d;
  real16_T s_pid_p_v;
  real16_T s_pid_i_v;
  real16_T s_pid_i2_v;
  real16_T s_pid_d_v;

  real16_T ff_duty_front;
  real16_T ff_duty_roll;
  real16_T ff_duty_rpm_r;
  real16_T ff_duty_rpm_l;
  real16_T ff_front_torque; // torque_mode==2で実際に使われるトルク系FF (ff_duty_frontとは別系統)
  real16_T ff_roll_torque;
  real16_T ff_friction_torque_r;
  real16_T ff_friction_torque_l;
  real16_T v_kf_l;
  real16_T v_kf_r;

  real16_T pos_x;
  real16_T pos_y;

  real16_T knym_v;
  real16_T knym_w;
  real16_T odm_x;
  real16_T odm_y;
  real16_T odm_theta;

  real16_T kim_x;
  real16_T kim_y;
  real16_T kim_theta;

  real16_T ang_i_bias;
  real16_T ang_i_bias_val;

  real16_T duty_suction;

  real16_T ang_kf_sum;
  real16_T img_ang_sum;
  real16_T duty_roll;
  real16_T duty_roll_before;
  real16_T mpc_d_estimated;
  real16_T sat_roll_dir; // apply_duty_limitter()判定のduty飽和方向(+1/-1/0)
  real16_T dbg_off_ang;   // デバッグ用一時フィールド(structs.hpp aw_log_t参照)
  real16_T dbg_off_wgain; // デバッグ用一時フィールド(structs.hpp aw_log_t参照)
  real16_T dbg_off_kny;   // デバッグ用一時フィールド(structs.hpp aw_log_t参照)

  real16_T accel_x; // ASM330LHH加速度計X軸[mm/s^2], gain補正前
  real16_T accel_y; // ASM330LHH加速度計Y軸[mm/s^2], gain補正前
  real16_T accel_z; // ASM330LHH加速度計Z軸[mm/s^2], gain補正前(ピッチング検知用)
  real16_T accel_x_corr; // gyro_pos補正後(車体基準点, 車体座標系)[mm/s^2]
  real16_T accel_y_corr;
  real16_T accel_z_corr;

  int16_t pln_t_ego;
  int16_t pln_t_sensor;
  int16_t pln_t_trj;
  int16_t pln_t_kanayama;
  int16_t pln_t_copy;
  int16_t pln_t_ctl;

  // battery(=batt_kf、duty%換算の分母に使うため強くLPF/KF済み)は速い電圧
  // 降下(加速時の瞬間的な負荷変動等)を捉えられない。battery_rawは生ADC値
  // (ego_estimator.cpp参照)で、フィルタなしの実電圧変動を直接確認する
  // ためのデバッグ用フィールド(2026-08-23追加)。
  real16_T battery_raw;
} log_data_t2;

typedef struct {
  real16_T v_l;
  real16_T v_c;
  real16_T v_r;
  real16_T w_lp;
  real16_T volt_l;
  real16_T volt_r;
} sysid_log;

typedef struct {
  int invalid_front_led;
  int invalid_duty_r_cnt;
  int invalid_duty_l_cnt;
  int invalid_v_cnt;
  int invalid_w_cnt;
} fail_safe_t;

typedef struct {
  float K;
  float k;
  float beta;
  float vx = 0;
  float vy = 0;
  float v = 0;
} slip_t;

typedef struct {
  float star_dist;

} sensor_ctrl_keep_dist_t;

typedef struct {
  float v_start = 0;
  float v_max = 0;
  float v_end = 0;
  float dist = 0;
  float lap_time = 0;
  float total_time = 0;
} planning_time_t;

typedef struct {
  int index = 1;
  float ideal_v = 2;
  float v_c = 3;
  float v_c2 = 4;
  float v_l = 5;
  float v_r = 6;
  int v_l_enc = 7;
  int v_r_enc = 8;
  float v_l_enc_sin = 9;
  float v_r_enc_sin = 10;
  float accl = 11;
  float accl_x = 12;
} LogStruct1;

typedef struct {
  float ideal_w = 13;
  float w_lp = 14;
  float alpha = 15;
  float ideal_dist = 16;

  float dist = 17;
  float dist_kf = 18;
  float ideal_ang = 19;
  float ang = 20;

  float ang_kf = 21;
  int left90 = 22;
  int left45 = 23;
  int front = 24;
} LogStruct2;

typedef struct {
  int right45 = 25;
  int right90 = 26;
  float left90_d = 27;
  float left45_d = 28;

  float front_d = 29;
  float right45_d = 30;
  float right90_d = 31;
  float left90_far_d = 32;

  float front_far_d = 33;
  float right90_far_d = 34;
  float battery = 35;
  float duty_l = 36;
} LogStruct3;

typedef struct {
  float duty_r = 37;
  int motion_state = 38;
  float duty_sen = 39;
  float dist_mod90 = 40;

  float sen_dist_l45 = 41;
  float sen_dist_r45 = 42;
  int timestamp = 43;
  int sen_calc_time = 44;

  int sen_calc_time2 = 45;
  int pln_calc_time = 46;
  int pln_calc_time2 = 47;
  int pln_time_diff = 48;
} LogStruct4;

typedef struct {
  float m_pid_p = 49;
  float m_pid_i = 50;
  float m_pid_i2 = 51;
  float m_pid_d = 52;

  float m_pid_p_v = 53;
  float m_pid_i_v = 54;
  float m_pid_i2_v = 55;
  float m_pid_d_v = 56;

  float g_pid_p = 57;
  float g_pid_i = 58;
  float g_pid_i2 = 59;
  float g_pid_d = 60;
} LogStruct5;

typedef struct {
  float g_pid_p_v = 61;
  float g_pid_i_v = 62;
  float g_pid_i2_v = 63;
  float g_pid_d_v = 64;

  float s_pid_p = 65;
  float s_pid_i = 66;
  float s_pid_i2 = 67;
  float s_pid_d = 68;

  float s_pid_p_v = 69;
  float s_pid_i_v = 70;
  float s_pid_i2_v = 71;
  float s_pid_d_v = 72;
} LogStruct6;

typedef struct {
  float ang_pid_p = 73;
  float ang_pid_i = 74;
  float ang_pid_d = 75;
  float ang_pid_p_v = 76;

  float ang_pid_i_v = 77;
  float ang_pid_d_v = 78;
  float ff_duty_front = 79;
  float ff_duty_roll = 80;

  float ff_duty_rpm_r = 81;
  float ff_duty_rpm_l = 82;
  float x = 83;
  float y = 84;
} LogStruct7;

typedef struct {
  int right45_2 = 85;
  int right45_3 = 86;
  int left45_2 = 87;
  int left45_3 = 88;

  float right45_2_d = 89;
  float right45_3_d = 90;
  float left45_2_d = 91;
  float left45_3_d = 92;

  float sen_dist_l45_2 = 93;
  float sen_dist_r45_2 = 94;
  float sen_dist_l45_3 = 95;
  float sen_dist_r45_3 = 96;

} LogStruct8;
typedef struct {
  float knym_v = 98;
  float knym_w = 99;
  float odm_x = 100;
  float odm_y = 101;

  float odm_theta = 102;
  float kim_x = 103;
  float kim_y = 104;
  float kim_theta = 105;

  float ang_i_bias = 106;
  float ang_i_bias_val = 107;
  float left90_d_diff = 108;
  float right90_d_diff = 109;

} LogStruct9;

typedef struct {
  float right45_3_d_diff = 110;
  float right45_2_d_diff = 111;
  float right45_d_diff = 112;
  float left45_d_diff = 113;

  float left45_2_d_diff = 114;
  float left45_3_d_diff = 115;
  float duty_suction = 116;
  float duty_roll = 117;
  float ang_kf_sum = 118;
  float img_ang_sum = 119;
  float duty_roll_before = 120;
  float mpc_d_estimated = 121; // 旧 reserve5。yawモデル外乱オブザーバの推定値(control_law.cpp calc_angle_velocity_ctrl)

} LogStruct10;

typedef struct {
  int pln_t_ego      = 122;
  int pln_t_sensor   = 123;
  int pln_t_trj      = 124;
  int pln_t_kanayama = 125;
  int pln_t_copy     = 126;
  int pln_t_ctl      = 127;
  float ff_front_torque = 128; // torque_mode==2 実適用値 (summation_duty)
  float ff_roll_torque  = 129;
  int continuous_turn   = 130; // 連続ターン中の壁切れ閾値切替フラグ (WallOffController)
  float sat_roll_dir    = 131; // duty飽和方向(conditional integrationの動作確認用)
  float ff_friction_torque_r = 132; // クーロン+粘性摩擦FF(summation_duty, torque_mode==2実適用値)
  float ff_friction_torque_l = 133;
  float v_kf_l = 134; // kf_v.update()に渡るkf_v_l/kf_v_rの状態(v_c2の入力の可視化用)
  float v_kf_r = 135;
  float accel_x = 136; // ASM330LHH加速度計X軸[mm/s^2], gain補正前
  float accel_y = 137;
  float accel_z = 138; // ピッチング検知用
  float accel_x_corr = 139; // gyro_pos補正後(車体基準点, 車体座標系)[mm/s^2]
  float accel_y_corr = 140;
  float accel_z_corr = 141;
  float battery_raw = 142; // フィルタ無しの生バッテリ電圧(structs.hpp log_data_t2参照)
  float dbg_off_ang   = 143; // デバッグ用一時フィールド(structs.hpp aw_log_t参照)
  float dbg_off_wgain = 144; // デバッグ用一時フィールド(structs.hpp aw_log_t参照)
  float dbg_off_kny   = 145; // デバッグ用一時フィールド(structs.hpp aw_log_t参照)
} LogStruct11;

#endif