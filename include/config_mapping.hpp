#pragma once
#include <ArduinoJson.h>
#include <cstring>
#include <type_traits>
#include <vector>
#include "structs.hpp"
#include "maze_solver.hpp"

// ─── テンプレートヘルパー ────────────────────────────────────────────────────
// convertFromJson 内でフィールドを読み取る際に使用する。
//
//  from_json_field(src, "key", dst.field)
//    スカラー (float/int/bool) および enum class フィールドに対応。
//    enum は JSON の int として格納し static_cast で変換する。
//    volatile T& のオーバーロードでそのまま volatile フィールドにも使える。
//
//  from_json_nested(src, "key", dst.sub)
//    入れ子の構造体を convertFromJson(ADL) に委譲する。
//    volatile T& の場合は tmp を介して memcpy する。
//
//  from_json_array(src, "key", dst.arr)   ← T (&)[N] 固定長 C 配列
//  from_json_vector(src, "key", dst.vec)  ← std::vector<T>
//
// 注意: ビットフィールドは C++ で参照を取れないため、
//       from_json_field ではなく従来の is<>/as<> を直接使うこと。

template<typename T>
inline void from_json_field(JsonVariantConst src, const char* key, T& dst) {
    if constexpr (std::is_enum_v<T>) {
        if (src[key].is<int>()) dst = static_cast<T>(src[key].as<int>());
    } else if constexpr (std::is_same_v<T, char>) {
        if (src[key].is<int>()) dst = static_cast<char>(src[key].as<int>());
    } else {
        if (src[key].is<T>()) dst = src[key].as<T>();
    }
}

// volatile スカラー / volatile enum
template<typename T>
inline void from_json_field(JsonVariantConst src, const char* key, volatile T& dst) {
    if constexpr (std::is_enum_v<T>) {
        if (src[key].is<int>()) dst = static_cast<T>(src[key].as<int>());
    } else if constexpr (std::is_same_v<T, char>) {
        if (src[key].is<int>()) dst = static_cast<char>(src[key].as<int>());
    } else {
        if (src[key].is<T>()) dst = src[key].as<T>();
    }
}

// 入れ子の構造体 (convertFromJson への ADL 委譲)
template<typename T>
inline void from_json_nested(JsonVariantConst src, const char* key, T& dst) {
    if (!src[key].isNull()) convertFromJson(src[key], dst);
}

// volatile 入れ子の構造体 (tmp 経由で memcpy)
template<typename T>
inline void from_json_nested(JsonVariantConst src, const char* key, volatile T& dst) {
    if (src[key].isNull()) return;
    T tmp{};
    convertFromJson(src[key], tmp);
    memcpy(const_cast<T*>(&dst), &tmp, sizeof(T));
}

// 固定長 C 配列
template<typename T, size_t N>
inline void from_json_array(JsonVariantConst src, const char* key, T (&dst)[N]) {
    if (!src[key].is<JsonArrayConst>()) return;
    size_t i = 0;
    for (JsonVariantConst v : src[key].as<JsonArrayConst>())
        if (i < N) dst[i++] = v.as<T>();
}

// std::vector / std::deque / std::list など clear() + push_back() を持つコンテナ全般
template<typename Container>
inline void from_json_vector(JsonVariantConst src, const char* key, Container& dst) {
    if (!src[key].is<JsonArrayConst>()) return;
    dst.clear();
    for (JsonVariantConst v : src[key].as<JsonArrayConst>())
        dst.push_back(v.as<typename Container::value_type>());
}
// ─────────────────────────────────────────────────────────────────────────────

/**
 * system.txt
 * root → goals[i]
 */
inline void convertFromJson(JsonVariantConst src, point_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
}

/**
 * hardware.txt
 * root → motor_pid | gyro_pid | front_ctrl_roll_pid
 *             | str_ang_pid | angle_pid | sensor_pid_dia | ...
 */
inline void convertFromJson(JsonVariantConst src, pid_param_t& dst) {
    from_json_field(src, "p", dst.p);
    from_json_field(src, "i", dst.i);
    from_json_field(src, "d", dst.d);
    from_json_field(src, "b", dst.b);
    from_json_field(src, "c", dst.c);
    from_json_field(src, "mode", dst.mode);
    from_json_field(src, "antiwindup", dst.antiwindup);
    from_json_field(src, "windup_gain", dst.windup_gain);
    from_json_field(src, "windup_dead_bind", dst.windup_dead_bind);
    from_json_field(src, "i_theta_tau", dst.i_theta_tau);
    from_json_field(src, "theta_gate", dst.theta_gate);
    from_json_field(src, "omega_gate", dst.omega_gate);
    from_json_field(src, "i_theta_slew", dst.i_theta_slew);
    from_json_field(src, "i_theta_max", dst.i_theta_max);
    from_json_field(src, "alpha_stop", dst.alpha_stop);
    from_json_field(src, "alpha_rate", dst.alpha_rate);
    from_json_field(src, "theta_damp_th", dst.theta_damp_th);
    from_json_field(src, "omega_damp", dst.omega_damp);
    from_json_field(src, "th", dst.th);
    from_json_field(src, "theta_gate_on", dst.theta_gate_on);
    from_json_field(src, "theta_gate_full", dst.theta_gate_full);
    from_json_field(src, "theta_kp", dst.theta_kp);
    from_json_field(src, "theta_kd", dst.theta_kd);
    from_json_field(src, "omega_add_max", dst.omega_add_max);
    from_json_field(src, "alpha_rate_end", dst.alpha_rate_end);
    from_json_field(src, "k_stop", dst.k_stop);
    from_json_field(src, "theta_eps", dst.theta_eps);
    from_json_field(src, "s_gate", dst.s_gate);
    from_json_field(src, "mpc_q_ang", dst.mpc_q_ang);
    from_json_field(src, "mpc_q_vel", dst.mpc_q_vel);
    from_json_field(src, "mpc_b", dst.mpc_b);
    from_json_field(src, "mpc_r", dst.mpc_r);
    from_json_field(src, "mpc_horizon", dst.mpc_horizon);
    from_json_field(src, "mpc_max_iter", dst.mpc_max_iter);
    from_json_field(src, "mpc_max_torque", dst.mpc_max_torque);
    from_json_field(src, "mpc_observer_k", dst.mpc_observer_k);
}

/**
 * hardware.txt
 * root → gyro_param | gyro2_param
 *             | battery_param | led_param
 */
inline void convertFromJson(JsonVariantConst src, gyro_param_t& dst) {
    from_json_field(src, "gyro_w_gain_right", dst.gyro_w_gain_right);
    from_json_field(src, "gyro_w_gain_left", dst.gyro_w_gain_left);
    from_json_field(src, "retry_min_th", dst.retry_min_th);
    from_json_field(src, "retry_max_th", dst.retry_max_th);
    from_json_field(src, "robust_th", dst.robust_th);
    from_json_field(src, "lp_delay", dst.lp_delay);
    from_json_field(src, "list_size", dst.list_size);
    from_json_field(src, "loop_size", dst.loop_size);
}

/**
 * hardware.txt
 * root → accel_x_param
 */
inline void convertFromJson(JsonVariantConst src, accel_param_t& dst) {
    from_json_field(src, "gain", dst.gain);
}

/**
 * hardware.txt
 * root → sen_param
 */
inline void convertFromJson(JsonVariantConst src, sen_param_t& dst) {
    from_json_field(src, "lp_delay", dst.lp_delay);
}

/**
 * sensor.txt
 * root → <mode> → ref | exist | search_ref
 */
inline void convertFromJson(JsonVariantConst src, sen_ref_param3_t& dst) {
    from_json_field(src, "right45", dst.right45);
    from_json_field(src, "left45", dst.left45);
    from_json_field(src, "right90", dst.right90);
    from_json_field(src, "left90", dst.left90);
    from_json_field(src, "front", dst.front);
    from_json_field(src, "kireme_r", dst.kireme_r);
    from_json_field(src, "kireme_l", dst.kireme_l);
    from_json_field(src, "kireme_r_fast", dst.kireme_r_fast);
    from_json_field(src, "kireme_l_fast", dst.kireme_l_fast);
    from_json_field(src, "kireme_r_wall_off", dst.kireme_r_wall_off);
    from_json_field(src, "kireme_l_wall_off", dst.kireme_l_wall_off);
    from_json_field(src, "kireme_r_wall_off2", dst.kireme_r_wall_off2);
    from_json_field(src, "kireme_l_wall_off2", dst.kireme_l_wall_off2);
}

/**
 * sensor.txt
 * root → <mode> → ref_search
 */
inline void convertFromJson(JsonVariantConst src, sen_search_param_t& dst) {
    from_json_field(src, "front", dst.front);
    from_json_field(src, "right45", dst.right45);
    from_json_field(src, "left45", dst.left45);
    from_json_field(src, "right90", dst.right90);
    from_json_field(src, "left90", dst.left90);
    from_json_field(src, "kireme_r", dst.kireme_r);
    from_json_field(src, "kireme_l", dst.kireme_l);
    from_json_field(src, "offset_r", dst.offset_r);
    from_json_field(src, "offset_l", dst.offset_l);
    from_json_field(src, "front_ctrl", dst.front_ctrl);
    from_json_field(src, "front_ctrl_th", dst.front_ctrl_th);
}

/**
 * sensor.txt
 * root → <mode> → expand
 */
inline void convertFromJson(JsonVariantConst src, sen_expand_param_t& dst) {
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "right45", dst.right45);
    from_json_field(src, "left45", dst.left45);
    from_json_field(src, "right45_2", dst.right45_2);
    from_json_field(src, "left45_2", dst.left45_2);
}

/**
 * sensor.txt
 * root → normal | normal2 | dia | search_exist | search_ref
 */
inline void convertFromJson(JsonVariantConst src, sen_ref_param2_t& dst) {
    from_json_nested(src, "ref", dst.ref);
    from_json_nested(src, "ref_search", dst.ref_search);
    from_json_nested(src, "exist", dst.exist);
    from_json_nested(src, "expand", dst.expand);
}

/**
 * sensor.txt
 * root
 */
inline void convertFromJson(JsonVariantConst src, sen_ref_param_t& dst) {
    from_json_nested(src, "normal", dst.normal);
    from_json_nested(src, "normal2", dst.normal2);
    from_json_nested(src, "dia", dst.dia);
    from_json_nested(src, "search_exist", dst.search_exist);
    from_json_nested(src, "search_ref", dst.search_ref);
}

/**
 * sensor.txt
 * root → sensor_gain → l90 | l45 | front | r45 | r90 | ...
 */
inline void convertFromJson(JsonVariantConst src, sensor_gain_param_t& dst) {
    from_json_field(src, "a", dst.a);
    from_json_field(src, "b", dst.b);
}

/**
 * sensor.txt
 * root → sensor_gain
 */
inline void convertFromJson(JsonVariantConst src, sensor_gain_t& dst) {
    from_json_nested(src, "l90", dst.l90);
    from_json_nested(src, "l45", dst.l45);
    from_json_nested(src, "front", dst.front);
    from_json_nested(src, "front2", dst.front2);
    from_json_nested(src, "front3", dst.front3);
    from_json_nested(src, "front4", dst.front4);
    from_json_nested(src, "front_ctrl_th", dst.front_ctrl_th);
    from_json_nested(src, "r45", dst.r45);
    from_json_nested(src, "l45_2", dst.l45_2);
    from_json_nested(src, "r45_2", dst.r45_2);
    from_json_nested(src, "l45_3", dst.l45_3);
    from_json_nested(src, "r45_3", dst.r45_3);
    from_json_nested(src, "r90", dst.r90);
    from_json_nested(src, "l90_far", dst.l90_far);
    from_json_nested(src, "r90_far", dst.r90_far);
    from_json_nested(src, "l90_mid", dst.l90_mid);
    from_json_nested(src, "r90_mid", dst.r90_mid);
}

/**
 * offset.txt
 * root → wall_off_dist
 */
inline void convertFromJson(JsonVariantConst src, wall_off_hold_dist_t& dst) {
    from_json_field(src, "left_str", dst.left_str);
    from_json_field(src, "right_str", dst.right_str);
    from_json_field(src, "left_diff_th", dst.left_diff_th);
    from_json_field(src, "right_diff_th", dst.right_diff_th);
    from_json_field(src, "left_str_exist", dst.left_str_exist);
    from_json_field(src, "right_str_exist", dst.right_str_exist);
    from_json_field(src, "left_dia", dst.left_dia);
    from_json_field(src, "right_dia", dst.right_dia);
    from_json_field(src, "left_dia_noexit", dst.left_dia_noexit);
    from_json_field(src, "right_dia_noexit", dst.right_dia_noexit);
    from_json_field(src, "left_dia_oppo", dst.left_dia_oppo);
    from_json_field(src, "right_dia_oppo", dst.right_dia_oppo);
    from_json_field(src, "left_dia2", dst.left_dia2);
    from_json_field(src, "right_dia2", dst.right_dia2);
    from_json_field(src, "exist_dist_l", dst.exist_dist_l);
    from_json_field(src, "exist_dist_r", dst.exist_dist_r);
    from_json_field(src, "exist_dist_l2", dst.exist_dist_l2);
    from_json_field(src, "exist_dist_r2", dst.exist_dist_r2);
    from_json_field(src, "noexist_th_l", dst.noexist_th_l);
    from_json_field(src, "noexist_th_r", dst.noexist_th_r);
    from_json_field(src, "noexist_th_l2", dst.noexist_th_l2);
    from_json_field(src, "noexist_th_r2", dst.noexist_th_r2);
    from_json_field(src, "div_th_l", dst.div_th_l);
    from_json_field(src, "div_th_r", dst.div_th_r);
    from_json_field(src, "div_th_l2", dst.div_th_l2);
    from_json_field(src, "div_th_r2", dst.div_th_r2);
    from_json_field(src, "div_th_l3", dst.div_th_l3);
    from_json_field(src, "div_th_r3", dst.div_th_r3);
    from_json_field(src, "div_th_dia_l", dst.div_th_dia_l);
    from_json_field(src, "div_th_dia_r", dst.div_th_dia_r);
    from_json_field(src, "exist_dia_th_l", dst.exist_dia_th_l);
    from_json_field(src, "exist_dia_th_r", dst.exist_dia_th_r);
    from_json_field(src, "exist_dia_th_l2", dst.exist_dia_th_l2);
    from_json_field(src, "exist_dia_th_r2", dst.exist_dia_th_r2);
    from_json_field(src, "noexist_dia_th_l", dst.noexist_dia_th_l);
    from_json_field(src, "noexist_dia_th_r", dst.noexist_dia_th_r);
    from_json_field(src, "noexist_dia_th_l2", dst.noexist_dia_th_l2);
    from_json_field(src, "noexist_dia_th_r2", dst.noexist_dia_th_r2);
    from_json_field(src, "wall_off_exist_wall_th_l", dst.wall_off_exist_wall_th_l);
    from_json_field(src, "wall_off_exist_wall_th_r", dst.wall_off_exist_wall_th_r);
    from_json_field(src, "wall_off_exist_dia_wall_th_l", dst.wall_off_exist_dia_wall_th_l);
    from_json_field(src, "wall_off_exist_dia_wall_th_r", dst.wall_off_exist_dia_wall_th_r);
    from_json_field(src, "search_wall_off_enable", dst.search_wall_off_enable);
    from_json_field(src, "search_wall_off_l_dist_offset", dst.search_wall_off_l_dist_offset);
    from_json_field(src, "search_wall_off_r_dist_offset", dst.search_wall_off_r_dist_offset);
    from_json_field(src, "search_wall_off_offset_dist", dst.search_wall_off_offset_dist);
    from_json_field(src, "ctrl_exist_wall_th_l", dst.ctrl_exist_wall_th_l);
    from_json_field(src, "ctrl_exist_wall_th_r", dst.ctrl_exist_wall_th_r);
    from_json_field(src, "go_straight_wide_ctrl_th", dst.go_straight_wide_ctrl_th);
    from_json_field(src, "diff_check_dist", dst.diff_check_dist);
    from_json_field(src, "diff_dist_th_l", dst.diff_dist_th_l);
    from_json_field(src, "diff_dist_th_r", dst.diff_dist_th_r);
    from_json_field(src, "diff_check_dist_dia", dst.diff_check_dist_dia);
    from_json_field(src, "diff_check_dist_dia_2", dst.diff_check_dist_dia_2);
}

/**
 * hardware.txt
 * root → fail_check
 */
inline void convertFromJson(JsonVariantConst src, fail_check_cnt_t& dst) {
    from_json_field(src, "duty", dst.duty);
    from_json_field(src, "v", dst.v);
    from_json_field(src, "w", dst.w);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "wall_off", dst.wall_off);
}

/**
 * hardware.txt
 * root → comp_param
 */
inline void convertFromJson(JsonVariantConst src, comp_param_t& dst) {
    from_json_field(src, "v_lp_gain", dst.v_lp_gain);
    from_json_field(src, "accl_x_hp_gain", dst.accl_x_hp_gain);
    from_json_field(src, "gain", dst.gain);
    from_json_field(src, "enable", dst.enable);
}

/**
 * offset.txt
 * root → kanayama
 */
inline void convertFromJson(JsonVariantConst src, kanayama_t& dst) {
    from_json_field(src, "kx", dst.kx);
    from_json_field(src, "ky", dst.ky);
    from_json_field(src, "k_theta", dst.k_theta);
    from_json_field(src, "enable", dst.enable);
    from_json_field(src, "windup", dst.windup);
    from_json_field(src, "windup_deg", dst.windup_deg);
}

/**
 * hardware.txt | offset.txt | sensor.txt
 * root
 */
inline void convertFromJson(JsonVariantConst src, input_param_t& dst) {
    from_json_field(src, "dt", dst.dt);
    from_json_field(src, "trj_length", dst.trj_length);
    from_json_field(src, "tire", dst.tire);
    from_json_field(src, "tire2", dst.tire2);
    from_json_field(src, "log_size", dst.log_size);
    from_json_field(src, "gear_a", dst.gear_a);
    from_json_field(src, "gear_b", dst.gear_b);
    from_json_field(src, "max_duty", dst.max_duty);
    from_json_field(src, "min_duty", dst.min_duty);
    from_json_field(src, "battery_gain", dst.battery_gain);
    from_json_field(src, "Ke", dst.Ke);
    from_json_field(src, "Km", dst.Km);
    from_json_field(src, "Resist", dst.Resist);
    from_json_field(src, "Mass", dst.Mass);
    from_json_field(src, "Lm", dst.Lm);
    from_json_field(src, "coulomb_friction", dst.coulomb_friction);
    from_json_field(src, "viscous_friction", dst.viscous_friction);
    from_json_field(src, "battery_init_cov", dst.battery_init_cov);
    from_json_field(src, "battery_p_noise", dst.battery_p_noise);
    from_json_field(src, "battery_m_noise", dst.battery_m_noise);
    from_json_field(src, "encoder_init_cov", dst.encoder_init_cov);
    from_json_field(src, "encoder_p_noise", dst.encoder_p_noise);
    from_json_field(src, "encoder_m_noise", dst.encoder_m_noise);
    from_json_field(src, "w_init_cov", dst.w_init_cov);
    from_json_field(src, "w_p_noise", dst.w_p_noise);
    from_json_field(src, "w_m_noise", dst.w_m_noise);
    from_json_field(src, "v_init_cov", dst.v_init_cov);
    from_json_field(src, "v_p_noise", dst.v_p_noise);
    from_json_field(src, "v_m_noise", dst.v_m_noise);
    from_json_field(src, "ang_init_cov", dst.ang_init_cov);
    from_json_field(src, "ang_p_noise", dst.ang_p_noise);
    from_json_field(src, "ang_m_noise", dst.ang_m_noise);
    from_json_field(src, "dist_init_cov", dst.dist_init_cov);
    from_json_field(src, "dist_p_noise", dst.dist_p_noise);
    from_json_field(src, "dist_m_noise", dst.dist_m_noise);
    from_json_field(src, "pos_init_cov", dst.pos_init_cov);
    from_json_field(src, "pos_p_noise", dst.pos_p_noise);
    from_json_field(src, "pos_m_noise", dst.pos_m_noise);
    from_json_field(src, "tread", dst.tread);
    from_json_field(src, "FF_front", dst.FF_front);
    from_json_field(src, "FF_roll", dst.FF_roll);
    from_json_field(src, "FF_keV", dst.FF_keV);
    from_json_field(src, "offset_start_dist", dst.offset_start_dist);
    from_json_field(src, "offset_start_dist_search", dst.offset_start_dist_search);
    from_json_field(src, "long_run_offset_dist", dst.long_run_offset_dist);
    from_json_field(src, "pivot_back_offset", dst.pivot_back_offset);
    from_json_field(src, "cell", dst.cell);
    from_json_field(src, "cell2", dst.cell2);
    from_json_field(src, "pivot_angle_180", dst.pivot_angle_180);
    from_json_field(src, "pivot_angle_90", dst.pivot_angle_90);
    from_json_field(src, "wall_off_front_move_dist_th", dst.wall_off_front_move_dist_th);
    from_json_field(src, "wall_off_front_move_dia_dist_th", dst.wall_off_front_move_dia_dist_th);
    from_json_field(src, "ff_front_gain_14", dst.ff_front_gain_14);
    from_json_field(src, "ff_roll_gain_before", dst.ff_roll_gain_before);
    from_json_field(src, "ff_roll_gain_after", dst.ff_roll_gain_after);
    from_json_field(src, "ff_front_gain_decel", dst.ff_front_gain_decel);
    from_json_nested(src, "front_ctrl_roll_pid", dst.front_ctrl_roll_pid);
    from_json_nested(src, "motor_pid", dst.motor_pid);
    from_json_nested(src, "motor_pid_gain_limitter", dst.motor_pid_gain_limitter);
    from_json_nested(src, "motor_pid2", dst.motor_pid2);
    from_json_nested(src, "motor2_pid_gain_limitter", dst.motor2_pid_gain_limitter);
    from_json_nested(src, "motor_pid3", dst.motor_pid3);
    from_json_nested(src, "gyro_pid", dst.gyro_pid);
    from_json_nested(src, "gyro_pid_gain_limitter", dst.gyro_pid_gain_limitter);
    from_json_nested(src, "str_ang_pid", dst.str_ang_pid);
    from_json_nested(src, "str_ang_dia_pid", dst.str_ang_dia_pid);
    from_json_nested(src, "angle_pid", dst.angle_pid);
    from_json_nested(src, "front_ctrl_angle_pid", dst.front_ctrl_angle_pid);
    from_json_nested(src, "front_ctrl_dist_pid", dst.front_ctrl_dist_pid);
    from_json_nested(src, "front_ctrl_keep_angle_pid", dst.front_ctrl_keep_angle_pid);
    from_json_nested(src, "sensor_pid_dia", dst.sensor_pid_dia);
    from_json_nested(src, "gyro_param", dst.gyro_param);
    from_json_nested(src, "gyro2_param", dst.gyro2_param);
    from_json_nested(src, "accel_x_param", dst.accel_x_param);
    from_json_nested(src, "comp_param", dst.comp_param);
    from_json_nested(src, "battery_param", dst.battery_param);
    from_json_nested(src, "led_param", dst.led_param);
    from_json_field(src, "motion_dir", dst.motion_dir);
    from_json_nested(src, "sen_ref_p", dst.sen_ref_p);
    from_json_nested(src, "sensor_gain", dst.sensor_gain);
    from_json_field(src, "sakiyomi_time", dst.sakiyomi_time);
    from_json_field(src, "search_sen_ctrl_limitter", dst.search_sen_ctrl_limitter);
    from_json_field(src, "clear_angle", dst.clear_angle);
    from_json_field(src, "clear_dist_order", dst.clear_dist_order);
    from_json_field(src, "front_dist_offset", dst.front_dist_offset);
    from_json_field(src, "front_dist_offset0", dst.front_dist_offset0);
    from_json_field(src, "front_dist_offset2", dst.front_dist_offset2);
    from_json_field(src, "front_dist_offset3", dst.front_dist_offset3);
    from_json_field(src, "front_dist_offset4", dst.front_dist_offset4);
    from_json_field(src, "front_dist_offset_dia_front", dst.front_dist_offset_dia_front);
    from_json_field(src, "front_dist_offset_dia_45_th", dst.front_dist_offset_dia_45_th);
    from_json_field(src, "front_dist_offset_dia_right45", dst.front_dist_offset_dia_right45);
    from_json_field(src, "front_dist_offset_dia_left45", dst.front_dist_offset_dia_left45);
    from_json_field(src, "sla_wall_ref_l", dst.sla_wall_ref_l);
    from_json_field(src, "sla_wall_ref_r", dst.sla_wall_ref_r);
    from_json_field(src, "sla_max_offset_dist", dst.sla_max_offset_dist);
    from_json_field(src, "large_offset_enable", dst.large_offset_enable);
    from_json_field(src, "dia45_offset_enable", dst.dia45_offset_enable);
    from_json_field(src, "dia135_offset_enable", dst.dia135_offset_enable);
    from_json_field(src, "orval_offset_enable", dst.orval_offset_enable);
    from_json_field(src, "large_offset_max_dist", dst.large_offset_max_dist);
    from_json_field(src, "dia45_offset_max_dist", dst.dia45_offset_max_dist);
    from_json_field(src, "dia135_offset_max_dist", dst.dia135_offset_max_dist);
    from_json_field(src, "orval_offset_max_dist", dst.orval_offset_max_dist);
    from_json_field(src, "dia45_2_offset_max_dist", dst.dia45_2_offset_max_dist);
    from_json_field(src, "dia135_2_offset_max_dist", dst.dia135_2_offset_max_dist);
    from_json_field(src, "dia90_offset_max_dist", dst.dia90_offset_max_dist);
    from_json_field(src, "lim_angle", dst.lim_angle);
    from_json_field(src, "front_ctrl_error_th", dst.front_ctrl_error_th);
    from_json_field(src, "clear_dist_ragne_from", dst.clear_dist_ragne_from);
    from_json_field(src, "clear_dist_ragne_to", dst.clear_dist_ragne_to);
    from_json_field(src, "clear_dist_ragne_to2", dst.clear_dist_ragne_to2);
    from_json_vector(src, "clear_dist_ragne_dist_list", dst.clear_dist_ragne_dist_list);
    from_json_vector(src, "clear_dist_ragne_th_list", dst.clear_dist_ragne_th_list);
    from_json_vector(src, "clear_dist_ragne_dist_list_fast", dst.clear_dist_ragne_dist_list_fast);
    from_json_field(src, "wall_off_hold_dist", dst.wall_off_hold_dist);
    from_json_nested(src, "wall_off_dist", dst.wall_off_dist);
    from_json_field(src, "wall_off_diff_ref_th", dst.wall_off_diff_ref_th);
    from_json_field(src, "wall_off_diff_ref_front_th", dst.wall_off_diff_ref_front_th);
    from_json_field(src, "wall_off_wait_dist", dst.wall_off_wait_dist);
    from_json_field(src, "wall_off_wait_dist_dia", dst.wall_off_wait_dist_dia);
    from_json_field(src, "search_log_enable", dst.search_log_enable);
    from_json_field(src, "seach_timer", dst.seach_timer);
    from_json_field(src, "test_log_enable", dst.test_log_enable);
    from_json_field(src, "fast_log_enable", dst.fast_log_enable);
    from_json_field(src, "front_dist_offset_pivot_th", dst.front_dist_offset_pivot_th);
    from_json_field(src, "front_dist_offset_pivot", dst.front_dist_offset_pivot);
    from_json_field(src, "pivot_back_dist0", dst.pivot_back_dist0);
    from_json_field(src, "pivot_back_dist1", dst.pivot_back_dist1);
    from_json_field(src, "sen_log_size", dst.sen_log_size);
    from_json_field(src, "sen_log_size2", dst.sen_log_size2);
    from_json_field(src, "led_light_delay_cnt", dst.led_light_delay_cnt);
    from_json_field(src, "led_light_delay_cnt2", dst.led_light_delay_cnt2);
    from_json_field(src, "set_param", dst.set_param);
    from_json_field(src, "logging_time", dst.logging_time);
    from_json_field(src, "offset_after_turn_l2", dst.offset_after_turn_l2);
    from_json_field(src, "offset_after_turn_r2", dst.offset_after_turn_r2);
    from_json_field(src, "offset_after_turn_l", dst.offset_after_turn_l);
    from_json_field(src, "offset_after_turn_r", dst.offset_after_turn_r);
    from_json_field(src, "offset_after_turn_dia_l", dst.offset_after_turn_dia_l);
    from_json_field(src, "offset_after_turn_dia_r", dst.offset_after_turn_dia_r);
    from_json_field(src, "dia_turn_exist_th_l", dst.dia_turn_exist_th_l);
    from_json_field(src, "dia_turn_exist_th_r", dst.dia_turn_exist_th_r);
    from_json_field(src, "dia_turn_th_l", dst.dia_turn_th_l);
    from_json_field(src, "dia_turn_th_r", dst.dia_turn_th_r);
    from_json_field(src, "dia_turn_ref_l", dst.dia_turn_ref_l);
    from_json_field(src, "dia_turn_ref_r", dst.dia_turn_ref_r);
    from_json_field(src, "dia_turn_max_dist_l", dst.dia_turn_max_dist_l);
    from_json_field(src, "dia_turn_max_dist_r", dst.dia_turn_max_dist_r);
    from_json_field(src, "wall_off_pass_dist", dst.wall_off_pass_dist);
    from_json_field(src, "dia_wall_off_ref_l", dst.dia_wall_off_ref_l);
    from_json_field(src, "dia_wall_off_ref_r", dst.dia_wall_off_ref_r);
    from_json_field(src, "dia_wall_off_ref_l_wall", dst.dia_wall_off_ref_l_wall);
    from_json_field(src, "dia_wall_off_ref_r_wall", dst.dia_wall_off_ref_r_wall);
    from_json_field(src, "dia_wall_off_ref_l_wall2", dst.dia_wall_off_ref_l_wall2);
    from_json_field(src, "dia_wall_off_ref_r_wall2", dst.dia_wall_off_ref_r_wall2);
    from_json_field(src, "dia_wall_off_ref_l_piller", dst.dia_wall_off_ref_l_piller);
    from_json_field(src, "dia_wall_off_ref_r_piller", dst.dia_wall_off_ref_r_piller);
    from_json_field(src, "dia_offset_max_dist", dst.dia_offset_max_dist);
    from_json_field(src, "slip_param_K", dst.slip_param_K);
    from_json_field(src, "slip_param_k2", dst.slip_param_k2);
    from_json_nested(src, "fail_check", dst.fail_check);
    from_json_field(src, "fail_check_ang_th", dst.fail_check_ang_th);
    from_json_field(src, "normal_sla_offset", dst.normal_sla_offset);
    from_json_field(src, "normal_sla_offset_front", dst.normal_sla_offset_front);
    from_json_field(src, "normal_sla_offset_back", dst.normal_sla_offset_back);
    from_json_field(src, "front_diff_th", dst.front_diff_th);
    from_json_field(src, "ff_v_th", dst.ff_v_th);
    from_json_field(src, "ff_front_dury", dst.ff_front_dury);
    from_json_field(src, "motor_driver_type", dst.motor_driver_type);
    from_json_field(src, "motor_debug_mode", dst.motor_debug_mode);
    from_json_field(src, "motor_r_cw_ccw_type", dst.motor_r_cw_ccw_type);
    from_json_field(src, "motor_l_cw_ccw_type", dst.motor_l_cw_ccw_type);
    from_json_field(src, "motor_debug_mode_duty_r", dst.motor_debug_mode_duty_r);
    from_json_field(src, "motor_debug_mode_duty_l", dst.motor_debug_mode_duty_l);
    from_json_field(src, "pivot_straight", dst.pivot_straight);
    from_json_field(src, "pivot_back_enable_front_th", dst.pivot_back_enable_front_th);
    from_json_field(src, "search_front_ctrl_th", dst.search_front_ctrl_th);
    from_json_field(src, "judge_pivot", dst.judge_pivot);
    from_json_field(src, "sensor_range_min", dst.sensor_range_min);
    from_json_field(src, "sensor_range_max", dst.sensor_range_max);
    from_json_field(src, "sensor_range_mid_max", dst.sensor_range_mid_max);
    from_json_field(src, "sensor_range_far_max", dst.sensor_range_far_max);
    from_json_field(src, "dist_mod_num", dst.dist_mod_num);
    from_json_field(src, "sen_ctrl_front_th", dst.sen_ctrl_front_th);
    from_json_field(src, "sen_ctrl_front_diff_th", dst.sen_ctrl_front_diff_th);
    from_json_field(src, "th_offset_dist", dst.th_offset_dist);
    from_json_field(src, "sla_front_ctrl_th", dst.sla_front_ctrl_th);
    from_json_field(src, "orval_front_ctrl_min", dst.orval_front_ctrl_min);
    from_json_field(src, "orval_front_ctrl_max", dst.orval_front_ctrl_max);
    from_json_field(src, "wall_off_front_ctrl_min", dst.wall_off_front_ctrl_min);
    from_json_field(src, "dia_turn_offset_calc_th", dst.dia_turn_offset_calc_th);
    from_json_field(src, "go_straight_wide_ctrl_th", dst.go_straight_wide_ctrl_th);
    from_json_field(src, "wall_off_pass_through_offset_r", dst.wall_off_pass_through_offset_r);
    from_json_field(src, "wall_off_pass_through_offset_l", dst.wall_off_pass_through_offset_l);
    from_json_field(src, "tire_tread", dst.tire_tread);
    from_json_field(src, "right_keep_dist_th", dst.right_keep_dist_th);
    from_json_field(src, "left_keep_dist_th", dst.left_keep_dist_th);
    from_json_field(src, "normal_sla_l_wall_off_th_in", dst.normal_sla_l_wall_off_th_in);
    from_json_field(src, "normal_sla_r_wall_off_th_in", dst.normal_sla_r_wall_off_th_in);
    from_json_field(src, "normal_sla_l_wall_off_th_out", dst.normal_sla_l_wall_off_th_out);
    from_json_field(src, "normal_sla_r_wall_off_th_out", dst.normal_sla_r_wall_off_th_out);
    from_json_field(src, "normal_sla_l_wall_off_ref_cnt", dst.normal_sla_l_wall_off_ref_cnt);
    from_json_field(src, "normal_sla_r_wall_off_ref_cnt", dst.normal_sla_r_wall_off_ref_cnt);
    from_json_field(src, "normal_sla_l_wall_off_dist", dst.normal_sla_l_wall_off_dist);
    from_json_field(src, "normal_sla_r_wall_off_dist", dst.normal_sla_r_wall_off_dist);
    from_json_field(src, "normal_sla_l_wall_off_margin", dst.normal_sla_l_wall_off_margin);
    from_json_field(src, "normal_sla_r_wall_off_margin", dst.normal_sla_r_wall_off_margin);
    from_json_field(src, "torque_mode", dst.torque_mode);
    from_json_field(src, "enable_kalman_gyro", dst.enable_kalman_gyro);
    from_json_field(src, "enable_kalman_encoder", dst.enable_kalman_encoder);
    from_json_field(src, "enable_mpc", dst.enable_mpc);
    from_json_field(src, "dia90_offset", dst.dia90_offset);
    from_json_nested(src, "kanayama", dst.kanayama);
}
/**
 * system.txt
 * root → test
 */
inline void convertFromJson(JsonVariantConst src, test_mode_t& dst) {
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "end_v", dst.end_v);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "decel", dst.decel);
    from_json_field(src, "dia_accl", dst.dia_accl);
    from_json_field(src, "dia_decel", dst.dia_decel);
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "w_max", dst.w_max);
    from_json_field(src, "w_end", dst.w_end);
    from_json_field(src, "alpha", dst.alpha);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "suction_active", dst.suction_active);
    from_json_field(src, "suction_duty", dst.suction_duty);
    from_json_field(src, "suction_duty_low", dst.suction_duty_low);
    from_json_field(src, "suction_duty_burst", dst.suction_duty_burst);
    from_json_field(src, "suction_duty_burst_low", dst.suction_duty_burst_low);
    from_json_field(src, "suction_gain", dst.suction_gain);
    from_json_field(src, "sla_dist", dst.sla_dist);
    from_json_field(src, "file_idx", dst.file_idx);
    from_json_field(src, "sla_type", dst.sla_type);
    from_json_field(src, "sla_return", dst.sla_return);
    from_json_field(src, "sla_type2", dst.sla_type2);
    from_json_field(src, "turn_times", dst.turn_times);
    from_json_field(src, "ignore_opp_sen", dst.ignore_opp_sen);
    from_json_field(src, "dia", dst.dia);
    from_json_field(src, "sysid_test_mode", dst.sysid_test_mode);
    from_json_field(src, "sysid_duty", dst.sysid_duty);
    from_json_field(src, "sysid_time", dst.sysid_time);
    from_json_field(src, "start_turn", dst.start_turn);
    from_json_field(src, "search_mode", dst.search_mode);
}
/**
 * system.txt
 * root
 */
inline void convertFromJson(JsonVariantConst src, system_t& dst) {
    from_json_vector(src, "goals", dst.goals);
    from_json_field(src, "maze_size", dst.maze_size);
    from_json_field(src, "mode", dst.user_mode);
    from_json_field(src, "circuit_mode", dst.circuit_mode);
    from_json_nested(src, "test", dst.test);
    from_json_field(src, "hf_cl", dst.hf_cl);
}

/**
 * profiles.txt
 * root → profile_list → <name>
 */
inline void convertFromJson(JsonVariantConst src, profile_idx_t& dst) {
    from_json_field(src, "normal", dst.normal);
    from_json_field(src, "large", dst.large);
    from_json_field(src, "orval", dst.orval);
    from_json_field(src, "dia45", dst.dia45);
    from_json_field(src, "dia45_2", dst.dia45_2);
    from_json_field(src, "dia135", dst.dia135);
    from_json_field(src, "dia135_2", dst.dia135_2);
    from_json_field(src, "dia90", dst.dia90);
}

/**
 * profiles.txt
 * root
 */
inline void convertFromJson(JsonVariantConst src, turn_param_profile_t& dst) {
    from_json_vector(src, "file_list", dst.file_list);
    from_json_field(src, "file_list_size", dst.file_list_size);
    from_json_field(src, "profile_idx_size", dst.profile_idx_size);
    // profile_list unordered_map skipped
    // profile_map unordered_map skipped
}

/**
 * <profile>.txt
 * root → <turn_type> → front | back
 */
inline void convertFromJson(JsonVariantConst src, slalom_offset_t& dst) {
    from_json_field(src, "right", dst.right);
    from_json_field(src, "left", dst.left);
}

/**
 * <profile>.txt
 * root → <turn_type>
 */
inline void convertFromJson(JsonVariantConst src, slalom_param2_t& dst) {
    from_json_field(src, "v", dst.v);
    from_json_field(src, "end_v", dst.end_v);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "ref_ang", dst.ref_ang);
    from_json_field(src, "rad", dst.rad);
    from_json_field(src, "rad2", dst.rad2);
    from_json_nested(src, "front", dst.front);
    from_json_nested(src, "back", dst.back);
    from_json_field(src, "pow_n", dst.pow_n);
    from_json_field(src, "time", dst.time);
    from_json_field(src, "time2", dst.time2);
    from_json_field(src, "type", dst.type);
}

/**
 * vel_prof.txt
 * root → v_prof[i] → <str_type>
 */
inline void convertFromJson(JsonVariantConst src, straight_param_t& dst) {
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "decel", dst.decel);
    from_json_field(src, "w_max", dst.w_max);
    from_json_field(src, "w_end", dst.w_end);
    from_json_field(src, "alpha", dst.alpha);
}

/**
 * run_prf.txt
 * root → exec_prof[i]
 */
inline void convertFromJson(JsonVariantConst src, exec_pram_t& dst) {
    from_json_field(src, "fast_idx", dst.fast_idx);
    from_json_field(src, "normal_idx", dst.normal_idx);
    from_json_field(src, "slow_idx", dst.slow_idx);
}
