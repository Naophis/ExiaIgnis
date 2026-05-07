#pragma once
#include <ArduinoJson.h>
#include "structs.hpp"

// convertFromJson の鏡像。全フィールドを JSON に書き出す。
// serializeJson(*param_, buf) のように使う。

inline void convertToJson(const point_t& src, JsonVariant dst) {
    JsonArray arr = dst.to<JsonArray>();
    arr.add(src.x);
    arr.add(src.y);
}

inline void convertToJson(const pid_param_t& src, JsonVariant dst) {
    dst["p"] = src.p;
    dst["i"] = src.i;
    dst["d"] = src.d;
    dst["b"] = src.b;
    dst["c"] = src.c;
    dst["mode"] = (int)src.mode;
    dst["antiwindup"] = (int)src.antiwindup;
    dst["windup_gain"] = src.windup_gain;
    dst["windup_dead_bind"] = src.windup_dead_bind;
    dst["i_theta_tau"] = src.i_theta_tau;
    dst["theta_gate"] = src.theta_gate;
    dst["omega_gate"] = src.omega_gate;
    dst["i_theta_slew"] = src.i_theta_slew;
    dst["i_theta_max"] = src.i_theta_max;
    dst["alpha_stop"] = src.alpha_stop;
    dst["alpha_rate"] = src.alpha_rate;
    dst["theta_damp_th"] = src.theta_damp_th;
    dst["omega_damp"] = src.omega_damp;
    dst["th"] = src.th;
    dst["theta_gate_on"] = src.theta_gate_on;
    dst["theta_gate_full"] = src.theta_gate_full;
    dst["theta_kp"] = src.theta_kp;
    dst["theta_kd"] = src.theta_kd;
    dst["omega_add_max"] = src.omega_add_max;
    dst["alpha_rate_end"] = src.alpha_rate_end;
    dst["k_stop"] = src.k_stop;
    dst["theta_eps"] = src.theta_eps;
    dst["s_gate"] = src.s_gate;
    dst["mpc_q_ang"] = src.mpc_q_ang;
    dst["mpc_q_vel"] = src.mpc_q_vel;
    dst["mpc_b"] = src.mpc_b;
    dst["mpc_r"] = src.mpc_r;
    dst["mpc_horizon"] = src.mpc_horizon;
    dst["mpc_max_iter"] = src.mpc_max_iter;
    dst["mpc_max_torque"] = src.mpc_max_torque;
    dst["mpc_observer_k"] = src.mpc_observer_k;
}

inline void convertToJson(const gyro_param_t& src, JsonVariant dst) {
    dst["gyro_w_gain_right"] = src.gyro_w_gain_right;
    dst["gyro_w_gain_left"]  = src.gyro_w_gain_left;
    dst["retry_min_th"]      = src.retry_min_th;
    dst["retry_max_th"]      = src.retry_max_th;
    dst["robust_th"]         = src.robust_th;
    dst["lp_delay"]          = src.lp_delay;
    dst["list_size"]         = src.list_size;
    dst["loop_size"]         = src.loop_size;
}

inline void convertToJson(const accel_param_t& src, JsonVariant dst) {
    dst["gain"] = src.gain;
}

inline void convertToJson(const sen_param_t& src, JsonVariant dst) {
    dst["lp_delay"] = src.lp_delay;
}

inline void convertToJson(const sen_ref_param3_t& src, JsonVariant dst) {
    dst["right45"]             = src.right45;
    dst["left45"]              = src.left45;
    dst["right90"]             = src.right90;
    dst["left90"]              = src.left90;
    dst["front"]               = src.front;
    dst["kireme_r"]            = src.kireme_r;
    dst["kireme_l"]            = src.kireme_l;
    dst["kireme_r_fast"]       = src.kireme_r_fast;
    dst["kireme_l_fast"]       = src.kireme_l_fast;
    dst["kireme_r_wall_off"]   = src.kireme_r_wall_off;
    dst["kireme_l_wall_off"]   = src.kireme_l_wall_off;
    dst["kireme_r_wall_off2"]  = src.kireme_r_wall_off2;
    dst["kireme_l_wall_off2"]  = src.kireme_l_wall_off2;
}

inline void convertToJson(const sen_search_param_t& src, JsonVariant dst) {
    dst["front"]        = src.front;
    dst["right45"]      = src.right45;
    dst["left45"]       = src.left45;
    dst["right90"]      = src.right90;
    dst["left90"]       = src.left90;
    dst["kireme_r"]     = src.kireme_r;
    dst["kireme_l"]     = src.kireme_l;
    dst["offset_r"]     = src.offset_r;
    dst["offset_l"]     = src.offset_l;
    dst["front_ctrl"]   = src.front_ctrl;
    dst["front_ctrl_th"] = src.front_ctrl_th;
}

inline void convertToJson(const sen_expand_param_t& src, JsonVariant dst) {
    dst["dist"]      = src.dist;
    dst["right45"]   = src.right45;
    dst["left45"]    = src.left45;
    dst["right45_2"] = src.right45_2;
    dst["left45_2"]  = src.left45_2;
}

inline void convertToJson(const sen_ref_param2_t& src, JsonVariant dst) {
    dst["ref"]        = src.ref;
    dst["ref_search"] = src.ref_search;
    dst["exist"]      = src.exist;
    dst["expand"]     = src.expand;
}

inline void convertToJson(const sen_ref_param_t& src, JsonVariant dst) {
    dst["normal"]       = src.normal;
    dst["normal2"]      = src.normal2;
    dst["dia"]          = src.dia;
    dst["search_exist"] = src.search_exist;
    dst["search_ref"]   = src.search_ref;
}

inline void convertToJson(const sensor_gain_param_t& src, JsonVariant dst) {
    dst["a"] = src.a;
    dst["b"] = src.b;
}

inline void convertToJson(const sensor_gain_t& src, JsonVariant dst) {
    dst["l90"]          = src.l90;
    dst["l45"]          = src.l45;
    dst["front"]        = src.front;
    dst["front2"]       = src.front2;
    dst["front3"]       = src.front3;
    dst["front4"]       = src.front4;
    dst["front_ctrl_th"] = src.front_ctrl_th;
    dst["r45"]          = src.r45;
    dst["l45_2"]        = src.l45_2;
    dst["r45_2"]        = src.r45_2;
    dst["l45_3"]        = src.l45_3;
    dst["r45_3"]        = src.r45_3;
    dst["r90"]          = src.r90;
    dst["l90_far"]      = src.l90_far;
    dst["r90_far"]      = src.r90_far;
    dst["l90_mid"]      = src.l90_mid;
    dst["r90_mid"]      = src.r90_mid;
}

inline void convertToJson(const wall_off_hold_dist_t& src, JsonVariant dst) {
    dst["left_str"]                    = src.left_str;
    dst["right_str"]                   = src.right_str;
    dst["left_diff_th"]                = src.left_diff_th;
    dst["right_diff_th"]               = src.right_diff_th;
    dst["left_str_exist"]              = src.left_str_exist;
    dst["right_str_exist"]             = src.right_str_exist;
    dst["left_dia"]                    = src.left_dia;
    dst["right_dia"]                   = src.right_dia;
    dst["left_dia_noexit"]             = src.left_dia_noexit;
    dst["right_dia_noexit"]            = src.right_dia_noexit;
    dst["left_dia_oppo"]               = src.left_dia_oppo;
    dst["right_dia_oppo"]              = src.right_dia_oppo;
    dst["left_dia2"]                   = src.left_dia2;
    dst["right_dia2"]                  = src.right_dia2;
    dst["exist_dist_l"]                = src.exist_dist_l;
    dst["exist_dist_r"]                = src.exist_dist_r;
    dst["exist_dist_l2"]               = src.exist_dist_l2;
    dst["exist_dist_r2"]               = src.exist_dist_r2;
    dst["noexist_th_l"]                = src.noexist_th_l;
    dst["noexist_th_r"]                = src.noexist_th_r;
    dst["noexist_th_l2"]               = src.noexist_th_l2;
    dst["noexist_th_r2"]               = src.noexist_th_r2;
    dst["div_th_l"]                    = src.div_th_l;
    dst["div_th_r"]                    = src.div_th_r;
    dst["div_th_l2"]                   = src.div_th_l2;
    dst["div_th_r2"]                   = src.div_th_r2;
    dst["div_th_l3"]                   = src.div_th_l3;
    dst["div_th_r3"]                   = src.div_th_r3;
    dst["div_th_dia_l"]                = src.div_th_dia_l;
    dst["div_th_dia_r"]                = src.div_th_dia_r;
    dst["exist_dia_th_l"]              = src.exist_dia_th_l;
    dst["exist_dia_th_r"]              = src.exist_dia_th_r;
    dst["exist_dia_th_l2"]             = src.exist_dia_th_l2;
    dst["exist_dia_th_r2"]             = src.exist_dia_th_r2;
    dst["noexist_dia_th_l"]            = src.noexist_dia_th_l;
    dst["noexist_dia_th_r"]            = src.noexist_dia_th_r;
    dst["noexist_dia_th_l2"]           = src.noexist_dia_th_l2;
    dst["noexist_dia_th_r2"]           = src.noexist_dia_th_r2;
    dst["wall_off_exist_wall_th_l"]    = src.wall_off_exist_wall_th_l;
    dst["wall_off_exist_wall_th_r"]    = src.wall_off_exist_wall_th_r;
    dst["wall_off_exist_dia_wall_th_l"] = src.wall_off_exist_dia_wall_th_l;
    dst["wall_off_exist_dia_wall_th_r"] = src.wall_off_exist_dia_wall_th_r;
    dst["search_wall_off_enable"]      = src.search_wall_off_enable;
    dst["search_wall_off_l_dist_offset"] = src.search_wall_off_l_dist_offset;
    dst["search_wall_off_r_dist_offset"] = src.search_wall_off_r_dist_offset;
    dst["search_wall_off_offset_dist"] = src.search_wall_off_offset_dist;
    dst["ctrl_exist_wall_th_l"]        = src.ctrl_exist_wall_th_l;
    dst["ctrl_exist_wall_th_r"]        = src.ctrl_exist_wall_th_r;
    dst["go_straight_wide_ctrl_th"]    = src.go_straight_wide_ctrl_th;
    dst["diff_check_dist"]             = src.diff_check_dist;
    dst["diff_dist_th_l"]              = src.diff_dist_th_l;
    dst["diff_dist_th_r"]              = src.diff_dist_th_r;
    dst["diff_check_dist_dia"]         = src.diff_check_dist_dia;
    dst["diff_check_dist_dia_2"]       = src.diff_check_dist_dia_2;
}

inline void convertToJson(const fail_check_cnt_t& src, JsonVariant dst) {
    dst["duty"]     = src.duty;
    dst["v"]        = src.v;
    dst["w"]        = src.w;
    dst["ang"]      = src.ang;
    dst["wall_off"] = src.wall_off;
}

inline void convertToJson(const comp_param_t& src, JsonVariant dst) {
    dst["v_lp_gain"]       = src.v_lp_gain;
    dst["accl_x_hp_gain"]  = src.accl_x_hp_gain;
    dst["gain"]            = src.gain;
    dst["enable"]          = src.enable;
}

inline void convertToJson(const kanayama_t& src, JsonVariant dst) {
    dst["kx"]         = src.kx;
    dst["ky"]         = src.ky;
    dst["k_theta"]    = src.k_theta;
    dst["enable"]     = (int)src.enable;
    dst["windup"]     = (int)src.windup;
    dst["windup_deg"] = src.windup_deg;
}

inline void convertToJson(const input_param_t& src, JsonVariant dst) {
    dst["dt"]               = src.dt;
    dst["trj_length"]       = src.trj_length;
    dst["tire"]             = src.tire;
    dst["tire2"]            = src.tire2;
    dst["log_size"]         = src.log_size;
    dst["gear_a"]           = src.gear_a;
    dst["gear_b"]           = src.gear_b;
    dst["max_duty"]         = src.max_duty;
    dst["min_duty"]         = src.min_duty;
    dst["battery_gain"]     = src.battery_gain;
    dst["Ke"]               = src.Ke;
    dst["Km"]               = src.Km;
    dst["Resist"]           = src.Resist;
    dst["Mass"]             = src.Mass;
    dst["Lm"]               = src.Lm;
    dst["coulomb_friction"] = src.coulomb_friction;
    dst["viscous_friction"] = src.viscous_friction;
    dst["battery_init_cov"] = src.battery_init_cov;
    dst["battery_p_noise"]  = src.battery_p_noise;
    dst["battery_m_noise"]  = src.battery_m_noise;
    dst["encoder_init_cov"] = src.encoder_init_cov;
    dst["encoder_p_noise"]  = src.encoder_p_noise;
    dst["encoder_m_noise"]  = src.encoder_m_noise;
    dst["w_init_cov"]       = src.w_init_cov;
    dst["w_p_noise"]        = src.w_p_noise;
    dst["w_m_noise"]        = src.w_m_noise;
    dst["v_init_cov"]       = src.v_init_cov;
    dst["v_p_noise"]        = src.v_p_noise;
    dst["v_m_noise"]        = src.v_m_noise;
    dst["ang_init_cov"]     = src.ang_init_cov;
    dst["ang_p_noise"]      = src.ang_p_noise;
    dst["ang_m_noise"]      = src.ang_m_noise;
    dst["dist_init_cov"]    = src.dist_init_cov;
    dst["dist_p_noise"]     = src.dist_p_noise;
    dst["dist_m_noise"]     = src.dist_m_noise;
    dst["pos_init_cov"]     = src.pos_init_cov;
    dst["pos_p_noise"]      = src.pos_p_noise;
    dst["pos_m_noise"]      = src.pos_m_noise;
    dst["tread"]            = src.tread;
    dst["FF_front"]         = src.FF_front;
    dst["FF_roll"]          = src.FF_roll;
    dst["FF_keV"]           = src.FF_keV;
    dst["offset_start_dist"]        = src.offset_start_dist;
    dst["offset_start_dist_search"] = src.offset_start_dist_search;
    dst["long_run_offset_dist"]     = src.long_run_offset_dist;
    dst["pivot_back_offset"]        = src.pivot_back_offset;
    dst["cell"]             = src.cell;
    dst["cell2"]            = src.cell2;
    dst["pivot_angle_180"]  = src.pivot_angle_180;
    dst["pivot_angle_90"]   = src.pivot_angle_90;
    dst["wall_off_front_move_dist_th"]     = src.wall_off_front_move_dist_th;
    dst["wall_off_front_move_dia_dist_th"] = src.wall_off_front_move_dia_dist_th;
    dst["ff_front_gain_14"]    = src.ff_front_gain_14;
    dst["ff_roll_gain_before"] = src.ff_roll_gain_before;
    dst["ff_roll_gain_after"]  = src.ff_roll_gain_after;
    dst["ff_front_gain_decel"] = src.ff_front_gain_decel;
    dst["front_ctrl_roll_pid"]         = src.front_ctrl_roll_pid;
    dst["motor_pid"]                   = src.motor_pid;
    dst["motor_pid_gain_limitter"]     = src.motor_pid_gain_limitter;
    dst["motor_pid2"]                  = src.motor_pid2;
    dst["motor2_pid_gain_limitter"]    = src.motor2_pid_gain_limitter;
    dst["motor_pid3"]                  = src.motor_pid3;
    dst["gyro_pid"]                    = src.gyro_pid;
    dst["gyro_pid_gain_limitter"]      = src.gyro_pid_gain_limitter;
    dst["str_ang_pid"]                 = src.str_ang_pid;
    dst["str_ang_dia_pid"]             = src.str_ang_dia_pid;
    dst["angle_pid"]                   = src.angle_pid;
    dst["front_ctrl_angle_pid"]        = src.front_ctrl_angle_pid;
    dst["front_ctrl_dist_pid"]         = src.front_ctrl_dist_pid;
    dst["front_ctrl_keep_angle_pid"]   = src.front_ctrl_keep_angle_pid;
    dst["sensor_pid_dia"]              = src.sensor_pid_dia;
    dst["gyro_param"]                  = src.gyro_param;
    dst["gyro2_param"]                 = src.gyro2_param;
    dst["accel_x_param"]               = src.accel_x_param;
    dst["comp_param"]                  = src.comp_param;
    dst["battery_param"]               = src.battery_param;
    dst["led_param"]                   = src.led_param;
    dst["motion_dir"]               = static_cast<int>(src.motion_dir);
    dst["sen_ref_p"]                = src.sen_ref_p;
    dst["sensor_gain"]              = src.sensor_gain;
    dst["sakiyomi_time"]            = src.sakiyomi_time;
    dst["search_sen_ctrl_limitter"] = src.search_sen_ctrl_limitter;
    dst["clear_angle"]              = src.clear_angle;
    dst["clear_dist_order"]         = src.clear_dist_order;
    dst["front_dist_offset"]        = src.front_dist_offset;
    dst["front_dist_offset0"]       = src.front_dist_offset0;
    dst["front_dist_offset2"]       = src.front_dist_offset2;
    dst["front_dist_offset3"]       = src.front_dist_offset3;
    dst["front_dist_offset4"]       = src.front_dist_offset4;
    dst["front_dist_offset_dia_front"]  = src.front_dist_offset_dia_front;
    dst["front_dist_offset_dia_45_th"]  = src.front_dist_offset_dia_45_th;
    dst["front_dist_offset_dia_right45"] = src.front_dist_offset_dia_right45;
    dst["front_dist_offset_dia_left45"]  = src.front_dist_offset_dia_left45;
    dst["sla_wall_ref_l"]           = src.sla_wall_ref_l;
    dst["sla_wall_ref_r"]           = src.sla_wall_ref_r;
    dst["sla_max_offset_dist"]      = src.sla_max_offset_dist;
    dst["large_offset_enable"]      = src.large_offset_enable;
    dst["dia45_offset_enable"]      = src.dia45_offset_enable;
    dst["dia135_offset_enable"]     = src.dia135_offset_enable;
    dst["orval_offset_enable"]      = src.orval_offset_enable;
    dst["large_offset_max_dist"]    = src.large_offset_max_dist;
    dst["dia45_offset_max_dist"]    = src.dia45_offset_max_dist;
    dst["dia135_offset_max_dist"]   = src.dia135_offset_max_dist;
    dst["orval_offset_max_dist"]    = src.orval_offset_max_dist;
    dst["dia45_2_offset_max_dist"]  = src.dia45_2_offset_max_dist;
    dst["dia135_2_offset_max_dist"] = src.dia135_2_offset_max_dist;
    dst["dia90_offset_max_dist"]    = src.dia90_offset_max_dist;
    dst["lim_angle"]                = src.lim_angle;
    dst["front_ctrl_error_th"]      = src.front_ctrl_error_th;
    dst["clear_dist_ragne_from"]    = src.clear_dist_ragne_from;
    dst["clear_dist_ragne_to"]      = src.clear_dist_ragne_to;
    dst["clear_dist_ragne_to2"]     = src.clear_dist_ragne_to2;
    {
        JsonArray a = dst["clear_dist_ragne_dist_list"].to<JsonArray>();
        for (float v : src.clear_dist_ragne_dist_list) a.add(v);
    }
    {
        JsonArray a = dst["clear_dist_ragne_th_list"].to<JsonArray>();
        for (float v : src.clear_dist_ragne_th_list) a.add(v);
    }
    {
        JsonArray a = dst["clear_dist_ragne_dist_list_fast"].to<JsonArray>();
        for (float v : src.clear_dist_ragne_dist_list_fast) a.add(v);
    }
    dst["wall_off_hold_dist"]       = src.wall_off_hold_dist;
    dst["wall_off_dist"]            = src.wall_off_dist;
    dst["wall_off_diff_ref_th"]     = src.wall_off_diff_ref_th;
    dst["wall_off_diff_ref_front_th"] = src.wall_off_diff_ref_front_th;
    dst["wall_off_wait_dist"]       = src.wall_off_wait_dist;
    dst["wall_off_wait_dist_dia"]   = src.wall_off_wait_dist_dia;
    dst["search_log_enable"]        = src.search_log_enable;
    dst["seach_timer"]              = src.seach_timer;
    dst["test_log_enable"]          = src.test_log_enable;
    dst["fast_log_enable"]          = src.fast_log_enable;
    dst["front_dist_offset_pivot_th"] = src.front_dist_offset_pivot_th;
    dst["front_dist_offset_pivot"]  = src.front_dist_offset_pivot;
    dst["pivot_back_dist0"]         = src.pivot_back_dist0;
    dst["pivot_back_dist1"]         = src.pivot_back_dist1;
    dst["sen_log_size"]             = src.sen_log_size;
    dst["sen_log_size2"]            = src.sen_log_size2;
    dst["led_light_delay_cnt"]      = src.led_light_delay_cnt;
    dst["led_light_delay_cnt2"]     = src.led_light_delay_cnt2;
    dst["set_param"]                = src.set_param;
    dst["logging_time"]             = src.logging_time;
    dst["offset_after_turn_l2"]     = src.offset_after_turn_l2;
    dst["offset_after_turn_r2"]     = src.offset_after_turn_r2;
    dst["offset_after_turn_l"]      = src.offset_after_turn_l;
    dst["offset_after_turn_r"]      = src.offset_after_turn_r;
    dst["offset_after_turn_dia_l"]  = src.offset_after_turn_dia_l;
    dst["offset_after_turn_dia_r"]  = src.offset_after_turn_dia_r;
    dst["dia_turn_exist_th_l"]      = src.dia_turn_exist_th_l;
    dst["dia_turn_exist_th_r"]      = src.dia_turn_exist_th_r;
    dst["dia_turn_th_l"]            = src.dia_turn_th_l;
    dst["dia_turn_th_r"]            = src.dia_turn_th_r;
    dst["dia_turn_ref_l"]           = src.dia_turn_ref_l;
    dst["dia_turn_ref_r"]           = src.dia_turn_ref_r;
    dst["dia_turn_max_dist_l"]      = src.dia_turn_max_dist_l;
    dst["dia_turn_max_dist_r"]      = src.dia_turn_max_dist_r;
    dst["wall_off_pass_dist"]       = src.wall_off_pass_dist;
    dst["dia_wall_off_ref_l"]       = src.dia_wall_off_ref_l;
    dst["dia_wall_off_ref_r"]       = src.dia_wall_off_ref_r;
    dst["dia_wall_off_ref_l_wall"]  = src.dia_wall_off_ref_l_wall;
    dst["dia_wall_off_ref_r_wall"]  = src.dia_wall_off_ref_r_wall;
    dst["dia_wall_off_ref_l_wall2"] = src.dia_wall_off_ref_l_wall2;
    dst["dia_wall_off_ref_r_wall2"] = src.dia_wall_off_ref_r_wall2;
    dst["dia_wall_off_ref_l_piller"] = src.dia_wall_off_ref_l_piller;
    dst["dia_wall_off_ref_r_piller"] = src.dia_wall_off_ref_r_piller;
    dst["dia_offset_max_dist"]      = src.dia_offset_max_dist;
    dst["slip_param_K"]             = src.slip_param_K;
    dst["slip_param_k2"]            = src.slip_param_k2;
    dst["fail_check"]               = src.fail_check;
    dst["fail_check_ang_th"]        = src.fail_check_ang_th;
    dst["normal_sla_offset"]        = src.normal_sla_offset;
    dst["normal_sla_offset_front"]  = src.normal_sla_offset_front;
    dst["normal_sla_offset_back"]   = src.normal_sla_offset_back;
    dst["front_diff_th"]            = src.front_diff_th;
    dst["ff_v_th"]                  = src.ff_v_th;
    dst["ff_front_dury"]            = src.ff_front_dury;
    dst["motor_driver_type"]        = static_cast<int>(src.motor_driver_type);
    dst["motor_debug_mode"]         = src.motor_debug_mode;
    dst["motor_r_cw_ccw_type"]      = src.motor_r_cw_ccw_type;
    dst["motor_l_cw_ccw_type"]      = src.motor_l_cw_ccw_type;
    dst["motor_debug_mode_duty_r"]  = src.motor_debug_mode_duty_r;
    dst["motor_debug_mode_duty_l"]  = src.motor_debug_mode_duty_l;
    dst["pivot_straight"]           = src.pivot_straight;
    dst["pivot_back_enable_front_th"] = src.pivot_back_enable_front_th;
    dst["search_front_ctrl_th"]     = src.search_front_ctrl_th;
    dst["judge_pivot"]              = src.judge_pivot;
    dst["sensor_range_min"]         = src.sensor_range_min;
    dst["sensor_range_max"]         = src.sensor_range_max;
    dst["sensor_range_mid_max"]     = src.sensor_range_mid_max;
    dst["sensor_range_far_max"]     = src.sensor_range_far_max;
    dst["dist_mod_num"]             = src.dist_mod_num;
    dst["sen_ctrl_front_th"]        = src.sen_ctrl_front_th;
    dst["sen_ctrl_front_diff_th"]   = src.sen_ctrl_front_diff_th;
    dst["th_offset_dist"]           = src.th_offset_dist;
    dst["sla_front_ctrl_th"]        = src.sla_front_ctrl_th;
    dst["orval_front_ctrl_min"]     = src.orval_front_ctrl_min;
    dst["orval_front_ctrl_max"]     = src.orval_front_ctrl_max;
    dst["wall_off_front_ctrl_min"]  = src.wall_off_front_ctrl_min;
    dst["dia_turn_offset_calc_th"]  = src.dia_turn_offset_calc_th;
    dst["go_straight_wide_ctrl_th"] = src.go_straight_wide_ctrl_th;
    dst["wall_off_pass_through_offset_r"] = src.wall_off_pass_through_offset_r;
    dst["wall_off_pass_through_offset_l"] = src.wall_off_pass_through_offset_l;
    dst["tire_tread"]               = src.tire_tread;
    dst["right_keep_dist_th"]       = src.right_keep_dist_th;
    dst["left_keep_dist_th"]        = src.left_keep_dist_th;
    dst["normal_sla_l_wall_off_th_in"]  = src.normal_sla_l_wall_off_th_in;
    dst["normal_sla_r_wall_off_th_in"]  = src.normal_sla_r_wall_off_th_in;
    dst["normal_sla_l_wall_off_th_out"] = src.normal_sla_l_wall_off_th_out;
    dst["normal_sla_r_wall_off_th_out"] = src.normal_sla_r_wall_off_th_out;
    dst["normal_sla_l_wall_off_ref_cnt"] = src.normal_sla_l_wall_off_ref_cnt;
    dst["normal_sla_r_wall_off_ref_cnt"] = src.normal_sla_r_wall_off_ref_cnt;
    dst["normal_sla_l_wall_off_dist"]   = src.normal_sla_l_wall_off_dist;
    dst["normal_sla_r_wall_off_dist"]   = src.normal_sla_r_wall_off_dist;
    dst["normal_sla_l_wall_off_margin"] = src.normal_sla_l_wall_off_margin;
    dst["normal_sla_r_wall_off_margin"] = src.normal_sla_r_wall_off_margin;
    dst["torque_mode"]              = (int)src.torque_mode;
    dst["enable_kalman_gyro"]       = (int)src.enable_kalman_gyro;
    dst["enable_kalman_encoder"]    = (int)src.enable_kalman_encoder;
    dst["enable_mpc"]               = (int)src.enable_mpc;
    dst["dia90_offset"]             = src.dia90_offset;
    dst["kanayama"]                 = src.kanayama;
}

inline void convertToJson(const test_mode_t& src, JsonVariant dst) {
    dst["v_max"]               = src.v_max;
    dst["end_v"]               = src.end_v;
    dst["accl"]                = src.accl;
    dst["decel"]               = src.decel;
    dst["dia_accl"]            = src.dia_accl;
    dst["dia_decel"]           = src.dia_decel;
    dst["dist"]                = src.dist;
    dst["w_max"]               = src.w_max;
    dst["w_end"]               = src.w_end;
    dst["alpha"]               = src.alpha;
    dst["ang"]                 = src.ang;
    dst["suction_active"]      = src.suction_active;
    dst["suction_duty"]        = src.suction_duty;
    dst["suction_duty_low"]    = src.suction_duty_low;
    dst["suction_duty_burst"]  = src.suction_duty_burst;
    dst["suction_duty_burst_low"] = src.suction_duty_burst_low;
    dst["suction_gain"]        = src.suction_gain;
    dst["sla_dist"]            = src.sla_dist;
    dst["file_idx"]            = src.file_idx;
    dst["sla_type"]            = src.sla_type;
    dst["sla_return"]          = src.sla_return;
    dst["sla_type2"]           = src.sla_type2;
    dst["turn_times"]          = src.turn_times;
    dst["ignore_opp_sen"]      = src.ignore_opp_sen;
    dst["dia"]                 = src.dia;
    dst["sysid_test_mode"]     = src.sysid_test_mode;
    dst["sysid_duty"]          = src.sysid_duty;
    dst["sysid_time"]          = src.sysid_time;
    dst["start_turn"]          = src.start_turn;
    dst["search_mode"]         = src.search_mode;
}

inline void convertToJson(const system_t& src, JsonVariant dst) {
    JsonArray goals = dst["goals"].to<JsonArray>();
    for (const auto& g : src.goals) {
        JsonArray pt = goals.add<JsonArray>();
        pt.add(g.x);
        pt.add(g.y);
    }
    dst["maze_size"]    = src.maze_size;
    dst["mode"]         = src.user_mode;
    dst["circuit_mode"] = src.circuit_mode;
    dst["hf_cl"]        = src.hf_cl;
    dst["test"]         = src.test;
}
