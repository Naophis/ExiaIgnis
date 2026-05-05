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
 * maze_solver
 * map[row][col]
 */
inline void convertFromJson(JsonVariantConst src, vector_map_t& dst) {
    from_json_field(src, "n", dst.n);
    from_json_field(src, "e", dst.e);
    from_json_field(src, "w", dst.w);
    from_json_field(src, "s", dst.s);
    // ビットフィールドは C++ で参照を取れないため is<>/as<> を直接使う
    if (src["v"].is<int>())    dst.v    = src["v"].as<int>();
    if (src["N1"].is<int>())   dst.N1   = src["N1"].as<int>();
    if (src["NE"].is<int>())   dst.NE   = src["NE"].as<int>();
    if (src["E1"].is<int>())   dst.E1   = src["E1"].as<int>();
    if (src["SE"].is<int>())   dst.SE   = src["SE"].as<int>();
    if (src["S1"].is<int>())   dst.S1   = src["S1"].as<int>();
    if (src["SW"].is<int>())   dst.SW   = src["SW"].as<int>();
    if (src["W1"].is<int>())   dst.W1   = src["W1"].as<int>();
    if (src["NW"].is<int>())   dst.NW   = src["NW"].as<int>();
    if (src["step"].is<int>()) dst.step = src["step"].as<int>();
}

/**
 * maze_solver
 * route[]
 */
inline void convertFromJson(JsonVariantConst src, route_t& dst) {
    from_json_field(src, "dir", dst.dir);
    from_json_field(src, "time", dst.time);
    from_json_field(src, "use", dst.use);
}

/**
 * maze_solver
 * path_create_status
 */
inline void convertFromJson(JsonVariantConst src, path_create_status_t& dst) {
    from_json_field(src, "time", dst.time);
    from_json_field(src, "state", dst.state);
    from_json_field(src, "use", dst.use);
}

/**
 * maze_solver
 * path_type
 */
inline void convertFromJson(JsonVariantConst src, path_type& dst) {
    from_json_vector(src, "s", dst.s);
    from_json_vector(src, "t", dst.t);
    from_json_field(src, "size", dst.size);
}

/**
 * system.txt
 * root/goals[i]
 */
inline void convertFromJson(JsonVariantConst src, point_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
}

/**
 * maze_solver
 * dir_pt
 */
inline void convertFromJson(JsonVariantConst src, dir_pt_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "dir", dst.dir);
    from_json_field(src, "dist2", dst.dist2);
}

/**
 * maze_solver
 * ego
 */
inline void convertFromJson(JsonVariantConst src, ego_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "dir", dst.dir);
    from_json_field(src, "prev_motion", dst.prev_motion);
}

/**
 * maze_solver
 * path element
 */
inline void convertFromJson(JsonVariantConst src, path_element& dst) {
    from_json_field(src, "s", dst.s);
    from_json_field(src, "t", dst.t);
}

/**
 * maze_solver
 * path struct
 */
inline void convertFromJson(JsonVariantConst src, path_struct& dst) {
    from_json_vector(src, "paths", dst.paths);
    from_json_field(src, "size", dst.size);
}

/**
 * sub-struct
 * trajectory
 */
inline void convertFromJson(JsonVariantConst src, trajectory_point_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "base_ang_accl", dst.base_ang_accl);
    from_json_field(src, "type", dst.type);
}

/**
 * sub-struct
 * odometry
 */
inline void convertFromJson(JsonVariantConst src, ego_odom_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "dir", dst.dir);
}

/**
 * sub-struct
 * slalom_data_t/<type>
 */
inline void convertFromJson(JsonVariantConst src, slalom_param_t& dst) {
    from_json_field(src, "radius", dst.radius);
    from_json_field(src, "time", dst.time);
    from_json_field(src, "n", dst.n);
    from_json_field(src, "front", dst.front);
    from_json_field(src, "back", dst.back);
}

/**
 * sub-struct
 * input_param_t/slalom_data
 */
inline void convertFromJson(JsonVariantConst src, slalom_data_t& dst) {
    from_json_nested(src, "normal",  dst.normal);
    from_json_nested(src, "orval",   dst.orval);
    from_json_nested(src, "large",   dst.large);
    from_json_nested(src, "dia45",   dst.dia45);
    from_json_nested(src, "dia45_2", dst.dia45_2);
    from_json_nested(src, "dia135",  dst.dia135);
    from_json_nested(src, "dia135_2",dst.dia135_2);
    from_json_nested(src, "dia90",   dst.dia90);
}

/**
 * sub-struct
 * run parameters
 */
inline void convertFromJson(JsonVariantConst src, run_param_t& dst) {
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "decel", dst.decel);
    from_json_field(src, "turn_v", dst.turn_v);
}

/**
 * sub-struct
 * trajectory pattern
 */
inline void convertFromJson(JsonVariantConst src, base_trajectory_pattern_t& dst) {
    from_json_vector(src, "normal",   dst.normal);
    from_json_vector(src, "orval",    dst.orval);
    from_json_vector(src, "large",    dst.large);
    from_json_vector(src, "dia45",    dst.dia45);
    from_json_vector(src, "dia45_2",  dst.dia45_2);
    from_json_vector(src, "dia135",   dst.dia135);
    from_json_vector(src, "dia135_2", dst.dia135_2);
    from_json_vector(src, "dia90",    dst.dia90);
}

/**
 * sub-struct
 * kinematics state
 */
inline void convertFromJson(JsonVariantConst src, t_kinematics_state& dst) {
    from_json_field(src, "x",     dst.x);
    from_json_field(src, "y",     dst.y);
    from_json_field(src, "theta", dst.theta);
    from_json_field(src, "v",     dst.v);
    from_json_field(src, "vx",    dst.vx);
    from_json_field(src, "vy",    dst.vy);
    from_json_field(src, "w",     dst.w);
    from_json_field(src, "accl",  dst.accl);
    from_json_field(src, "alpha", dst.alpha);
}

/**
 * sub-struct
 * sensing_result_entity_t/encoder
 */
inline void convertFromJson(JsonVariantConst src, encoder_data_t& dst) {
    from_json_field(src, "right",     dst.right);
    from_json_field(src, "left",      dst.left);
    from_json_field(src, "right_old", dst.right_old);
    from_json_field(src, "left_old",  dst.left_old);
}

/**
 * sub-struct
 * sensing
 */
inline void convertFromJson(JsonVariantConst src, sensing_data_t& dst) {
    from_json_field(src, "raw", dst.raw);
    from_json_field(src, "data", dst.data);
}

/**
 * sub-struct
 * sensing_result_entity_t/led_sen
 */
inline void convertFromJson(JsonVariantConst src, led_sensor_t& dst) {
    from_json_nested(src, "right90", dst.right90);
    from_json_nested(src, "right45", dst.right45);
    from_json_nested(src, "right45_2", dst.right45_2);
    from_json_nested(src, "right45_3", dst.right45_3);
    from_json_nested(src, "front", dst.front);
    from_json_nested(src, "left45", dst.left45);
    from_json_nested(src, "left45_2", dst.left45_2);
    from_json_nested(src, "left45_3", dst.left45_3);
    from_json_nested(src, "left90", dst.left90);
}

/**
 * sub-struct
 * ego_entity_t/rpm
 */
inline void convertFromJson(JsonVariantConst src, rpm_t& dst) {
    from_json_field(src, "right", dst.right);
    from_json_field(src, "left", dst.left);
}

/**
 * sub-struct
 * ego_entity_t/duty
 */
inline void convertFromJson(JsonVariantConst src, duty_t& dst) {
    from_json_field(src, "duty_l", dst.duty_l);
    from_json_field(src, "duty_r", dst.duty_r);
    from_json_field(src, "duty_suction", dst.duty_suction);
    from_json_field(src, "duty_suction_low", dst.duty_suction_low);
    from_json_field(src, "sen", dst.sen);
    from_json_field(src, "sen_ang", dst.sen_ang);
    from_json_field(src, "ff_duty_front", dst.ff_duty_front);
    from_json_field(src, "ff_duty_roll", dst.ff_duty_roll);
    from_json_field(src, "ff_duty_rpm_r", dst.ff_duty_rpm_r);
    from_json_field(src, "ff_duty_rpm_l", dst.ff_duty_rpm_l);
}

/**
 * sub-struct
 * ego_entity_t/ff_duty
 */
inline void convertFromJson(JsonVariantConst src, ff_duty_t& dst) {
    from_json_field(src, "front", dst.front);
    from_json_field(src, "roll", dst.roll);
}

/**
 * logging
 * ego state
 */
inline void convertFromJson(JsonVariantConst src, ego_entity_t& dst) {
    from_json_field(src, "v_r", dst.v_r);
    from_json_field(src, "v_l", dst.v_l);
    from_json_field(src, "v_r_old", dst.v_r_old);
    from_json_field(src, "v_l_old", dst.v_l_old);
    from_json_field(src, "v_c", dst.v_c);
    from_json_field(src, "filter_v", dst.filter_v);
    from_json_field(src, "main_v", dst.main_v);
    from_json_field(src, "w_raw", dst.w_raw);
    from_json_field(src, "w_raw2", dst.w_raw2);
    from_json_field(src, "w_lp", dst.w_lp);
    from_json_field(src, "w_lp2", dst.w_lp2);
    from_json_field(src, "w_kf", dst.w_kf);
    from_json_field(src, "w_kf2", dst.w_kf2);
    from_json_field(src, "v_kf", dst.v_kf);
    from_json_field(src, "dist_kf", dst.dist_kf);
    from_json_field(src, "ang_kf", dst.ang_kf);
    from_json_field(src, "ang_kf2", dst.ang_kf2);
    from_json_field(src, "batt_kf", dst.batt_kf);
    from_json_field(src, "accel_x_raw", dst.accel_x_raw);
    from_json_field(src, "v_ave", dst.v_ave);
    from_json_field(src, "v_lp", dst.v_lp);
    from_json_field(src, "integrate_accl_x_ave", dst.integrate_accl_x_ave);
    from_json_field(src, "sum_v_ave", dst.sum_v_ave);
    from_json_field(src, "sum_integrate_accl_x_ave", dst.sum_integrate_accl_x_ave);
    from_json_field(src, "w_kalman", dst.w_kalman);
    from_json_field(src, "ang_kalman", dst.ang_kalman);
    from_json_field(src, "battery_raw", dst.battery_raw);
    from_json_field(src, "battery_lp", dst.battery_lp);
    from_json_field(src, "right90_raw", dst.right90_raw);
    from_json_field(src, "right90_lp", dst.right90_lp);
    from_json_field(src, "right45_raw", dst.right45_raw);
    from_json_field(src, "right45_lp", dst.right45_lp);
    from_json_field(src, "front_raw", dst.front_raw);
    from_json_field(src, "front_lp", dst.front_lp);
    from_json_field(src, "left45_raw", dst.left45_raw);
    from_json_field(src, "left45_lp", dst.left45_lp);
    from_json_field(src, "left90_raw", dst.left90_raw);
    from_json_field(src, "left90_lp", dst.left90_lp);
    from_json_field(src, "right45_2_raw", dst.right45_2_raw);
    from_json_field(src, "right45_2_lp", dst.right45_2_lp);
    from_json_field(src, "left45_2_raw", dst.left45_2_raw);
    from_json_field(src, "left45_2_lp", dst.left45_2_lp);
    from_json_field(src, "right45_3_raw", dst.right45_3_raw);
    from_json_field(src, "right45_3_lp", dst.right45_3_lp);
    from_json_field(src, "left45_3_raw", dst.left45_3_raw);
    from_json_field(src, "left45_3_lp", dst.left45_3_lp);
    from_json_field(src, "front_lp_old", dst.front_lp_old);
    from_json_field(src, "left45_lp_old", dst.left45_lp_old);
    from_json_field(src, "left90_lp_old", dst.left90_lp_old);
    from_json_field(src, "right45_lp_old", dst.right45_lp_old);
    from_json_field(src, "right90_lp_old", dst.right90_lp_old);
    from_json_field(src, "left45_2_lp_old", dst.left45_2_lp_old);
    from_json_field(src, "right45_2_lp_old", dst.right45_2_lp_old);
    from_json_field(src, "left45_3_lp_old", dst.left45_3_lp_old);
    from_json_field(src, "right45_3_lp_old", dst.right45_3_lp_old);
    from_json_field(src, "front_dist", dst.front_dist);
    from_json_field(src, "left45_dist", dst.left45_dist);
    from_json_field(src, "left45_2_dist", dst.left45_2_dist);
    from_json_field(src, "left45_3_dist", dst.left45_3_dist);
    from_json_field(src, "left90_dist", dst.left90_dist);
    from_json_field(src, "right45_dist", dst.right45_dist);
    from_json_field(src, "right45_2_dist", dst.right45_2_dist);
    from_json_field(src, "right45_3_dist", dst.right45_3_dist);
    from_json_field(src, "right90_dist", dst.right90_dist);
    from_json_field(src, "front_far_dist", dst.front_far_dist);
    from_json_field(src, "left90_far_dist", dst.left90_far_dist);
    from_json_field(src, "right90_far_dist", dst.right90_far_dist);
    from_json_field(src, "left90_mid_dist", dst.left90_mid_dist);
    from_json_field(src, "right90_mid_dist", dst.right90_mid_dist);
    from_json_field(src, "front_mid_dist", dst.front_mid_dist);
    from_json_field(src, "left45_dist_diff", dst.left45_dist_diff);
    from_json_field(src, "left45_2_dist_diff", dst.left45_2_dist_diff);
    from_json_field(src, "left45_3_dist_diff", dst.left45_3_dist_diff);
    from_json_field(src, "right45_dist_diff", dst.right45_dist_diff);
    from_json_field(src, "right45_2_dist_diff", dst.right45_2_dist_diff);
    from_json_field(src, "right45_3_dist_diff", dst.right45_3_dist_diff);
    from_json_field(src, "left90_dist_diff", dst.left90_dist_diff);
    from_json_field(src, "right90_dist_diff", dst.right90_dist_diff);
    from_json_field(src, "temp", dst.temp);
    from_json_field(src, "front_dist_old", dst.front_dist_old);
    from_json_field(src, "left45_dist_old", dst.left45_dist_old);
    from_json_field(src, "left45_2_dist_old", dst.left45_2_dist_old);
    from_json_field(src, "left45_3_dist_old", dst.left45_3_dist_old);
    from_json_field(src, "left90_dist_old", dst.left90_dist_old);
    from_json_field(src, "right45_dist_old", dst.right45_dist_old);
    from_json_field(src, "right45_2_dist_old", dst.right45_2_dist_old);
    from_json_field(src, "right45_3_dist_old", dst.right45_3_dist_old);
    from_json_field(src, "right90_dist_old", dst.right90_dist_old);
    from_json_field(src, "exist_r_wall", dst.exist_r_wall);
    from_json_field(src, "exist_l_wall", dst.exist_l_wall);
    from_json_nested(src, "rpm", dst.rpm);
    from_json_nested(src, "duty", dst.duty);
    from_json_nested(src, "ff_duty", dst.ff_duty);
    from_json_field(src, "motion_type", dst.motion_type);
    from_json_field(src, "pos_x", dst.pos_x);
    from_json_field(src, "pos_y", dst.pos_y);
    from_json_field(src, "pos_ang", dst.pos_ang);
    from_json_field(src, "knym_v", dst.knym_v);
    from_json_field(src, "knym_w", dst.knym_w);
    from_json_field(src, "odm_x", dst.odm_x);
    from_json_field(src, "odm_y", dst.odm_y);
    from_json_field(src, "odm_theta", dst.odm_theta);
    from_json_field(src, "kim_x", dst.kim_x);
    from_json_field(src, "kim_y", dst.kim_y);
    from_json_field(src, "kim_theta", dst.kim_theta);
}

/**
 * logging
 * kinematics
 */
inline void convertFromJson(JsonVariantConst src, kinematics_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "theta", dst.theta);
    from_json_field(src, "v", dst.v);
    from_json_field(src, "w", dst.w);
}

/**
 * logging
 * sensor distance log
 */
inline void convertFromJson(JsonVariantConst src, sen_log_t& dst) {
    from_json_field(src, "sensor_dist", dst.sensor_dist);
    from_json_field(src, "global_run_dist", dst.global_run_dist);
    from_json_field(src, "angle", dst.angle);
}

/**
 * logging
 * sensor distance log (2)
 */
inline void convertFromJson(JsonVariantConst src, sen_log2_t& dst) {
    from_json_field(src, "r45_dist", dst.r45_dist);
    from_json_field(src, "l45_dist", dst.l45_dist);
    from_json_field(src, "global_run_dist", dst.global_run_dist);
}

/**
 * logging
 * sensor logs (l90/l45/r45/r90 etc.)
 */
inline void convertFromJson(JsonVariantConst src, sen_logs_t& dst) {
    from_json_nested(src, "l90", dst.l90);
    from_json_nested(src, "l45", dst.l45);
    from_json_nested(src, "l45_2", dst.l45_2);
    from_json_nested(src, "l45_3", dst.l45_3);
    from_json_nested(src, "r45", dst.r45);
    from_json_nested(src, "r45_2", dst.r45_2);
    from_json_nested(src, "r45_3", dst.r45_3);
    from_json_nested(src, "r90", dst.r90);
}

/**
 * logging
 * sensor distance log list
 */
inline void convertFromJson(JsonVariantConst src, sen_dist_log_t& dst) {
    from_json_vector(src, "list", dst.list);
}

/**
 * logging
 * sensing result (all sensors)
 */
inline void convertFromJson(JsonVariantConst src, sensing_result_entity_t& dst) {
    from_json_nested(src, "led_sen", dst.led_sen);
    from_json_nested(src, "led_sen_after", dst.led_sen_after);
    from_json_nested(src, "led_sen_before", dst.led_sen_before);
    from_json_nested(src, "gyro", dst.gyro);
    from_json_nested(src, "gyro2", dst.gyro2);
    from_json_nested(src, "accel_x",   dst.accel_x);
    from_json_nested(src, "accel_y",   dst.accel_y);
    from_json_array(src,  "gyro_list", dst.gyro_list);  // int[5] 固定長配列
    from_json_vector(src, "enc_r_list",dst.enc_r_list);
    from_json_vector(src, "enc_l_list",dst.enc_l_list);
    from_json_nested(src, "battery",   dst.battery);
    from_json_nested(src, "encoder_raw", dst.encoder_raw);
    from_json_nested(src, "encoder", dst.encoder);
    from_json_nested(src, "ego", dst.ego);
    from_json_nested(src, "sen", dst.sen);
    from_json_nested(src, "sen_dist_log", dst.sen_dist_log);
    from_json_field(src, "calc_time", dst.calc_time);
    from_json_field(src, "calc_time2", dst.calc_time2);
    from_json_field(src, "sensing_timestamp", dst.sensing_timestamp);
    from_json_field(src, "ang_kf_sum", dst.ang_kf_sum);
    from_json_field(src, "img_ang_sum", dst.img_ang_sum);
    from_json_field(src, "img_ang_z", dst.img_ang_z);
}

/**
 * logging
 * velocity / acceleration
 */
inline void convertFromJson(JsonVariantConst src, xva_t& dst) {
    from_json_field(src, "vel", dst.vel);
    from_json_field(src, "speed", dst.speed);
    from_json_field(src, "accl", dst.accl);
}

/**
 * hardware.txt
 * root/<pid_key>  (motor_pid, gyro_pid, front_ctrl_roll_pid, ...)
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
 * root/gyro_param | root/gyro2_param
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
 * root/accel_x_param
 */
inline void convertFromJson(JsonVariantConst src, accel_param_t& dst) {
    from_json_field(src, "gain", dst.gain);
}

/**
 * hardware.txt
 * root/sen_param
 */
inline void convertFromJson(JsonVariantConst src, sen_param_t& dst) {
    from_json_field(src, "lp_delay", dst.lp_delay);
}

/**
 * sensor.txt
 * root/<mode>/ref | exist | search_ref
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
 * root/<mode>/ref_search
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
 * root/<mode>/expand
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
 * root/normal | root/normal2 | root/dia | root/search_*
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
 * root/sensor_gain/<sensor>
 */
inline void convertFromJson(JsonVariantConst src, sensor_gain_param_t& dst) {
    from_json_field(src, "a", dst.a);
    from_json_field(src, "b", dst.b);
}

/**
 * sensor.txt
 * root/sensor_gain
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
 * root/wall_off_dist
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
 * root/fail_check
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
 * root/comp_param
 */
inline void convertFromJson(JsonVariantConst src, comp_param_t& dst) {
    from_json_field(src, "v_lp_gain", dst.v_lp_gain);
    from_json_field(src, "accl_x_hp_gain", dst.accl_x_hp_gain);
    from_json_field(src, "gain", dst.gain);
    from_json_field(src, "enable", dst.enable);
}

/**
 * offset.txt
 * root/kanayama
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
 * logging
 * PID error terms
 */
inline void convertFromJson(JsonVariantConst src, pid_error_t& dst) {
    from_json_field(src, "error_p", dst.error_p);
    from_json_field(src, "error_i", dst.error_i);
    from_json_field(src, "error_i_keep", dst.error_i_keep);
    from_json_field(src, "error_d", dst.error_d);
    from_json_field(src, "error_dd", dst.error_dd);
    from_json_field(src, "i_slow", dst.i_slow);
    from_json_field(src, "i_bias", dst.i_bias);
}

/**
 * logging
 * PID error terms (2)
 */
inline void convertFromJson(JsonVariantConst src, pid_error2_t& dst) {
    from_json_field(src, "p", dst.p);
    from_json_field(src, "i", dst.i);
    from_json_field(src, "i2", dst.i2);
    from_json_field(src, "d", dst.d);
    from_json_field(src, "p_val", dst.p_val);
    from_json_field(src, "i_val", dst.i_val);
    from_json_field(src, "i2_val", dst.i2_val);
    from_json_field(src, "d_val", dst.d_val);
    from_json_field(src, "zz", dst.zz);
    from_json_field(src, "z", dst.z);
}

/**
 * logging
 * gain log
 */
inline void convertFromJson(JsonVariantConst src, gain_log_t& dst) {
    from_json_field(src, "gain_z", dst.gain_z);
    from_json_field(src, "gain_zz", dst.gain_zz);
    from_json_field(src, "omega_ref_prev", dst.omega_ref_prev);
    from_json_field(src, "prev_motion_type", dst.prev_motion_type);
}

/**
 * logging
 * anti-windup log
 */
inline void convertFromJson(JsonVariantConst src, aw_log_t& dst) {
    from_json_field(src, "was_aw", dst.was_aw);
    from_json_field(src, "enter_aw", dst.enter_aw);
    from_json_field(src, "keep_aw", dst.keep_aw);
    from_json_field(src, "w_i_base", dst.w_i_base);
    from_json_field(src, "w_error_i_raw", dst.w_error_i_raw);
    from_json_field(src, "w_error_i_clamped", dst.w_error_i_clamped);
    from_json_field(src, "gyro_pid_histerisis_i", dst.gyro_pid_histerisis_i);
    from_json_field(src, "sat_flag", dst.sat_flag);
    from_json_field(src, "duty_roll", dst.duty_roll);
    from_json_field(src, "duty_roll_before", dst.duty_roll_before);
}

/**
 * logging
 * all PID errors
 */
inline void convertFromJson(JsonVariantConst src, pid_error_entity_t& dst) {
    from_json_nested(src, "v", dst.v);
    from_json_nested(src, "v_kf", dst.v_kf);
    from_json_nested(src, "dist", dst.dist);
    from_json_nested(src, "w", dst.w);
    from_json_nested(src, "v_r", dst.v_r);
    from_json_nested(src, "v_l", dst.v_l);
    from_json_nested(src, "w_kf", dst.w_kf);
    from_json_nested(src, "ang", dst.ang);
    from_json_nested(src, "v_log", dst.v_log);
    from_json_nested(src, "dist_log", dst.dist_log);
    from_json_nested(src, "w_log", dst.w_log);
    from_json_nested(src, "v_r_log", dst.v_r_log);
    from_json_nested(src, "v_l_log", dst.v_l_log);
    from_json_nested(src, "ang_log", dst.ang_log);
    from_json_nested(src, "sen_log", dst.sen_log);
    from_json_nested(src, "sen_log_dia", dst.sen_log_dia);
    from_json_nested(src, "sen", dst.sen);
    from_json_nested(src, "sen_dia", dst.sen_dia);
    from_json_nested(src, "v_val", dst.v_val);
    from_json_nested(src, "w_val", dst.w_val);
    from_json_nested(src, "ang_val", dst.ang_val);
    from_json_nested(src, "s_val", dst.s_val);
    from_json_nested(src, "aw_log", dst.aw_log);
}

/**
 * sub-struct
 * motion target
 */
inline void convertFromJson(JsonVariantConst src, motion_tgt_t& dst) {
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "w_max", dst.w_max);
    from_json_field(src, "alpha", dst.alpha);
}

/**
 * sub-struct
 * motion_tgt_val_t/buzzer
 */
inline void convertFromJson(JsonVariantConst src, buzzer_t& dst) {
    from_json_field(src, "hz", dst.hz);
    from_json_field(src, "time", dst.time);
    from_json_field(src, "timstamp", dst.timstamp);
}

/**
 * sub-struct
 * motion_tgt_val_t/pl_req
 */
inline void convertFromJson(JsonVariantConst src, planning_req_t& dst) {
    from_json_field(src, "time_stamp", dst.time_stamp);
    from_json_field(src, "error_gyro_reset", dst.error_gyro_reset);
    from_json_field(src, "error_vel_reset", dst.error_vel_reset);
    from_json_field(src, "error_led_reset", dst.error_led_reset);
    from_json_field(src, "error_ang_reset", dst.error_ang_reset);
    from_json_field(src, "error_dist_reset", dst.error_dist_reset);
}

/**
 * sub-struct
 * motion_tgt_val_t/fss
 */
inline void convertFromJson(JsonVariantConst src, fail_safe_state_t& dst) {
    from_json_field(src, "error", dst.error);
}

/**
 * sub-struct
 * new_motion_req_t/sys_id
 */
inline void convertFromJson(JsonVariantConst src, sys_id_t& dst) {
    from_json_field(src, "right_v", dst.right_v);
    from_json_field(src, "left_v", dst.left_v);
    from_json_field(src, "enable", dst.enable);
}

/**
 * sub-struct
 * motion_tgt_val_t/nmr
 */
inline void convertFromJson(JsonVariantConst src, new_motion_req_t& dst) {
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "v_end", dst.v_end);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "decel", dst.decel);
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "w_max", dst.w_max);
    from_json_field(src, "w_end", dst.w_end);
    from_json_field(src, "alpha", dst.alpha);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "sla_alpha", dst.sla_alpha);
    from_json_field(src, "sla_time", dst.sla_time);
    from_json_field(src, "sla_pow_n", dst.sla_pow_n);
    from_json_field(src, "sla_rad", dst.sla_rad);
    from_json_field(src, "dia90_offset", dst.dia90_offset);
    from_json_field(src, "td", dst.td);
    from_json_field(src, "tt", dst.tt);
    from_json_field(src, "motion_mode", dst.motion_mode);
    from_json_field(src, "motion_type", dst.motion_type);
    from_json_field(src, "timstamp", dst.timstamp);
    from_json_field(src, "motion_dir", dst.motion_dir);
    from_json_field(src, "dia_mode", dst.dia_mode);
    from_json_field(src, "sct", dst.sct);
    from_json_nested(src, "sys_id", dst.sys_id);
    from_json_field(src, "tgt_reset_req", dst.tgt_reset_req);
    from_json_field(src, "ego_reset_req", dst.ego_reset_req);
}

/**
 * sub-struct
 * motion_tgt_val_t/global_pos
 */
inline void convertFromJson(JsonVariantConst src, global_ego_pos_t& dst) {
    from_json_field(src, "img_dist", dst.img_dist);
    from_json_field(src, "img_ang", dst.img_ang);
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "ang", dst.ang);
}

/**
 * sub-struct
 * motion_tgt_val_t/p
 */
inline void convertFromJson(JsonVariantConst src, pos_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
}

/**
 * sub-struct
 * motion_tgt_val_t/dia_state
 */
inline void convertFromJson(JsonVariantConst src, dia_state_t& dst) {
    from_json_field(src, "right_old", dst.right_old);
    from_json_field(src, "left_old", dst.left_old);
    from_json_field(src, "right_save", dst.right_save);
    from_json_field(src, "left_save", dst.left_save);
    from_json_field(src, "dia90_offset", dst.dia90_offset);
}

/**
 * sub-struct
 * t_tgt/kanayama_gain
 */
inline void convertFromJson(JsonVariantConst src, t_kanayama_gain& dst) {
    from_json_field(src, "k_x",     dst.k_x);
    from_json_field(src, "k_y",     dst.k_y);
    from_json_field(src, "k_theta", dst.k_theta);
}

/**
 * sub-struct
 * t_tgt/accl_param
 */
inline void convertFromJson(JsonVariantConst src, t_accl_param& dst) {
    from_json_field(src, "limit",          dst.limit);
    from_json_field(src, "n",              dst.n);
    from_json_field(src, "decel_delay_cnt",dst.decel_delay_cnt);
    from_json_field(src, "decel_delay_n",  dst.decel_delay_n);
}

/**
 * sub-struct
 * t_ego/sla_param
 */
inline void convertFromJson(JsonVariantConst src, t_slalom& dst) {
    from_json_field(src, "base_alpha",       dst.base_alpha);
    from_json_field(src, "base_time",        dst.base_time);
    from_json_field(src, "limit_time_count", dst.limit_time_count);
    from_json_field(src, "pow_n",            dst.pow_n);
    from_json_field(src, "state",            dst.state);
    from_json_field(src, "counter",          dst.counter);
}

/**
 * sub-struct
 * t_ego/trj_diff
 */
inline void convertFromJson(JsonVariantConst src, t_trajectory_diff& dst) {
    from_json_field(src, "x",     dst.x);
    from_json_field(src, "y",     dst.y);
    from_json_field(src, "theta", dst.theta);
}

/**
 * sub-struct
 * t_ego/kanayama_point
 */
inline void convertFromJson(JsonVariantConst src, t_kanayama_tgt_point& dst) {
    from_json_field(src, "x",     dst.x);
    from_json_field(src, "y",     dst.y);
    from_json_field(src, "theta", dst.theta);
    from_json_field(src, "v",     dst.v);
    from_json_field(src, "w",     dst.w);
}

/**
 * sub-struct
 * t_ego/ideal_point | t_ego/slip_point
 */
inline void convertFromJson(JsonVariantConst src, t_point& dst) {
    from_json_field(src, "x",          dst.x);
    from_json_field(src, "y",          dst.y);
    from_json_field(src, "theta",      dst.theta);
    from_json_field(src, "v",          dst.v);
    from_json_field(src, "w",          dst.w);
    from_json_field(src, "slip_angle", dst.slip_angle);
}

/**
 * sub-struct
 * t_ego/slip
 */
inline void convertFromJson(JsonVariantConst src, t_slip& dst) {
    from_json_field(src, "beta", dst.beta);
    from_json_field(src, "vx",   dst.vx);
    from_json_field(src, "vy",   dst.vy);
    from_json_field(src, "v",    dst.v);
    from_json_field(src, "accl", dst.accl);
}

/**
 * sub-struct
 * motion_tgt_val_t/tgt_in
 */
inline void convertFromJson(JsonVariantConst src, t_tgt& dst) {
    from_json_field(src,  "v_max",                  dst.v_max);
    from_json_field(src,  "end_v",                  dst.end_v);
    from_json_field(src,  "accl",                   dst.accl);
    from_json_field(src,  "decel",                  dst.decel);
    from_json_field(src,  "w_max",                  dst.w_max);
    from_json_field(src,  "end_w",                  dst.end_w);
    from_json_field(src,  "alpha",                  dst.alpha);
    from_json_field(src,  "tgt_dist",               dst.tgt_dist);
    from_json_field(src,  "tgt_angle",              dst.tgt_angle);
    from_json_field(src,  "trajectory_point_size",  dst.trajectory_point_size);
    from_json_nested(src, "kanayama_gain",           dst.kanayama_gain);
    from_json_nested(src, "accl_param",              dst.accl_param);
    from_json_field(src,  "slip_gain",               dst.slip_gain);
    from_json_field(src,  "limit_accl_ratio_cnt",    dst.limit_accl_ratio_cnt);
    from_json_field(src,  "limit_decel_ratio_cnt",   dst.limit_decel_ratio_cnt);
    from_json_field(src,  "slip_gain_K1",            dst.slip_gain_K1);
    from_json_field(src,  "slip_gain_K2",            dst.slip_gain_K2);
    from_json_field(src,  "time_step2",              dst.time_step2);
    from_json_field(src,  "axel_degenerate_gain",    dst.axel_degenerate_gain);
    from_json_field(src,  "enable_slip_decel",       dst.enable_slip_decel);
}

/**
 * sub-struct
 * motion_tgt_val_t/ego_in
 */
inline void convertFromJson(JsonVariantConst src, t_ego& dst) {
    from_json_field(src,  "v",                     dst.v);
    from_json_field(src,  "v_r",                   dst.v_r);
    from_json_field(src,  "v_l",                   dst.v_l);
    from_json_field(src,  "pos_x",                 dst.pos_x);
    from_json_field(src,  "pos_y",                 dst.pos_y);
    from_json_field(src,  "ideal_px",              dst.ideal_px);
    from_json_field(src,  "ideal_py",              dst.ideal_py);
    from_json_field(src,  "accl",                  dst.accl);
    from_json_field(src,  "w",                     dst.w);
    from_json_field(src,  "alpha",                 dst.alpha);
    from_json_field(src,  "alpha2",                dst.alpha2);
    from_json_field(src,  "dist",                  dst.dist);
    from_json_field(src,  "ang",                   dst.ang);
    from_json_field(src,  "img_dist",              dst.img_dist);
    from_json_field(src,  "img_ang",               dst.img_ang);
    from_json_nested(src, "sla_param",             dst.sla_param);
    from_json_field(src,  "state",                 dst.state);
    from_json_field(src,  "pivot_state",           dst.pivot_state);
    from_json_nested(src, "ideal_point",           dst.ideal_point);
    from_json_nested(src, "slip_point",            dst.slip_point);
    from_json_nested(src, "kanayama_point",        dst.kanayama_point);
    from_json_nested(src, "trj_diff",              dst.trj_diff);
    from_json_field(src,  "delay_accl",            dst.delay_accl);
    from_json_field(src,  "delay_v",               dst.delay_v);
    from_json_field(src,  "cnt_delay_accl_ratio",  dst.cnt_delay_accl_ratio);
    from_json_field(src,  "cnt_delay_decel_ratio", dst.cnt_delay_decel_ratio);
    from_json_nested(src, "slip",                  dst.slip);
    from_json_field(src,  "ff_duty_l",             dst.ff_duty_l);
    from_json_field(src,  "ff_duty_r",             dst.ff_duty_r);
    from_json_field(src,  "ff_duty_low_th",        dst.ff_duty_low_th);
    from_json_field(src,  "ff_duty_low_v_th",      dst.ff_duty_low_v_th);
    from_json_field(src,  "ff_duty_front",         dst.ff_duty_front);
    from_json_field(src,  "ff_duty_roll",          dst.ff_duty_roll);
    from_json_field(src,  "ff_duty_rpm_r",         dst.ff_duty_rpm_r);
    from_json_field(src, "ff_duty_rpm_l", dst.ff_duty_rpm_l);
    from_json_field(src, "ff_front_torque", dst.ff_front_torque);
    from_json_field(src, "ff_roll_torque", dst.ff_roll_torque);
    from_json_field(src, "ff_friction_torque_l", dst.ff_friction_torque_l);
    from_json_field(src, "ff_friction_torque_r", dst.ff_friction_torque_r);
    from_json_field(src, "decel_delay_cnt", dst.decel_delay_cnt);
}

/**
 * logging
 * motion target values (runtime state)
 */
inline void convertFromJson(JsonVariantConst src, motion_tgt_val_t& dst) {
    from_json_nested(src, "tgt_in", dst.tgt_in);
    from_json_nested(src, "ego_in", dst.ego_in);
    from_json_field(src, "calc_time", dst.calc_time);
    from_json_field(src, "calc_time2", dst.calc_time2);
    from_json_field(src, "calc_time_diff", dst.calc_time_diff);
    from_json_nested(src, "global_pos", dst.global_pos);
    from_json_field(src, "motion_mode", dst.motion_mode);
    from_json_field(src, "motion_type", dst.motion_type);
    from_json_field(src, "motion_dir", dst.motion_dir);
    from_json_field(src, "dia_mode", dst.dia_mode);
    from_json_nested(src, "pl_req", dst.pl_req);
    from_json_nested(src, "fss", dst.fss);
    from_json_field(src, "gyro_zero_p_offset", dst.gyro_zero_p_offset);
    from_json_field(src, "var_unbiased_dps2", dst.var_unbiased_dps2);
    from_json_field(src, "var_robust_dps2", dst.var_robust_dps2);
    from_json_field(src, "gyro_retry", dst.gyro_retry);
    from_json_field(src, "calibration_mode", dst.calibration_mode);
    from_json_field(src, "gyro2_zero_p_offset", dst.gyro2_zero_p_offset);
    from_json_field(src, "accel_x_zero_p_offset", dst.accel_x_zero_p_offset);
    from_json_field(src, "accel_y_zero_p_offset", dst.accel_y_zero_p_offset);
    from_json_field(src, "temp_zero_p_offset", dst.temp_zero_p_offset);
    from_json_nested(src, "buzzer", dst.buzzer);
    from_json_nested(src, "nmr",       dst.nmr);
    from_json_nested(src, "p",         dst.p);
    from_json_nested(src, "dia_state", dst.dia_state);
    from_json_field(src,  "v_error",   dst.v_error);
    from_json_field(src,  "w_error",   dst.w_error);
    from_json_field(src,  "td",        dst.td);   // TurnDirection — enum, static_cast 自動
    from_json_field(src,  "tt",        dst.tt);   // TurnType      — enum, static_cast 自動
    from_json_field(src,  "duty_suction", dst.duty_suction);
}

/**
 * sub-struct
 * planning straight param
 */
inline void convertFromJson(JsonVariantConst src, param_straight_t& dst) {
    from_json_field(src, "v_max",                dst.v_max);
    from_json_field(src, "v_end",                dst.v_end);
    from_json_field(src, "accl",                 dst.accl);
    from_json_field(src, "decel",                dst.decel);
    from_json_field(src, "dist",                 dst.dist);
    from_json_field(src, "motion_type",          dst.motion_type);   // enum
    from_json_field(src, "sct",                  dst.sct);           // enum
    from_json_field(src, "wall_off_req",         dst.wall_off_req);  // enum
    from_json_field(src, "wall_ctrl_mode",       dst.wall_ctrl_mode);// enum
    from_json_field(src, "wall_off_dist_r",      dst.wall_off_dist_r);
    from_json_field(src, "wall_off_dist_l",      dst.wall_off_dist_l);
    from_json_field(src, "dia_mode",             dst.dia_mode);
    from_json_field(src, "skil_wall_off",        dst.skil_wall_off);
    from_json_field(src, "search_str_wide_ctrl_r", dst.search_str_wide_ctrl_r);
    from_json_field(src, "search_str_wide_ctrl_l", dst.search_str_wide_ctrl_l);
    from_json_field(src, "dia90_offset",         dst.dia90_offset);
}

/**
 * sub-struct
 * planning roll param
 */
inline void convertFromJson(JsonVariantConst src, param_roll_t& dst) {
    from_json_field(src, "w_max", dst.w_max);
    from_json_field(src, "w_end", dst.w_end);
    from_json_field(src, "alpha", dst.alpha);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "RorL", dst.RorL);
}

/**
 * sub-struct
 * planning normal slalom param
 */
inline void convertFromJson(JsonVariantConst src, param_normal_slalom_t& dst) {
    from_json_field(src, "radius", dst.radius);
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "v_end", dst.v_end);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "RorL", dst.RorL);
}
/**
 * system.yaml
 * root/test
 */
/**
 * system.txt
 * root/test
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
 * system.yaml
 * root
 */
/**
 * system.txt
 * root
 */
inline void convertFromJson(JsonVariantConst src, system_t& dst) {
    from_json_vector(src, "goals", dst.goals);
    from_json_field(src, "maze_size", dst.maze_size);
    from_json_field(src, "user_mode", dst.user_mode);
    from_json_field(src, "circuit_mode", dst.circuit_mode);
    from_json_nested(src, "test", dst.test);
    from_json_field(src, "hf_cl", dst.hf_cl);
}

/**
 * profiles.txt
 * root/profile_list/<name>
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
 * sub-struct
 * motor request
 */
inline void convertFromJson(JsonVariantConst src, motor_req_t& dst) {
    from_json_field(src, "enable", dst.enable);
    from_json_field(src, "timestamp", dst.timestamp);
}

/**
 * sub-struct
 * planning
 */
inline void convertFromJson(JsonVariantConst src, slalom_offset_t& dst) {
    from_json_field(src, "right", dst.right);
    from_json_field(src, "left", dst.left);
}

/**
 * <profile>.txt
 * root/<turn_type>
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
 * root/v_prof[i]/<str_type>
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
 * sub-struct
 * parameter set
 */
inline void convertFromJson(JsonVariantConst src, param_set_t& dst) {
    // map unordered_map skipped
    // map_slow unordered_map skipped
    // map_fast unordered_map skipped
    // str_map unordered_map skipped
    from_json_field(src, "suction", dst.suction);
    from_json_field(src, "suction_duty", dst.suction_duty);
    from_json_field(src, "suction_duty_low", dst.suction_duty_low);
    from_json_field(src, "cell_size", dst.cell_size);
    from_json_field(src, "start_offset", dst.start_offset);
}

/**
 * sub-struct
 * path set
 */
inline void convertFromJson(JsonVariantConst src, path_set_t& dst) {
    from_json_vector(src, "path_s", dst.path_s);
    // path_t vector<unsigned char> skipped
    from_json_field(src, "time", dst.time);
    from_json_field(src, "result", dst.result);
    from_json_field(src, "type", dst.type);
}

/**
 * sub-struct
 * path request
 */
inline void convertFromJson(JsonVariantConst src, path_req_t& dst) {
    from_json_field(src, "time", dst.time);
}

/**
 * sub-struct
 * path creation result
 */
inline void convertFromJson(JsonVariantConst src, create_path_result_t& dst) {
    from_json_field(src, "time", dst.time);
}

/**
 * sub-struct
 * next motion
 */
inline void convertFromJson(JsonVariantConst src, next_motion_t& dst) {
    from_json_field(src, "is_turn", dst.is_turn);
    from_json_field(src, "next_turn_type", dst.next_turn_type);
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "v_end", dst.v_end);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "decel", dst.decel);
    from_json_field(src, "skip_wall_off", dst.skip_wall_off);
    from_json_field(src, "carry_over_dist", dst.carry_over_dist);
}

/**
 * logging
 * motion log (float)
 */
inline void convertFromJson(JsonVariantConst src, log_data_t& dst) {
    from_json_field(src, "img_v", dst.img_v);
    from_json_field(src, "v_l", dst.v_l);
    from_json_field(src, "v_c", dst.v_c);
    from_json_field(src, "v_r", dst.v_r);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "img_w", dst.img_w);
    from_json_field(src, "w_lp", dst.w_lp);
    from_json_field(src, "alpha", dst.alpha);
    from_json_field(src, "img_dist", dst.img_dist);
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "img_ang", dst.img_ang);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "duty_l", dst.duty_l);
    from_json_field(src, "duty_r", dst.duty_r);
    from_json_field(src, "left90_lp", dst.left90_lp);
    from_json_field(src, "left45_lp", dst.left45_lp);
    from_json_field(src, "front_lp", dst.front_lp);
    from_json_field(src, "right45_lp", dst.right45_lp);
    from_json_field(src, "right90_lp", dst.right90_lp);
    from_json_field(src, "battery_lp", dst.battery_lp);
    from_json_field(src, "motion_type", dst.motion_type);
    from_json_field(src, "duty_ff_front", dst.duty_ff_front);
    from_json_field(src, "duty_ff_roll", dst.duty_ff_roll);
    from_json_field(src, "duty_sensor_ctrl", dst.duty_sensor_ctrl);
    from_json_field(src, "pos_x", dst.pos_x);
    from_json_field(src, "pos_y", dst.pos_y);
}

/**
 * run_prf.txt
 * root/exec_prof[i]
 */
inline void convertFromJson(JsonVariantConst src, exec_pram_t& dst) {
    from_json_field(src, "fast_idx", dst.fast_idx);
    from_json_field(src, "normal_idx", dst.normal_idx);
    from_json_field(src, "slow_idx", dst.slow_idx);
}

/**
 * logging
 * motion log (compact / half_t)
 */
inline void convertFromJson(JsonVariantConst src, log_data_t2& dst) {
    // img_v is half_t, skipping
    // v_l is half_t, skipping
    // v_c is half_t, skipping
    // v_c2 is half_t, skipping
    // v_r is half_t, skipping
    from_json_field(src, "v_r_enc", dst.v_r_enc);
    from_json_field(src, "v_l_enc", dst.v_l_enc);
    // accl is half_t, skipping
    // accl_x is half_t, skipping
    // dist_kf is half_t, skipping
    // img_w is half_t, skipping
    // w_lp is half_t, skipping
    // alpha is half_t, skipping
    // img_dist is half_t, skipping
    // dist is half_t, skipping
    // img_ang is half_t, skipping
    // ang is half_t, skipping
    // ang_kf is half_t, skipping
    // duty_l is half_t, skipping
    // duty_r is half_t, skipping
    from_json_field(src, "left90_lp", dst.left90_lp);
    from_json_field(src, "left45_lp", dst.left45_lp);
    from_json_field(src, "right45_lp", dst.right45_lp);
    from_json_field(src, "right90_lp", dst.right90_lp);
    // battery_lp is half_t, skipping
    from_json_field(src, "left45_2_lp", dst.left45_2_lp);
    from_json_field(src, "right45_2_lp", dst.right45_2_lp);
    from_json_field(src, "left45_3_lp", dst.left45_3_lp);
    from_json_field(src, "right45_3_lp", dst.right45_3_lp);
    from_json_field(src, "motion_type", dst.motion_type);
    from_json_field(src, "motion_timestamp", dst.motion_timestamp);
    // duty_sensor_ctrl is half_t, skipping
    // sen_log_l45 is half_t, skipping
    // sen_log_r45 is half_t, skipping
    // sen_log_l45_2 is half_t, skipping
    // sen_log_r45_2 is half_t, skipping
    // sen_log_l45_3 is half_t, skipping
    // sen_log_r45_3 is half_t, skipping
    from_json_field(src, "sen_calc_time", dst.sen_calc_time);
    from_json_field(src, "sen_calc_time2", dst.sen_calc_time2);
    from_json_field(src, "pln_calc_time", dst.pln_calc_time);
    from_json_field(src, "pln_time_diff", dst.pln_time_diff);
    // m_pid_p is half_t, skipping
    // m_pid_i is half_t, skipping
    // m_pid_i2 is half_t, skipping
    // m_pid_d is half_t, skipping
    // m_pid_p_v is half_t, skipping
    // m_pid_i_v is half_t, skipping
    // m_pid_i2_v is half_t, skipping
    // m_pid_d_v is half_t, skipping
    // g_pid_p is half_t, skipping
    // g_pid_i is half_t, skipping
    // g_pid_i2 is half_t, skipping
    // g_pid_d is half_t, skipping
    // g_pid_p_v is half_t, skipping
    // g_pid_i_v is half_t, skipping
    // g_pid_i2_v is half_t, skipping
    // g_pid_d_v is half_t, skipping
    // ang_pid_p is half_t, skipping
    // ang_pid_i is half_t, skipping
    // ang_pid_d is half_t, skipping
    // ang_pid_p_v is half_t, skipping
    // ang_pid_i_v is half_t, skipping
    // ang_pid_d_v is half_t, skipping
    // s_pid_p is half_t, skipping
    // s_pid_i is half_t, skipping
    // s_pid_i2 is half_t, skipping
    // s_pid_d is half_t, skipping
    // s_pid_p_v is half_t, skipping
    // s_pid_i_v is half_t, skipping
    // s_pid_i2_v is half_t, skipping
    // s_pid_d_v is half_t, skipping
    // ff_duty_front is half_t, skipping
    // ff_duty_roll is half_t, skipping
    // ff_duty_rpm_r is half_t, skipping
    // ff_duty_rpm_l is half_t, skipping
    // pos_x is half_t, skipping
    // pos_y is half_t, skipping
    // knym_v is half_t, skipping
    // knym_w is half_t, skipping
    // odm_x is half_t, skipping
    // odm_y is half_t, skipping
    // odm_theta is half_t, skipping
    // kim_x is half_t, skipping
    // kim_y is half_t, skipping
    // kim_theta is half_t, skipping
    // ang_i_bias is half_t, skipping
    // ang_i_bias_val is half_t, skipping
    // duty_suction is half_t, skipping
    // ang_kf_sum is half_t, skipping
    // img_ang_sum is half_t, skipping
    // duty_roll is half_t, skipping
    // duty_roll_before is half_t, skipping
}

/**
 * logging
 * system identification log
 */
inline void convertFromJson(JsonVariantConst src, sysid_log& dst) {
    // v_l is half_t, skipping
    // v_c is half_t, skipping
    // v_r is half_t, skipping
    // w_lp is half_t, skipping
    // volt_l is half_t, skipping
    // volt_r is half_t, skipping
}

/**
 * sub-struct
 * fail safe
 */
inline void convertFromJson(JsonVariantConst src, fail_safe_t& dst) {
    from_json_field(src, "invalid_front_led", dst.invalid_front_led);
    from_json_field(src, "invalid_duty_r_cnt", dst.invalid_duty_r_cnt);
    from_json_field(src, "invalid_duty_l_cnt", dst.invalid_duty_l_cnt);
    from_json_field(src, "invalid_v_cnt", dst.invalid_v_cnt);
    from_json_field(src, "invalid_w_cnt", dst.invalid_w_cnt);
}

/**
 * sub-struct
 * slip
 */
inline void convertFromJson(JsonVariantConst src, slip_t& dst) {
    from_json_field(src, "K", dst.K);
    from_json_field(src, "k", dst.k);
    from_json_field(src, "beta", dst.beta);
    from_json_field(src, "vx", dst.vx);
    from_json_field(src, "vy", dst.vy);
    from_json_field(src, "v", dst.v);
}

/**
 * sub-struct
 * sensor ctrl keep dist
 */
inline void convertFromJson(JsonVariantConst src, sensor_ctrl_keep_dist_t& dst) {
    from_json_field(src, "star_dist", dst.star_dist);
}

/**
 * sub-struct
 * planning time
 */
inline void convertFromJson(JsonVariantConst src, planning_time_t& dst) {
    from_json_field(src, "v_start", dst.v_start);
    from_json_field(src, "v_max", dst.v_max);
    from_json_field(src, "v_end", dst.v_end);
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "lap_time", dst.lap_time);
    from_json_field(src, "total_time", dst.total_time);
}

/**
 * logging
 * log chunk 1
 */
inline void convertFromJson(JsonVariantConst src, LogStruct1& dst) {
    from_json_field(src, "index", dst.index);
    from_json_field(src, "ideal_v", dst.ideal_v);
    from_json_field(src, "v_c", dst.v_c);
    from_json_field(src, "v_c2", dst.v_c2);
    from_json_field(src, "v_l", dst.v_l);
    from_json_field(src, "v_r", dst.v_r);
    from_json_field(src, "v_l_enc", dst.v_l_enc);
    from_json_field(src, "v_r_enc", dst.v_r_enc);
    from_json_field(src, "v_l_enc_sin", dst.v_l_enc_sin);
    from_json_field(src, "v_r_enc_sin", dst.v_r_enc_sin);
    from_json_field(src, "accl", dst.accl);
    from_json_field(src, "accl_x", dst.accl_x);
}

/**
 * logging
 * log chunk 2
 */
inline void convertFromJson(JsonVariantConst src, LogStruct2& dst) {
    from_json_field(src, "ideal_w", dst.ideal_w);
    from_json_field(src, "w_lp", dst.w_lp);
    from_json_field(src, "alpha", dst.alpha);
    from_json_field(src, "ideal_dist", dst.ideal_dist);
    from_json_field(src, "dist", dst.dist);
    from_json_field(src, "dist_kf", dst.dist_kf);
    from_json_field(src, "ideal_ang", dst.ideal_ang);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "ang_kf", dst.ang_kf);
    from_json_field(src, "left90", dst.left90);
    from_json_field(src, "left45", dst.left45);
    from_json_field(src, "front", dst.front);
}

/**
 * logging
 * log chunk 3
 */
inline void convertFromJson(JsonVariantConst src, LogStruct3& dst) {
    from_json_field(src, "right45", dst.right45);
    from_json_field(src, "right90", dst.right90);
    from_json_field(src, "left90_d", dst.left90_d);
    from_json_field(src, "left45_d", dst.left45_d);
    from_json_field(src, "front_d", dst.front_d);
    from_json_field(src, "right45_d", dst.right45_d);
    from_json_field(src, "right90_d", dst.right90_d);
    from_json_field(src, "left90_far_d", dst.left90_far_d);
    from_json_field(src, "front_far_d", dst.front_far_d);
    from_json_field(src, "right90_far_d", dst.right90_far_d);
    from_json_field(src, "battery", dst.battery);
    from_json_field(src, "duty_l", dst.duty_l);
}

/**
 * logging
 * log chunk 4
 */
inline void convertFromJson(JsonVariantConst src, LogStruct4& dst) {
    from_json_field(src, "duty_r", dst.duty_r);
    from_json_field(src, "motion_state", dst.motion_state);
    from_json_field(src, "duty_sen", dst.duty_sen);
    from_json_field(src, "dist_mod90", dst.dist_mod90);
    from_json_field(src, "sen_dist_l45", dst.sen_dist_l45);
    from_json_field(src, "sen_dist_r45", dst.sen_dist_r45);
    from_json_field(src, "timestamp", dst.timestamp);
    from_json_field(src, "sen_calc_time", dst.sen_calc_time);
    from_json_field(src, "sen_calc_time2", dst.sen_calc_time2);
    from_json_field(src, "pln_calc_time", dst.pln_calc_time);
    from_json_field(src, "pln_calc_time2", dst.pln_calc_time2);
    from_json_field(src, "pln_time_diff", dst.pln_time_diff);
}

/**
 * logging
 * log chunk 5
 */
inline void convertFromJson(JsonVariantConst src, LogStruct5& dst) {
    from_json_field(src, "m_pid_p", dst.m_pid_p);
    from_json_field(src, "m_pid_i", dst.m_pid_i);
    from_json_field(src, "m_pid_i2", dst.m_pid_i2);
    from_json_field(src, "m_pid_d", dst.m_pid_d);
    from_json_field(src, "m_pid_p_v", dst.m_pid_p_v);
    from_json_field(src, "m_pid_i_v", dst.m_pid_i_v);
    from_json_field(src, "m_pid_i2_v", dst.m_pid_i2_v);
    from_json_field(src, "m_pid_d_v", dst.m_pid_d_v);
    from_json_field(src, "g_pid_p", dst.g_pid_p);
    from_json_field(src, "g_pid_i", dst.g_pid_i);
    from_json_field(src, "g_pid_i2", dst.g_pid_i2);
    from_json_field(src, "g_pid_d", dst.g_pid_d);
}

/**
 * logging
 * log chunk 6
 */
inline void convertFromJson(JsonVariantConst src, LogStruct6& dst) {
    from_json_field(src, "g_pid_p_v", dst.g_pid_p_v);
    from_json_field(src, "g_pid_i_v", dst.g_pid_i_v);
    from_json_field(src, "g_pid_i2_v", dst.g_pid_i2_v);
    from_json_field(src, "g_pid_d_v", dst.g_pid_d_v);
    from_json_field(src, "s_pid_p", dst.s_pid_p);
    from_json_field(src, "s_pid_i", dst.s_pid_i);
    from_json_field(src, "s_pid_i2", dst.s_pid_i2);
    from_json_field(src, "s_pid_d", dst.s_pid_d);
    from_json_field(src, "s_pid_p_v", dst.s_pid_p_v);
    from_json_field(src, "s_pid_i_v", dst.s_pid_i_v);
    from_json_field(src, "s_pid_i2_v", dst.s_pid_i2_v);
    from_json_field(src, "s_pid_d_v", dst.s_pid_d_v);
}

/**
 * logging
 * log chunk 7
 */
inline void convertFromJson(JsonVariantConst src, LogStruct7& dst) {
    from_json_field(src, "ang_pid_p", dst.ang_pid_p);
    from_json_field(src, "ang_pid_i", dst.ang_pid_i);
    from_json_field(src, "ang_pid_d", dst.ang_pid_d);
    from_json_field(src, "ang_pid_p_v", dst.ang_pid_p_v);
    from_json_field(src, "ang_pid_i_v", dst.ang_pid_i_v);
    from_json_field(src, "ang_pid_d_v", dst.ang_pid_d_v);
    from_json_field(src, "ff_duty_front", dst.ff_duty_front);
    from_json_field(src, "ff_duty_roll", dst.ff_duty_roll);
    from_json_field(src, "ff_duty_rpm_r", dst.ff_duty_rpm_r);
    from_json_field(src, "ff_duty_rpm_l", dst.ff_duty_rpm_l);
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
}

/**
 * logging
 * log chunk 8
 */
inline void convertFromJson(JsonVariantConst src, LogStruct8& dst) {
    from_json_field(src, "right45_2", dst.right45_2);
    from_json_field(src, "right45_3", dst.right45_3);
    from_json_field(src, "left45_2", dst.left45_2);
    from_json_field(src, "left45_3", dst.left45_3);
    from_json_field(src, "right45_2_d", dst.right45_2_d);
    from_json_field(src, "right45_3_d", dst.right45_3_d);
    from_json_field(src, "left45_2_d", dst.left45_2_d);
    from_json_field(src, "left45_3_d", dst.left45_3_d);
    from_json_field(src, "sen_dist_l45_2", dst.sen_dist_l45_2);
    from_json_field(src, "sen_dist_r45_2", dst.sen_dist_r45_2);
    from_json_field(src, "sen_dist_l45_3", dst.sen_dist_l45_3);
    from_json_field(src, "sen_dist_r45_3", dst.sen_dist_r45_3);
}

/**
 * logging
 * log chunk 9
 */
inline void convertFromJson(JsonVariantConst src, LogStruct9& dst) {
    from_json_field(src, "knym_v", dst.knym_v);
    from_json_field(src, "knym_w", dst.knym_w);
    from_json_field(src, "odm_x", dst.odm_x);
    from_json_field(src, "odm_y", dst.odm_y);
    from_json_field(src, "odm_theta", dst.odm_theta);
    from_json_field(src, "kim_x", dst.kim_x);
    from_json_field(src, "kim_y", dst.kim_y);
    from_json_field(src, "kim_theta", dst.kim_theta);
    from_json_field(src, "ang_i_bias", dst.ang_i_bias);
    from_json_field(src, "ang_i_bias_val", dst.ang_i_bias_val);
    from_json_field(src, "left90_d_diff", dst.left90_d_diff);
    from_json_field(src, "right90_d_diff", dst.right90_d_diff);
}

/**
 * logging
 * log chunk 10
 */
inline void convertFromJson(JsonVariantConst src, LogStruct10& dst) {
    from_json_field(src, "right45_3_d_diff", dst.right45_3_d_diff);
    from_json_field(src, "right45_2_d_diff", dst.right45_2_d_diff);
    from_json_field(src, "right45_d_diff", dst.right45_d_diff);
    from_json_field(src, "left45_d_diff", dst.left45_d_diff);
    from_json_field(src, "left45_2_d_diff", dst.left45_2_d_diff);
    from_json_field(src, "left45_3_d_diff", dst.left45_3_d_diff);
    from_json_field(src, "duty_suction", dst.duty_suction);
    from_json_field(src, "duty_roll", dst.duty_roll);
    from_json_field(src, "ang_kf_sum", dst.ang_kf_sum);
    from_json_field(src, "img_ang_sum", dst.img_ang_sum);
    from_json_field(src, "duty_roll_before", dst.duty_roll_before);
    from_json_field(src, "reserve5", dst.reserve5);
}

