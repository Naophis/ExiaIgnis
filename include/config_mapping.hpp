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
    } else {
        if (src[key].is<T>()) dst = src[key].as<T>();
    }
}

// volatile スカラー / volatile enum
template<typename T>
inline void from_json_field(JsonVariantConst src, const char* key, volatile T& dst) {
    if constexpr (std::is_enum_v<T>) {
        if (src[key].is<int>()) dst = static_cast<T>(src[key].as<int>());
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

inline void convertFromJson(JsonVariantConst src, route_t& dst) {
    from_json_nested(src, "dir", dst.dir);
    from_json_field(src, "time", dst.time);
    from_json_field(src, "use", dst.use);
}

inline void convertFromJson(JsonVariantConst src, path_create_status_t& dst) {
    from_json_field(src, "time", dst.time);
    from_json_field(src, "state", dst.state);
    from_json_field(src, "use", dst.use);
}

inline void convertFromJson(JsonVariantConst src, path_type& dst) {
    from_json_vector(src, "s", dst.s);
    from_json_vector(src, "t", dst.t);
    from_json_field(src, "size", dst.size);
}

inline void convertFromJson(JsonVariantConst src, point_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
}

inline void convertFromJson(JsonVariantConst src, dir_pt_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_nested(src, "dir", dst.dir);
    from_json_field(src, "dist2", dst.dist2);
}

inline void convertFromJson(JsonVariantConst src, ego_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_nested(src, "dir", dst.dir);
    from_json_field(src, "prev_motion", dst.prev_motion);
}

inline void convertFromJson(JsonVariantConst src, path_element& dst) {
    from_json_field(src, "s", dst.s);
    from_json_field(src, "t", dst.t);
}

inline void convertFromJson(JsonVariantConst src, path_struct& dst) {
    from_json_vector(src, "paths", dst.paths);
    from_json_field(src, "size", dst.size);
}

inline void convertFromJson(JsonVariantConst src, trajectory_point_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "ang", dst.ang);
    from_json_field(src, "base_ang_accl", dst.base_ang_accl);
    from_json_field(src, "type", dst.type);
}

inline void convertFromJson(JsonVariantConst src, ego_odom_t& dst) {
    from_json_field(src, "x", dst.x);
    from_json_field(src, "y", dst.y);
    from_json_field(src, "ang", dst.ang);
    from_json_nested(src, "dir", dst.dir);
}

inline void convertFromJson(JsonVariantConst src, slalom_param_t& dst) {
    from_json_field(src, "radius", dst.radius);
    from_json_field(src, "time", dst.time);
    from_json_field(src, "n", dst.n);
    from_json_field(src, "front", dst.front);
    from_json_field(src, "back", dst.back);
}

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

inline void convertFromJson(JsonVariantConst src, run_param_t& dst) {
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["decel"].is<float>()) dst.decel = src["decel"].as<float>();
    if (src["turn_v"].is<float>()) dst.turn_v = src["turn_v"].as<float>();
}

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

inline void convertFromJson(JsonVariantConst src, encoder_data_t& dst) {
    from_json_field(src, "right",     dst.right);
    from_json_field(src, "left",      dst.left);
    from_json_field(src, "right_old", dst.right_old);
    from_json_field(src, "left_old",  dst.left_old);
}

inline void convertFromJson(JsonVariantConst src, sensing_data_t& dst) {
    if (src["raw"].is<int>()) dst.raw = src["raw"].as<int>();
    if (src["data"].is<float>()) dst.data = src["data"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, led_sensor_t& dst) {
    if (!src["right90"].isNull()) convertFromJson(src["right90"], dst.right90);
    if (!src["right45"].isNull()) convertFromJson(src["right45"], dst.right45);
    if (!src["right45_2"].isNull()) convertFromJson(src["right45_2"], dst.right45_2);
    if (!src["right45_3"].isNull()) convertFromJson(src["right45_3"], dst.right45_3);
    if (!src["front"].isNull()) convertFromJson(src["front"], dst.front);
    if (!src["left45"].isNull()) convertFromJson(src["left45"], dst.left45);
    if (!src["left45_2"].isNull()) convertFromJson(src["left45_2"], dst.left45_2);
    if (!src["left45_3"].isNull()) convertFromJson(src["left45_3"], dst.left45_3);
    if (!src["left90"].isNull()) convertFromJson(src["left90"], dst.left90);
}

inline void convertFromJson(JsonVariantConst src, rpm_t& dst) {
    if (src["right"].is<float>()) dst.right = src["right"].as<float>();
    if (src["left"].is<float>()) dst.left = src["left"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, duty_t& dst) {
    if (src["duty_l"].is<float>()) dst.duty_l = src["duty_l"].as<float>();
    if (src["duty_r"].is<float>()) dst.duty_r = src["duty_r"].as<float>();
    if (src["duty_suction"].is<float>()) dst.duty_suction = src["duty_suction"].as<float>();
    if (src["duty_suction_low"].is<float>()) dst.duty_suction_low = src["duty_suction_low"].as<float>();
    if (src["sen"].is<float>()) dst.sen = src["sen"].as<float>();
    if (src["sen_ang"].is<float>()) dst.sen_ang = src["sen_ang"].as<float>();
    if (src["ff_duty_front"].is<float>()) dst.ff_duty_front = src["ff_duty_front"].as<float>();
    if (src["ff_duty_roll"].is<float>()) dst.ff_duty_roll = src["ff_duty_roll"].as<float>();
    if (src["ff_duty_rpm_r"].is<float>()) dst.ff_duty_rpm_r = src["ff_duty_rpm_r"].as<float>();
    if (src["ff_duty_rpm_l"].is<float>()) dst.ff_duty_rpm_l = src["ff_duty_rpm_l"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, ff_duty_t& dst) {
    if (src["front"].is<float>()) dst.front = src["front"].as<float>();
    if (src["roll"].is<float>()) dst.roll = src["roll"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, ego_entity_t& dst) {
    if (src["v_r"].is<float>()) dst.v_r = src["v_r"].as<float>();
    if (src["v_l"].is<float>()) dst.v_l = src["v_l"].as<float>();
    if (src["v_r_old"].is<float>()) dst.v_r_old = src["v_r_old"].as<float>();
    if (src["v_l_old"].is<float>()) dst.v_l_old = src["v_l_old"].as<float>();
    if (src["v_c"].is<float>()) dst.v_c = src["v_c"].as<float>();
    if (src["filter_v"].is<float>()) dst.filter_v = src["filter_v"].as<float>();
    if (src["main_v"].is<float>()) dst.main_v = src["main_v"].as<float>();
    if (src["w_raw"].is<float>()) dst.w_raw = src["w_raw"].as<float>();
    if (src["w_raw2"].is<float>()) dst.w_raw2 = src["w_raw2"].as<float>();
    if (src["w_lp"].is<float>()) dst.w_lp = src["w_lp"].as<float>();
    if (src["w_lp2"].is<float>()) dst.w_lp2 = src["w_lp2"].as<float>();
    if (src["w_kf"].is<float>()) dst.w_kf = src["w_kf"].as<float>();
    if (src["w_kf2"].is<float>()) dst.w_kf2 = src["w_kf2"].as<float>();
    if (src["v_kf"].is<float>()) dst.v_kf = src["v_kf"].as<float>();
    if (src["dist_kf"].is<float>()) dst.dist_kf = src["dist_kf"].as<float>();
    if (src["ang_kf"].is<float>()) dst.ang_kf = src["ang_kf"].as<float>();
    if (src["ang_kf2"].is<float>()) dst.ang_kf2 = src["ang_kf2"].as<float>();
    if (src["batt_kf"].is<float>()) dst.batt_kf = src["batt_kf"].as<float>();
    if (src["accel_x_raw"].is<float>()) dst.accel_x_raw = src["accel_x_raw"].as<float>();
    if (src["v_ave"].is<float>()) dst.v_ave = src["v_ave"].as<float>();
    if (src["v_lp"].is<float>()) dst.v_lp = src["v_lp"].as<float>();
    if (src["integrate_accl_x_ave"].is<float>()) dst.integrate_accl_x_ave = src["integrate_accl_x_ave"].as<float>();
    if (src["sum_v_ave"].is<float>()) dst.sum_v_ave = src["sum_v_ave"].as<float>();
    if (src["sum_integrate_accl_x_ave"].is<float>()) dst.sum_integrate_accl_x_ave = src["sum_integrate_accl_x_ave"].as<float>();
    if (src["w_kalman"].is<float>()) dst.w_kalman = src["w_kalman"].as<float>();
    if (src["ang_kalman"].is<float>()) dst.ang_kalman = src["ang_kalman"].as<float>();
    if (src["battery_raw"].is<float>()) dst.battery_raw = src["battery_raw"].as<float>();
    if (src["battery_lp"].is<float>()) dst.battery_lp = src["battery_lp"].as<float>();
    if (src["right90_raw"].is<float>()) dst.right90_raw = src["right90_raw"].as<float>();
    if (src["right90_lp"].is<float>()) dst.right90_lp = src["right90_lp"].as<float>();
    if (src["right45_raw"].is<float>()) dst.right45_raw = src["right45_raw"].as<float>();
    if (src["right45_lp"].is<float>()) dst.right45_lp = src["right45_lp"].as<float>();
    if (src["front_raw"].is<float>()) dst.front_raw = src["front_raw"].as<float>();
    if (src["front_lp"].is<float>()) dst.front_lp = src["front_lp"].as<float>();
    if (src["left45_raw"].is<float>()) dst.left45_raw = src["left45_raw"].as<float>();
    if (src["left45_lp"].is<float>()) dst.left45_lp = src["left45_lp"].as<float>();
    if (src["left90_raw"].is<float>()) dst.left90_raw = src["left90_raw"].as<float>();
    if (src["left90_lp"].is<float>()) dst.left90_lp = src["left90_lp"].as<float>();
    if (src["right45_2_raw"].is<float>()) dst.right45_2_raw = src["right45_2_raw"].as<float>();
    if (src["right45_2_lp"].is<float>()) dst.right45_2_lp = src["right45_2_lp"].as<float>();
    if (src["left45_2_raw"].is<float>()) dst.left45_2_raw = src["left45_2_raw"].as<float>();
    if (src["left45_2_lp"].is<float>()) dst.left45_2_lp = src["left45_2_lp"].as<float>();
    if (src["right45_3_raw"].is<float>()) dst.right45_3_raw = src["right45_3_raw"].as<float>();
    if (src["right45_3_lp"].is<float>()) dst.right45_3_lp = src["right45_3_lp"].as<float>();
    if (src["left45_3_raw"].is<float>()) dst.left45_3_raw = src["left45_3_raw"].as<float>();
    if (src["left45_3_lp"].is<float>()) dst.left45_3_lp = src["left45_3_lp"].as<float>();
    if (src["front_lp_old"].is<float>()) dst.front_lp_old = src["front_lp_old"].as<float>();
    if (src["left45_lp_old"].is<float>()) dst.left45_lp_old = src["left45_lp_old"].as<float>();
    if (src["left90_lp_old"].is<float>()) dst.left90_lp_old = src["left90_lp_old"].as<float>();
    if (src["right45_lp_old"].is<float>()) dst.right45_lp_old = src["right45_lp_old"].as<float>();
    if (src["right90_lp_old"].is<float>()) dst.right90_lp_old = src["right90_lp_old"].as<float>();
    if (src["left45_2_lp_old"].is<float>()) dst.left45_2_lp_old = src["left45_2_lp_old"].as<float>();
    if (src["right45_2_lp_old"].is<float>()) dst.right45_2_lp_old = src["right45_2_lp_old"].as<float>();
    if (src["left45_3_lp_old"].is<float>()) dst.left45_3_lp_old = src["left45_3_lp_old"].as<float>();
    if (src["right45_3_lp_old"].is<float>()) dst.right45_3_lp_old = src["right45_3_lp_old"].as<float>();
    if (src["front_dist"].is<float>()) dst.front_dist = src["front_dist"].as<float>();
    if (src["left45_dist"].is<float>()) dst.left45_dist = src["left45_dist"].as<float>();
    if (src["left45_2_dist"].is<float>()) dst.left45_2_dist = src["left45_2_dist"].as<float>();
    if (src["left45_3_dist"].is<float>()) dst.left45_3_dist = src["left45_3_dist"].as<float>();
    if (src["left90_dist"].is<float>()) dst.left90_dist = src["left90_dist"].as<float>();
    if (src["right45_dist"].is<float>()) dst.right45_dist = src["right45_dist"].as<float>();
    if (src["right45_2_dist"].is<float>()) dst.right45_2_dist = src["right45_2_dist"].as<float>();
    if (src["right45_3_dist"].is<float>()) dst.right45_3_dist = src["right45_3_dist"].as<float>();
    if (src["right90_dist"].is<float>()) dst.right90_dist = src["right90_dist"].as<float>();
    if (src["front_far_dist"].is<float>()) dst.front_far_dist = src["front_far_dist"].as<float>();
    if (src["left90_far_dist"].is<float>()) dst.left90_far_dist = src["left90_far_dist"].as<float>();
    if (src["right90_far_dist"].is<float>()) dst.right90_far_dist = src["right90_far_dist"].as<float>();
    if (src["left90_mid_dist"].is<float>()) dst.left90_mid_dist = src["left90_mid_dist"].as<float>();
    if (src["right90_mid_dist"].is<float>()) dst.right90_mid_dist = src["right90_mid_dist"].as<float>();
    if (src["front_mid_dist"].is<float>()) dst.front_mid_dist = src["front_mid_dist"].as<float>();
    if (src["left45_dist_diff"].is<float>()) dst.left45_dist_diff = src["left45_dist_diff"].as<float>();
    if (src["left45_2_dist_diff"].is<float>()) dst.left45_2_dist_diff = src["left45_2_dist_diff"].as<float>();
    if (src["left45_3_dist_diff"].is<float>()) dst.left45_3_dist_diff = src["left45_3_dist_diff"].as<float>();
    if (src["right45_dist_diff"].is<float>()) dst.right45_dist_diff = src["right45_dist_diff"].as<float>();
    if (src["right45_2_dist_diff"].is<float>()) dst.right45_2_dist_diff = src["right45_2_dist_diff"].as<float>();
    if (src["right45_3_dist_diff"].is<float>()) dst.right45_3_dist_diff = src["right45_3_dist_diff"].as<float>();
    if (src["left90_dist_diff"].is<float>()) dst.left90_dist_diff = src["left90_dist_diff"].as<float>();
    if (src["right90_dist_diff"].is<float>()) dst.right90_dist_diff = src["right90_dist_diff"].as<float>();
    if (src["temp"].is<float>()) dst.temp = src["temp"].as<float>();
    if (src["front_dist_old"].is<float>()) dst.front_dist_old = src["front_dist_old"].as<float>();
    if (src["left45_dist_old"].is<float>()) dst.left45_dist_old = src["left45_dist_old"].as<float>();
    if (src["left45_2_dist_old"].is<float>()) dst.left45_2_dist_old = src["left45_2_dist_old"].as<float>();
    if (src["left45_3_dist_old"].is<float>()) dst.left45_3_dist_old = src["left45_3_dist_old"].as<float>();
    if (src["left90_dist_old"].is<float>()) dst.left90_dist_old = src["left90_dist_old"].as<float>();
    if (src["right45_dist_old"].is<float>()) dst.right45_dist_old = src["right45_dist_old"].as<float>();
    if (src["right45_2_dist_old"].is<float>()) dst.right45_2_dist_old = src["right45_2_dist_old"].as<float>();
    if (src["right45_3_dist_old"].is<float>()) dst.right45_3_dist_old = src["right45_3_dist_old"].as<float>();
    if (src["right90_dist_old"].is<float>()) dst.right90_dist_old = src["right90_dist_old"].as<float>();
    if (src["exist_r_wall"].is<bool>()) dst.exist_r_wall = src["exist_r_wall"].as<bool>();
    if (src["exist_l_wall"].is<bool>()) dst.exist_l_wall = src["exist_l_wall"].as<bool>();
    if (!src["rpm"].isNull()) convertFromJson(src["rpm"], dst.rpm);
    if (!src["duty"].isNull()) convertFromJson(src["duty"], dst.duty);
    if (!src["ff_duty"].isNull()) convertFromJson(src["ff_duty"], dst.ff_duty);
    if (src["motion_type"].is<int8_t>()) dst.motion_type = src["motion_type"].as<int8_t>();
    if (src["pos_x"].is<float>()) dst.pos_x = src["pos_x"].as<float>();
    if (src["pos_y"].is<float>()) dst.pos_y = src["pos_y"].as<float>();
    if (src["pos_ang"].is<float>()) dst.pos_ang = src["pos_ang"].as<float>();
    if (src["knym_v"].is<float>()) dst.knym_v = src["knym_v"].as<float>();
    if (src["knym_w"].is<float>()) dst.knym_w = src["knym_w"].as<float>();
    if (src["odm_x"].is<float>()) dst.odm_x = src["odm_x"].as<float>();
    if (src["odm_y"].is<float>()) dst.odm_y = src["odm_y"].as<float>();
    if (src["odm_theta"].is<float>()) dst.odm_theta = src["odm_theta"].as<float>();
    if (src["kim_x"].is<float>()) dst.kim_x = src["kim_x"].as<float>();
    if (src["kim_y"].is<float>()) dst.kim_y = src["kim_y"].as<float>();
    if (src["kim_theta"].is<float>()) dst.kim_theta = src["kim_theta"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, kinematics_t& dst) {
    if (src["x"].is<float>()) dst.x = src["x"].as<float>();
    if (src["y"].is<float>()) dst.y = src["y"].as<float>();
    if (src["theta"].is<float>()) dst.theta = src["theta"].as<float>();
    if (src["v"].is<float>()) dst.v = src["v"].as<float>();
    if (src["w"].is<float>()) dst.w = src["w"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_log_t& dst) {
    if (src["sensor_dist"].is<float>()) dst.sensor_dist = src["sensor_dist"].as<float>();
    if (src["global_run_dist"].is<float>()) dst.global_run_dist = src["global_run_dist"].as<float>();
    if (src["angle"].is<float>()) dst.angle = src["angle"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_log2_t& dst) {
    if (src["r45_dist"].is<float>()) dst.r45_dist = src["r45_dist"].as<float>();
    if (src["l45_dist"].is<float>()) dst.l45_dist = src["l45_dist"].as<float>();
    if (src["global_run_dist"].is<float>()) dst.global_run_dist = src["global_run_dist"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_logs_t& dst) {
    if (!src["l90"].isNull()) convertFromJson(src["l90"], dst.l90);
    if (!src["l45"].isNull()) convertFromJson(src["l45"], dst.l45);
    if (!src["l45_2"].isNull()) convertFromJson(src["l45_2"], dst.l45_2);
    if (!src["l45_3"].isNull()) convertFromJson(src["l45_3"], dst.l45_3);
    if (!src["r45"].isNull()) convertFromJson(src["r45"], dst.r45);
    if (!src["r45_2"].isNull()) convertFromJson(src["r45_2"], dst.r45_2);
    if (!src["r45_3"].isNull()) convertFromJson(src["r45_3"], dst.r45_3);
    if (!src["r90"].isNull()) convertFromJson(src["r90"], dst.r90);
}

inline void convertFromJson(JsonVariantConst src, sen_dist_log_t& dst) {
    if (src["list"].is<JsonArrayConst>()) {
        dst.list.clear();
        for (auto v : src["list"].as<JsonArrayConst>()) {
            sen_log2_t temp;
            convertFromJson(v, temp);
            dst.list.push_back(temp);
        }
    }
}

inline void convertFromJson(JsonVariantConst src, sensing_result_entity_t& dst) {
    if (!src["led_sen"].isNull()) convertFromJson(src["led_sen"], dst.led_sen);
    if (!src["led_sen_after"].isNull()) convertFromJson(src["led_sen_after"], dst.led_sen_after);
    if (!src["led_sen_before"].isNull()) convertFromJson(src["led_sen_before"], dst.led_sen_before);
    if (!src["gyro"].isNull()) convertFromJson(src["gyro"], dst.gyro);
    if (!src["gyro2"].isNull()) convertFromJson(src["gyro2"], dst.gyro2);
    from_json_nested(src, "accel_x",   dst.accel_x);
    from_json_nested(src, "accel_y",   dst.accel_y);
    from_json_array(src,  "gyro_list", dst.gyro_list);  // int[5] 固定長配列
    from_json_vector(src, "enc_r_list",dst.enc_r_list);
    from_json_vector(src, "enc_l_list",dst.enc_l_list);
    from_json_nested(src, "battery",   dst.battery);
    if (!src["encoder_raw"].isNull()) convertFromJson(src["encoder_raw"], dst.encoder_raw);
    if (!src["encoder"].isNull()) convertFromJson(src["encoder"], dst.encoder);
    if (!src["ego"].isNull()) convertFromJson(src["ego"], dst.ego);
    if (!src["sen"].isNull()) convertFromJson(src["sen"], dst.sen);
    if (!src["sen_dist_log"].isNull()) convertFromJson(src["sen_dist_log"], dst.sen_dist_log);
    if (src["calc_time"].is<int16_t>()) dst.calc_time = src["calc_time"].as<int16_t>();
    if (src["calc_time2"].is<int16_t>()) dst.calc_time2 = src["calc_time2"].as<int16_t>();
    if (src["sensing_timestamp"].is<int64_t>()) dst.sensing_timestamp = src["sensing_timestamp"].as<int64_t>();
    if (src["ang_kf_sum"].is<float>()) dst.ang_kf_sum = src["ang_kf_sum"].as<float>();
    if (src["img_ang_sum"].is<float>()) dst.img_ang_sum = src["img_ang_sum"].as<float>();
    if (src["img_ang_z"].is<float>()) dst.img_ang_z = src["img_ang_z"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, xva_t& dst) {
    if (src["vel"].is<float>()) dst.vel = src["vel"].as<float>();
    if (src["speed"].is<float>()) dst.speed = src["speed"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, pid_param_t& dst) {
    if (src["p"].is<float>()) dst.p = src["p"].as<float>();
    if (src["i"].is<float>()) dst.i = src["i"].as<float>();
    if (src["d"].is<float>()) dst.d = src["d"].as<float>();
    if (src["b"].is<float>()) dst.b = src["b"].as<float>();
    if (src["c"].is<float>()) dst.c = src["c"].as<float>();
    if (src["mode"].is<int8_t>()) dst.mode = src["mode"].as<int8_t>();
    if (src["antiwindup"].is<int8_t>()) dst.antiwindup = src["antiwindup"].as<int8_t>();
    if (src["windup_gain"].is<float>()) dst.windup_gain = src["windup_gain"].as<float>();
    if (src["windup_dead_bind"].is<float>()) dst.windup_dead_bind = src["windup_dead_bind"].as<float>();
    if (src["i_theta_tau"].is<float>()) dst.i_theta_tau = src["i_theta_tau"].as<float>();
    if (src["theta_gate"].is<float>()) dst.theta_gate = src["theta_gate"].as<float>();
    if (src["omega_gate"].is<float>()) dst.omega_gate = src["omega_gate"].as<float>();
    if (src["i_theta_slew"].is<float>()) dst.i_theta_slew = src["i_theta_slew"].as<float>();
    if (src["i_theta_max"].is<float>()) dst.i_theta_max = src["i_theta_max"].as<float>();
    if (src["alpha_stop"].is<float>()) dst.alpha_stop = src["alpha_stop"].as<float>();
    if (src["alpha_rate"].is<float>()) dst.alpha_rate = src["alpha_rate"].as<float>();
    if (src["theta_damp_th"].is<float>()) dst.theta_damp_th = src["theta_damp_th"].as<float>();
    if (src["omega_damp"].is<float>()) dst.omega_damp = src["omega_damp"].as<float>();
    if (src["th"].is<float>()) dst.th = src["th"].as<float>();
    if (src["theta_gate_on"].is<float>()) dst.theta_gate_on = src["theta_gate_on"].as<float>();
    if (src["theta_gate_full"].is<float>()) dst.theta_gate_full = src["theta_gate_full"].as<float>();
    if (src["theta_kp"].is<float>()) dst.theta_kp = src["theta_kp"].as<float>();
    if (src["theta_kd"].is<float>()) dst.theta_kd = src["theta_kd"].as<float>();
    if (src["omega_add_max"].is<float>()) dst.omega_add_max = src["omega_add_max"].as<float>();
    if (src["alpha_rate_end"].is<float>()) dst.alpha_rate_end = src["alpha_rate_end"].as<float>();
    if (src["k_stop"].is<float>()) dst.k_stop = src["k_stop"].as<float>();
    if (src["theta_eps"].is<float>()) dst.theta_eps = src["theta_eps"].as<float>();
    if (src["s_gate"].is<float>()) dst.s_gate = src["s_gate"].as<float>();
    if (src["mpc_q_ang"].is<float>()) dst.mpc_q_ang = src["mpc_q_ang"].as<float>();
    if (src["mpc_q_vel"].is<float>()) dst.mpc_q_vel = src["mpc_q_vel"].as<float>();
    if (src["mpc_b"].is<float>()) dst.mpc_b = src["mpc_b"].as<float>();
    if (src["mpc_r"].is<float>()) dst.mpc_r = src["mpc_r"].as<float>();
    if (src["mpc_horizon"].is<int>()) dst.mpc_horizon = src["mpc_horizon"].as<int>();
    if (src["mpc_max_iter"].is<int>()) dst.mpc_max_iter = src["mpc_max_iter"].as<int>();
    if (src["mpc_max_torque"].is<float>()) dst.mpc_max_torque = src["mpc_max_torque"].as<float>();
    if (src["mpc_observer_k"].is<float>()) dst.mpc_observer_k = src["mpc_observer_k"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, gyro_param_t& dst) {
    if (src["gyro_w_gain_right"].is<float>()) dst.gyro_w_gain_right = src["gyro_w_gain_right"].as<float>();
    if (src["gyro_w_gain_left"].is<float>()) dst.gyro_w_gain_left = src["gyro_w_gain_left"].as<float>();
    if (src["retry_min_th"].is<float>()) dst.retry_min_th = src["retry_min_th"].as<float>();
    if (src["retry_max_th"].is<float>()) dst.retry_max_th = src["retry_max_th"].as<float>();
    if (src["robust_th"].is<float>()) dst.robust_th = src["robust_th"].as<float>();
    if (src["lp_delay"].is<float>()) dst.lp_delay = src["lp_delay"].as<float>();
    if (src["list_size"].is<int>()) dst.list_size = src["list_size"].as<int>();
    if (src["loop_size"].is<int>()) dst.loop_size = src["loop_size"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, accel_param_t& dst) {
    if (src["gain"].is<float>()) dst.gain = src["gain"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_param_t& dst) {
    if (src["lp_delay"].is<float>()) dst.lp_delay = src["lp_delay"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_ref_param3_t& dst) {
    if (src["right45"].is<float>()) dst.right45 = src["right45"].as<float>();
    if (src["left45"].is<float>()) dst.left45 = src["left45"].as<float>();
    if (src["right90"].is<float>()) dst.right90 = src["right90"].as<float>();
    if (src["left90"].is<float>()) dst.left90 = src["left90"].as<float>();
    if (src["front"].is<float>()) dst.front = src["front"].as<float>();
    if (src["kireme_r"].is<float>()) dst.kireme_r = src["kireme_r"].as<float>();
    if (src["kireme_l"].is<float>()) dst.kireme_l = src["kireme_l"].as<float>();
    if (src["kireme_r_fast"].is<float>()) dst.kireme_r_fast = src["kireme_r_fast"].as<float>();
    if (src["kireme_l_fast"].is<float>()) dst.kireme_l_fast = src["kireme_l_fast"].as<float>();
    if (src["kireme_r_wall_off"].is<float>()) dst.kireme_r_wall_off = src["kireme_r_wall_off"].as<float>();
    if (src["kireme_l_wall_off"].is<float>()) dst.kireme_l_wall_off = src["kireme_l_wall_off"].as<float>();
    if (src["kireme_r_wall_off2"].is<float>()) dst.kireme_r_wall_off2 = src["kireme_r_wall_off2"].as<float>();
    if (src["kireme_l_wall_off2"].is<float>()) dst.kireme_l_wall_off2 = src["kireme_l_wall_off2"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_search_param_t& dst) {
    if (src["front"].is<float>()) dst.front = src["front"].as<float>();
    if (src["right45"].is<float>()) dst.right45 = src["right45"].as<float>();
    if (src["left45"].is<float>()) dst.left45 = src["left45"].as<float>();
    if (src["right90"].is<float>()) dst.right90 = src["right90"].as<float>();
    if (src["left90"].is<float>()) dst.left90 = src["left90"].as<float>();
    if (src["kireme_r"].is<float>()) dst.kireme_r = src["kireme_r"].as<float>();
    if (src["kireme_l"].is<float>()) dst.kireme_l = src["kireme_l"].as<float>();
    if (src["offset_r"].is<float>()) dst.offset_r = src["offset_r"].as<float>();
    if (src["offset_l"].is<float>()) dst.offset_l = src["offset_l"].as<float>();
    if (src["front_ctrl"].is<float>()) dst.front_ctrl = src["front_ctrl"].as<float>();
    if (src["front_ctrl_th"].is<float>()) dst.front_ctrl_th = src["front_ctrl_th"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_expand_param_t& dst) {
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["right45"].is<float>()) dst.right45 = src["right45"].as<float>();
    if (src["left45"].is<float>()) dst.left45 = src["left45"].as<float>();
    if (src["right45_2"].is<float>()) dst.right45_2 = src["right45_2"].as<float>();
    if (src["left45_2"].is<float>()) dst.left45_2 = src["left45_2"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sen_ref_param2_t& dst) {
    if (!src["ref"].isNull()) convertFromJson(src["ref"], dst.ref);
    if (!src["ref_search"].isNull()) convertFromJson(src["ref_search"], dst.ref_search);
    if (!src["exist"].isNull()) convertFromJson(src["exist"], dst.exist);
    if (!src["expand"].isNull()) convertFromJson(src["expand"], dst.expand);
}

inline void convertFromJson(JsonVariantConst src, sen_ref_param_t& dst) {
    if (!src["normal"].isNull()) convertFromJson(src["normal"], dst.normal);
    if (!src["normal2"].isNull()) convertFromJson(src["normal2"], dst.normal2);
    if (!src["dia"].isNull()) convertFromJson(src["dia"], dst.dia);
    if (!src["search_exist"].isNull()) convertFromJson(src["search_exist"], dst.search_exist);
    if (!src["search_ref"].isNull()) convertFromJson(src["search_ref"], dst.search_ref);
}

inline void convertFromJson(JsonVariantConst src, sensor_gain_param_t& dst) {
    if (src["a"].is<float>()) dst.a = src["a"].as<float>();
    if (src["b"].is<float>()) dst.b = src["b"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sensor_gain_t& dst) {
    if (!src["l90"].isNull()) convertFromJson(src["l90"], dst.l90);
    if (!src["l45"].isNull()) convertFromJson(src["l45"], dst.l45);
    if (!src["front"].isNull()) convertFromJson(src["front"], dst.front);
    if (!src["front2"].isNull()) convertFromJson(src["front2"], dst.front2);
    if (!src["front3"].isNull()) convertFromJson(src["front3"], dst.front3);
    if (!src["front4"].isNull()) convertFromJson(src["front4"], dst.front4);
    if (!src["front_ctrl_th"].isNull()) convertFromJson(src["front_ctrl_th"], dst.front_ctrl_th);
    if (!src["r45"].isNull()) convertFromJson(src["r45"], dst.r45);
    if (!src["l45_2"].isNull()) convertFromJson(src["l45_2"], dst.l45_2);
    if (!src["r45_2"].isNull()) convertFromJson(src["r45_2"], dst.r45_2);
    if (!src["l45_3"].isNull()) convertFromJson(src["l45_3"], dst.l45_3);
    if (!src["r45_3"].isNull()) convertFromJson(src["r45_3"], dst.r45_3);
    if (!src["r90"].isNull()) convertFromJson(src["r90"], dst.r90);
    if (!src["l90_far"].isNull()) convertFromJson(src["l90_far"], dst.l90_far);
    if (!src["r90_far"].isNull()) convertFromJson(src["r90_far"], dst.r90_far);
    if (!src["l90_mid"].isNull()) convertFromJson(src["l90_mid"], dst.l90_mid);
    if (!src["r90_mid"].isNull()) convertFromJson(src["r90_mid"], dst.r90_mid);
}

inline void convertFromJson(JsonVariantConst src, wall_off_hold_dist_t& dst) {
    if (src["left_str"].is<float>()) dst.left_str = src["left_str"].as<float>();
    if (src["right_str"].is<float>()) dst.right_str = src["right_str"].as<float>();
    if (src["left_diff_th"].is<float>()) dst.left_diff_th = src["left_diff_th"].as<float>();
    if (src["right_diff_th"].is<float>()) dst.right_diff_th = src["right_diff_th"].as<float>();
    if (src["left_str_exist"].is<float>()) dst.left_str_exist = src["left_str_exist"].as<float>();
    if (src["right_str_exist"].is<float>()) dst.right_str_exist = src["right_str_exist"].as<float>();
    if (src["left_dia"].is<float>()) dst.left_dia = src["left_dia"].as<float>();
    if (src["right_dia"].is<float>()) dst.right_dia = src["right_dia"].as<float>();
    if (src["left_dia_noexit"].is<float>()) dst.left_dia_noexit = src["left_dia_noexit"].as<float>();
    if (src["right_dia_noexit"].is<float>()) dst.right_dia_noexit = src["right_dia_noexit"].as<float>();
    if (src["left_dia_oppo"].is<float>()) dst.left_dia_oppo = src["left_dia_oppo"].as<float>();
    if (src["right_dia_oppo"].is<float>()) dst.right_dia_oppo = src["right_dia_oppo"].as<float>();
    if (src["left_dia2"].is<float>()) dst.left_dia2 = src["left_dia2"].as<float>();
    if (src["right_dia2"].is<float>()) dst.right_dia2 = src["right_dia2"].as<float>();
    if (src["exist_dist_l"].is<float>()) dst.exist_dist_l = src["exist_dist_l"].as<float>();
    if (src["exist_dist_r"].is<float>()) dst.exist_dist_r = src["exist_dist_r"].as<float>();
    if (src["exist_dist_l2"].is<float>()) dst.exist_dist_l2 = src["exist_dist_l2"].as<float>();
    if (src["exist_dist_r2"].is<float>()) dst.exist_dist_r2 = src["exist_dist_r2"].as<float>();
    if (src["noexist_th_l"].is<float>()) dst.noexist_th_l = src["noexist_th_l"].as<float>();
    if (src["noexist_th_r"].is<float>()) dst.noexist_th_r = src["noexist_th_r"].as<float>();
    if (src["noexist_th_l2"].is<float>()) dst.noexist_th_l2 = src["noexist_th_l2"].as<float>();
    if (src["noexist_th_r2"].is<float>()) dst.noexist_th_r2 = src["noexist_th_r2"].as<float>();
    if (src["div_th_l"].is<float>()) dst.div_th_l = src["div_th_l"].as<float>();
    if (src["div_th_r"].is<float>()) dst.div_th_r = src["div_th_r"].as<float>();
    if (src["div_th_l2"].is<float>()) dst.div_th_l2 = src["div_th_l2"].as<float>();
    if (src["div_th_r2"].is<float>()) dst.div_th_r2 = src["div_th_r2"].as<float>();
    if (src["div_th_l3"].is<float>()) dst.div_th_l3 = src["div_th_l3"].as<float>();
    if (src["div_th_r3"].is<float>()) dst.div_th_r3 = src["div_th_r3"].as<float>();
    if (src["div_th_dia_l"].is<float>()) dst.div_th_dia_l = src["div_th_dia_l"].as<float>();
    if (src["div_th_dia_r"].is<float>()) dst.div_th_dia_r = src["div_th_dia_r"].as<float>();
    if (src["exist_dia_th_l"].is<float>()) dst.exist_dia_th_l = src["exist_dia_th_l"].as<float>();
    if (src["exist_dia_th_r"].is<float>()) dst.exist_dia_th_r = src["exist_dia_th_r"].as<float>();
    if (src["exist_dia_th_l2"].is<float>()) dst.exist_dia_th_l2 = src["exist_dia_th_l2"].as<float>();
    if (src["exist_dia_th_r2"].is<float>()) dst.exist_dia_th_r2 = src["exist_dia_th_r2"].as<float>();
    if (src["noexist_dia_th_l"].is<float>()) dst.noexist_dia_th_l = src["noexist_dia_th_l"].as<float>();
    if (src["noexist_dia_th_r"].is<float>()) dst.noexist_dia_th_r = src["noexist_dia_th_r"].as<float>();
    if (src["noexist_dia_th_l2"].is<float>()) dst.noexist_dia_th_l2 = src["noexist_dia_th_l2"].as<float>();
    if (src["noexist_dia_th_r2"].is<float>()) dst.noexist_dia_th_r2 = src["noexist_dia_th_r2"].as<float>();
    if (src["wall_off_exist_wall_th_l"].is<float>()) dst.wall_off_exist_wall_th_l = src["wall_off_exist_wall_th_l"].as<float>();
    if (src["wall_off_exist_wall_th_r"].is<float>()) dst.wall_off_exist_wall_th_r = src["wall_off_exist_wall_th_r"].as<float>();
    if (src["wall_off_exist_dia_wall_th_l"].is<float>()) dst.wall_off_exist_dia_wall_th_l = src["wall_off_exist_dia_wall_th_l"].as<float>();
    if (src["wall_off_exist_dia_wall_th_r"].is<float>()) dst.wall_off_exist_dia_wall_th_r = src["wall_off_exist_dia_wall_th_r"].as<float>();
    if (src["search_wall_off_enable"].is<bool>()) dst.search_wall_off_enable = src["search_wall_off_enable"].as<bool>();
    if (src["search_wall_off_l_dist_offset"].is<float>()) dst.search_wall_off_l_dist_offset = src["search_wall_off_l_dist_offset"].as<float>();
    if (src["search_wall_off_r_dist_offset"].is<float>()) dst.search_wall_off_r_dist_offset = src["search_wall_off_r_dist_offset"].as<float>();
    if (src["search_wall_off_offset_dist"].is<float>()) dst.search_wall_off_offset_dist = src["search_wall_off_offset_dist"].as<float>();
    if (src["ctrl_exist_wall_th_l"].is<float>()) dst.ctrl_exist_wall_th_l = src["ctrl_exist_wall_th_l"].as<float>();
    if (src["ctrl_exist_wall_th_r"].is<float>()) dst.ctrl_exist_wall_th_r = src["ctrl_exist_wall_th_r"].as<float>();
    if (src["go_straight_wide_ctrl_th"].is<float>()) dst.go_straight_wide_ctrl_th = src["go_straight_wide_ctrl_th"].as<float>();
    if (src["diff_check_dist"].is<float>()) dst.diff_check_dist = src["diff_check_dist"].as<float>();
    if (src["diff_dist_th_l"].is<float>()) dst.diff_dist_th_l = src["diff_dist_th_l"].as<float>();
    if (src["diff_dist_th_r"].is<float>()) dst.diff_dist_th_r = src["diff_dist_th_r"].as<float>();
    if (src["diff_check_dist_dia"].is<float>()) dst.diff_check_dist_dia = src["diff_check_dist_dia"].as<float>();
    if (src["diff_check_dist_dia_2"].is<float>()) dst.diff_check_dist_dia_2 = src["diff_check_dist_dia_2"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, fail_check_cnt_t& dst) {
    if (src["duty"].is<int>()) dst.duty = src["duty"].as<int>();
    if (src["v"].is<int>()) dst.v = src["v"].as<int>();
    if (src["w"].is<int>()) dst.w = src["w"].as<int>();
    if (src["ang"].is<int>()) dst.ang = src["ang"].as<int>();
    if (src["wall_off"].is<int>()) dst.wall_off = src["wall_off"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, comp_param_t& dst) {
    if (src["v_lp_gain"].is<float>()) dst.v_lp_gain = src["v_lp_gain"].as<float>();
    if (src["accl_x_hp_gain"].is<float>()) dst.accl_x_hp_gain = src["accl_x_hp_gain"].as<float>();
    if (src["gain"].is<float>()) dst.gain = src["gain"].as<float>();
    if (src["enable"].is<int>()) dst.enable = src["enable"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, kanayama_t& dst) {
    if (src["kx"].is<float>()) dst.kx = src["kx"].as<float>();
    if (src["ky"].is<float>()) dst.ky = src["ky"].as<float>();
    if (src["k_theta"].is<float>()) dst.k_theta = src["k_theta"].as<float>();
    if (src["enable"].is<int8_t>()) dst.enable = src["enable"].as<int8_t>();
    if (src["windup"].is<int8_t>()) dst.windup = src["windup"].as<int8_t>();
    if (src["windup_deg"].is<float>()) dst.windup_deg = src["windup_deg"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, input_param_t& dst) {
    if (src["dt"].is<float>()) dst.dt = src["dt"].as<float>();
    if (src["trj_length"].is<int>()) dst.trj_length = src["trj_length"].as<int>();
    if (src["tire"].is<float>()) dst.tire = src["tire"].as<float>();
    if (src["tire2"].is<float>()) dst.tire2 = src["tire2"].as<float>();
    if (src["log_size"].is<int>()) dst.log_size = src["log_size"].as<int>();
    if (src["gear_a"].is<float>()) dst.gear_a = src["gear_a"].as<float>();
    if (src["gear_b"].is<float>()) dst.gear_b = src["gear_b"].as<float>();
    if (src["max_duty"].is<float>()) dst.max_duty = src["max_duty"].as<float>();
    if (src["min_duty"].is<float>()) dst.min_duty = src["min_duty"].as<float>();
    if (src["battery_gain"].is<float>()) dst.battery_gain = src["battery_gain"].as<float>();
    if (src["Ke"].is<float>()) dst.Ke = src["Ke"].as<float>();
    if (src["Km"].is<float>()) dst.Km = src["Km"].as<float>();
    if (src["Resist"].is<float>()) dst.Resist = src["Resist"].as<float>();
    if (src["Mass"].is<float>()) dst.Mass = src["Mass"].as<float>();
    if (src["Lm"].is<float>()) dst.Lm = src["Lm"].as<float>();
    if (src["coulomb_friction"].is<float>()) dst.coulomb_friction = src["coulomb_friction"].as<float>();
    if (src["viscous_friction"].is<float>()) dst.viscous_friction = src["viscous_friction"].as<float>();
    if (src["battery_init_cov"].is<float>()) dst.battery_init_cov = src["battery_init_cov"].as<float>();
    if (src["battery_p_noise"].is<float>()) dst.battery_p_noise = src["battery_p_noise"].as<float>();
    if (src["battery_m_noise"].is<float>()) dst.battery_m_noise = src["battery_m_noise"].as<float>();
    if (src["encoder_init_cov"].is<float>()) dst.encoder_init_cov = src["encoder_init_cov"].as<float>();
    if (src["encoder_p_noise"].is<float>()) dst.encoder_p_noise = src["encoder_p_noise"].as<float>();
    if (src["encoder_m_noise"].is<float>()) dst.encoder_m_noise = src["encoder_m_noise"].as<float>();
    if (src["w_init_cov"].is<float>()) dst.w_init_cov = src["w_init_cov"].as<float>();
    if (src["w_p_noise"].is<float>()) dst.w_p_noise = src["w_p_noise"].as<float>();
    if (src["w_m_noise"].is<float>()) dst.w_m_noise = src["w_m_noise"].as<float>();
    if (src["v_init_cov"].is<float>()) dst.v_init_cov = src["v_init_cov"].as<float>();
    if (src["v_p_noise"].is<float>()) dst.v_p_noise = src["v_p_noise"].as<float>();
    if (src["v_m_noise"].is<float>()) dst.v_m_noise = src["v_m_noise"].as<float>();
    if (src["ang_init_cov"].is<float>()) dst.ang_init_cov = src["ang_init_cov"].as<float>();
    if (src["ang_p_noise"].is<float>()) dst.ang_p_noise = src["ang_p_noise"].as<float>();
    if (src["ang_m_noise"].is<float>()) dst.ang_m_noise = src["ang_m_noise"].as<float>();
    if (src["dist_init_cov"].is<float>()) dst.dist_init_cov = src["dist_init_cov"].as<float>();
    if (src["dist_p_noise"].is<float>()) dst.dist_p_noise = src["dist_p_noise"].as<float>();
    if (src["dist_m_noise"].is<float>()) dst.dist_m_noise = src["dist_m_noise"].as<float>();
    if (src["pos_init_cov"].is<float>()) dst.pos_init_cov = src["pos_init_cov"].as<float>();
    if (src["pos_p_noise"].is<float>()) dst.pos_p_noise = src["pos_p_noise"].as<float>();
    if (src["pos_m_noise"].is<float>()) dst.pos_m_noise = src["pos_m_noise"].as<float>();
    if (src["tread"].is<float>()) dst.tread = src["tread"].as<float>();
    if (src["FF_front"].is<int>()) dst.FF_front = src["FF_front"].as<int>();
    if (src["FF_roll"].is<int>()) dst.FF_roll = src["FF_roll"].as<int>();
    if (src["FF_keV"].is<int>()) dst.FF_keV = src["FF_keV"].as<int>();
    if (src["offset_start_dist"].is<float>()) dst.offset_start_dist = src["offset_start_dist"].as<float>();
    if (src["offset_start_dist_search"].is<float>()) dst.offset_start_dist_search = src["offset_start_dist_search"].as<float>();
    if (src["long_run_offset_dist"].is<float>()) dst.long_run_offset_dist = src["long_run_offset_dist"].as<float>();
    if (src["pivot_back_offset"].is<float>()) dst.pivot_back_offset = src["pivot_back_offset"].as<float>();
    if (src["cell"].is<float>()) dst.cell = src["cell"].as<float>();
    if (src["cell2"].is<float>()) dst.cell2 = src["cell2"].as<float>();
    if (src["pivot_angle_180"].is<float>()) dst.pivot_angle_180 = src["pivot_angle_180"].as<float>();
    if (src["pivot_angle_90"].is<float>()) dst.pivot_angle_90 = src["pivot_angle_90"].as<float>();
    if (src["wall_off_front_move_dist_th"].is<float>()) dst.wall_off_front_move_dist_th = src["wall_off_front_move_dist_th"].as<float>();
    if (src["wall_off_front_move_dia_dist_th"].is<float>()) dst.wall_off_front_move_dia_dist_th = src["wall_off_front_move_dia_dist_th"].as<float>();
    if (src["ff_front_gain_14"].is<float>()) dst.ff_front_gain_14 = src["ff_front_gain_14"].as<float>();
    if (src["ff_roll_gain_before"].is<float>()) dst.ff_roll_gain_before = src["ff_roll_gain_before"].as<float>();
    if (src["ff_roll_gain_after"].is<float>()) dst.ff_roll_gain_after = src["ff_roll_gain_after"].as<float>();
    if (src["ff_front_gain_decel"].is<float>()) dst.ff_front_gain_decel = src["ff_front_gain_decel"].as<float>();
    if (!src["front_ctrl_roll_pid"].isNull()) convertFromJson(src["front_ctrl_roll_pid"], dst.front_ctrl_roll_pid);
    if (!src["motor_pid"].isNull()) convertFromJson(src["motor_pid"], dst.motor_pid);
    if (!src["motor_pid_gain_limitter"].isNull()) convertFromJson(src["motor_pid_gain_limitter"], dst.motor_pid_gain_limitter);
    if (!src["motor_pid2"].isNull()) convertFromJson(src["motor_pid2"], dst.motor_pid2);
    if (!src["motor2_pid_gain_limitter"].isNull()) convertFromJson(src["motor2_pid_gain_limitter"], dst.motor2_pid_gain_limitter);
    if (!src["motor_pid3"].isNull()) convertFromJson(src["motor_pid3"], dst.motor_pid3);
    if (!src["gyro_pid"].isNull()) convertFromJson(src["gyro_pid"], dst.gyro_pid);
    if (!src["gyro_pid_gain_limitter"].isNull()) convertFromJson(src["gyro_pid_gain_limitter"], dst.gyro_pid_gain_limitter);
    if (!src["str_ang_pid"].isNull()) convertFromJson(src["str_ang_pid"], dst.str_ang_pid);
    if (!src["str_ang_dia_pid"].isNull()) convertFromJson(src["str_ang_dia_pid"], dst.str_ang_dia_pid);
    if (!src["angle_pid"].isNull()) convertFromJson(src["angle_pid"], dst.angle_pid);
    if (!src["front_ctrl_angle_pid"].isNull()) convertFromJson(src["front_ctrl_angle_pid"], dst.front_ctrl_angle_pid);
    if (!src["front_ctrl_dist_pid"].isNull()) convertFromJson(src["front_ctrl_dist_pid"], dst.front_ctrl_dist_pid);
    if (!src["front_ctrl_keep_angle_pid"].isNull()) convertFromJson(src["front_ctrl_keep_angle_pid"], dst.front_ctrl_keep_angle_pid);
    if (!src["sensor_pid_dia"].isNull()) convertFromJson(src["sensor_pid_dia"], dst.sensor_pid_dia);
    if (!src["gyro_param"].isNull()) convertFromJson(src["gyro_param"], dst.gyro_param);
    if (!src["gyro2_param"].isNull()) convertFromJson(src["gyro2_param"], dst.gyro2_param);
    if (!src["accel_x_param"].isNull()) convertFromJson(src["accel_x_param"], dst.accel_x_param);
    if (!src["comp_param"].isNull()) convertFromJson(src["comp_param"], dst.comp_param);
    if (!src["battery_param"].isNull()) convertFromJson(src["battery_param"], dst.battery_param);
    if (!src["led_param"].isNull()) convertFromJson(src["led_param"], dst.led_param);
    if (src["motion_dir"].is<int>()) dst.motion_dir = (MotionDirection)src["motion_dir"].as<int>();
    if (!src["sen_ref_p"].isNull()) convertFromJson(src["sen_ref_p"], dst.sen_ref_p);
    if (!src["sensor_gain"].isNull()) convertFromJson(src["sensor_gain"], dst.sensor_gain);
    if (src["sakiyomi_time"].is<float>()) dst.sakiyomi_time = src["sakiyomi_time"].as<float>();
    if (src["search_sen_ctrl_limitter"].is<float>()) dst.search_sen_ctrl_limitter = src["search_sen_ctrl_limitter"].as<float>();
    if (src["clear_angle"].is<float>()) dst.clear_angle = src["clear_angle"].as<float>();
    if (src["clear_dist_order"].is<float>()) dst.clear_dist_order = src["clear_dist_order"].as<float>();
    if (src["front_dist_offset"].is<float>()) dst.front_dist_offset = src["front_dist_offset"].as<float>();
    if (src["front_dist_offset0"].is<float>()) dst.front_dist_offset0 = src["front_dist_offset0"].as<float>();
    if (src["front_dist_offset2"].is<float>()) dst.front_dist_offset2 = src["front_dist_offset2"].as<float>();
    if (src["front_dist_offset3"].is<float>()) dst.front_dist_offset3 = src["front_dist_offset3"].as<float>();
    if (src["front_dist_offset4"].is<float>()) dst.front_dist_offset4 = src["front_dist_offset4"].as<float>();
    if (src["front_dist_offset_dia_front"].is<float>()) dst.front_dist_offset_dia_front = src["front_dist_offset_dia_front"].as<float>();
    if (src["front_dist_offset_dia_45_th"].is<float>()) dst.front_dist_offset_dia_45_th = src["front_dist_offset_dia_45_th"].as<float>();
    if (src["front_dist_offset_dia_right45"].is<float>()) dst.front_dist_offset_dia_right45 = src["front_dist_offset_dia_right45"].as<float>();
    if (src["front_dist_offset_dia_left45"].is<float>()) dst.front_dist_offset_dia_left45 = src["front_dist_offset_dia_left45"].as<float>();
    if (src["sla_wall_ref_l"].is<float>()) dst.sla_wall_ref_l = src["sla_wall_ref_l"].as<float>();
    if (src["sla_wall_ref_r"].is<float>()) dst.sla_wall_ref_r = src["sla_wall_ref_r"].as<float>();
    if (src["sla_max_offset_dist"].is<float>()) dst.sla_max_offset_dist = src["sla_max_offset_dist"].as<float>();
    if (src["large_offset_enable"].is<bool>()) dst.large_offset_enable = src["large_offset_enable"].as<bool>();
    if (src["dia45_offset_enable"].is<bool>()) dst.dia45_offset_enable = src["dia45_offset_enable"].as<bool>();
    if (src["dia135_offset_enable"].is<bool>()) dst.dia135_offset_enable = src["dia135_offset_enable"].as<bool>();
    if (src["orval_offset_enable"].is<bool>()) dst.orval_offset_enable = src["orval_offset_enable"].as<bool>();
    if (src["large_offset_max_dist"].is<float>()) dst.large_offset_max_dist = src["large_offset_max_dist"].as<float>();
    if (src["dia45_offset_max_dist"].is<float>()) dst.dia45_offset_max_dist = src["dia45_offset_max_dist"].as<float>();
    if (src["dia135_offset_max_dist"].is<float>()) dst.dia135_offset_max_dist = src["dia135_offset_max_dist"].as<float>();
    if (src["orval_offset_max_dist"].is<float>()) dst.orval_offset_max_dist = src["orval_offset_max_dist"].as<float>();
    if (src["dia45_2_offset_max_dist"].is<float>()) dst.dia45_2_offset_max_dist = src["dia45_2_offset_max_dist"].as<float>();
    if (src["dia135_2_offset_max_dist"].is<float>()) dst.dia135_2_offset_max_dist = src["dia135_2_offset_max_dist"].as<float>();
    if (src["dia90_offset_max_dist"].is<float>()) dst.dia90_offset_max_dist = src["dia90_offset_max_dist"].as<float>();
    if (src["lim_angle"].is<float>()) dst.lim_angle = src["lim_angle"].as<float>();
    if (src["front_ctrl_error_th"].is<float>()) dst.front_ctrl_error_th = src["front_ctrl_error_th"].as<float>();
    if (src["clear_dist_ragne_from"].is<float>()) dst.clear_dist_ragne_from = src["clear_dist_ragne_from"].as<float>();
    if (src["clear_dist_ragne_to"].is<float>()) dst.clear_dist_ragne_to = src["clear_dist_ragne_to"].as<float>();
    if (src["clear_dist_ragne_to2"].is<float>()) dst.clear_dist_ragne_to2 = src["clear_dist_ragne_to2"].as<float>();
    if (src["clear_dist_ragne_dist_list"].is<JsonArrayConst>()) {
        dst.clear_dist_ragne_dist_list.clear();
        for (auto v : src["clear_dist_ragne_dist_list"].as<JsonArrayConst>()) {
            dst.clear_dist_ragne_dist_list.push_back(v.as<float>());
        }
    }
    if (src["clear_dist_ragne_th_list"].is<JsonArrayConst>()) {
        dst.clear_dist_ragne_th_list.clear();
        for (auto v : src["clear_dist_ragne_th_list"].as<JsonArrayConst>()) {
            dst.clear_dist_ragne_th_list.push_back(v.as<float>());
        }
    }
    if (src["clear_dist_ragne_dist_list_fast"].is<JsonArrayConst>()) {
        dst.clear_dist_ragne_dist_list_fast.clear();
        for (auto v : src["clear_dist_ragne_dist_list_fast"].as<JsonArrayConst>()) {
            dst.clear_dist_ragne_dist_list_fast.push_back(v.as<float>());
        }
    }
    if (src["wall_off_hold_dist"].is<float>()) dst.wall_off_hold_dist = src["wall_off_hold_dist"].as<float>();
    if (!src["wall_off_dist"].isNull()) convertFromJson(src["wall_off_dist"], dst.wall_off_dist);
    if (src["wall_off_diff_ref_th"].is<float>()) dst.wall_off_diff_ref_th = src["wall_off_diff_ref_th"].as<float>();
    if (src["wall_off_diff_ref_front_th"].is<float>()) dst.wall_off_diff_ref_front_th = src["wall_off_diff_ref_front_th"].as<float>();
    if (src["wall_off_wait_dist"].is<float>()) dst.wall_off_wait_dist = src["wall_off_wait_dist"].as<float>();
    if (src["wall_off_wait_dist_dia"].is<float>()) dst.wall_off_wait_dist_dia = src["wall_off_wait_dist_dia"].as<float>();
    if (src["search_log_enable"].is<int>()) dst.search_log_enable = src["search_log_enable"].as<int>();
    if (src["seach_timer"].is<int>()) dst.seach_timer = src["seach_timer"].as<int>();
    if (src["test_log_enable"].is<int>()) dst.test_log_enable = src["test_log_enable"].as<int>();
    if (src["fast_log_enable"].is<int>()) dst.fast_log_enable = src["fast_log_enable"].as<int>();
    if (src["front_dist_offset_pivot_th"].is<float>()) dst.front_dist_offset_pivot_th = src["front_dist_offset_pivot_th"].as<float>();
    if (src["front_dist_offset_pivot"].is<float>()) dst.front_dist_offset_pivot = src["front_dist_offset_pivot"].as<float>();
    if (src["pivot_back_dist0"].is<float>()) dst.pivot_back_dist0 = src["pivot_back_dist0"].as<float>();
    if (src["pivot_back_dist1"].is<float>()) dst.pivot_back_dist1 = src["pivot_back_dist1"].as<float>();
    if (src["sen_log_size"].is<int>()) dst.sen_log_size = src["sen_log_size"].as<int>();
    if (src["sen_log_size2"].is<int>()) dst.sen_log_size2 = src["sen_log_size2"].as<int>();
    if (src["led_light_delay_cnt"].is<int>()) dst.led_light_delay_cnt = src["led_light_delay_cnt"].as<int>();
    if (src["led_light_delay_cnt2"].is<int>()) dst.led_light_delay_cnt2 = src["led_light_delay_cnt2"].as<int>();
    if (src["set_param"].is<bool>()) dst.set_param = src["set_param"].as<bool>();
    if (src["logging_time"].is<float>()) dst.logging_time = src["logging_time"].as<float>();
    if (src["offset_after_turn_l2"].is<float>()) dst.offset_after_turn_l2 = src["offset_after_turn_l2"].as<float>();
    if (src["offset_after_turn_r2"].is<float>()) dst.offset_after_turn_r2 = src["offset_after_turn_r2"].as<float>();
    if (src["offset_after_turn_l"].is<float>()) dst.offset_after_turn_l = src["offset_after_turn_l"].as<float>();
    if (src["offset_after_turn_r"].is<float>()) dst.offset_after_turn_r = src["offset_after_turn_r"].as<float>();
    if (src["offset_after_turn_dia_l"].is<float>()) dst.offset_after_turn_dia_l = src["offset_after_turn_dia_l"].as<float>();
    if (src["offset_after_turn_dia_r"].is<float>()) dst.offset_after_turn_dia_r = src["offset_after_turn_dia_r"].as<float>();
    if (src["dia_turn_exist_th_l"].is<float>()) dst.dia_turn_exist_th_l = src["dia_turn_exist_th_l"].as<float>();
    if (src["dia_turn_exist_th_r"].is<float>()) dst.dia_turn_exist_th_r = src["dia_turn_exist_th_r"].as<float>();
    if (src["dia_turn_th_l"].is<float>()) dst.dia_turn_th_l = src["dia_turn_th_l"].as<float>();
    if (src["dia_turn_th_r"].is<float>()) dst.dia_turn_th_r = src["dia_turn_th_r"].as<float>();
    if (src["dia_turn_ref_l"].is<float>()) dst.dia_turn_ref_l = src["dia_turn_ref_l"].as<float>();
    if (src["dia_turn_ref_r"].is<float>()) dst.dia_turn_ref_r = src["dia_turn_ref_r"].as<float>();
    if (src["dia_turn_max_dist_l"].is<float>()) dst.dia_turn_max_dist_l = src["dia_turn_max_dist_l"].as<float>();
    if (src["dia_turn_max_dist_r"].is<float>()) dst.dia_turn_max_dist_r = src["dia_turn_max_dist_r"].as<float>();
    if (src["wall_off_pass_dist"].is<float>()) dst.wall_off_pass_dist = src["wall_off_pass_dist"].as<float>();
    if (src["dia_wall_off_ref_l"].is<float>()) dst.dia_wall_off_ref_l = src["dia_wall_off_ref_l"].as<float>();
    if (src["dia_wall_off_ref_r"].is<float>()) dst.dia_wall_off_ref_r = src["dia_wall_off_ref_r"].as<float>();
    if (src["dia_wall_off_ref_l_wall"].is<float>()) dst.dia_wall_off_ref_l_wall = src["dia_wall_off_ref_l_wall"].as<float>();
    if (src["dia_wall_off_ref_r_wall"].is<float>()) dst.dia_wall_off_ref_r_wall = src["dia_wall_off_ref_r_wall"].as<float>();
    if (src["dia_wall_off_ref_l_wall2"].is<float>()) dst.dia_wall_off_ref_l_wall2 = src["dia_wall_off_ref_l_wall2"].as<float>();
    if (src["dia_wall_off_ref_r_wall2"].is<float>()) dst.dia_wall_off_ref_r_wall2 = src["dia_wall_off_ref_r_wall2"].as<float>();
    if (src["dia_wall_off_ref_l_piller"].is<float>()) dst.dia_wall_off_ref_l_piller = src["dia_wall_off_ref_l_piller"].as<float>();
    if (src["dia_wall_off_ref_r_piller"].is<float>()) dst.dia_wall_off_ref_r_piller = src["dia_wall_off_ref_r_piller"].as<float>();
    if (src["dia_offset_max_dist"].is<float>()) dst.dia_offset_max_dist = src["dia_offset_max_dist"].as<float>();
    if (src["slip_param_K"].is<float>()) dst.slip_param_K = src["slip_param_K"].as<float>();
    if (src["slip_param_k2"].is<float>()) dst.slip_param_k2 = src["slip_param_k2"].as<float>();
    if (!src["fail_check"].isNull()) convertFromJson(src["fail_check"], dst.fail_check);
    if (src["fail_check_ang_th"].is<float>()) dst.fail_check_ang_th = src["fail_check_ang_th"].as<float>();
    if (src["normal_sla_offset"].is<float>()) dst.normal_sla_offset = src["normal_sla_offset"].as<float>();
    if (src["normal_sla_offset_front"].is<float>()) dst.normal_sla_offset_front = src["normal_sla_offset_front"].as<float>();
    if (src["normal_sla_offset_back"].is<float>()) dst.normal_sla_offset_back = src["normal_sla_offset_back"].as<float>();
    if (src["front_diff_th"].is<float>()) dst.front_diff_th = src["front_diff_th"].as<float>();
    if (src["ff_v_th"].is<float>()) dst.ff_v_th = src["ff_v_th"].as<float>();
    if (src["ff_front_dury"].is<float>()) dst.ff_front_dury = src["ff_front_dury"].as<float>();
    if (src["motor_driver_type"].is<int>()) dst.motor_driver_type = (MotorDriveType)src["motor_driver_type"].as<int>();
    if (src["motor_debug_mode"].is<uint8_t>()) dst.motor_debug_mode = src["motor_debug_mode"].as<uint8_t>();
    if (src["motor_r_cw_ccw_type"].is<uint8_t>()) dst.motor_r_cw_ccw_type = src["motor_r_cw_ccw_type"].as<uint8_t>();
    if (src["motor_l_cw_ccw_type"].is<uint8_t>()) dst.motor_l_cw_ccw_type = src["motor_l_cw_ccw_type"].as<uint8_t>();
    if (src["motor_debug_mode_duty_r"].is<float>()) dst.motor_debug_mode_duty_r = src["motor_debug_mode_duty_r"].as<float>();
    if (src["motor_debug_mode_duty_l"].is<float>()) dst.motor_debug_mode_duty_l = src["motor_debug_mode_duty_l"].as<float>();
    if (src["pivot_straight"].is<float>()) dst.pivot_straight = src["pivot_straight"].as<float>();
    if (src["pivot_back_enable_front_th"].is<float>()) dst.pivot_back_enable_front_th = src["pivot_back_enable_front_th"].as<float>();
    if (src["search_front_ctrl_th"].is<float>()) dst.search_front_ctrl_th = src["search_front_ctrl_th"].as<float>();
    if (src["judge_pivot"].is<float>()) dst.judge_pivot = src["judge_pivot"].as<float>();
    if (src["sensor_range_min"].is<float>()) dst.sensor_range_min = src["sensor_range_min"].as<float>();
    if (src["sensor_range_max"].is<float>()) dst.sensor_range_max = src["sensor_range_max"].as<float>();
    if (src["sensor_range_mid_max"].is<float>()) dst.sensor_range_mid_max = src["sensor_range_mid_max"].as<float>();
    if (src["sensor_range_far_max"].is<float>()) dst.sensor_range_far_max = src["sensor_range_far_max"].as<float>();
    if (src["dist_mod_num"].is<float>()) dst.dist_mod_num = src["dist_mod_num"].as<float>();
    if (src["sen_ctrl_front_th"].is<float>()) dst.sen_ctrl_front_th = src["sen_ctrl_front_th"].as<float>();
    if (src["sen_ctrl_front_diff_th"].is<float>()) dst.sen_ctrl_front_diff_th = src["sen_ctrl_front_diff_th"].as<float>();
    if (src["th_offset_dist"].is<float>()) dst.th_offset_dist = src["th_offset_dist"].as<float>();
    if (src["sla_front_ctrl_th"].is<float>()) dst.sla_front_ctrl_th = src["sla_front_ctrl_th"].as<float>();
    if (src["orval_front_ctrl_min"].is<float>()) dst.orval_front_ctrl_min = src["orval_front_ctrl_min"].as<float>();
    if (src["orval_front_ctrl_max"].is<float>()) dst.orval_front_ctrl_max = src["orval_front_ctrl_max"].as<float>();
    if (src["wall_off_front_ctrl_min"].is<float>()) dst.wall_off_front_ctrl_min = src["wall_off_front_ctrl_min"].as<float>();
    if (src["dia_turn_offset_calc_th"].is<float>()) dst.dia_turn_offset_calc_th = src["dia_turn_offset_calc_th"].as<float>();
    if (src["go_straight_wide_ctrl_th"].is<float>()) dst.go_straight_wide_ctrl_th = src["go_straight_wide_ctrl_th"].as<float>();
    if (src["wall_off_pass_through_offset_r"].is<float>()) dst.wall_off_pass_through_offset_r = src["wall_off_pass_through_offset_r"].as<float>();
    if (src["wall_off_pass_through_offset_l"].is<float>()) dst.wall_off_pass_through_offset_l = src["wall_off_pass_through_offset_l"].as<float>();
    if (src["tire_tread"].is<float>()) dst.tire_tread = src["tire_tread"].as<float>();
    if (src["right_keep_dist_th"].is<float>()) dst.right_keep_dist_th = src["right_keep_dist_th"].as<float>();
    if (src["left_keep_dist_th"].is<float>()) dst.left_keep_dist_th = src["left_keep_dist_th"].as<float>();
    if (src["normal_sla_l_wall_off_th_in"].is<float>()) dst.normal_sla_l_wall_off_th_in = src["normal_sla_l_wall_off_th_in"].as<float>();
    if (src["normal_sla_r_wall_off_th_in"].is<float>()) dst.normal_sla_r_wall_off_th_in = src["normal_sla_r_wall_off_th_in"].as<float>();
    if (src["normal_sla_l_wall_off_th_out"].is<float>()) dst.normal_sla_l_wall_off_th_out = src["normal_sla_l_wall_off_th_out"].as<float>();
    if (src["normal_sla_r_wall_off_th_out"].is<float>()) dst.normal_sla_r_wall_off_th_out = src["normal_sla_r_wall_off_th_out"].as<float>();
    if (src["normal_sla_l_wall_off_ref_cnt"].is<float>()) dst.normal_sla_l_wall_off_ref_cnt = src["normal_sla_l_wall_off_ref_cnt"].as<float>();
    if (src["normal_sla_r_wall_off_ref_cnt"].is<float>()) dst.normal_sla_r_wall_off_ref_cnt = src["normal_sla_r_wall_off_ref_cnt"].as<float>();
    if (src["normal_sla_l_wall_off_dist"].is<float>()) dst.normal_sla_l_wall_off_dist = src["normal_sla_l_wall_off_dist"].as<float>();
    if (src["normal_sla_r_wall_off_dist"].is<float>()) dst.normal_sla_r_wall_off_dist = src["normal_sla_r_wall_off_dist"].as<float>();
    if (src["normal_sla_l_wall_off_margin"].is<float>()) dst.normal_sla_l_wall_off_margin = src["normal_sla_l_wall_off_margin"].as<float>();
    if (src["normal_sla_r_wall_off_margin"].is<float>()) dst.normal_sla_r_wall_off_margin = src["normal_sla_r_wall_off_margin"].as<float>();
    if (src["torque_mode"].is<int8_t>()) dst.torque_mode = src["torque_mode"].as<int8_t>();
    if (src["enable_kalman_gyro"].is<int8_t>()) dst.enable_kalman_gyro = src["enable_kalman_gyro"].as<int8_t>();
    if (src["enable_kalman_encoder"].is<int8_t>()) dst.enable_kalman_encoder = src["enable_kalman_encoder"].as<int8_t>();
    if (src["enable_mpc"].is<int8_t>()) dst.enable_mpc = src["enable_mpc"].as<int8_t>();
    if (src["dia90_offset"].is<float>()) dst.dia90_offset = src["dia90_offset"].as<float>();
    if (!src["kanayama"].isNull()) convertFromJson(src["kanayama"], dst.kanayama);
}

inline void convertFromJson(JsonVariantConst src, pid_error_t& dst) {
    if (src["error_p"].is<float>()) dst.error_p = src["error_p"].as<float>();
    if (src["error_i"].is<float>()) dst.error_i = src["error_i"].as<float>();
    if (src["error_i_keep"].is<float>()) dst.error_i_keep = src["error_i_keep"].as<float>();
    if (src["error_d"].is<float>()) dst.error_d = src["error_d"].as<float>();
    if (src["error_dd"].is<float>()) dst.error_dd = src["error_dd"].as<float>();
    if (src["i_slow"].is<float>()) dst.i_slow = src["i_slow"].as<float>();
    if (src["i_bias"].is<float>()) dst.i_bias = src["i_bias"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, pid_error2_t& dst) {
    if (src["p"].is<float>()) dst.p = src["p"].as<float>();
    if (src["i"].is<float>()) dst.i = src["i"].as<float>();
    if (src["i2"].is<float>()) dst.i2 = src["i2"].as<float>();
    if (src["d"].is<float>()) dst.d = src["d"].as<float>();
    if (src["p_val"].is<float>()) dst.p_val = src["p_val"].as<float>();
    if (src["i_val"].is<float>()) dst.i_val = src["i_val"].as<float>();
    if (src["i2_val"].is<float>()) dst.i2_val = src["i2_val"].as<float>();
    if (src["d_val"].is<float>()) dst.d_val = src["d_val"].as<float>();
    if (src["zz"].is<float>()) dst.zz = src["zz"].as<float>();
    if (src["z"].is<float>()) dst.z = src["z"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, gain_log_t& dst) {
    if (src["gain_z"].is<float>()) dst.gain_z = src["gain_z"].as<float>();
    if (src["gain_zz"].is<float>()) dst.gain_zz = src["gain_zz"].as<float>();
    if (src["omega_ref_prev"].is<float>()) dst.omega_ref_prev = src["omega_ref_prev"].as<float>();
    if (src["prev_motion_type"].is<int>()) dst.prev_motion_type = (MotionType)src["prev_motion_type"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, aw_log_t& dst) {
    if (src["was_aw"].is<float>()) dst.was_aw = src["was_aw"].as<float>();
    if (src["enter_aw"].is<float>()) dst.enter_aw = src["enter_aw"].as<float>();
    if (src["keep_aw"].is<float>()) dst.keep_aw = src["keep_aw"].as<float>();
    if (src["w_i_base"].is<float>()) dst.w_i_base = src["w_i_base"].as<float>();
    if (src["w_error_i_raw"].is<float>()) dst.w_error_i_raw = src["w_error_i_raw"].as<float>();
    if (src["w_error_i_clamped"].is<float>()) dst.w_error_i_clamped = src["w_error_i_clamped"].as<float>();
    if (src["gyro_pid_histerisis_i"].is<float>()) dst.gyro_pid_histerisis_i = src["gyro_pid_histerisis_i"].as<float>();
    if (src["sat_flag"].is<float>()) dst.sat_flag = src["sat_flag"].as<float>();
    if (src["duty_roll"].is<float>()) dst.duty_roll = src["duty_roll"].as<float>();
    if (src["duty_roll_before"].is<float>()) dst.duty_roll_before = src["duty_roll_before"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, pid_error_entity_t& dst) {
    if (!src["v"].isNull()) convertFromJson(src["v"], dst.v);
    if (!src["v_kf"].isNull()) convertFromJson(src["v_kf"], dst.v_kf);
    if (!src["dist"].isNull()) convertFromJson(src["dist"], dst.dist);
    if (!src["w"].isNull()) convertFromJson(src["w"], dst.w);
    if (!src["v_r"].isNull()) convertFromJson(src["v_r"], dst.v_r);
    if (!src["v_l"].isNull()) convertFromJson(src["v_l"], dst.v_l);
    if (!src["w_kf"].isNull()) convertFromJson(src["w_kf"], dst.w_kf);
    if (!src["ang"].isNull()) convertFromJson(src["ang"], dst.ang);
    if (!src["v_log"].isNull()) convertFromJson(src["v_log"], dst.v_log);
    if (!src["dist_log"].isNull()) convertFromJson(src["dist_log"], dst.dist_log);
    if (!src["w_log"].isNull()) convertFromJson(src["w_log"], dst.w_log);
    if (!src["v_r_log"].isNull()) convertFromJson(src["v_r_log"], dst.v_r_log);
    if (!src["v_l_log"].isNull()) convertFromJson(src["v_l_log"], dst.v_l_log);
    if (!src["ang_log"].isNull()) convertFromJson(src["ang_log"], dst.ang_log);
    if (!src["sen_log"].isNull()) convertFromJson(src["sen_log"], dst.sen_log);
    if (!src["sen_log_dia"].isNull()) convertFromJson(src["sen_log_dia"], dst.sen_log_dia);
    if (!src["sen"].isNull()) convertFromJson(src["sen"], dst.sen);
    if (!src["sen_dia"].isNull()) convertFromJson(src["sen_dia"], dst.sen_dia);
    if (!src["v_val"].isNull()) convertFromJson(src["v_val"], dst.v_val);
    if (!src["w_val"].isNull()) convertFromJson(src["w_val"], dst.w_val);
    if (!src["ang_val"].isNull()) convertFromJson(src["ang_val"], dst.ang_val);
    if (!src["s_val"].isNull()) convertFromJson(src["s_val"], dst.s_val);
    if (!src["aw_log"].isNull()) convertFromJson(src["aw_log"], dst.aw_log);
}

inline void convertFromJson(JsonVariantConst src, motion_tgt_t& dst) {
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["w_max"].is<float>()) dst.w_max = src["w_max"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, buzzer_t& dst) {
    if (src["hz"].is<int>()) dst.hz = src["hz"].as<int>();
    if (src["time"].is<int>()) dst.time = src["time"].as<int>();
    if (src["timstamp"].is<int>()) dst.timstamp = src["timstamp"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, planning_req_t& dst) {
    if (src["time_stamp"].is<int>()) dst.time_stamp = src["time_stamp"].as<int>();
    if (src["error_gyro_reset"].is<int>()) dst.error_gyro_reset = src["error_gyro_reset"].as<int>();
    if (src["error_vel_reset"].is<int>()) dst.error_vel_reset = src["error_vel_reset"].as<int>();
    if (src["error_led_reset"].is<int>()) dst.error_led_reset = src["error_led_reset"].as<int>();
    if (src["error_ang_reset"].is<int>()) dst.error_ang_reset = src["error_ang_reset"].as<int>();
    if (src["error_dist_reset"].is<int>()) dst.error_dist_reset = src["error_dist_reset"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, fail_safe_state_t& dst) {
    if (src["error"].is<int>()) dst.error = src["error"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, sys_id_t& dst) {
    if (src["right_v"].is<float>()) dst.right_v = src["right_v"].as<float>();
    if (src["left_v"].is<float>()) dst.left_v = src["left_v"].as<float>();
    if (src["enable"].is<bool>()) dst.enable = src["enable"].as<bool>();
}

inline void convertFromJson(JsonVariantConst src, new_motion_req_t& dst) {
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["v_end"].is<float>()) dst.v_end = src["v_end"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["decel"].is<float>()) dst.decel = src["decel"].as<float>();
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["w_max"].is<float>()) dst.w_max = src["w_max"].as<float>();
    if (src["w_end"].is<float>()) dst.w_end = src["w_end"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["sla_alpha"].is<float>()) dst.sla_alpha = src["sla_alpha"].as<float>();
    if (src["sla_time"].is<float>()) dst.sla_time = src["sla_time"].as<float>();
    if (src["sla_pow_n"].is<float>()) dst.sla_pow_n = src["sla_pow_n"].as<float>();
    if (src["sla_rad"].is<float>()) dst.sla_rad = src["sla_rad"].as<float>();
    if (src["dia90_offset"].is<float>()) dst.dia90_offset = src["dia90_offset"].as<float>();
    if (src["td"].is<int>()) dst.td = (TurnDirection)src["td"].as<int>();
    if (src["tt"].is<int>()) dst.tt = (TurnType)src["tt"].as<int>();
    if (src["motion_mode"].is<int>()) dst.motion_mode = (RUN_MODE2)src["motion_mode"].as<int>();
    if (src["motion_type"].is<int>()) dst.motion_type = (MotionType)src["motion_type"].as<int>();
    if (src["timstamp"].is<int>()) dst.timstamp = src["timstamp"].as<int>();
    if (src["motion_dir"].is<int>()) dst.motion_dir = (MotionDirection)src["motion_dir"].as<int>();
    if (src["dia_mode"].is<bool>()) dst.dia_mode = src["dia_mode"].as<bool>();
    if (src["sct"].is<int>()) dst.sct = (SensorCtrlType)src["sct"].as<int>();
    if (!src["sys_id"].isNull()) convertFromJson(src["sys_id"], dst.sys_id);
    if (src["tgt_reset_req"].is<bool>()) dst.tgt_reset_req = src["tgt_reset_req"].as<bool>();
    if (src["ego_reset_req"].is<bool>()) dst.ego_reset_req = src["ego_reset_req"].as<bool>();
}

inline void convertFromJson(JsonVariantConst src, global_ego_pos_t& dst) {
    if (src["img_dist"].is<float>()) dst.img_dist = src["img_dist"].as<float>();
    if (src["img_ang"].is<float>()) dst.img_ang = src["img_ang"].as<float>();
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, pos_t& dst) {
    if (src["x"].is<float>()) dst.x = src["x"].as<float>();
    if (src["y"].is<float>()) dst.y = src["y"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, dia_state_t& dst) {
    if (src["right_old"].is<float>()) dst.right_old = src["right_old"].as<float>();
    if (src["left_old"].is<float>()) dst.left_old = src["left_old"].as<float>();
    if (src["right_save"].is<bool>()) dst.right_save = src["right_save"].as<bool>();
    if (src["left_save"].is<bool>()) dst.left_save = src["left_save"].as<bool>();
    if (src["dia90_offset"].is<float>()) dst.dia90_offset = src["dia90_offset"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, t_kanayama_gain& dst) {
    from_json_field(src, "k_x",     dst.k_x);
    from_json_field(src, "k_y",     dst.k_y);
    from_json_field(src, "k_theta", dst.k_theta);
}

inline void convertFromJson(JsonVariantConst src, t_accl_param& dst) {
    from_json_field(src, "limit",          dst.limit);
    from_json_field(src, "n",              dst.n);
    from_json_field(src, "decel_delay_cnt",dst.decel_delay_cnt);
    from_json_field(src, "decel_delay_n",  dst.decel_delay_n);
}

inline void convertFromJson(JsonVariantConst src, t_slalom& dst) {
    from_json_field(src, "base_alpha",       dst.base_alpha);
    from_json_field(src, "base_time",        dst.base_time);
    from_json_field(src, "limit_time_count", dst.limit_time_count);
    from_json_field(src, "pow_n",            dst.pow_n);
    from_json_field(src, "state",            dst.state);
    from_json_field(src, "counter",          dst.counter);
}

inline void convertFromJson(JsonVariantConst src, t_trajectory_diff& dst) {
    from_json_field(src, "x",     dst.x);
    from_json_field(src, "y",     dst.y);
    from_json_field(src, "theta", dst.theta);
}

inline void convertFromJson(JsonVariantConst src, t_kanayama_tgt_point& dst) {
    from_json_field(src, "x",     dst.x);
    from_json_field(src, "y",     dst.y);
    from_json_field(src, "theta", dst.theta);
    from_json_field(src, "v",     dst.v);
    from_json_field(src, "w",     dst.w);
}

inline void convertFromJson(JsonVariantConst src, t_point& dst) {
    from_json_field(src, "x",          dst.x);
    from_json_field(src, "y",          dst.y);
    from_json_field(src, "theta",      dst.theta);
    from_json_field(src, "v",          dst.v);
    from_json_field(src, "w",          dst.w);
    from_json_field(src, "slip_angle", dst.slip_angle);
}

inline void convertFromJson(JsonVariantConst src, t_slip& dst) {
    from_json_field(src, "beta", dst.beta);
    from_json_field(src, "vx",   dst.vx);
    from_json_field(src, "vy",   dst.vy);
    from_json_field(src, "v",    dst.v);
    from_json_field(src, "accl", dst.accl);
}

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
    if (src["ff_duty_rpm_l"].is<float>()) dst.ff_duty_rpm_l = src["ff_duty_rpm_l"].as<float>();
    if (src["ff_front_torque"].is<float>()) dst.ff_front_torque = src["ff_front_torque"].as<float>();
    if (src["ff_roll_torque"].is<float>()) dst.ff_roll_torque = src["ff_roll_torque"].as<float>();
    if (src["ff_friction_torque_l"].is<float>()) dst.ff_friction_torque_l = src["ff_friction_torque_l"].as<float>();
    if (src["ff_friction_torque_r"].is<float>()) dst.ff_friction_torque_r = src["ff_friction_torque_r"].as<float>();
    if (src["decel_delay_cnt"].is<int>()) dst.decel_delay_cnt = src["decel_delay_cnt"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, motion_tgt_val_t& dst) {
    if (!src["tgt_in"].isNull()) convertFromJson(src["tgt_in"], dst.tgt_in);
    if (!src["ego_in"].isNull()) convertFromJson(src["ego_in"], dst.ego_in);
    if (src["calc_time"].is<int16_t>()) dst.calc_time = src["calc_time"].as<int16_t>();
    if (src["calc_time2"].is<int16_t>()) dst.calc_time2 = src["calc_time2"].as<int16_t>();
    if (src["calc_time_diff"].is<int16_t>()) dst.calc_time_diff = src["calc_time_diff"].as<int16_t>();
    if (!src["global_pos"].isNull()) {
        global_ego_pos_t tmp{};
        convertFromJson(src["global_pos"], tmp);
        memcpy(const_cast<global_ego_pos_t*>(&dst.global_pos), &tmp, sizeof(global_ego_pos_t));
    }
    if (src["motion_mode"].is<int32_t>()) dst.motion_mode = src["motion_mode"].as<int32_t>();
    if (src["motion_type"].is<int>()) dst.motion_type = (MotionType)src["motion_type"].as<int>();
    if (src["motion_dir"].is<int>()) dst.motion_dir = (MotionDirection)src["motion_dir"].as<int>();
    if (src["dia_mode"].is<bool>()) dst.dia_mode = src["dia_mode"].as<bool>();
    if (!src["pl_req"].isNull()) convertFromJson(src["pl_req"], dst.pl_req);
    if (!src["fss"].isNull()) convertFromJson(src["fss"], dst.fss);
    if (src["gyro_zero_p_offset"].is<float>()) dst.gyro_zero_p_offset = src["gyro_zero_p_offset"].as<float>();
    if (src["var_unbiased_dps2"].is<float>()) dst.var_unbiased_dps2 = src["var_unbiased_dps2"].as<float>();
    if (src["var_robust_dps2"].is<float>()) dst.var_robust_dps2 = src["var_robust_dps2"].as<float>();
    if (src["gyro_retry"].is<int>()) dst.gyro_retry = src["gyro_retry"].as<int>();
    if (src["calibration_mode"].is<int>()) dst.calibration_mode = (CalibrationMode)src["calibration_mode"].as<int>();
    if (src["gyro2_zero_p_offset"].is<float>()) dst.gyro2_zero_p_offset = src["gyro2_zero_p_offset"].as<float>();
    if (src["accel_x_zero_p_offset"].is<float>()) dst.accel_x_zero_p_offset = src["accel_x_zero_p_offset"].as<float>();
    if (src["accel_y_zero_p_offset"].is<float>()) dst.accel_y_zero_p_offset = src["accel_y_zero_p_offset"].as<float>();
    if (src["temp_zero_p_offset"].is<float>()) dst.temp_zero_p_offset = src["temp_zero_p_offset"].as<float>();
    if (!src["buzzer"].isNull()) convertFromJson(src["buzzer"], dst.buzzer);
    from_json_nested(src, "nmr",       dst.nmr);
    from_json_nested(src, "p",         dst.p);
    from_json_nested(src, "dia_state", dst.dia_state);
    from_json_field(src,  "v_error",   dst.v_error);
    from_json_field(src,  "w_error",   dst.w_error);
    from_json_field(src,  "td",        dst.td);   // TurnDirection — enum, static_cast 自動
    from_json_field(src,  "tt",        dst.tt);   // TurnType      — enum, static_cast 自動
    from_json_field(src,  "duty_suction", dst.duty_suction);
}

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

inline void convertFromJson(JsonVariantConst src, param_roll_t& dst) {
    if (src["w_max"].is<float>()) dst.w_max = src["w_max"].as<float>();
    if (src["w_end"].is<float>()) dst.w_end = src["w_end"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["RorL"].is<int>()) dst.RorL = (TurnDirection)src["RorL"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, param_normal_slalom_t& dst) {
    if (src["radius"].is<float>()) dst.radius = src["radius"].as<float>();
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["v_end"].is<float>()) dst.v_end = src["v_end"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["RorL"].is<int>()) dst.RorL = (TurnDirection)src["RorL"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, test_mode_t& dst) {
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["end_v"].is<float>()) dst.end_v = src["end_v"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["decel"].is<float>()) dst.decel = src["decel"].as<float>();
    if (src["dia_accl"].is<float>()) dst.dia_accl = src["dia_accl"].as<float>();
    if (src["dia_decel"].is<float>()) dst.dia_decel = src["dia_decel"].as<float>();
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["w_max"].is<float>()) dst.w_max = src["w_max"].as<float>();
    if (src["w_end"].is<float>()) dst.w_end = src["w_end"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["suction_active"].is<int>()) dst.suction_active = src["suction_active"].as<int>();
    if (src["suction_duty"].is<float>()) dst.suction_duty = src["suction_duty"].as<float>();
    if (src["suction_duty_low"].is<float>()) dst.suction_duty_low = src["suction_duty_low"].as<float>();
    if (src["suction_duty_burst"].is<float>()) dst.suction_duty_burst = src["suction_duty_burst"].as<float>();
    if (src["suction_duty_burst_low"].is<float>()) dst.suction_duty_burst_low = src["suction_duty_burst_low"].as<float>();
    if (src["suction_gain"].is<float>()) dst.suction_gain = src["suction_gain"].as<float>();
    if (src["sla_dist"].is<float>()) dst.sla_dist = src["sla_dist"].as<float>();
    if (src["file_idx"].is<int>()) dst.file_idx = src["file_idx"].as<int>();
    if (src["sla_type"].is<int>()) dst.sla_type = src["sla_type"].as<int>();
    if (src["sla_return"].is<int>()) dst.sla_return = src["sla_return"].as<int>();
    if (src["sla_type2"].is<int>()) dst.sla_type2 = src["sla_type2"].as<int>();
    if (src["turn_times"].is<int>()) dst.turn_times = src["turn_times"].as<int>();
    if (src["ignore_opp_sen"].is<int>()) dst.ignore_opp_sen = src["ignore_opp_sen"].as<int>();
    if (src["dia"].is<int>()) dst.dia = src["dia"].as<int>();
    if (src["sysid_test_mode"].is<int>()) dst.sysid_test_mode = src["sysid_test_mode"].as<int>();
    if (src["sysid_duty"].is<float>()) dst.sysid_duty = src["sysid_duty"].as<float>();
    if (src["sysid_time"].is<float>()) dst.sysid_time = src["sysid_time"].as<float>();
    if (src["start_turn"].is<int>()) dst.start_turn = src["start_turn"].as<int>();
    if (src["search_mode"].is<int>()) dst.search_mode = src["search_mode"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, system_t& dst) {
    if (src["goals"].is<JsonArrayConst>()) {
        dst.goals.clear();
        for (auto v : src["goals"].as<JsonArrayConst>()) {
            point_t temp;
            convertFromJson(v, temp);
            dst.goals.push_back(temp);
        }
    }
    if (src["maze_size"].is<int>()) dst.maze_size = src["maze_size"].as<int>();
    if (src["user_mode"].is<int>()) dst.user_mode = src["user_mode"].as<int>();
    if (src["circuit_mode"].is<int>()) dst.circuit_mode = src["circuit_mode"].as<int>();
    if (!src["test"].isNull()) convertFromJson(src["test"], dst.test);
    if (src["hf_cl"].is<int>()) dst.hf_cl = src["hf_cl"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, profile_idx_t& dst) {
    if (src["normal"].is<int>()) dst.normal = src["normal"].as<int>();
    if (src["large"].is<int>()) dst.large = src["large"].as<int>();
    if (src["orval"].is<int>()) dst.orval = src["orval"].as<int>();
    if (src["dia45"].is<int>()) dst.dia45 = src["dia45"].as<int>();
    if (src["dia45_2"].is<int>()) dst.dia45_2 = src["dia45_2"].as<int>();
    if (src["dia135"].is<int>()) dst.dia135 = src["dia135"].as<int>();
    if (src["dia135_2"].is<int>()) dst.dia135_2 = src["dia135_2"].as<int>();
    if (src["dia90"].is<int>()) dst.dia90 = src["dia90"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, turn_param_profile_t& dst) {
    if (src["file_list"].is<JsonArrayConst>()) {
        dst.file_list.clear();
        for (auto v : src["file_list"].as<JsonArrayConst>()) {
            std::string temp;
            convertFromJson(v, temp);
            dst.file_list.push_back(temp);
        }
    }
    if (src["file_list_size"].is<int>()) dst.file_list_size = src["file_list_size"].as<int>();
    if (src["profile_idx_size"].is<int>()) dst.profile_idx_size = src["profile_idx_size"].as<int>();
    // profile_list unordered_map skipped
    // profile_map unordered_map skipped
}

inline void convertFromJson(JsonVariantConst src, motor_req_t& dst) {
    if (src["enable"].is<bool>()) dst.enable = src["enable"].as<bool>();
    if (src["timestamp"].is<int>()) dst.timestamp = src["timestamp"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, slalom_offset_t& dst) {
    if (src["right"].is<float>()) dst.right = src["right"].as<float>();
    if (src["left"].is<float>()) dst.left = src["left"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, slalom_param2_t& dst) {
    if (src["v"].is<float>()) dst.v = src["v"].as<float>();
    if (src["end_v"].is<float>()) dst.end_v = src["end_v"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["ref_ang"].is<float>()) dst.ref_ang = src["ref_ang"].as<float>();
    if (src["rad"].is<float>()) dst.rad = src["rad"].as<float>();
    if (src["rad2"].is<float>()) dst.rad2 = src["rad2"].as<float>();
    if (!src["front"].isNull()) convertFromJson(src["front"], dst.front);
    if (!src["back"].isNull()) convertFromJson(src["back"], dst.back);
    if (src["pow_n"].is<int>()) dst.pow_n = src["pow_n"].as<int>();
    if (src["time"].is<float>()) dst.time = src["time"].as<float>();
    if (src["time2"].is<float>()) dst.time2 = src["time2"].as<float>();
    if (src["type"].is<int>()) dst.type = (TurnType)src["type"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, straight_param_t& dst) {
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["decel"].is<float>()) dst.decel = src["decel"].as<float>();
    if (src["w_max"].is<float>()) dst.w_max = src["w_max"].as<float>();
    if (src["w_end"].is<float>()) dst.w_end = src["w_end"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, param_set_t& dst) {
    // map unordered_map skipped
    // map_slow unordered_map skipped
    // map_fast unordered_map skipped
    // str_map unordered_map skipped
    if (src["suction"].is<int8_t>()) dst.suction = src["suction"].as<int8_t>();
    if (src["suction_duty"].is<float>()) dst.suction_duty = src["suction_duty"].as<float>();
    if (src["suction_duty_low"].is<float>()) dst.suction_duty_low = src["suction_duty_low"].as<float>();
    if (src["cell_size"].is<float>()) dst.cell_size = src["cell_size"].as<float>();
    if (src["start_offset"].is<float>()) dst.start_offset = src["start_offset"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, path_set_t& dst) {
    if (src["path_s"].is<JsonArrayConst>()) {
        dst.path_s.clear();
        for (auto v : src["path_s"].as<JsonArrayConst>()) {
            dst.path_s.push_back(v.as<float>());
        }
    }
    // path_t vector<unsigned char> skipped
    if (src["time"].is<float>()) dst.time = src["time"].as<float>();
    if (src["result"].is<bool>()) dst.result = src["result"].as<bool>();
    if (src["type"].is<int8_t>()) dst.type = src["type"].as<int8_t>();
}

inline void convertFromJson(JsonVariantConst src, path_req_t& dst) {
    if (src["time"].is<float>()) dst.time = src["time"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, create_path_result_t& dst) {
    if (src["time"].is<float>()) dst.time = src["time"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, next_motion_t& dst) {
    if (src["is_turn"].is<bool>()) dst.is_turn = src["is_turn"].as<bool>();
    if (src["next_turn_type"].is<int>()) dst.next_turn_type = (TurnType)src["next_turn_type"].as<int>();
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["v_end"].is<float>()) dst.v_end = src["v_end"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["decel"].is<float>()) dst.decel = src["decel"].as<float>();
    if (src["skip_wall_off"].is<bool>()) dst.skip_wall_off = src["skip_wall_off"].as<bool>();
    if (src["carry_over_dist"].is<float>()) dst.carry_over_dist = src["carry_over_dist"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, log_data_t& dst) {
    if (src["img_v"].is<float>()) dst.img_v = src["img_v"].as<float>();
    if (src["v_l"].is<float>()) dst.v_l = src["v_l"].as<float>();
    if (src["v_c"].is<float>()) dst.v_c = src["v_c"].as<float>();
    if (src["v_r"].is<float>()) dst.v_r = src["v_r"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["img_w"].is<float>()) dst.img_w = src["img_w"].as<float>();
    if (src["w_lp"].is<float>()) dst.w_lp = src["w_lp"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
    if (src["img_dist"].is<float>()) dst.img_dist = src["img_dist"].as<float>();
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["img_ang"].is<float>()) dst.img_ang = src["img_ang"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["duty_l"].is<float>()) dst.duty_l = src["duty_l"].as<float>();
    if (src["duty_r"].is<float>()) dst.duty_r = src["duty_r"].as<float>();
    if (src["left90_lp"].is<float>()) dst.left90_lp = src["left90_lp"].as<float>();
    if (src["left45_lp"].is<float>()) dst.left45_lp = src["left45_lp"].as<float>();
    if (src["front_lp"].is<float>()) dst.front_lp = src["front_lp"].as<float>();
    if (src["right45_lp"].is<float>()) dst.right45_lp = src["right45_lp"].as<float>();
    if (src["right90_lp"].is<float>()) dst.right90_lp = src["right90_lp"].as<float>();
    if (src["battery_lp"].is<float>()) dst.battery_lp = src["battery_lp"].as<float>();
    if (src["motion_type"].is<int8_t>()) dst.motion_type = src["motion_type"].as<int8_t>();
    if (src["duty_ff_front"].is<float>()) dst.duty_ff_front = src["duty_ff_front"].as<float>();
    if (src["duty_ff_roll"].is<float>()) dst.duty_ff_roll = src["duty_ff_roll"].as<float>();
    if (src["duty_sensor_ctrl"].is<float>()) dst.duty_sensor_ctrl = src["duty_sensor_ctrl"].as<float>();
    if (src["pos_x"].is<float>()) dst.pos_x = src["pos_x"].as<float>();
    if (src["pos_y"].is<float>()) dst.pos_y = src["pos_y"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, exec_pram_t& dst) {
    if (src["fast_idx"].is<int8_t>()) dst.fast_idx = src["fast_idx"].as<int8_t>();
    if (src["normal_idx"].is<int8_t>()) dst.normal_idx = src["normal_idx"].as<int8_t>();
    if (src["slow_idx"].is<int8_t>()) dst.slow_idx = src["slow_idx"].as<int8_t>();
}

inline void convertFromJson(JsonVariantConst src, log_data_t2& dst) {
    // img_v is half_t, skipping
    // v_l is half_t, skipping
    // v_c is half_t, skipping
    // v_c2 is half_t, skipping
    // v_r is half_t, skipping
    if (src["v_r_enc"].is<int16_t>()) dst.v_r_enc = src["v_r_enc"].as<int16_t>();
    if (src["v_l_enc"].is<int16_t>()) dst.v_l_enc = src["v_l_enc"].as<int16_t>();
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
    if (src["left90_lp"].is<int16_t>()) dst.left90_lp = src["left90_lp"].as<int16_t>();
    if (src["left45_lp"].is<int16_t>()) dst.left45_lp = src["left45_lp"].as<int16_t>();
    if (src["right45_lp"].is<int16_t>()) dst.right45_lp = src["right45_lp"].as<int16_t>();
    if (src["right90_lp"].is<int16_t>()) dst.right90_lp = src["right90_lp"].as<int16_t>();
    // battery_lp is half_t, skipping
    if (src["left45_2_lp"].is<int16_t>()) dst.left45_2_lp = src["left45_2_lp"].as<int16_t>();
    if (src["right45_2_lp"].is<int16_t>()) dst.right45_2_lp = src["right45_2_lp"].as<int16_t>();
    if (src["left45_3_lp"].is<int16_t>()) dst.left45_3_lp = src["left45_3_lp"].as<int16_t>();
    if (src["right45_3_lp"].is<int16_t>()) dst.right45_3_lp = src["right45_3_lp"].as<int16_t>();
    if (src["motion_type"].is<uint8_t>()) dst.motion_type = src["motion_type"].as<uint8_t>();
    if (src["motion_timestamp"].is<int16_t>()) dst.motion_timestamp = src["motion_timestamp"].as<int16_t>();
    // duty_sensor_ctrl is half_t, skipping
    // sen_log_l45 is half_t, skipping
    // sen_log_r45 is half_t, skipping
    // sen_log_l45_2 is half_t, skipping
    // sen_log_r45_2 is half_t, skipping
    // sen_log_l45_3 is half_t, skipping
    // sen_log_r45_3 is half_t, skipping
    if (src["sen_calc_time"].is<int16_t>()) dst.sen_calc_time = src["sen_calc_time"].as<int16_t>();
    if (src["sen_calc_time2"].is<int16_t>()) dst.sen_calc_time2 = src["sen_calc_time2"].as<int16_t>();
    if (src["pln_calc_time"].is<int16_t>()) dst.pln_calc_time = src["pln_calc_time"].as<int16_t>();
    if (src["pln_time_diff"].is<int16_t>()) dst.pln_time_diff = src["pln_time_diff"].as<int16_t>();
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

inline void convertFromJson(JsonVariantConst src, sysid_log& dst) {
    // v_l is half_t, skipping
    // v_c is half_t, skipping
    // v_r is half_t, skipping
    // w_lp is half_t, skipping
    // volt_l is half_t, skipping
    // volt_r is half_t, skipping
}

inline void convertFromJson(JsonVariantConst src, fail_safe_t& dst) {
    if (src["invalid_front_led"].is<int>()) dst.invalid_front_led = src["invalid_front_led"].as<int>();
    if (src["invalid_duty_r_cnt"].is<int>()) dst.invalid_duty_r_cnt = src["invalid_duty_r_cnt"].as<int>();
    if (src["invalid_duty_l_cnt"].is<int>()) dst.invalid_duty_l_cnt = src["invalid_duty_l_cnt"].as<int>();
    if (src["invalid_v_cnt"].is<int>()) dst.invalid_v_cnt = src["invalid_v_cnt"].as<int>();
    if (src["invalid_w_cnt"].is<int>()) dst.invalid_w_cnt = src["invalid_w_cnt"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, slip_t& dst) {
    if (src["K"].is<float>()) dst.K = src["K"].as<float>();
    if (src["k"].is<float>()) dst.k = src["k"].as<float>();
    if (src["beta"].is<float>()) dst.beta = src["beta"].as<float>();
    if (src["vx"].is<float>()) dst.vx = src["vx"].as<float>();
    if (src["vy"].is<float>()) dst.vy = src["vy"].as<float>();
    if (src["v"].is<float>()) dst.v = src["v"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, sensor_ctrl_keep_dist_t& dst) {
    if (src["star_dist"].is<float>()) dst.star_dist = src["star_dist"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, planning_time_t& dst) {
    if (src["v_start"].is<float>()) dst.v_start = src["v_start"].as<float>();
    if (src["v_max"].is<float>()) dst.v_max = src["v_max"].as<float>();
    if (src["v_end"].is<float>()) dst.v_end = src["v_end"].as<float>();
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["lap_time"].is<float>()) dst.lap_time = src["lap_time"].as<float>();
    if (src["total_time"].is<float>()) dst.total_time = src["total_time"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct1& dst) {
    if (src["index"].is<int>()) dst.index = src["index"].as<int>();
    if (src["ideal_v"].is<float>()) dst.ideal_v = src["ideal_v"].as<float>();
    if (src["v_c"].is<float>()) dst.v_c = src["v_c"].as<float>();
    if (src["v_c2"].is<float>()) dst.v_c2 = src["v_c2"].as<float>();
    if (src["v_l"].is<float>()) dst.v_l = src["v_l"].as<float>();
    if (src["v_r"].is<float>()) dst.v_r = src["v_r"].as<float>();
    if (src["v_l_enc"].is<int>()) dst.v_l_enc = src["v_l_enc"].as<int>();
    if (src["v_r_enc"].is<int>()) dst.v_r_enc = src["v_r_enc"].as<int>();
    if (src["v_l_enc_sin"].is<float>()) dst.v_l_enc_sin = src["v_l_enc_sin"].as<float>();
    if (src["v_r_enc_sin"].is<float>()) dst.v_r_enc_sin = src["v_r_enc_sin"].as<float>();
    if (src["accl"].is<float>()) dst.accl = src["accl"].as<float>();
    if (src["accl_x"].is<float>()) dst.accl_x = src["accl_x"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct2& dst) {
    if (src["ideal_w"].is<float>()) dst.ideal_w = src["ideal_w"].as<float>();
    if (src["w_lp"].is<float>()) dst.w_lp = src["w_lp"].as<float>();
    if (src["alpha"].is<float>()) dst.alpha = src["alpha"].as<float>();
    if (src["ideal_dist"].is<float>()) dst.ideal_dist = src["ideal_dist"].as<float>();
    if (src["dist"].is<float>()) dst.dist = src["dist"].as<float>();
    if (src["dist_kf"].is<float>()) dst.dist_kf = src["dist_kf"].as<float>();
    if (src["ideal_ang"].is<float>()) dst.ideal_ang = src["ideal_ang"].as<float>();
    if (src["ang"].is<float>()) dst.ang = src["ang"].as<float>();
    if (src["ang_kf"].is<float>()) dst.ang_kf = src["ang_kf"].as<float>();
    if (src["left90"].is<int>()) dst.left90 = src["left90"].as<int>();
    if (src["left45"].is<int>()) dst.left45 = src["left45"].as<int>();
    if (src["front"].is<int>()) dst.front = src["front"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct3& dst) {
    if (src["right45"].is<int>()) dst.right45 = src["right45"].as<int>();
    if (src["right90"].is<int>()) dst.right90 = src["right90"].as<int>();
    if (src["left90_d"].is<float>()) dst.left90_d = src["left90_d"].as<float>();
    if (src["left45_d"].is<float>()) dst.left45_d = src["left45_d"].as<float>();
    if (src["front_d"].is<float>()) dst.front_d = src["front_d"].as<float>();
    if (src["right45_d"].is<float>()) dst.right45_d = src["right45_d"].as<float>();
    if (src["right90_d"].is<float>()) dst.right90_d = src["right90_d"].as<float>();
    if (src["left90_far_d"].is<float>()) dst.left90_far_d = src["left90_far_d"].as<float>();
    if (src["front_far_d"].is<float>()) dst.front_far_d = src["front_far_d"].as<float>();
    if (src["right90_far_d"].is<float>()) dst.right90_far_d = src["right90_far_d"].as<float>();
    if (src["battery"].is<float>()) dst.battery = src["battery"].as<float>();
    if (src["duty_l"].is<float>()) dst.duty_l = src["duty_l"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct4& dst) {
    if (src["duty_r"].is<float>()) dst.duty_r = src["duty_r"].as<float>();
    if (src["motion_state"].is<int>()) dst.motion_state = src["motion_state"].as<int>();
    if (src["duty_sen"].is<float>()) dst.duty_sen = src["duty_sen"].as<float>();
    if (src["dist_mod90"].is<float>()) dst.dist_mod90 = src["dist_mod90"].as<float>();
    if (src["sen_dist_l45"].is<float>()) dst.sen_dist_l45 = src["sen_dist_l45"].as<float>();
    if (src["sen_dist_r45"].is<float>()) dst.sen_dist_r45 = src["sen_dist_r45"].as<float>();
    if (src["timestamp"].is<int>()) dst.timestamp = src["timestamp"].as<int>();
    if (src["sen_calc_time"].is<int>()) dst.sen_calc_time = src["sen_calc_time"].as<int>();
    if (src["sen_calc_time2"].is<int>()) dst.sen_calc_time2 = src["sen_calc_time2"].as<int>();
    if (src["pln_calc_time"].is<int>()) dst.pln_calc_time = src["pln_calc_time"].as<int>();
    if (src["pln_calc_time2"].is<int>()) dst.pln_calc_time2 = src["pln_calc_time2"].as<int>();
    if (src["pln_time_diff"].is<int>()) dst.pln_time_diff = src["pln_time_diff"].as<int>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct5& dst) {
    if (src["m_pid_p"].is<float>()) dst.m_pid_p = src["m_pid_p"].as<float>();
    if (src["m_pid_i"].is<float>()) dst.m_pid_i = src["m_pid_i"].as<float>();
    if (src["m_pid_i2"].is<float>()) dst.m_pid_i2 = src["m_pid_i2"].as<float>();
    if (src["m_pid_d"].is<float>()) dst.m_pid_d = src["m_pid_d"].as<float>();
    if (src["m_pid_p_v"].is<float>()) dst.m_pid_p_v = src["m_pid_p_v"].as<float>();
    if (src["m_pid_i_v"].is<float>()) dst.m_pid_i_v = src["m_pid_i_v"].as<float>();
    if (src["m_pid_i2_v"].is<float>()) dst.m_pid_i2_v = src["m_pid_i2_v"].as<float>();
    if (src["m_pid_d_v"].is<float>()) dst.m_pid_d_v = src["m_pid_d_v"].as<float>();
    if (src["g_pid_p"].is<float>()) dst.g_pid_p = src["g_pid_p"].as<float>();
    if (src["g_pid_i"].is<float>()) dst.g_pid_i = src["g_pid_i"].as<float>();
    if (src["g_pid_i2"].is<float>()) dst.g_pid_i2 = src["g_pid_i2"].as<float>();
    if (src["g_pid_d"].is<float>()) dst.g_pid_d = src["g_pid_d"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct6& dst) {
    if (src["g_pid_p_v"].is<float>()) dst.g_pid_p_v = src["g_pid_p_v"].as<float>();
    if (src["g_pid_i_v"].is<float>()) dst.g_pid_i_v = src["g_pid_i_v"].as<float>();
    if (src["g_pid_i2_v"].is<float>()) dst.g_pid_i2_v = src["g_pid_i2_v"].as<float>();
    if (src["g_pid_d_v"].is<float>()) dst.g_pid_d_v = src["g_pid_d_v"].as<float>();
    if (src["s_pid_p"].is<float>()) dst.s_pid_p = src["s_pid_p"].as<float>();
    if (src["s_pid_i"].is<float>()) dst.s_pid_i = src["s_pid_i"].as<float>();
    if (src["s_pid_i2"].is<float>()) dst.s_pid_i2 = src["s_pid_i2"].as<float>();
    if (src["s_pid_d"].is<float>()) dst.s_pid_d = src["s_pid_d"].as<float>();
    if (src["s_pid_p_v"].is<float>()) dst.s_pid_p_v = src["s_pid_p_v"].as<float>();
    if (src["s_pid_i_v"].is<float>()) dst.s_pid_i_v = src["s_pid_i_v"].as<float>();
    if (src["s_pid_i2_v"].is<float>()) dst.s_pid_i2_v = src["s_pid_i2_v"].as<float>();
    if (src["s_pid_d_v"].is<float>()) dst.s_pid_d_v = src["s_pid_d_v"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct7& dst) {
    if (src["ang_pid_p"].is<float>()) dst.ang_pid_p = src["ang_pid_p"].as<float>();
    if (src["ang_pid_i"].is<float>()) dst.ang_pid_i = src["ang_pid_i"].as<float>();
    if (src["ang_pid_d"].is<float>()) dst.ang_pid_d = src["ang_pid_d"].as<float>();
    if (src["ang_pid_p_v"].is<float>()) dst.ang_pid_p_v = src["ang_pid_p_v"].as<float>();
    if (src["ang_pid_i_v"].is<float>()) dst.ang_pid_i_v = src["ang_pid_i_v"].as<float>();
    if (src["ang_pid_d_v"].is<float>()) dst.ang_pid_d_v = src["ang_pid_d_v"].as<float>();
    if (src["ff_duty_front"].is<float>()) dst.ff_duty_front = src["ff_duty_front"].as<float>();
    if (src["ff_duty_roll"].is<float>()) dst.ff_duty_roll = src["ff_duty_roll"].as<float>();
    if (src["ff_duty_rpm_r"].is<float>()) dst.ff_duty_rpm_r = src["ff_duty_rpm_r"].as<float>();
    if (src["ff_duty_rpm_l"].is<float>()) dst.ff_duty_rpm_l = src["ff_duty_rpm_l"].as<float>();
    if (src["x"].is<float>()) dst.x = src["x"].as<float>();
    if (src["y"].is<float>()) dst.y = src["y"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct8& dst) {
    if (src["right45_2"].is<int>()) dst.right45_2 = src["right45_2"].as<int>();
    if (src["right45_3"].is<int>()) dst.right45_3 = src["right45_3"].as<int>();
    if (src["left45_2"].is<int>()) dst.left45_2 = src["left45_2"].as<int>();
    if (src["left45_3"].is<int>()) dst.left45_3 = src["left45_3"].as<int>();
    if (src["right45_2_d"].is<float>()) dst.right45_2_d = src["right45_2_d"].as<float>();
    if (src["right45_3_d"].is<float>()) dst.right45_3_d = src["right45_3_d"].as<float>();
    if (src["left45_2_d"].is<float>()) dst.left45_2_d = src["left45_2_d"].as<float>();
    if (src["left45_3_d"].is<float>()) dst.left45_3_d = src["left45_3_d"].as<float>();
    if (src["sen_dist_l45_2"].is<float>()) dst.sen_dist_l45_2 = src["sen_dist_l45_2"].as<float>();
    if (src["sen_dist_r45_2"].is<float>()) dst.sen_dist_r45_2 = src["sen_dist_r45_2"].as<float>();
    if (src["sen_dist_l45_3"].is<float>()) dst.sen_dist_l45_3 = src["sen_dist_l45_3"].as<float>();
    if (src["sen_dist_r45_3"].is<float>()) dst.sen_dist_r45_3 = src["sen_dist_r45_3"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct9& dst) {
    if (src["knym_v"].is<float>()) dst.knym_v = src["knym_v"].as<float>();
    if (src["knym_w"].is<float>()) dst.knym_w = src["knym_w"].as<float>();
    if (src["odm_x"].is<float>()) dst.odm_x = src["odm_x"].as<float>();
    if (src["odm_y"].is<float>()) dst.odm_y = src["odm_y"].as<float>();
    if (src["odm_theta"].is<float>()) dst.odm_theta = src["odm_theta"].as<float>();
    if (src["kim_x"].is<float>()) dst.kim_x = src["kim_x"].as<float>();
    if (src["kim_y"].is<float>()) dst.kim_y = src["kim_y"].as<float>();
    if (src["kim_theta"].is<float>()) dst.kim_theta = src["kim_theta"].as<float>();
    if (src["ang_i_bias"].is<float>()) dst.ang_i_bias = src["ang_i_bias"].as<float>();
    if (src["ang_i_bias_val"].is<float>()) dst.ang_i_bias_val = src["ang_i_bias_val"].as<float>();
    if (src["left90_d_diff"].is<float>()) dst.left90_d_diff = src["left90_d_diff"].as<float>();
    if (src["right90_d_diff"].is<float>()) dst.right90_d_diff = src["right90_d_diff"].as<float>();
}

inline void convertFromJson(JsonVariantConst src, LogStruct10& dst) {
    if (src["right45_3_d_diff"].is<float>()) dst.right45_3_d_diff = src["right45_3_d_diff"].as<float>();
    if (src["right45_2_d_diff"].is<float>()) dst.right45_2_d_diff = src["right45_2_d_diff"].as<float>();
    if (src["right45_d_diff"].is<float>()) dst.right45_d_diff = src["right45_d_diff"].as<float>();
    if (src["left45_d_diff"].is<float>()) dst.left45_d_diff = src["left45_d_diff"].as<float>();
    if (src["left45_2_d_diff"].is<float>()) dst.left45_2_d_diff = src["left45_2_d_diff"].as<float>();
    if (src["left45_3_d_diff"].is<float>()) dst.left45_3_d_diff = src["left45_3_d_diff"].as<float>();
    if (src["duty_suction"].is<float>()) dst.duty_suction = src["duty_suction"].as<float>();
    if (src["duty_roll"].is<float>()) dst.duty_roll = src["duty_roll"].as<float>();
    if (src["ang_kf_sum"].is<float>()) dst.ang_kf_sum = src["ang_kf_sum"].as<float>();
    if (src["img_ang_sum"].is<float>()) dst.img_ang_sum = src["img_ang_sum"].as<float>();
    if (src["duty_roll_before"].is<float>()) dst.duty_roll_before = src["duty_roll_before"].as<float>();
    if (src["reserve5"].is<int>()) dst.reserve5 = src["reserve5"].as<int>();
}

