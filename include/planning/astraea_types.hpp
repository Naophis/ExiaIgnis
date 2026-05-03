#pragma once
// Astraea sample からの移植に必要な型定義。
// FreeRTOS / ESP-IDF 依存を除去し、標準 C++ + Pico SDK のみで動作するよう適合。
// 元ソース: sample/Astraea/include/{enums,structs,maze_solver}.hpp
//          sample/Astraea/model/gen_code_mpc/bus.h

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// ============================================================
// 定数 (maze_solver.hpp / defines.hpp から)
// ============================================================
// #ifndef ROOT2
// constexpr float ROOT2 = 1.41421356237f;
// #endif
// constexpr float dt     = 0.001f;        // 制御周期 [s]
// constexpr float m_PI   = 3.14159265358979f;
constexpr uint16_t RESET_GYRO_LOOP_CNT = 256;

#define ABS(x) ((x) < 0 ? -(x) : (x))
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static const std::string slalom_log_file("/log/sla.log");

// ============================================================
// 列挙型 (enums.hpp から)
// ============================================================
// enum class SensorCtrlType : int {
//     NONE     = 0,
//     Straight = 1,
//     Dia      = 2,
// };

// enum class RUN_MODE2 : int {
//     NONE_MODE   = 0,
//     KEEP        = 0,
//     SLAROM_RUN  = 1,
//     PIVOT_TURN  = 2,
//     ST_RUN      = 3,
//     SLALOM_RUN2 = 4,
//     FRONT_CTRL  = 5,
// };

// enum class MotionType : int {
//     NONE          = 0,
//     STRAIGHT      = 1,
//     PIVOT         = 2,
//     SLA_FRONT_STR = 3,
//     SLALOM        = 4,
//     BACK_STRAIGHT = 5,
//     WALL_OFF      = 6,
//     READY         = 7,
//     PIVOT_PRE     = 8,
//     PIVOT_PRE2    = 9,
//     PIVOT_AFTER   = 10,
//     FRONT_CTRL    = 11,
//     PIVOT_OFFSET  = 12,
//     WALL_OFF_DIA  = 13,
//     SLA_BACK_STR  = 14,
//     SYS_ID_PARA   = 15,
//     SYS_ID_ROLL   = 16,
//     SENSING_DUMP  = 17,
// };

// enum class MotionResult : int {
//     NONE              = 0,
//     ERROR             = 1,
//     WALL_OFF_DETECTED = 2,
// };

// enum class MotionDirection : int {
//     NONE  = 0,
//     RIGHT = 1,
//     LEFT  = 2,
// };

// enum class WallOffReq : int {
//     NONE     = 0,
//     SEARCH   = 1,
//     FAST_RUN = 2,
// };

// enum class WallCtrlMode : int {
//     NONE       = 0,
//     LEFT_ONLY  = 1,
//     RIGHT_ONLY = 2,
//     BOTH       = 3,
// };

// enum class CalibrationMode : int {
//     NONE  = 0,
//     DOING = 1,
// };

// enum class FailSafe : int {
//     NONE  = 0,
//     ERROR = 1,
// };

// // maze_solver.hpp
// enum class TurnDirection : int {
//     None     = 0,
//     Right    = 1,
//     Left     = 2,
//     Pivot180 = 128,
//     End      = 256,
// };

// enum class TurnType : int {
//     None     = 0,
//     Normal   = 1,
//     Orval    = 2,
//     Large    = 3,
//     Dia45    = 4,
//     Dia135   = 5,
//     Dia90    = 6,
//     Kojima   = 7,
//     Dia45_2  = 8,
//     Dia135_2 = 9,
//     Finish   = 255,
// };

// enum class StraightType : int {
//     Search     = 0,
//     FastRun    = 1,
//     FastRunDia = 2,
// };

// enum class Direction : int {
//     North     = 1,
//     East      = 2,
//     NorthEast = 3,
//     West      = 4,
//     NorthWest = 5,
//     SouthEast = 6,
//     SouthWest = 7,
//     South     = 8,
//     Undefined = 255,
//     Null      = 0,
// };

// // ============================================================
// // bus.h から (MPC 生成コードの構造体)
// // ============================================================
// typedef struct {
//     float limit;
//     float n;
//     int   decel_delay_cnt;
//     float decel_delay_n;
// } t_accl_param;

// typedef struct {
//     float v_max;
//     float end_v;
//     float accl;
//     float decel;
//     float w_max;
//     float end_w;
//     float alpha;
//     float tgt_dist;
//     float tgt_angle;
//     int   trajectory_point_size;
//     t_accl_param accl_param;
//     float slip_gain;
//     float limit_accl_ratio_cnt;
//     float limit_decel_ratio_cnt;
//     float slip_gain_K1;
//     float slip_gain_K2;
//     int   time_step2;
//     float axel_degenerate_gain;
//     char  enable_slip_decel;
// } t_tgt;

// typedef struct {
//     float base_alpha;
//     float base_time;
//     float limit_time_count;
//     float pow_n;
//     int   state;
//     int   counter;
// } t_slalom;

// typedef struct {
//     float v;
//     float v_r;
//     float v_l;
//     float pos_x;
//     float pos_y;
//     float ideal_px;
//     float ideal_py;
//     float accl;
//     float w;
//     float alpha;
//     float alpha2;
//     float dist;
//     float ang;
//     float img_dist;
//     float img_ang;
//     t_slalom sla_param;
//     int   state;
//     int   pivot_state;
//     float delay_accl;
//     float delay_v;
//     float ff_duty_l;
//     float ff_duty_r;
//     float ff_duty_front;
//     float ff_duty_roll;
//     float ff_duty_rpm_r;
//     float ff_duty_rpm_l;
//     int   decel_delay_cnt;
// } t_ego;

// // ============================================================
// // maze_solver.hpp から
// // ============================================================
// typedef struct {
//     unsigned char x;
//     unsigned char y;
// } point_t;

// typedef struct {
//     float x;
//     float y;
//     float ang;
//     Direction dir;
// } ego_odom_t;

// // ============================================================
// // structs.hpp から (motion_planning.cpp で使用するフィールドのみ)
// // ============================================================

// typedef struct {
//     float sensor_dist    = 300.0f;
//     float global_run_dist = 0.0f;
//     float angle          = 0.0f;
// } sen_log_t;

// typedef struct {
//     sen_log_t l90;
//     sen_log_t l45;
//     sen_log_t l45_2;
//     sen_log_t l45_3;
//     sen_log_t r45;
//     sen_log_t r45_2;
//     sen_log_t r45_3;
//     sen_log_t r90;
// } sen_logs_t;

// typedef struct {
//     int raw  = 0;
//     float data = 0.0f;
// } sensing_data_t;

// typedef struct {
//     // 距離 [mm] (要キャリブレーション変換; デフォルト 300 = 壁なし)
//     volatile float left45_dist     = 300.0f;
//     volatile float right45_dist    = 300.0f;
//     volatile float front_dist      = 300.0f;
//     volatile float left90_dist     = 300.0f;
//     volatile float right90_dist    = 300.0f;
//     volatile float left90_mid_dist = 300.0f;
//     volatile float right90_mid_dist= 300.0f;
//     volatile float front_mid_dist  = 300.0f;
//     volatile float left45_2_dist   = 300.0f;
//     volatile float right45_2_dist  = 300.0f;
//     volatile float left45_3_dist   = 300.0f;
//     volatile float right45_3_dist  = 300.0f;
//     volatile float temp            = 0.0f;
//     float battery_raw              = 0.0f;
// } ego_entity_t;

// typedef struct {
//     ego_entity_t  ego;
//     sensing_data_t gyro;
//     sensing_data_t gyro2;
//     sensing_data_t accel_x;
//     sensing_data_t accel_y;
//     sen_logs_t    sen;
// } sensing_result_entity_t;

// typedef struct {
//     float right45;
//     float left45;
//     float right90;
//     float left90;
//     float front;
//     float kireme_r;
//     float kireme_l;
//     float kireme_r_fast;
//     float kireme_l_fast;
//     float kireme_r_wall_off;
//     float kireme_l_wall_off;
//     float kireme_r_wall_off2;
//     float kireme_l_wall_off2;
// } sen_ref_param3_t;

// typedef struct {
//     float front;
//     float right45;
//     float left45;
//     float right90;
//     float left90;
//     float kireme_r;
//     float kireme_l;
//     float offset_r;
//     float offset_l;
//     float front_ctrl;
//     float front_ctrl_th;
// } sen_search_param_t;

// typedef struct {
//     sen_ref_param3_t ref;
//     sen_ref_param3_t ref_search;
//     sen_ref_param3_t exist;
// } sen_ref_param2_t;

// typedef struct {
//     sen_ref_param2_t normal;
//     sen_ref_param2_t normal2;
//     sen_ref_param2_t dia;
//     sen_search_param_t search_exist;
//     sen_search_param_t search_ref;
// } sen_ref_param_t;

// typedef struct {
//     float left_str               = 0;
//     float right_str              = 0;
//     float left_diff_th           = 0;
//     float right_diff_th          = 0;
//     float left_str_exist         = 0;
//     float right_str_exist        = 0;
//     float left_dia               = 0;
//     float right_dia              = 0;
//     float left_dia_noexit        = 0;
//     float right_dia_noexit       = 0;
//     float left_dia_oppo          = 0;
//     float right_dia_oppo         = 0;
//     float left_dia2              = 0;
//     float right_dia2             = 0;
//     float exist_dist_l           = 0;
//     float exist_dist_r           = 0;
//     float exist_dist_l2          = 0;
//     float exist_dist_r2          = 0;
//     float noexist_th_l           = 150.0f;
//     float noexist_th_r           = 150.0f;
//     float noexist_th_l2          = 150.0f;
//     float noexist_th_r2          = 150.0f;
//     float div_th_l               = 0;
//     float div_th_r               = 0;
//     float div_th_l2              = 0;
//     float div_th_r2              = 0;
//     float div_th_l3              = 0;
//     float div_th_r3              = 0;
//     float div_th_dia_l           = 0;
//     float div_th_dia_r           = 0;
//     float exist_dia_th_l         = 0;
//     float exist_dia_th_r         = 0;
//     float exist_dia_th_l2        = 0;
//     float exist_dia_th_r2        = 0;
//     float noexist_dia_th_l       = 150.0f;
//     float noexist_dia_th_r       = 150.0f;
//     float noexist_dia_th_l2      = 150.0f;
//     float noexist_dia_th_r2      = 150.0f;
//     float wall_off_exist_wall_th_l    = 0;
//     float wall_off_exist_wall_th_r    = 0;
//     float wall_off_exist_dia_wall_th_l = 0;
//     float wall_off_exist_dia_wall_th_r = 0;
//     bool  search_wall_off_enable = false;
//     float search_wall_off_l_dist_offset = 0;
//     float search_wall_off_r_dist_offset = 0;
//     float search_wall_off_offset_dist   = 0;
//     float ctrl_exist_wall_th_l   = 0;
//     float ctrl_exist_wall_th_r   = 0;
//     float go_straight_wide_ctrl_th = 60.0f;
//     float diff_check_dist        = 20.0f;
//     float diff_dist_th_l         = 20.0f;
//     float diff_dist_th_r         = 20.0f;
//     float diff_check_dist_dia    = 15.0f;
//     float diff_check_dist_dia_2  = 5.0f;
// } wall_off_hold_dist_t;

// typedef struct {
//     float gyro_w_gain_right = 0;
//     float gyro_w_gain_left  = 0;
//     float retry_min_th      = 0;
//     float retry_max_th      = 0;
//     float robust_th         = 0;
//     float lp_delay          = 0;
//     int   list_size         = 256;
//     int   loop_size         = 10;
// } gyro_param_t;

// typedef struct {
//     int   time_stamp         = 0;
//     int   error_gyro_reset   = 0;
//     int   error_vel_reset    = 0;
//     int   error_led_reset    = 0;
//     int   error_ang_reset    = 0;
//     int   error_dist_reset   = 0;
// } planning_req_t;

// typedef struct {
//     int error = 0;
// } fail_safe_state_t;

// typedef struct {
//     volatile float img_dist = 0;
//     volatile float img_ang  = 0;
//     volatile float dist     = 0;
//     volatile float ang      = 0;
// } global_ego_pos_t;

// typedef struct {
//     volatile float right_v;
//     volatile float left_v;
//     volatile bool  enable = false;
// } sys_id_t;

// typedef struct {
//     volatile float     v_max        = 0;
//     volatile float     v_end        = 0;
//     volatile float     accl         = 0;
//     volatile float     decel        = 0;
//     volatile float     dist         = 0;
//     volatile float     w_max        = 0;
//     volatile float     w_end        = 0;
//     volatile float     alpha        = 0;
//     volatile float     ang          = 0;
//     volatile float     sla_alpha    = 0;
//     volatile float     sla_time     = 0;
//     volatile float     sla_pow_n    = 0;
//     volatile float     sla_rad      = 0;
//     volatile float     dia90_offset = 0;
//     volatile TurnDirection  td      = TurnDirection::None;
//     volatile TurnType       tt      = TurnType::None;
//     volatile RUN_MODE2 motion_mode  = RUN_MODE2::NONE_MODE;
//     volatile MotionType motion_type = MotionType::NONE;
//     volatile int       timstamp     = 0;
//     MotionDirection    motion_dir   = MotionDirection::RIGHT;
//     volatile bool      dia_mode     = false;
//     SensorCtrlType     sct          = SensorCtrlType::NONE;
//     sys_id_t           sys_id;
//     volatile bool      tgt_reset_req  = false;
//     volatile bool      ego_reset_req  = false;
// } new_motion_req_t;

// typedef struct {
//     t_tgt              tgt_in;
//     t_ego              ego_in;
//     volatile int16_t   calc_time      = 0;
//     volatile int16_t   calc_time2     = 0;
//     volatile int16_t   calc_time_diff = 0;
//     volatile global_ego_pos_t global_pos;
//     volatile int32_t   motion_mode    = 0;
//     MotionType         motion_type    = MotionType::NONE;
//     MotionDirection    motion_dir     = MotionDirection::RIGHT;
//     volatile bool      dia_mode       = false;
//     planning_req_t     pl_req;
//     fail_safe_state_t  fss;
//     volatile float     gyro_zero_p_offset  = 0;
//     volatile float     var_unbiased_dps2   = 0;
//     volatile float     var_robust_dps2     = 0;
//     volatile int       gyro_retry          = 0;
//     volatile CalibrationMode calibration_mode = CalibrationMode::NONE;
//     volatile float     gyro2_zero_p_offset    = 0;
//     volatile float     accel_x_zero_p_offset  = 0;
//     volatile float     accel_y_zero_p_offset  = 0;
//     volatile float     temp_zero_p_offset     = 0;
//     new_motion_req_t   nmr;
//     float              v_error       = 0;
//     float              w_error       = 0;
//     TurnDirection      td            = TurnDirection::None;
//     TurnType           tt            = TurnType::None;
//     float              duty_suction  = 0;
// } motion_tgt_val_t;

// // ============================================================
// // motion_planning で使う入力パラメータ
// // ============================================================
// typedef struct {
//     float v_max  = 0;
//     float v_end  = 0;
//     float accl   = 0;
//     float decel  = 0;
//     float dist   = 0;
//     MotionType   motion_type  = MotionType::NONE;
//     SensorCtrlType sct        = SensorCtrlType::NONE;
//     WallOffReq   wall_off_req = WallOffReq::NONE;
//     WallCtrlMode wall_ctrl_mode = WallCtrlMode::NONE;
//     volatile float wall_off_dist_r  = 0;
//     volatile float wall_off_dist_l  = 0;
//     volatile bool  dia_mode         = false;
//     volatile bool  skil_wall_off    = false;
//     volatile bool  search_str_wide_ctrl_r = false;
//     volatile bool  search_str_wide_ctrl_l = false;
//     volatile float dia90_offset     = 0;
// } param_straight_t;

// typedef struct {
//     volatile float w_max = 0;
//     volatile float w_end = 0;
//     volatile float alpha = 0;
//     volatile float ang   = 0;
//     TurnDirection RorL   = TurnDirection::None;
// } param_roll_t;

// typedef struct {
//     float radius = 0;
//     float v_max  = 0;
//     float v_end  = 0;
//     float ang    = 0;
//     TurnDirection RorL = TurnDirection::None;
// } param_normal_slalom_t;

// typedef struct {
//     float right = 0;
//     float left  = 0;
// } slalom_offset_t;

// typedef struct {
//     float v       = 0;
//     float end_v   = 0;
//     float ang     = 0;
//     float ref_ang = 0;
//     float rad     = 0;
//     float rad2    = 0;
//     slalom_offset_t front;
//     slalom_offset_t back;
//     int   pow_n = 0;
//     float time  = 0;
//     float time2 = 0;
//     TurnType type = TurnType::None;
// } slalom_param2_t;

// typedef struct {
//     bool  is_turn        = false;
//     TurnType next_turn_type = TurnType::None;
//     float v_max          = 0;
//     float v_end          = 0;
//     float accl           = 0;
//     float decel          = 0;
//     bool  skip_wall_off  = false;
//     float carry_over_dist = 0;
// } next_motion_t;

// typedef struct {
//     float v_max  = 0;
//     float accl   = 0;
//     float decel  = 0;
//     float w_max  = 0;
//     float w_end  = 0;
//     float alpha  = 0;
// } straight_param_t;

// typedef struct {
//     std::unordered_map<TurnType, slalom_param2_t> map;
//     std::unordered_map<TurnType, slalom_param2_t> map_slow;
//     std::unordered_map<TurnType, slalom_param2_t> map_fast;
//     std::unordered_map<StraightType, straight_param_t> str_map;
//     char  suction      = 0;
//     float suction_duty = 0;
//     float suction_duty_low = 0;
//     float cell_size    = 90.0f;
//     float start_offset = 16.0f;
// } param_set_t;

// // ============================================================
// // input_param_t (motion_planning.cpp で参照するフィールドのみ)
// // ============================================================
// typedef struct {
//     sen_ref_param_t     sen_ref_p;
//     wall_off_hold_dist_t wall_off_dist;
//     gyro_param_t        gyro_param;

//     float lim_angle                  = 0;
//     float dia_turn_offset_calc_th    = 52.0f;
//     float front_dist_offset          = 0;
//     float front_dist_offset2         = 0;
//     float front_dist_offset_pivot_th = 0;
//     float sla_front_ctrl_th          = 110.0f;
//     float th_offset_dist             = 58.0f;
//     float normal_sla_offset_front    = 4.0f;
//     float normal_sla_offset_back     = 4.0f;
//     float sla_wall_ref_l             = 45.0f;
//     float sla_wall_ref_r             = 45.0f;
//     float offset_after_turn_r        = 0;
//     float offset_after_turn_l        = 0;
//     float offset_after_turn_r2       = 0;
//     float offset_after_turn_l2       = 0;
//     float offset_after_turn_dia_r    = 0;
//     float offset_after_turn_dia_l    = 0;
//     float orval_front_ctrl_min       = 40.0f;
//     float orval_front_ctrl_max       = 130.0f;
//     bool  orval_offset_enable        = false;
//     bool  large_offset_enable        = false;
//     bool  dia45_offset_enable        = false;
//     bool  dia135_offset_enable       = false;
//     float orval_offset_max_dist      = 0;
//     float large_offset_max_dist      = 5.0f;
//     float dia45_offset_max_dist      = 0;
//     float dia135_offset_max_dist     = 0;
//     float cell                       = 90.0f;
//     float cell2                      = 90.0f;
//     float long_run_offset_dist       = 5.0f;
//     float offset_start_dist          = 0;
//     int   fast_log_enable            = 0;
//     float normal_sla_l_wall_off_th_in  = 100.0f;
//     float normal_sla_r_wall_off_th_in  = 100.0f;
//     float normal_sla_l_wall_off_th_out = 100.0f;
//     float normal_sla_r_wall_off_th_out = 100.0f;
//     float normal_sla_l_wall_off_ref_cnt = 100.0f;
//     float normal_sla_r_wall_off_ref_cnt = 100.0f;
//     float normal_sla_l_wall_off_dist   = 5.0f;
//     float normal_sla_r_wall_off_dist   = 5.0f;
//     float normal_sla_l_wall_off_margin = 10.0f;
//     float normal_sla_r_wall_off_margin = 10.0f;
//     float dia90_offset               = 0;
//     float sensor_range_min           = 5.0f;
//     float sensor_range_max           = 180.0f;
//     float sensor_range_mid_max       = 150.0f;
//     float sensor_range_far_max       = 150.0f;
//     float logging_time               = 4.0f;
//     float front_ctrl_error_th        = 4.0f;
//     float wall_off_pass_dist         = 10.0f;
//     bool  set_param                  = false;
//     float pivot_back_offset          = 0;
//     float pivot_straight             = 43.0f;
//     float dist_mod_num               = 90.0f;
//     float clear_dist_ragne_to2       = 45.0f;  // スペルは元コードのまま
// } input_param_t;

// クラス定義は include/action/ 以下の専用ヘッダーに移動済み
