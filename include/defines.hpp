#pragma once

#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <cstdint>
#include "gen_code_conv_single2half/bus.h"
#include "include/maze_solver.hpp"

#include "include/enums.hpp"
#include "include/structs.hpp"
#include <bitset>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

// #define portTICK_RATE_MS portTICK_PERIOD_MS
// #define xTaskHandle TaskHandle_t

#define ABS(IN) ((IN) < 0 ? -(IN) : (IN))

// constexpr int GY_DQ_SIZE = 2;
// constexpr int GY_CYCLE = 2500; // 2500=1/4msec
// // constexpr int GY_CYCLE = 1250; // 1250=1/8msec
// constexpr int GY_MODE = 0;
// constexpr float cell_size = 90;

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// constexpr int16_t ENCODER_H_LIM_VAL = 32767;
// constexpr int16_t ENCODER_L_LIM_VAL = -32767;

const int16_t ENC_RESOLUTION = 16384 - 1;
// constexpr uint8_t READ_FLAG = 0x80;
// constexpr uint16_t READ_FLAG2 = 0b01000000;
// constexpr uint16_t PARITY_FLAG = 0b10000000;
// constexpr uint8_t ESC = 0x1B;
// constexpr uint16_t BUF_SIZE = 8192;

// constexpr uint16_t MOTION_CHECK_TH = 1000;
// constexpr uint16_t ENC_OPE_V_R_TH = 90 * 1;

// constexpr uint16_t LOG_SIZE = 1300;
// constexpr uint16_t LINE_BUF_SIZE = 1024;
// constexpr float BATTERY_GAIN = 3.503919806809492;

// constexpr uint8_t LEDC_HIGH_SPEED_MODE = 0;
// constexpr float LOW_BATTERY_TH = 11.8;

// constexpr uint16_t RESET_GYRO_LOOP_CNT = 256;

// static const std::string slalom_log_file("/spiflash/sla.log");
// static const std::string sysid_log_file("/spiflash/sysid.log");
// static const std::string maze_log_file("/spiflash/maze.txt");
// static const std::string maze_log_kata_file("/spiflash/maze_kata.log");
// static const std::string maze_log_return_file("/spiflash/maze_return.log");

// static const std::string format1("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%0.3f,"
//                                  "%0.3f,%0.3f,%0.3f,");
// static const std::string
//     format2("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
// static const std::string format3("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0."
//                                  "3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,"
//                                  "%0.3f,%d,");
// static const std::string format4("%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,%d,%d,");
// static const std::string format5(
//     "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
// static const std::string
//     format6("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,"
//             "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
// static const std::string format7("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n");
// static const std::string
//     formatsysid("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n");
// static char line_buf[LINE_BUF_SIZE];

// std::shared_ptr<sensing_result_entity_t> get_sensing_entity();

#endif