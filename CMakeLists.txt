# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g -std=c++17") 

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ExiaIgnis C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ---- ArduinoJson ----
include(FetchContent)
FetchContent_Declare(ArduinoJson
    GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
    GIT_TAG        v7.0.4
)
FetchContent_MakeAvailable(ArduinoJson)

# ---- LittleFS ----
FetchContent_Declare(littlefs
    GIT_REPOSITORY https://github.com/littlefs-project/littlefs.git
    GIT_TAG        v2.5.0
)
FetchContent_GetProperties(littlefs)
if(NOT littlefs_POPULATED)
    FetchContent_Populate(littlefs)
    add_library(littlefs STATIC
        ${littlefs_SOURCE_DIR}/lfs.c
        ${littlefs_SOURCE_DIR}/lfs_util.c
    )
    target_include_directories(littlefs PUBLIC ${littlefs_SOURCE_DIR})
    # LFS_NO_MALLOC must NOT be defined so lfs_file_open() is available
endif()

# ---- SparkFun PSRAM (sfe_psram only, no allocator/tlsf) ----
FetchContent_Declare(sparkfun_pico_src
    GIT_REPOSITORY https://github.com/sparkfun/sparkfun-pico.git
    GIT_TAG        main
)
FetchContent_GetProperties(sparkfun_pico_src)
if(NOT sparkfun_pico_src_POPULATED)
    FetchContent_Populate(sparkfun_pico_src)
    add_library(sfe_psram STATIC
        ${sparkfun_pico_src_SOURCE_DIR}/sparkfun_pico/sfe_psram.c
    )
    target_include_directories(sfe_psram PUBLIC
        ${sparkfun_pico_src_SOURCE_DIR}/sparkfun_pico
    )
    target_link_libraries(sfe_psram PUBLIC
        pico_stdlib hardware_gpio hardware_clocks hardware_sync hardware_spi pico_flash
    )
endif()

# Add executable. Default name is the project name, version 0.1

add_executable(ExiaIgnis
    src/main.cpp
    src/sensing_task.cpp
    src/planning/planning_task.cpp
    src/planning/motor_actuator.cpp
    src/planning/ego_estimator.cpp
    src/planning/sensor_processor.cpp
    src/planning/trajectory_generator.cpp
    src/planning/control_law.cpp
    src/action/motion_planning.cpp
    src/action/wall_off_controller.cpp
    src/action/trajectory_creator.cpp
    src/main/main_task.cpp
    src/main/main_task_usb.cpp
    src/main/main_task_util.cpp
    src/main/main_task_run.cpp
    src/main/main_task_test.cpp
    src/logging/logging_task.cpp
    src/ui.cpp
    src/driver/asm330lhh.cpp
    src/driver/as5147p.cpp
    src/driver/ads7042.cpp
    src/config_loader.cpp

    src/utils/kalman_filter.cpp
    src/utils/kalman_filter_matrix.cpp
    gen_code_conv_single2half/half_type.c

    gen_code_mpc/mpc_tgt_calc_data.cpp
    gen_code_mpc/mpc_tgt_calc.cpp
    gen_code_mpc/rtGetInf.cpp
    gen_code_mpc/rtGetNaN.cpp
    gen_code_mpc/rt_nonfinite.cpp
    gen_code_simple_pid/simple_pid_controller_data.cpp
    gen_code_simple_pid/simple_pid_controller.cpp

    src/search/adachi.cpp
    src/search/logic.cpp
    src/search/search_controller.cpp
)

pico_set_program_name(ExiaIgnis "ExiaIgnis")
pico_set_program_version(ExiaIgnis "0.1")

# Generate PIO header
pico_generate_pio_header(ExiaIgnis 
    ${CMAKE_CURRENT_LIST_DIR}/pio/blink.pio
)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ExiaIgnis 0)
pico_enable_stdio_usb(ExiaIgnis 1)

# Add the standard library to the build
target_link_libraries(ExiaIgnis
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(ExiaIgnis PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/include/search
        ${CMAKE_CURRENT_LIST_DIR}/gen_code_conv_single2half
        ${CMAKE_CURRENT_LIST_DIR}/gen_code_mpc
)

# Add any user requested libraries
target_link_libraries(ExiaIgnis
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_pwm
        hardware_clocks
        hardware_adc
        hardware_irq
        hardware_timer
        hardware_flash
        hardware_sync
        hardware_xip_cache
        pico_multicore
        pico_flash
        ArduinoJson
        littlefs
        sfe_psram
        )

# Core1 stack lives in SCRATCH_X (4KB hard limit; SCRATCH_Y is Core0 stack).
# Default 2KB overflows under deep IRQ chain. 4KB = SCRATCH_X limit, no collision.
# target_compile_definitions is required — set() is not propagated to -D flags by the SDK.
target_compile_definitions(ExiaIgnis PRIVATE PICO_CORE1_STACK_SIZE=0x1000)

pico_add_extra_outputs(ExiaIgnis)

