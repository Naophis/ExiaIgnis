# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# ---- cJSON ----
include(FetchContent)
FetchContent_Declare(cjson
    GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
    GIT_TAG        v1.7.18
)
set(ENABLE_CJSON_TEST   OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS   OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cjson)

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

# Add executable. Default name is the project name, version 0.1

add_executable(ExiaIgnis
    src/main.cpp
    src/sensing_task.cpp
    src/planning/planning_task.cpp
    src/main/main_task.cpp
    src/logging/logging_task.cpp
    src/ui.cpp
    src/driver/asm330lhh.cpp
    src/driver/as5147p.cpp
    src/driver/ads7042.cpp
    src/config_loader.cpp
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
        ${cjson_SOURCE_DIR}
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
        pico_multicore
        pico_flash
        cjson
        littlefs
        )

pico_add_extra_outputs(ExiaIgnis)

