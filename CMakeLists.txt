# CMakeLists.txt - Integrated Robot System

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pico SDK setup
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(robo_pico C CXX ASM)

pico_sdk_init()

# ============================================================================
# THIS IS THE CRITICAL PART YOU WERE MISSING!
# Tell CMake where to find the header files (.h files)
# ============================================================================
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration
    ${CMAKE_CURRENT_SOURCE_DIR}/control_algo
    ${CMAKE_CURRENT_SOURCE_DIR}/hardware_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/main_program
)

# ============================================================================
# INTEGRATED SYSTEM - Main executable
# ============================================================================
add_executable(main
    main_program/main.c
    
    # Hardware drivers
    hardware_drivers/motor.c
    hardware_drivers/encoder.c
    hardware_drivers/imu.c
    hardware_drivers/ir_sensor.c
    hardware_drivers/ultrasonic.c
    hardware_drivers/servo.c
    hardware_drivers/calibration.c
    
    # Control algorithms
    control_algo/pid.c
    control_algo/heading_control.c
    control_algo/line_following.c
    control_algo/obstacle_scanner.c
    control_algo/avoidance_maneuver.c
    control_algo/obstacle_control.c
    
    # Utilities
    utilities/timer_manager.c
    utilities/telemetry.c
)

target_link_libraries(main
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_adc
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)