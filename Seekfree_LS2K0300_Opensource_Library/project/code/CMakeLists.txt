cmake_minimum_required(VERSION 3.16)
project(three_wheel_no_servo_integrated CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 说明:
# 1. 这里只编译当前工程自己的源码。
# 2. 逐飞 SDK 头文件与库路径，请按你的板端环境自行补充。
# 3. 如果你的环境里 zf_driver_encoder 不是预编译库，而是源码文件，
#    请把对应的 .cpp 再加入 add_executable()。
# -----------------------------------------------------------------------------
add_executable(three_wheel_no_servo_integrated
    main.cpp
    uvc_camera.cpp
    vision_uart.cpp
    line_track.cpp
    control_utils.cpp
    seekfree_hal.cpp
)

target_include_directories(three_wheel_no_servo_integrated PRIVATE .)
