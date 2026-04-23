cmake_minimum_required(VERSION 3.16)
project(ls2k0300_seekfree_port_simplified CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ls2k0300_seekfree_port_simplified
    main.cpp
    uvc_camera.cpp
    vision_uart.cpp
    line_track.cpp
    control_utils.cpp
    seekfree_hal.cpp
    zf_driver_encoder.cpp
)

target_include_directories(ls2k0300_seekfree_port_simplified PRIVATE .)
