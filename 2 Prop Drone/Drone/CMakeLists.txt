cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(2PropDrone)

pico_sdk_init()

add_executable(2PropDrone
    src/NRF24L01.cpp
    src/MPU6050.cpp
    src/BLDC.cpp
    src/SERVO.cpp
    src/main.cpp
)

target_include_directories(2PropDrone PUBLIC
    inc
)

target_link_libraries(2PropDrone
    pico_stdlib
    hardware_gpio
    hardware_spi
    hardware_i2c
    hardware_pwm
)

pico_enable_stdio_usb(2PropDrone 1)
pico_enable_stdio_uart(2PropDrone 0)

pico_add_extra_outputs(2PropDrone)
