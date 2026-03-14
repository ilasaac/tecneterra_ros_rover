cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(rc_link_sx1278_slave C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rc_link_sx1278_slave
    main.cpp
)

pico_generate_pio_header(rc_link_sx1278_slave ${CMAKE_CURRENT_LIST_DIR}/ppm_tx.pio)

target_link_libraries(rc_link_sx1278_slave
    pico_stdlib
    pico_stdio_usb
    hardware_pio
    hardware_dma
    hardware_spi
    hardware_gpio
    hardware_irq
    hardware_clocks
)

pico_enable_stdio_usb(rc_link_sx1278_slave  1)
pico_enable_stdio_uart(rc_link_sx1278_slave 0)

pico_add_extra_outputs(rc_link_sx1278_slave)

target_compile_options(rc_link_sx1278_slave PRIVATE -Wall -Wextra)
