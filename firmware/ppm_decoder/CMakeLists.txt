cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(ppm_decoder C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ppm_decoder
    main.cpp
)

pico_generate_pio_header(ppm_decoder ${CMAKE_CURRENT_LIST_DIR}/ppm_rx.pio)

target_link_libraries(ppm_decoder
    pico_stdlib
    hardware_pio
    hardware_clocks
)

# USB CDC only — Jetson reads decoded PPM over USB serial
pico_enable_stdio_usb(ppm_decoder  1)
pico_enable_stdio_uart(ppm_decoder 0)

pico_add_extra_outputs(ppm_decoder)

target_compile_options(ppm_decoder PRIVATE -Wall -Wextra)
