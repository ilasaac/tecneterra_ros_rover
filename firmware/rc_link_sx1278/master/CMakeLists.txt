cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(rc_link_sx1278_master C CXX ASM)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ── Generate PIO headers from .pio files ──────────────────────────────────────
# pico_generate_pio_header() compiles each .pio file into a .pio.h header
# that exposes the program blob and the c-sdk init functions.

add_executable(rc_link_sx1278_master
    main.cpp
)

pico_generate_pio_header(rc_link_sx1278_master ${CMAKE_CURRENT_LIST_DIR}/sbus_rx.pio)
pico_generate_pio_header(rc_link_sx1278_master ${CMAKE_CURRENT_LIST_DIR}/ppm_tx.pio)

target_link_libraries(rc_link_sx1278_master
    pico_stdlib          # includes stdio framework; pico_enable_stdio_usb adds USB transport
    hardware_pio
    hardware_dma
    hardware_spi
    hardware_gpio
    hardware_irq
    hardware_clocks
)

# Route stdio to USB only (not UART)
pico_enable_stdio_usb(rc_link_sx1278_master  1)
pico_enable_stdio_uart(rc_link_sx1278_master 0)

# Generate UF2, bin, hex outputs for drag-and-drop flashing
pico_add_extra_outputs(rc_link_sx1278_master)

# Optional: print memory usage after build
target_compile_options(rc_link_sx1278_master PRIVATE -Wall -Wextra)
