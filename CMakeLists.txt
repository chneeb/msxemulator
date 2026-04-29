# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

pico_sdk_init()

project(msxemulator)

set(DVI_DEFAULT_SERIAL_CONFIG waveshare_rp2040_pizero)
add_compile_definitions(
    DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
    # Place pico_mem_ops (memset/memcpy wrappers) in SRAM so core1's render
    # loop can call memset without stalling when XIP is disabled on core0.
    PICO_MEM_IN_RAM=1
)
add_subdirectory(PicoDVI/software/libdvi)

add_subdirectory(msxemulator)