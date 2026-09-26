add_executable(msxemulator)

# audio_i2s.pio is only needed when USE_I2S is defined; kept for optional builds
pico_generate_pio_header(msxemulator ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(msxemulator PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/fatfs
    ${CMAKE_SOURCE_DIR}/PicoDVI/software/include
)

target_sources(msxemulator PRIVATE
    msxemulator.c Z80.c hid_app.c joystick.c hidparser/hidparser.c
    dvi_adapter.c nunchuck.c nes_controller.c
    sd_loader.c fatfs/ff.c fatfs/ffsystem.c fatfs/ffunicode.c fatfs/tf_card.c
    lfs.c lfs_util.c
    tms9918/vrEmuTms9918.c tms9918/vrEmuTms9918Util.c
    emu2149/emu2149.c emu2413/emu2413.c emu2212/emu2212.c
    fdc.c
)

pico_enable_stdio_usb(msxemulator 0)
pico_enable_stdio_uart(msxemulator 1)

target_compile_definitions(msxemulator PRIVATE
    Z80_STATIC Z80_WITH_LOCAL_HEADER Z80_WITH_Q Z80_WITH_ZILOG_NMOS_LD_A_IR_BUG
    PICO_STACK_SIZE=4096
    PICO_CORE1_STACK_SIZE=2048
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=4
    PICO_DEFAULT_UART_RX_PIN=5
    # More TMDS scanline buffers — each buffer is 3840 bytes; 8 total gives
    # ~507 µs of burst slack so sprite-heavy scanlines don't underrun the queue.
    DVI_N_TMDS_BUFFERS=8
    # Unroll the TMDS encode inner loop 2x — halves branch overhead per scanline.
    TMDS_ENCODE_UNROLL=2
    # RGB555 pixel format for wren's TMDS encoder (bit15=0, R[14:10], G[9:5], B[4:0])
    DVI_16BPP_RED_MSB=14
    DVI_16BPP_RED_LSB=10
    DVI_16BPP_GREEN_MSB=9
    DVI_16BPP_GREEN_LSB=5
    DVI_16BPP_BLUE_MSB=4
    DVI_16BPP_BLUE_LSB=0
)

target_link_libraries(msxemulator PRIVATE
    pico_stdlib hardware_pio hardware_timer hardware_dma hardware_uart
    hardware_flash pico_multicore hardware_pwm hardware_vreg hardware_i2c hardware_spi
    tinyusb_host tinyusb_board
    libdvi
)

# Redirect pico_util queue functions to SRAM-resident wrappers in dvi_adapter.c.
# Core1's DMA IRQ handler calls these during DVI rendering; if XIP is disabled
# on core0 (flash erase/program), any flash-resident call on core1 will stall.
# queue_try_peek is called from dvi_dma_irq_handler (libdvi/dvi.c:236) and is
# NOT __dvi_func — must be wrapped like the others.
target_link_options(msxemulator PRIVATE
    "-Wl,--wrap=queue_try_add"
    "-Wl,--wrap=queue_try_remove"
    "-Wl,--wrap=queue_try_peek"
    "-Wl,--wrap=queue_add_blocking"
    "-Wl,--wrap=queue_remove_blocking"
    # interp_save/restore are called per scanline by tmds_encode_data_channel_16bpp.
    "-Wl,--wrap=interp_save"
    "-Wl,--wrap=interp_restore"
)

pico_add_extra_outputs(msxemulator)
