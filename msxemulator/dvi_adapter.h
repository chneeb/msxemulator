#pragma once
#include <stdint.h>
#include "tms9918/vrEmuTms9918.h"

#define DVI_FRAME_H  240
#define DVI_LINE_W   320  // half of 640; wren's encoder pixel-doubles automatically
#define V_MARGIN     24   // (240-192)/2 — vertical centering of 192 active lines
#define H_MARGIN     32   // (320-256)/2 — horizontal centering of 256 active pixels

// Initialize DVI hardware (pio0 + DMA) and launch the core1 DVI scan loop.
// Must be called after vreg_set_voltage() and set_sys_clock_khz(252000).
// Must NOT be preceded by board_init() (which resets the clock to 120MHz).
// active_screen starts NULL — core1 shows black until dvi_adapter_set_screen().
void dvi_adapter_init(void);

// Set the active TMS9918 screen rendered by core1.
// Call after vrEmuTms9918New() returns a valid pointer, and again at each
// VBlank to switch between mainscreen and menuscreen.
// Non-blocking; safe to call from core0 at any time.
void dvi_adapter_set_screen(VrEmuTms9918 *screen);

// Count of DVI frames where the TMDS queue was empty (stall events).
// Incremented by the DMA IRQ handler. Core0 can read this to detect DVI stalls.
extern volatile uint32_t dvi_stall_count;
