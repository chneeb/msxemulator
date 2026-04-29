// dvi_adapter.c — PicoDVI (wren's libdvi) integration for msxemulator
//
// Architecture: core1 continuously renders DVI by calling vrEmuTms9918ScanLine
// directly on the active TMS9918 screen pointer (set by dvi_adapter_set_screen).
// No intermediate index_fb — core1 renders the TMS9918 state as it is at render
// time.  Minor tearing is acceptable; no sync with core0 is needed.
//
// Pixel format: RGB555 (bit15=0, R[14:10], G[9:5], B[4:0]).
// The DVI_16BPP_* compile-time overrides in CMakeLists.txt configure the
// wren TMDS encoder to use this layout instead of the default RGB565.

#include "dvi_adapter.h"  // includes tms9918/vrEmuTms9918.h
#include "dvi.h"
#include "dvi_timing.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"

#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/sync.h"

// TMS9918A 16-color palette in RGB555 (bit15=0, R[14:10], G[9:5], B[4:0])
// Source: TMS9918A datasheet + standard MSX colour approximations.
// __not_in_flash forces this into SRAM: core1 reads it every scanline, and XIP
// is disabled on core0 during flash_range_erase/program (the const/non-const
// distinction doesn't reliably prevent GCC from placing it in .rodata).
static const uint16_t tms_palette[16] __not_in_flash("tms_palette") = {
    0x0000, // 0:  Transparent  → black
    0x0000, // 1:  Black        (0,0,0)
    0x1A26, // 2:  Medium Green (33,200,66)  → R=4,  G=25, B=8
    0x2F77, // 3:  Light Green  (94,220,120) → R=11, G=27, B=15
    0x10BD, // 4:  Dark Blue    (84,85,237)  → R=10, G=10, B=29
    0x18FD, // 5:  Light Blue   (125,118,252)→ R=15, G=14, B=31
    0xD529, // 6:  Dark Red     (212,82,77)  → R=26, G=10, B=9
    0x23DE, // 7:  Cyan         (66,235,245) → R=8,  G=29, B=30
    0xF529, // 8:  Medium Red   (252,85,84)  → R=31, G=10, B=10
    0xFC8F, // 9:  Light Red    (255,121,120)→ R=31, G=15, B=15
    0xD626, // 10: Dark Yellow  (212,193,84) → R=26, G=24, B=10
    0xDE90, // 11: Light Yellow (230,206,128)→ R=28, G=25, B=16
    0x1A05, // 12: Dark Green   (33,176,59)  → R=4,  G=22, B=7
    0xC976, // 13: Magenta      (201,91,186) → R=25, G=11, B=23
    0xCE59, // 14: Gray         (204,204,204)→ R=25, G=25, B=25
    0x7FFF, // 15: White        (255,255,255)→ R=31, G=31, B=31
};

// Active TMS9918 screen pointer.  NULL until dvi_adapter_set_screen() is called
// (boot shows black until the BIOS has allocated and initialised the VDP).
// Written by core0, read by core1 — volatile to prevent caching.
static VrEmuTms9918 * volatile active_screen = NULL;

// Frame counter incremented by core1 on every completed DVI frame.
// Core0 can read this to detect whether core1 is still running.
volatile uint32_t dvi_stall_count = 0;

// Per-line palette-index scratch (256 bytes) and RGB555 scratch (320 pixels),
// used exclusively by core1.
static uint8_t  line_pal[256];
static uint16_t line_rgb[DVI_LINE_W];

// Static TMDS encode buffers — avoids heap allocation in dvi_init.
// Each buffer holds 3 channels × (640/DVI_SYMBOLS_PER_WORD) = 3×320 uint32_t words.
#define TMDS_BUF_WORDS (3 * (640 / DVI_SYMBOLS_PER_WORD))  // = 960
static uint32_t tmds_buf0[TMDS_BUF_WORDS];
static uint32_t tmds_buf1[TMDS_BUF_WORDS];
static uint32_t tmds_buf2[TMDS_BUF_WORDS];
static uint32_t tmds_buf3[TMDS_BUF_WORDS];
static uint32_t tmds_buf4[TMDS_BUF_WORDS];
static uint32_t tmds_buf5[TMDS_BUF_WORDS];
static uint32_t tmds_buf6[TMDS_BUF_WORDS];
static uint32_t tmds_buf7[TMDS_BUF_WORDS];
static uint32_t *tmds_bufs[DVI_N_TMDS_BUFFERS] = {
    tmds_buf0, tmds_buf1, tmds_buf2, tmds_buf3,
    tmds_buf4, tmds_buf5, tmds_buf6, tmds_buf7
};

static struct dvi_inst dvi0;

// Core1: own the DVI display loop.  For each DVI frame, render 240 lines by
// calling vrEmuTms9918ScanLine for the 192 active rows and showing black borders.
// Runs from __not_in_flash_func (SRAM) — safe alongside flash writes on core0.
static void __not_in_flash_func(core1_main)(void) {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    // No multicore_lockout_victim_init: core1's entire render path is in SRAM
    // (vrEmuTms9918ScanLine via __time_critical_func, queue ops via --wrap,
    //  tms_palette in .data, line_rgb/line_pal in .bss).  Core0 can therefore
    // disable XIP for flash erase/program without pausing core1.

    const uint words_per_channel = dvi0.timing->h_active_pixels / DVI_SYMBOLS_PER_WORD;
    const uint n_pix              = dvi0.timing->h_active_pixels / 2;

    while (true) {
        uint32_t frame_late = dvi0.late_scanline_ctr;
        for (int y = 0; y < DVI_FRAME_H; y++) {
            int fmsx_y = y - V_MARGIN;

            if (fmsx_y < 0 || fmsx_y >= 192 || active_screen == NULL) {
                // Border / pre-init — black (avoid memset: it calls flash-resident libc)
                for (int i = 0; i < DVI_LINE_W; i++) line_rgb[i] = 0;
            } else {
                // Active MSX row: render TMS9918 → palette indices → RGB555
                vrEmuTms9918ScanLine(active_screen, fmsx_y, line_pal);
                for (int i = 0; i < H_MARGIN; i++) line_rgb[i] = 0;
                /* Palette expand: 2 pixels per 32-bit store to halve store count */
                uint32_t *dst32 = (uint32_t *)(line_rgb + H_MARGIN);
                const uint8_t *pal_src = line_pal;
                for (int x = 0; x < 128; x++) {
                    uint32_t p0 = tms_palette[*pal_src++ & 0x0F];
                    uint32_t p1 = tms_palette[*pal_src++ & 0x0F];
                    *dst32++ = p0 | (p1 << 16);
                }
                uint16_t *dst = (uint16_t *)dst32;
                for (int i = 0; i < H_MARGIN; i++) *dst++ = 0;
            }

            // Acquire a free TMDS buffer and encode all three channels.
            uint32_t *tmdsbuf;
            queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmdsbuf);
            const uint32_t *row = (const uint32_t *)line_rgb;
            tmds_encode_data_channel_16bpp(row, tmdsbuf + 0 * words_per_channel,
                                           n_pix, DVI_16BPP_BLUE_MSB,  DVI_16BPP_BLUE_LSB);
            tmds_encode_data_channel_16bpp(row, tmdsbuf + 1 * words_per_channel,
                                           n_pix, DVI_16BPP_GREEN_MSB, DVI_16BPP_GREEN_LSB);
            tmds_encode_data_channel_16bpp(row, tmdsbuf + 2 * words_per_channel,
                                           n_pix, DVI_16BPP_RED_MSB,   DVI_16BPP_RED_LSB);
            queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmdsbuf);
        }
        // Accumulate late-scanline events so core0 can observe stalls.
        if (dvi0.late_scanline_ctr != frame_late)
            dvi_stall_count++;
    }
}

void dvi_adapter_init(void) {
    dvi0.timing  = &dvi_timing_640x480p_60hz;
    dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;   // waveshare_rp2040_pizero via CMake define
    // Use static TMDS buffers to avoid heap allocation (3×3840 bytes = 11.5KB)
    dvi_init_with_buffers(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num(),
                          tmds_bufs, DVI_N_TMDS_BUFFERS);

    // active_screen is NULL — core1 shows black until dvi_adapter_set_screen().
    multicore_launch_core1(core1_main);
}

// Switch the active TMS9918 screen rendered by core1.
// Call after vrEmuTms9918New() returns a valid pointer.
// Also used at VBlank to switch between mainscreen and menuscreen.
void dvi_adapter_set_screen(VrEmuTms9918 *screen) {
    active_screen = screen;
}

// -----------------------------------------------------------------------
// SRAM-resident queue wrappers (activated by --wrap linker flags).
//
// pico_util queue functions live in flash.  Core1's DMA IRQ handler calls
// queue_try_add/remove, and core1's render loop calls queue_add/remove_
// blocking.  If XIP is disabled on core0 (during flash_range_erase/program)
// while core1 is executing any of those flash-resident functions, core1
// will stall waiting for XIP — corrupting the DVI signal.
//
// These wrappers reimplement the same logic with __not_in_flash_func so the
// code lives in SRAM.  spin_lock_blocking / spin_unlock / queue_get_level_unsafe
// are all __force_inline / static inline and therefore also end up in SRAM.
//
// Element size is always sizeof(uint32_t) for the DVI TMDS queues; the casts
// below are safe for any queue with element_size == 4 (all queues in this build).
// -----------------------------------------------------------------------

// Advance ring-buffer index: the queue allocates element_count+1 slots.
static inline uint16_t __q_inc(const queue_t *q, uint16_t idx) {
    return (++idx > q->element_count) ? 0 : idx;
}

bool __not_in_flash_func(__wrap_queue_try_add)(queue_t *q, const void *data) {
    uint32_t save = spin_lock_blocking(q->core.spin_lock);
    bool ok = (queue_get_level_unsafe(q) != q->element_count);
    if (ok) {
        *(uint32_t *)(q->data + (uint32_t)q->wptr * q->element_size) = *(const uint32_t *)data;
        q->wptr = __q_inc(q, q->wptr);
        spin_unlock(q->core.spin_lock, save);
        __sev();
    } else {
        spin_unlock(q->core.spin_lock, save);
    }
    return ok;
}

bool __not_in_flash_func(__wrap_queue_try_peek)(queue_t *q, void *data) {
    uint32_t save = spin_lock_blocking(q->core.spin_lock);
    bool ok = (queue_get_level_unsafe(q) != 0);
    if (ok && data)
        *(uint32_t *)data = *(const uint32_t *)(q->data + (uint32_t)q->rptr * q->element_size);
    spin_unlock(q->core.spin_lock, save);
    return ok;
}

bool __not_in_flash_func(__wrap_queue_try_remove)(queue_t *q, void *data) {
    uint32_t save = spin_lock_blocking(q->core.spin_lock);
    bool ok = (queue_get_level_unsafe(q) != 0);
    if (ok) {
        if (data)
            *(uint32_t *)data = *(const uint32_t *)(q->data + (uint32_t)q->rptr * q->element_size);
        q->rptr = __q_inc(q, q->rptr);
        spin_unlock(q->core.spin_lock, save);
        __sev();
    } else {
        spin_unlock(q->core.spin_lock, save);
    }
    return ok;
}

void __not_in_flash_func(__wrap_queue_add_blocking)(queue_t *q, const void *data) {
    while (true) {
        uint32_t save = spin_lock_blocking(q->core.spin_lock);
        if (queue_get_level_unsafe(q) != q->element_count) {
            *(uint32_t *)(q->data + (uint32_t)q->wptr * q->element_size) = *(const uint32_t *)data;
            q->wptr = __q_inc(q, q->wptr);
            spin_unlock(q->core.spin_lock, save);
            __sev();
            return;
        }
        spin_unlock(q->core.spin_lock, save);
        __wfe();
    }
}

void __not_in_flash_func(__wrap_queue_remove_blocking)(queue_t *q, void *data) {
    while (true) {
        uint32_t save = spin_lock_blocking(q->core.spin_lock);
        if (queue_get_level_unsafe(q) != 0) {
            if (data)
                *(uint32_t *)data = *(const uint32_t *)(q->data + (uint32_t)q->rptr * q->element_size);
            q->rptr = __q_inc(q, q->rptr);
            spin_unlock(q->core.spin_lock, save);
            __sev();
            return;
        }
        spin_unlock(q->core.spin_lock, save);
        __wfe();
    }
}
