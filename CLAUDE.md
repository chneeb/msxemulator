# msxemulator — RP2040-PiZero DVI Port

## Status: building; stable DVI output; SD ROM loading uses reboot-to-flash pattern

The port of `shippoiincho/msxemulator` to the Waveshare RP2040-PiZero board is done.
PIO-based VGA output has been replaced with PicoDVI (HDMI) using `Wren6991/PicoDVI`
as a git submodule.

## Build

```sh
cd /home/chneeb/Source/msxemulator/build
cmake .. -DPICO_SDK_PATH=/home/chneeb/Source/pico-sdk
make msxemulator
# flash msxemulator/msxemulator.uf2
```

BIOS arrays (`msxemulator/bios/msx.h`, `disk.h`, `fmpac.h`) are compiled in from
`/home/chneeb/Source/msx2pico/src/picomsx/bios/` (gitignored — regenerate with
`tools/rom2h.py` from that repo if needed).

## Board: Waveshare RP2040-PiZero

- `PICO_BOARD=pico`, SDK board: `pico`
- Clock: **252 MHz** (`vreg_set_voltage(VREG_VOLTAGE_1_20)` before clock change)
- **Do NOT call `board_init()`** — it resets the clock to 120 MHz. `tuh_init(0)` is safe.
- UART: **9600 baud** on uart1 / GPIO 4-5 (for Adafruit BT UART adapter on the Pi-Zero header)

### Pin mapping

| Function        | GPIO | Notes            |
|-----------------|------|------------------|
| DVI D0 (Blue)   | 26   | pair base        |
| DVI D1 (Green)  | 24   | pair base        |
| DVI D2 (Red)    | 22   | pair base        |
| DVI CLK         | 28   | pair base        |
| SD SCLK         | 18   | SPI0             |
| SD MOSI         | 19   | SPI0             |
| SD MISO         | 20   | SPI0             |
| SD CS           | 21   | SPI0             |
| PWM audio       | 6    |                  |
| UART TX         | 4    | uart1, 9600 baud |
| UART RX         | 5    | uart1, 9600 baud |
| Nunchuck SDA    | 2    | I2C1             |
| Nunchuck SCL    | 3    | I2C1             |
| USB D+/D−       | 15/16| hardware USB     |

## Architecture

### DVI (core1) — `dvi_adapter.c` / `dvi_adapter.h`

- Library: `Wren6991/PicoDVI` submodule at `PicoDVI/`; linked as `libdvi` (INTERFACE target)
- `DVI_DEFAULT_SERIAL_CONFIG=waveshare_rp2040_pizero` set in root `CMakeLists.txt`
- Pixel format: RGB555 — `DVI_16BPP_{RED,GREEN,BLUE}_{MSB,LSB}` set in `msxemulator/CMakeLists.txt`
  (propagates to libdvi because it is an INTERFACE library)
- `dvi_timing_640x480p_60hz` with `DVI_VERTICAL_REPEAT=2` (default) → 240 line pushes per frame
- `DVI_FRAME_H=240`, `DVI_LINE_W=320`, `V_MARGIN=24`, `H_MARGIN=32`

**core1 startup sequence (critical):**
```c
dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
dvi_start(&dvi0);          // called immediately — no wait for first buffer
multicore_lockout_victim_init();
// then enters continuous render loop
```
`dvi_start()` must be called unconditionally. `active_screen` starts NULL so core1
renders black lines (border) until `dvi_adapter_set_screen()` is called with a real
TMS9918 pointer. DVI sync is live from the moment core1 starts.

### HSync / VBlank timing — `msxemulator.c`

```c
add_repeating_timer_us(-64, hsync_cb, NULL, &timer);
```

`hsync_cb` fires every 64 µs. **262 ticks** per VBlank (≈ 59.6 Hz NTSC), NOT 313 (PAL 50 Hz).
Using 262 closely matches the DVI 60 Hz frame period (16.67 ms vs 16.77 ms).
With 313 ticks there is a 3.3 ms gap per frame where DVI shows solid-red error lines.

### Frame rendering

Core1 runs a continuous render loop (no `dvi_scanbuf_main_16bpp`). For each DVI
frame it calls `vrEmuTms9918ScanLine(active_screen, y, line_pal)` for the 192 active
rows and outputs black borders for the remaining lines, encoding directly with
`tmds_encode_data_channel_16bpp`. No persistent framebuffer; no `q_colour_valid`.

At VBlank, core0 calls `dvi_adapter_set_screen(screen)` — **non-blocking**, just
updates the volatile `active_screen` pointer that core1 reads on its next frame.

### Flash and multicore

Core1's entire **runtime** render path is SRAM-resident — no multicore lockout is needed.
Flash erase/program on core0 only requires `save_and_disable_interrupts` / `restore_interrupts`.

**Why no lockout:** `multicore_lockout` pauses core1 including its DMA IRQ handler. A 50ms
sector erase starves the TMDS queue after ~100 µs (3 buffers × 34.7 µs/line), causing the
monitor to show solid red. Since all code/data accessed by core1 at runtime is in SRAM, XIP
can be disabled without pausing core1.

**How core1's path is made SRAM-resident:**
- `vrEmuTms9918ScanLine` — SRAM via `__time_critical_func` (in `vrEmuTms9918.c`)
- `tms_palette[]` — SRAM via `__not_in_flash("tms_palette")` (const array in `dvi_adapter.c`)
- `memset` / `memcpy` — SRAM via `PICO_MEM_IN_RAM=1` in root `CMakeLists.txt`
  (makes `pico_mem_ops` `__wrap_memset` use `.time_critical` section)
- `queue_try_add/remove/peek`, `queue_add/remove_blocking` — SRAM via `--wrap` linker flags
  (SRAM-resident re-implementations in `dvi_adapter.c`; activated in `msxemulator/CMakeLists.txt`).
  **`queue_try_peek` must be wrapped**: `dvi_dma_irq_handler` (libdvi/dvi.c:236) calls
  `queue_try_peek_u32` → `queue_try_peek` which is flash-resident. Without the wrap, core1
  stalls mid-IRQ whenever XIP is disabled for `flash_range_program` → DVI PIO starves
  → solid red then black screen → monitor loses sync → stuck.
- `interp_save` / `interp_restore` — SRAM via `--wrap` (RAM copies in `dvi_adapter.c`);
  `tmds_encode_data_channel_16bpp` calls them on every encode (3× per scanline).
- `core1_main`, `dvi_dma_irq_handler`, `tmds_encode_data_channel_16bpp`, `dvi_update_scanline_data_dma` — SRAM via `__not_in_flash_func` / `__dvi_func`

Flash write sites use only `save_and_disable_interrupts` / `restore_interrupts` — no
`multicore_lockout_*`. For SD-sourced ROMs, flashing now happens pre-core1 (see ROM loading
section), so `save_and_disable_interrupts` there is a precaution only; core1 is not running.

**Flash erase strategy:** Both `cart_write` (LFS fallback) and `flash_rom_from_sd` (SD path)
use a **single 64KB block erase** (`flash_range_erase` with count rounded up to 65536) rather
than per-sector 4KB erases. This is because specific 4KB sector addresses (0xA4000, 0xC4000)
on the board's W25Q128 flash hang indefinitely with the 4KB erase command.

## Flash layout

| Region           | Address      | Size   | Content                         |
|------------------|--------------|--------|---------------------------------|
| Firmware + BIOS  | 0x10000000   | ≤640KB | msxemulator + MSX/DISK/FMPAC arrays |
| Cart slot 1      | 0x10060000   | 256KB  | Game ROM (flashed via menu)     |
| Cart slot 2      | 0x100E0000   | 128KB  | Game ROM (flashed via menu)     |
| LittleFS         | 0x10100000   | ~1MB   | Save states, tape images        |

`HW_FLASH_STORAGE_MEGABYTES=2`, `HW_SYSTEM_RESERVED=1024` (KB) in `msxemulator.h`.
`CART1BASE=0x10060000`, `CART2BASE=0x100E0000` defined in `msxrom.h`.

**Why these addresses:** Multiple flash blocks on this board are defective — page-program
hangs (WIP bit never clears):
- Block 10 (0xA0000): bad — original CART1 address
- Block 12 (0xC0000, specifically 0xC4000): bad — original CART2 address  
- Block 13 (0xD0000): page-program hangs after 2 successful erase/program cycles

CART1 at blocks 6–9 (0x60000) and CART2 at blocks 14–15 (0xE0000) are the next untouched
regions. SD-sourced ROM writes go through `flash_rom_from_sd()` (pre-core1); LFS-sourced
writes go through `cart_write()` (fallback path, still uses `cart_compare()` to skip
unchanged ROMs).

## ROM loading

### From SD card — reboot-to-flash pattern (primary)

SD card must be FAT/FAT32 formatted with ROMs in a `/msx/` subfolder:

```
SD card
└── msx/
    ├── game.rom
    ├── SELROM1      ← written by menu; full path of ROM selected for slot 1
    └── SELROM2      ← written by menu; full path of ROM selected for slot 2
```

FatFs driver is in `msxemulator/fatfs/` (write-enabled, LFN enabled).
`sd_loader.c` handles SD init (`spi0`, GPIO 18-21).

**SD load flow** (as of 2026-04-28):

1. **Menu** — user selects a ROM via the SD file browser (menu item 2 or 5). The menu
   writes the full SD path (e.g. `/msx/GAME.ROM`) into `/msx/SELROM1` (or `SELROM2`)
   on the SD card, then triggers a watchdog reboot (`watchdog_enable(1, 1)`).

2. **Boot** — `main()` runs `sd_init()` before `dvi_adapter_init()`. Before core1 starts,
   it checks `watchdog_enable_caused_reboot()` and reads `SELROM1`/`SELROM2`. If a watchdog
   boot, `flash_rom_from_sd()` streams the ROM from SD directly to `CART1BASE`/`CART2BASE`
   in 4KB chunks (64KB block erase, watchdog-guarded). Then core1 / DVI start normally.

3. **Non-watchdog reset** — `SELROM1`/`SELROM2` still exist from the previous session.
   Boot reads the file size from SD (no flash write) and runs `cart_type_checker()` to
   restore the mapper type. ROM is accessed via XIP pointer as before.

`flash_rom_from_sd()` is a static helper in `msxemulator.c` (near `cart_write`). It reuses
the global `flash_buffer[4096]` and the same 64KB erase strategy as `cart_write`.

### From LittleFS (no SD card)

If no SD card is present the menu falls back to the LittleFS file browser, which uses
the old `cart_write()` path (flash write while core1/DVI is live — acceptable as a
fallback since the queue_try_peek wrap and interrupt-disable approach still works).

To create a LittleFS image with `mklittlefs`:

```sh
mklittlefs -c staging_dir -b 65536 -p 256 -s 1048576 lfs.bin
picotool load -t bin lfs.bin -o 0x10100000   # device in BOOTSEL mode
```

To write a ROM directly to cart flash (bypassing the menu, BOOTSEL mode required):
```sh
picotool load -t bin game.rom -o 0x10060000   # cart slot 1
picotool load -t bin game.rom -o 0x100E0000   # cart slot 2
```

### Planned: remove LFS ROM staging (Step 3)

The `sd_to_lfs()` / `lfs_cart1` / `lfs_cart2` / `cart_compare()` code paths are now
dead for the SD case. Next cleanup step:

- Delete `sd_to_lfs()` from `sd_loader.c` / `sd_loader.h`
- Delete `cart_compare()`, `lfs_cart1`, `lfs_cart2` from `msxemulator.c`
- Delete `cart_write()` if the LFS fallback path is also removed, or keep it for LFS-only use
- Remove `sd_filesize()` (only used by old SD path)
- Consider removing LittleFS entirely if save-state / tape support is also dropped

### Menu navigation

Press **F12** on the USB keyboard to open the menu.

| Menu item | Action                                        |
|-----------|-----------------------------------------------|
| 0         | Save tape/state                               |
| 1         | Load tape/state from LittleFS                 |
| **2**     | **Load ROM → Cart Slot 1** (SD or LittleFS)  |
| 3         | Toggle Cart Slot 1 enable/disable             |
| 4         | Set Cart Slot 1 mapper type                   |
| **5**     | **Load ROM → Cart Slot 2** (SD or LittleFS)  |
| 6         | Toggle Cart Slot 2 enable/disable             |
| 12        | Reset                                         |
| 13        | Power cycle                                   |

Arrow keys to navigate, Enter to select, F12/Escape to return to emulator.
After loading a ROM, use Reset (item 12) to boot it.

## Known issues / fixes applied

### flash_range_program hangs on defective flash blocks (watchdog + CART2BASE move)

This board has multiple defective flash blocks where page-program hangs (WIP bit never
clears). Blocks 10, 12, and 13 are confirmed bad. Fix: moved CART2BASE from block 13
(0x0D0000) to block 14 (0x0E0000). `flash_rom_from_sd()` wraps each `flash_range_program`
call with `watchdog_enable(1000, false)` / `watchdog_update()` / `watchdog_disable()`;
if a bad block causes a hang, the RP2040 reboots within 1s instead of freezing forever.

### DVI shows solid red / monitor goes black during ROM loading (queue_try_peek wrap)

`dvi_dma_irq_handler` calls `queue_try_peek_u32` → `queue_try_peek` (flash-resident,
not `__dvi_func`). When `flash_range_program` disables XIP on core0, core1's DMA IRQ
fires and stalls trying to fetch `queue_try_peek` from XIP → DVI PIO runs dry → solid
red → monitor loses sync. Fix: added `__wrap_queue_try_peek` (SRAM-resident) in
`dvi_adapter.c` and `--wrap=queue_try_peek` in `CMakeLists.txt`.

This fix remains in place for the LFS fallback path (`cart_write()`), which still runs
while core1 is live. For the SD path, flashing now happens pre-core1 so the conflict
no longer exists.

### Screen goes black after SD ROM loading (stale USB keypressed)

The 64KB erase (400ms, interrupts disabled) starves TinyUSB. A key pressed during
loading is buffered; on the next `tuh_task()` in the menu loop, a stale F12 fires
`menumode=0` → DVI switches to `mainscreen` (display disabled after VDP reset) → black.
Fix: `tuh_task(); keypressed=0;` after each ROM load handler (`if(menuitem==2/5)`).

With the reboot-to-flash pattern this is no longer an issue for the SD path (the reboot
clears all state), but the fix remains in the LFS fallback branch.

## Nunchuck joystick — `nunchuck.c` / `nunchuck.h`

I2C1 on GPIO 2 (SDA) / GPIO 3 (SCL). Polled at VBlank. Bit mapping into PSG reg 0x0E:

| `nunchuck_joy_bits` | PSG bit | Direction/Button |
|---------------------|---------|-----------------|
| 0x0004 (up)         | bit 0   | Up              |
| 0x0008 (down)       | bit 1   | Down            |
| 0x0001 (left)       | bit 2   | Left            |
| 0x0002 (right)      | bit 3   | Right           |
| 0x0010 (Z button)   | bit 4   | Fire A          |
| 0x0040 (C button)   | bit 5   | Fire B          |

## Key gotchas

- **`board_init()` must not be called** — resets clock to 120 MHz
- **pio0 exclusive to DVI** — any other PIO use must target pio1
- **DMA channels 0–5 claimed by DVI** — other DMA must use channels 6+
- **No multicore lockout for flash ops** — core1's runtime path is entirely SRAM-resident; flash ops need only `save_and_disable_interrupts` / `restore_interrupts` on core0 (lockout would pause the DMA IRQ and kill DVI after ~100 µs)
- **262 HSync ticks, not 313** — 313 (PAL) causes 3.3 ms DVI data gaps (red stripes)
- **`active_screen` starts NULL** — core1 renders black until `dvi_adapter_set_screen()` is called; DVI sync signal is live from core1 start so monitors don't power-save during boot
- **Defective flash blocks** — 0xA0000 (block 10), 0xC4000 (block 12), 0xD0000 (block 13) hang on page-program; avoid these blocks entirely. Watchdog in `sd_loader.c` provides recovery if a new bad block is hit.
- **64KB block erase interrupt blackout** — `flash_range_erase` for 64KB holds interrupts off for up to 400ms; TinyUSB host stack on core0 is not serviced during this window. For the SD path this happens pre-core1 (no DVI, no USB yet), so it is benign.
- **FatFs write enabled** — `FF_FS_READONLY 0` in `fatfs/ffconf.h`; needed to write `SELROM1`/`SELROM2` marker files. The SD card itself is only written by the menu (two tiny files); ROM data is only ever read.
- **DVI TMDS timing** — `DVI_N_TMDS_BUFFERS=8` (8 × 3840 bytes = 30.7 KB) gives ~507 µs of burst slack. `TMDS_ENCODE_UNROLL=2` halves loop branch overhead in the encode inner loop. The Graphics I/II tile inner loops are fully unrolled (8 pixels, running pointer). Palette expansion uses 32-bit paired stores. Together these eliminated solid-red scanline artifacts under heavy sprite load.

## Lessons from galagino (2026-09-26)

Found while porting Galagino to the RP2350-PiZero (`~/Source/galagino/galagino_pizero`). Most
of it is RP2350/pico_lib-specific; what carries over to this RP2040 + PicoDVI port:

- **Flash audit on core 1.** A single flash fetch on core 1's DVI path shows up as red lines
  when core 0 thrashes the XIP cache. This port already moved the queue ops and `memset` into
  RAM. To find anything left (libdvi IRQ path, newlib `memcpy`, SDK helpers, const tables read
  per line), run `arm-none-eabi-objdump -D -j .data <elf>` and look for `*_veneer` calls from
  core 1 functions. Galagino fixed its leftovers with `-Wl,--wrap=<fn>` to RAM copies
  (galagino_pizero/ram_wrappers.c), and a lambda in the DVI IRQ, since lambdas don't inherit
  `__not_in_flash_func`.
- **USB gamepads:** the LUFA `hidparser` + `joystick.c` logic misreads a cheap SNES pad
  (`0079:0011`): byte 0 is a constant `01`, parsed as the X axis, so Left is held constantly.
  Fixed byte maps for this and other cheap pads are in frank-snes
  (`~/Source/frank-snes/drivers/usbhid/hid_app.c`, adopted by galagino_pizero/usb_input.c).
  **Applied here:** `pad_maps[]` in `joystick.c` (VID:PID → byte layout, A/B → TRIG A/B);
  `hid_app.c` skips the descriptor parser for known pads and calls `parse_mapped_report()`.
- **On-screen diagnostics** in the picture's side margins (timings, missed lines, USB IDs and
  raw reports) proved more practical than a serial console; see galagino_pizero/main.cpp.
- The RP2350's SIO TMDS encoder (a big win there) doesn't exist on RP2040.
