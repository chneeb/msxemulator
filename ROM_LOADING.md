# ROM Loading: Reboot-to-Flash Pattern

Reference implementation: `pico-infonesPlus` (`pico_shared/FrensHelpers.cpp`, `pico_shared/menu.cpp`)

## Problem with the current approach

msxemulator currently copies SD card ROMs into LittleFS (internal flash) while the emulator is
running — core1 DVI is live at the time. Flash erase/write halts XIP for the entire chip, which
stalls core1 mid-scanline and produces display glitches or crashes. This is why ROM loading is
unreliable.

## The fix: write flash at boot, before core1 starts

The core insight from pico-infonesPlus: **flash writes must happen before
`multicore_launch_core1()` is called**. At that point core1 is idle, XIP is not in use by
anyone, and the write is safe.

The menu cannot write flash directly (core1 DVI is running). Instead it saves the ROM path to
SD and reboots. The next boot detects the watchdog reboot, writes flash, then starts core1.

---

## Full flow

### Step 1 — Menu: save path + reboot

When the user confirms a ROM selection:

```c
// Write full SD path to a marker file on SD card
FIL fil;
f_open(&fil, "/MSX/SELROM", FA_CREATE_ALWAYS | FA_WRITE);
f_puts("/MSX/GAME.ROM", &fil);   // full path, e.g. built from curdir + filename
f_close(&fil);

// Reboot via watchdog (1 ms timeout)
watchdog_enable(1, 1);
while (1);   // wait for reset
```

The existing LittleFS path and `lfs_cart1`/`lfs_cart2` logic can be removed entirely.

### Step 2 — Boot: detect watchdog reboot and write flash

In `main()`, before `multicore_launch_core1()`:

```c
// Compute ROM address: first 4KB-aligned byte after firmware
extern uint8_t __flash_binary_end;
uint32_t rom_flash_offset = ((uintptr_t)&__flash_binary_end + 0xFFF) & ~0xFFF;
rom_flash_offset -= XIP_BASE;          // convert to flash offset for the API

if (watchdog_enable_caused_reboot()) {
    // Reboot was triggered by the menu — flash the selected ROM
    char rompath[64] = {0};
    FIL fil;
    UINT br;
    if (f_open(&fil, "/MSX/SELROM", FA_READ) == FR_OK) {
        f_read(&fil, rompath, sizeof(rompath) - 1, &br);
        f_close(&fil);
        rompath[br] = 0;
    }

    if (rompath[0]) {
        // Stream ROM from SD → flash in 4KB chunks
        static uint8_t buf[4096];
        UINT bytes_read;
        uint32_t ofs = rom_flash_offset;

        if (f_open(&fil, rompath, FA_READ) == FR_OK) {
            for (;;) {
                memset(buf, 0xFF, sizeof(buf));
                f_read(&fil, buf, sizeof(buf), &bytes_read);
                if (bytes_read == 0) break;

                uint32_t ints = save_and_disable_interrupts();
                flash_range_erase(ofs, sizeof(buf));
                flash_range_program(ofs, buf, sizeof(buf));
                restore_interrupts(ints);

                ofs += sizeof(buf);
            }
            f_close(&fil);
        }
    }
}

// ROM is now in flash — point cartrom1 at it via XIP
uint8_t *cartrom1 = (uint8_t *)(XIP_BASE + rom_flash_offset);

// NOW safe to start core1
multicore_launch_core1(core1_main);
```

### Step 3 — Runtime: read ROM via XIP pointer

`cartrom1` is a plain pointer into XIP flash. The emulator's memory read callback:

```c
// In the Z80 memory read handler — unchanged from current code
return cartrom1[(address - 0x4000) & banked_mask];
```

No `lfs_file_read`, no heap buffer. ROM bytes come straight from flash cache.

---

## Skipping re-flash (same ROM, reset within game)

If the user resets the board while in-game, `watchdog_enable_caused_reboot()` will be false
(hardware reset, not watchdog). `/MSX/SELROM` still contains the last-played ROM path.
Read it and point `cartrom1` at the existing flash contents — no re-flash needed.

```c
if (watchdog_enable_caused_reboot()) {
    // ... flash as above ...
} else {
    // Power-on or hardware reset: ROM already in flash from previous session.
    // Just point the pointer — do not erase/write.
}
// Either way, cartrom1 = (uint8_t *)(XIP_BASE + rom_flash_offset);
```

pico-infonesPlus uses a `/START` sentinel file on SD to distinguish these cases. A simpler
approach for msxemulator: always skip flash on non-watchdog reboot, since `SELROM` is only
written by the menu immediately before a watchdog reboot.

---

## ROM address calculation

```c
extern uint8_t __flash_binary_end;   // provided by the linker

// 4KB-aligned address in XIP space (for pointer arithmetic)
uintptr_t rom_xip_addr = ((uintptr_t)&__flash_binary_end + 0xFFF) & ~0xFFF;

// Flash offset (for flash_range_erase / flash_range_program)
uint32_t rom_flash_offset = rom_xip_addr - XIP_BASE;

// Maximum ROM size before hitting LittleFS partition
// LittleFS starts at 0x10100000 (1MB into flash = XIP_BASE + 0x100000)
uint32_t max_rom_size = (XIP_BASE + 0x100000) - rom_xip_addr;
```

The current msxemulator firmware is ~430 KB. On a 2MB RP2040-PiZero, that leaves
~1MB - 430KB ≈ 570KB available for ROM before LittleFS begins. Enough for a 512 KB MegaROM.

If LittleFS is no longer needed (once ROM loading no longer uses it), the LittleFS partition
can be reclaimed, giving the full ~1.5 MB for ROMs.

---

## RAM impact

| Before (LittleFS stream) | After (XIP pointer) |
|---|---|
| 0 bytes (streamed byte-by-byte) | 0 bytes |
| But: unreliable due to core1 conflict | Reliable: write before core1 starts |

Note: msxemulator already streams from LittleFS byte-by-byte, so the RAM cost was already
zero — the flash approach doesn't change that. The improvement is **reliability**, not RAM.
(msx2pico heap-copies the ROM, so for that project the gain would be RAM as well.)

---

## Files to change in msxemulator

| File | Change |
|---|---|
| `msxemulator.c` — menu selection | Write `/MSX/SELROM` + `watchdog_enable(1,1)` instead of `lfs_file_open` write path |
| `msxemulator.c` — `main()` / init | Add watchdog-reboot check + flash write before `multicore_launch_core1()` |
| `msxrom.h` | Replace fixed `CART1BASE 0x10060000` with dynamic `rom_xip_addr` from `__flash_binary_end` |
| `msxemulator.c` — memory read | `cartrom1` pointer already used — no change needed in the read path |
| `CMakeLists.txt` | Can remove LittleFS sources if LFS is no longer used for ROM storage |
