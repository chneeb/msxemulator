#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "lfs.h"

// Initialize SPI0 SD card (GPIO18=SCK, GPIO19=MOSI, GPIO20=MISO, GPIO21=CS)
// Returns true if a card was found and mounted.
bool sd_init(void);

// True if SD card is mounted.
bool sd_available(void);

// Return the size of a ROM file on SD (/msx/<sd_path>) without reading it.
// Returns size (>0) on success, -1 if file not found or empty.
int sd_filesize(const char *sd_path);

// Stream a ROM file from SD (path = bare filename under /msx/, e.g. "game.rom")
// into an LFS file at lfs_name.  The LFS file is created/truncated; caller opens
// it for reading afterwards.
// Returns filesize (>0) on success, -1 on error.
int sd_to_lfs(const char *sd_path, lfs_t *lfs, const char *lfs_name);
