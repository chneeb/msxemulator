// MSX ROM layout for RP2040-PiZero DVI port
//
// System ROMs are compiled in as C arrays (from bios/ headers).
// Cart ROMs live in flash at adjusted addresses, above the 1MB firmware reserve.
// LittleFS starts at 0x10100000 (1MB into flash).

#include "bios/msx.h"    // MSX[32768]  — main BIOS
#ifdef USE_FDC
#include "bios/disk.h"   // DISK[16384] — floppy disk BIOS
#endif
#ifdef USE_OPLL
#include "bios/fmpac.h"  // FMPAC[16384] — FM-PAC BIOS
#endif

// System ROM pointers (compiled-in arrays, not fixed flash addresses)
const uint8_t *basicrom = MSX;

#ifdef USE_FDC
const uint8_t *extrom1 = DISK;
#else
static uint8_t _extrom1_dummy[16384];
const uint8_t *extrom1 = _extrom1_dummy;
#endif

#ifdef USE_OPLL
const uint8_t *extrom2 = FMPAC;
#else
static uint8_t _extrom2_dummy[16384];
const uint8_t *extrom2 = _extrom2_dummy;
#endif

// Cart ROM slots in flash (LFS starts at 0x10100000).
// Addresses chosen to skip bad-sector regions at 0xA0000, 0xC4000, and 0xD0000.
// Block 13 (0x0D0000) was tried but page-program hangs after 2 successful loads.
// Block 14 (0x0E0000) is the next untouched region; 128KB fits before LFS.
#define CART1BASE 0x10060000u   // 384KB into flash (blocks 6-9, 256KB max)
#define CART2BASE 0x100E0000u   // 896KB into flash (blocks 14-15, 128KB max)
uint8_t *cartrom1 = (uint8_t *)(CART1BASE);
uint8_t *cartrom2 = (uint8_t *)(CART2BASE);
