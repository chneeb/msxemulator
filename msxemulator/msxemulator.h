// Configuration
#define HW_FLASH_STORAGE_MEGABYTES 2     // Define the pico's FLASH size in Megabytes (1MB - 16MB)
//#define USE_I2S     // Enable I2S DAC Output and SCC emulation
//#define USE_OPLL    // Enable OPLL emulation. need FM-PAC BIOS.
#define USE_FDC     // Enable SONY HBD-F1 emulation. need DISKBIOS.
// DVI port: always 252MHz, always 1.2V core
#define USE_CORE_VOLTAGE12    // Required for 252MHz (PicoDVI)
// Reserve 1MB for firmware+carts in flash (DVI firmware is larger than original VGA)
#define HW_SYSTEM_RESERVED  1024     // KiB (was 512)


// Dependency
#ifdef USE_OPLL
// OPLL emulation require I2S DAC output and more CPU power
#define USE_I2S
#define USE_MORE_OVERCLOCK      
#endif
