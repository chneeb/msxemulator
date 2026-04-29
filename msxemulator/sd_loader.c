// sd_loader.c — SD card ROM loader for msxemulator (RP2040-PiZero DVI port)
//
// SD card is on SPI0: SCK=GPIO18, MOSI=GPIO19, MISO=GPIO20, CS=GPIO21.
// ROMs are read from the /msx/ folder on the SD card and streamed into LittleFS.
// Flash programming is handled by cart_write() in msxemulator.c via the LFS path.

#include "sd_loader.h"
#include "fatfs/ff.h"
#include "fatfs/tf_card.h"
#include "hardware/uart.h"
#include <string.h>
#include <stdio.h>

#define SD_SCK  18
#define SD_MOSI 19
#define SD_MISO 20
#define SD_CS   21

#define SD_ROM_DIR "/msx"

static FATFS sd_fs;
static bool  sd_mounted = false;

// 4KB chunk buffer for SD→LFS streaming
static uint8_t _sd_buf[4096];

bool sd_init(void) {
    static const pico_fatfs_spi_config_t cfg = {
        .spi_inst = spi0,
        .clk_slow = 100 * 1000,
        .clk_fast = 32 * 1000 * 1000,
        .pin_miso = SD_MISO,
        .pin_cs   = SD_CS,
        .pin_sck  = SD_SCK,
        .pin_mosi = SD_MOSI,
        .pullup   = true,
    };
    pico_fatfs_set_config((pico_fatfs_spi_config_t *)&cfg);
    sd_mounted = (f_mount(&sd_fs, "", 1) == FR_OK);
    return sd_mounted;
}

bool sd_available(void) {
    return sd_mounted;
}

// Return the byte size of an SD ROM file without reading its contents.
int sd_filesize(const char *sd_path) {
    char full_path[80];
    snprintf(full_path, sizeof(full_path), "%s/%s", SD_ROM_DIR, sd_path);
    FIL fil;
    FRESULT fr = f_open(&fil, full_path, FA_READ);
    if (fr != FR_OK) return -1;
    FSIZE_t sz = f_size(&fil);
    f_close(&fil);
    return (sz > 0 && sz <= 262144u) ? (int)sz : -1;
}

// Stream a ROM file from SD (/msx/<sd_path>) into LFS file <lfs_name>.
// The LFS file is created/truncated on write, then closed.  Caller opens
// it read-only and calls cart_compare() / cart_write() via the normal path.
// Returns filesize (>0) on success, -1 on error.
int sd_to_lfs(const char *sd_path, lfs_t *lfs, const char *lfs_name) {
    char full_path[80];
    snprintf(full_path, sizeof(full_path), "%s/%s", SD_ROM_DIR, sd_path);

    FIL fil;
    FRESULT fr = f_open(&fil, full_path, FA_READ);
    if (fr != FR_OK) return -1;

    FSIZE_t filesize = f_size(&fil);
    // 256KB is the larger of the two cart slot limits; LFS itself enforces no
    // cart-slot maximum here — cart_size_check() in msxemulator.c does that.
    if (filesize == 0 || filesize > 262144u) {
        f_close(&fil);
        return -1;
    }

    printf("[SD→LFS] %s → %s (%lu bytes)\n",
           full_path, lfs_name, (unsigned long)filesize);
    uart_tx_wait_blocking(uart1);

    lfs_file_t lf;
    int lfs_err = lfs_file_open(lfs, &lf, lfs_name,
                                LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC);
    if (lfs_err < 0) {
        printf("[SD→LFS] lfs_file_open err=%d\n", lfs_err);
        uart_tx_wait_blocking(uart1);
        f_close(&fil);
        return -1;
    }

    UINT br;
    uint32_t written = 0;
    while (written < (uint32_t)filesize) {
        memset(_sd_buf, 0xFF, sizeof(_sd_buf));
        fr = f_read(&fil, _sd_buf, sizeof(_sd_buf), &br);
        if (fr != FR_OK || br == 0) break;
        lfs_ssize_t lw = lfs_file_write(lfs, &lf, _sd_buf, br);
        if (lw < 0) {
            printf("[SD→LFS] lfs_file_write err=%d\n", (int)lw);
            uart_tx_wait_blocking(uart1);
            lfs_file_close(lfs, &lf);
            f_close(&fil);
            return -1;
        }
        written += (uint32_t)br;
        printf(".");
        uart_tx_wait_blocking(uart1);
    }

    lfs_file_close(lfs, &lf);
    f_close(&fil);
    printf(" done %lu bytes\n", (unsigned long)written);
    uart_tx_wait_blocking(uart1);

    return (written > 0) ? (int)written : -1;
}
