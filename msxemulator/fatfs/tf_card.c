/* tf_card.c — FatFs disk I/O via hardware SPI (no PIO SPI)
 * Adapted from pico-infonesPlus pico_fatfs driver by fhoedemakers.
 * Only the hardware SPI path is kept; PIO SPI is removed.
 */

#include "tf_card.h"
#include "ff.h"
#include "diskio.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

static pico_fatfs_spi_config_t _config = {
    spi1,
    CLK_SLOW_DEFAULT,
    CLK_FAST_DEFAULT,
    40,  /* MISO */
    43,  /* CS   */
    30,  /* SCK  */
    31,  /* MOSI */
    true
};

/* MMC/SD command codes */
#define CMD0    (0)
#define CMD1    (1)
#define ACMD41  (0x80+41)
#define CMD8    (8)
#define CMD9    (9)
#define CMD10   (10)
#define CMD12   (12)
#define ACMD13  (0x80+13)
#define CMD16   (16)
#define CMD17   (17)
#define CMD18   (18)
#define CMD23   (23)
#define ACMD23  (0x80+23)
#define CMD24   (24)
#define CMD25   (25)
#define CMD32   (32)
#define CMD33   (33)
#define CMD38   (38)
#define CMD55   (55)
#define CMD58   (58)

#define CT_MMC   0x01
#define CT_SD1   0x02
#define CT_SD2   0x04
#define CT_SDC   (CT_SD1|CT_SD2)
#define CT_BLOCK 0x08

static volatile DSTATUS Stat = STA_NOINIT;
static BYTE CardType;

static inline uint32_t _millis(void) {
    return to_ms_since_boot(get_absolute_time());
}

static inline void cs_select(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(_config.pin_cs, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(_config.pin_cs, 1);
    asm volatile("nop \n nop \n nop");
}

static void FCLK_SLOW(void) { spi_set_baudrate(_config.spi_inst, _config.clk_slow); }
static void FCLK_FAST(void) { spi_set_baudrate(_config.spi_inst, _config.clk_fast); }

void pico_fatfs_set_config(pico_fatfs_spi_config_t *config) {
    _config = *config;
}

static void init_spi(void) {
    gpio_init(_config.pin_sck);
    gpio_disable_pulls(_config.pin_sck);
    gpio_init(_config.pin_miso);
    if (_config.pullup) gpio_pull_up(_config.pin_miso);
    else gpio_disable_pulls(_config.pin_miso);
    gpio_init(_config.pin_mosi);
    gpio_disable_pulls(_config.pin_mosi);
    gpio_init(_config.pin_cs);
    gpio_disable_pulls(_config.pin_cs);

    gpio_set_function(_config.pin_sck,  GPIO_FUNC_SPI);
    gpio_set_function(_config.pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(_config.pin_mosi, GPIO_FUNC_SPI);
    gpio_set_dir(_config.pin_cs, GPIO_OUT);
    cs_deselect();

    spi_init(_config.spi_inst, _config.clk_slow);
    spi_set_format(_config.spi_inst, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

static BYTE xchg_spi(BYTE dat) {
    uint8_t buf = dat;
    spi_write_read_blocking(_config.spi_inst, &buf, &buf, 1);
    return (BYTE)buf;
}

static void rcvr_spi_multi(BYTE *buff, UINT btr) {
    spi_read_blocking(_config.spi_inst, 0xFF, buff, btr);
}

static int wait_ready(UINT wt) {
    BYTE d;
    uint32_t t = _millis();
    do {
        d = xchg_spi(0xFF);
    } while (d != 0xFF && _millis() < t + wt);
    return (d == 0xFF) ? 1 : 0;
}

static void deselect(void) {
    cs_deselect();
    xchg_spi(0xFF);
}

static int _select(void) {
    cs_select();
    xchg_spi(0xFF);
    if (wait_ready(500)) return 1;
    deselect();
    return 0;
}

static int rcvr_datablock(BYTE *buff, UINT btr) {
    BYTE token;
    uint32_t t = _millis();
    do {
        token = xchg_spi(0xFF);
    } while (token == 0xFF && _millis() < t + 200);
    if (token != 0xFE) return 0;
    rcvr_spi_multi(buff, btr);
    xchg_spi(0xFF); xchg_spi(0xFF);   /* discard CRC */
    return 1;
}

static BYTE send_cmd(BYTE cmd, DWORD arg) {
    BYTE n, res;
    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }
    if (cmd != CMD12) { deselect(); if (!_select()) return 0xFF; }
    xchg_spi(0x40 | cmd);
    xchg_spi((BYTE)(arg >> 24));
    xchg_spi((BYTE)(arg >> 16));
    xchg_spi((BYTE)(arg >> 8));
    xchg_spi((BYTE)arg);
    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    xchg_spi(n);
    if (cmd == CMD12) xchg_spi(0xFF);
    n = 10;
    do { res = xchg_spi(0xFF); } while ((res & 0x80) && --n);
    return res;
}

DSTATUS disk_initialize(BYTE drv) {
    BYTE n, cmd, ty, ocr[4];
    uint32_t t;

    if (drv) return STA_NOINIT;
    init_spi();
    sleep_ms(10);
    if (Stat & STA_NODISK) return Stat;

    FCLK_SLOW();
    for (n = 10; n; n--) xchg_spi(0xFF);

    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {
        t = _millis();
        if (send_cmd(CMD8, 0x1AA) == 1) {
            for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                while (_millis() < t + 1000 && send_cmd(ACMD41, 1UL << 30));
                if (_millis() < t + 1000 && send_cmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        } else {
            if (send_cmd(ACMD41, 0) <= 1) {
                ty = CT_SD1; cmd = ACMD41;
            } else {
                ty = CT_MMC; cmd = CMD1;
            }
            while (_millis() < t + 1000 && send_cmd(cmd, 0));
            if (_millis() >= t + 1000 || send_cmd(CMD16, 512) != 0)
                ty = 0;
        }
    }
    CardType = ty;
    deselect();

    if (ty) { FCLK_FAST(); Stat &= ~STA_NOINIT; }
    else    { Stat = STA_NOINIT; }

    return Stat;
}

DSTATUS disk_status(BYTE drv) {
    if (drv) return STA_NOINIT;
    return Stat;
}

DRESULT disk_read(BYTE drv, BYTE *buff, LBA_t sector, UINT count) {
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (!(CardType & CT_BLOCK)) sector *= 512;
    if (count == 1) {
        if ((send_cmd(CMD17, sector) == 0) && rcvr_datablock(buff, 512))
            count = 0;
    } else {
        if (send_cmd(CMD18, sector) == 0) {
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);
        }
    }
    deselect();
    return count ? RES_ERROR : RES_OK;
}

#if FF_FS_READONLY == 0
static void xmit_spi_multi(const BYTE *buff, UINT btx) {
    spi_write_blocking(_config.spi_inst, buff, btx);
}

static int xmit_datablock(const BYTE *buff, BYTE token) {
    BYTE resp;
    if (!wait_ready(500)) return 0;
    xchg_spi(token);
    if (token != 0xFD) {
        xmit_spi_multi(buff, 512);
        xchg_spi(0xFF); xchg_spi(0xFF);
        resp = xchg_spi(0xFF);
        if ((resp & 0x1F) != 0x05) return 0;
    }
    return 1;
}

DRESULT disk_write(BYTE drv, const BYTE *buff, LBA_t sector, UINT count) {
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;
    if (!(CardType & CT_BLOCK)) sector *= 512;
    if (!_select()) return RES_NOTRDY;
    if (count == 1) {
        if ((send_cmd(CMD24, sector) == 0) && xmit_datablock(buff, 0xFE))
            count = 0;
    } else {
        if (CardType & CT_SDC) send_cmd(ACMD23, count);
        if (send_cmd(CMD25, sector) == 0) {
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD)) count = 1;
        }
    }
    deselect();
    return count ? RES_ERROR : RES_OK;
}
#endif /* FF_FS_READONLY == 0 */

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void *buff) {
    DRESULT res;
    BYTE n, csd[16];
    DWORD *dp, st, ed, csize;

    if (drv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    res = RES_ERROR;
    switch (cmd) {
    case CTRL_SYNC:
        if (_select()) res = RES_OK;
        break;
    case GET_SECTOR_COUNT:
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {
                csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(DWORD *)buff = csize << 10;
            } else {
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(DWORD *)buff = csize << (n - 9);
            }
            res = RES_OK;
        }
        break;
    case GET_BLOCK_SIZE:
        if (CardType & CT_SD2) {
            if (send_cmd(ACMD13, 0) == 0) {
                xchg_spi(0xFF);
                if (rcvr_datablock(csd, 16)) {
                    for (n = 64 - 16; n; n--) xchg_spi(0xFF);
                    *(DWORD *)buff = 16UL << (csd[10] >> 4);
                    res = RES_OK;
                }
            }
        } else {
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
                if (CardType & CT_SD1) {
                    *(DWORD *)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                } else {
                    *(DWORD *)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                }
                res = RES_OK;
            }
        }
        break;
    case CTRL_TRIM:
        if (!(CardType & CT_SDC)) break;
        if (disk_ioctl(drv, MMC_GET_CSD, csd)) break;
        if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;
        dp = buff; st = dp[0]; ed = dp[1];
        if (!(CardType & CT_BLOCK)) { st *= 512; ed *= 512; }
        if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 &&
            send_cmd(CMD38, 0) == 0 && wait_ready(30000))
            res = RES_OK;
        break;
    default:
        res = RES_PARERR;
    }
    deselect();
    return res;
}
