#pragma once

#include "hardware/clocks.h"
#include "hardware/spi.h"

#define CLK_SLOW_DEFAULT  (100 * 1000)
#define CLK_FAST_DEFAULT  (32 * 1000 * 1000)

typedef struct {
    spi_inst_t *spi_inst;
    uint        clk_slow;
    uint        clk_fast;
    uint        pin_miso;
    uint        pin_cs;
    uint        pin_sck;
    uint        pin_mosi;
    bool        pullup;
} pico_fatfs_spi_config_t;

#ifdef __cplusplus
extern "C" {
#endif

void pico_fatfs_set_config(pico_fatfs_spi_config_t *config);

#ifdef __cplusplus
}
#endif
