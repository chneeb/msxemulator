//  MSX1 emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue0
//  GP3: Blue1
//  GP4: Red0
//  GP5: Red1
//  GP6: Red2
//  GP7: Green0
//  GP8: Green1
//  GP9: Green2
//  GP10: Audio
//  GP14: I2S DATA
//  GP15: I2S BCLK
//  GP16: I2S LRCLK

#include "msxemulator.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"

#include "tusb.h"
// NOTE: bsp/board.h is intentionally NOT included — board_init() forces the
// clock back to 120MHz which breaks PicoDVI at 252MHz.
#include "hidparser/hidparser.h"

#include "dvi_adapter.h"
#include "nunchuck.h"
#include "nes_controller.h"
#include "sd_loader.h"
#include "fatfs/ff.h"
#include "tms9918/vrEmuTms9918.h"
#include "tms9918/vrEmuTms9918Util.h"

#include "msxkeymap.h"
#include "Z80.h"
#include "msxmisc.h"
#include "font_jp.h"
#include "msxrom.h"

#include "emu2149/emu2149.h"
#include "emu2212/emu2212.h"
#ifdef USE_OPLL
#include "emu2413/emu2413.h"
#endif
#include "lfs.h"
#include "fdc.h"

// DVI output at 252MHz (required by PicoDVI for 640x480p60Hz)
#define SYSCLOCK_KHZ 252000

// MSX screen dimensions used for menu/text rendering (40 chars × 24 rows)
#define VGA_CHARS_X 40
#define VGA_CHARS_Y 24

volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline,vsync_scanline;

struct repeating_timer timer,timer2;

// PC configuration

static Z80 cpu;
uint32_t cpu_clocks=0;
uint32_t cpu_ei=0;
uint32_t cpu_cycles=0;
uint32_t cpu_hsync=0;

uint32_t cpu_trace=0;   // DEBUG
uint32_t cpu_boost=0;

// Slot configuration
// Slot0: BASIC
// Slot1: Cart
// Slot2: EMPTY
// Slot3: RAM

uint8_t extslot[4];
uint8_t megarom[8];
uint8_t memmap[4];

uint8_t mainram[0x20000];
uint8_t ioport[0x100];

//uint32_t colormode=0;
//uint32_t screenmode=0;
uint32_t carttype[2]={0,0};            // 0: Plain,1:ASCII8,2:ASCII16,3:KONAMI8,4:KONAMISCC
uint32_t cart_enable[2]={0,0};
uint32_t cart_loaded[2]={0,0};


// VDP — heap-allocated via vrEmuTms9918New().
// TMDS buffers are now static (in dvi_adapter.c), so the heap has ~42KB free
// for these two instances (~33KB combined) plus runtime file caches.
VrEmuTms9918 *mainscreen;
VrEmuTms9918 *menuscreen;
uint8_t scandata[256];

uint8_t timer_enable_irq=0;

volatile uint8_t redraw_flag=0;

uint8_t keymap[11];

volatile uint8_t keypressed=0;  //last pressed usbkeycode
uint32_t key_caps=0;            // Keyboard LED status
uint32_t key_kana=0;
uint32_t key_kana_jis=1;        // Kana Keyboard select 

// BEEP & PSG

uint32_t beep_enable=0;
uint32_t pwm_slice_num;
volatile uint32_t sound_tick=0;

uint8_t psg_register_number=0;

uint8_t psg_register[16];
uint32_t psg_osc_interval[4];
uint32_t psg_osc_counter[4];

uint32_t psg_noise_interval;
uint32_t psg_noise_counter;
uint8_t psg_noise_output;
uint32_t psg_noise_seed;
uint32_t psg_envelope_interval;
uint32_t psg_envelope_counter;
uint32_t psg_master_clock = 3579545/2;
uint16_t psg_master_volume = 0;

uint8_t psg_tone_on[4], psg_noise_on[4];

const uint16_t psg_volume[] = { 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
       0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1b, 0x20,
       0x26, 0x2d, 0x36, 0x40, 0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff };

#ifdef USE_I2S
#define I2S_NUMSAMPLES    8
int32_t i2s_data;
uint16_t i2s_buffer[8];
uint16_t wave_buffer[8];
volatile uint8_t wave_in=0,wave_out=7;
//int i2s_buffer[8];
uint16_t __attribute__  ((aligned(256)))  i2s_buffer0[I2S_NUMSAMPLES*2];
uint16_t __attribute__  ((aligned(256)))  i2s_buffer1[I2S_NUMSAMPLES*2];
uint32_t i2s_active_dma=0;
uint i2s_chan_0 = 3;
uint i2s_chan_1 = 4;
// Static audio chip instances — avoid heap allocation for PSG/SCC structs
static PSG _psg_inst;
static SCC _scc1_inst;
static SCC _scc2_inst;
PSG *msxpsg  = &_psg_inst;
SCC *msxscc1 = &_scc1_inst;
SCC *msxscc2 = &_scc2_inst;
#ifdef USE_OPLL
OPLL *msxopll;
#endif
#endif


#ifndef USE_OPLL
//#define SAMPLING_FREQ 44100    
#define SAMPLING_FREQ 22050
#else                             
#define SAMPLING_FREQ 50000  // USE with OPLL
//#define SAMPLING_FREQ 25000  // USE with OPLL
#endif

#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ) 
// #define YM2203_TIMER_INTERVAL (1000000/SAMPLING_FREQ)

// Tape

uint32_t tape_ready=0;
uint32_t tape_ptr=0;
uint32_t tape_phase=0;
uint32_t tape_count=0;

uint32_t tape_read_wait=0;
uint32_t tape_leader=0;
uint32_t tape_autoclose=0;          // Default value of TAPE autoclose
uint32_t tape_skip=0;               // Default value of TAPE load accelaration
uint32_t tape_cycles;

const uint32_t tape_waits[] = { 650 ,1300 , 325 , 650 } ;  // Tape signal width
const uint8_t tape_cas_header[] = { 0x1F, 0xA6, 0xDE , 0xBA , 0xCC , 0x13,  0x7D, 0x74  };

#define TAPE_WAIT_WIDTH 20

#define TAPE_THRESHOLD 20000000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

// UI

uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);
extern volatile uint8_t gamepad_info;
uint32_t gamepad_select=0;

uint32_t usbcheck_count=0;
uint32_t kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

// Define the flash sizes
// This is the setup to read a block of the flash from the end
// HW_SYSTEM_RESERVED is defined in msxemulator.h (1024 KiB for DVI port, was 512 KiB)
// Use 64KB block erase size to match the W25Q128 block-erase opcode (0xD8).
// The board has defective sectors where 4KB sector erase (0x20) hangs forever
// (WIP bit never clears).  flash_range_erase with a 64KB-aligned count uses
// the 0xD8 block erase which is reliable on all tested blocks.
// LFS partition: 1MB / 64KB = 16 blocks (down from 256 4KB-blocks).
// Changing this invalidates existing LFS data — firmware auto-reformats on boot.
#define BLOCK_SIZE_BYTES (65536)
// Use the outcome of HW_FLASH_STORAGE_BYTES for creating the little file system
// Example: 2MB flash, 1024KiB reserved -> (2048-1024)*1024 = 1MB LFS
#define HW_FLASH_STORAGE_BYTES  (((HW_FLASH_STORAGE_MEGABYTES * 1024) - HW_SYSTEM_RESERVED) * 1024)
#define HW_FLASH_STORAGE_BASE   (1024*1024*HW_FLASH_STORAGE_MEGABYTES - HW_FLASH_STORAGE_BYTES)

uint8_t __attribute__  ((aligned(sizeof(unsigned char *)*4096))) flash_buffer[4096];

lfs_t lfs;
lfs_file_t lfs_file,lfs_fd,lfs_cart1,lfs_cart2;

#define FILE_THREHSOLD 20000000
#define LFS_LS_FILES 12

volatile uint32_t load_enabled=0;
volatile uint32_t save_enabled=0;
//uint32_t file_cycle=0;

unsigned char filename[16];
unsigned char tape_filename[16];
unsigned char fd_filename[16];
unsigned char cart1_filename[16];
unsigned char cart2_filename[16];

static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);

// HSync callback — replaces the old VGA PIO interrupt handler.
// Fires every 64µs via repeating_timer, pacing the Z80 emulation.
// 262 ticks × 64µs = 16.768ms ≈ 59.6Hz (NTSC), which closely matches
// the DVI 60Hz frame period (16.67ms).  Using 313 (PAL 50Hz) leaves a
// 3.3ms gap each DVI frame where the display shows solid-red error lines.
static uint32_t _hsync_count = 0;

bool __not_in_flash_func(hsync_cb)(struct repeating_timer *t) {
    _hsync_count++;
    if (_hsync_count >= 262) {  // 262 lines × 64µs ≈ 16.77ms → ~60Hz NTSC
        _hsync_count = 0;
        video_vsync = 1;
    }
    video_hsync = 1;
    return true;
}

// BEEP and PSG emulation
bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {

    uint16_t timer_diffs;
    uint32_t pon_count;
    uint16_t master_volume;

    uint8_t tone_output[3], noise_output[3], envelope_volume;

#ifndef USE_I2S
    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);

    // PSG

    master_volume = 0;

    // Run Noise generator

        psg_noise_counter += SAMPLING_INTERVAL;
        if (psg_noise_counter > psg_noise_interval) {
            psg_noise_seed = (psg_noise_seed >> 1)
                    | (((psg_noise_seed << 14) ^ (psg_noise_seed << 16))
                            & 0x10000);
            psg_noise_output = psg_noise_seed & 1;
            psg_noise_counter -= psg_noise_interval;
        }
        if (psg_noise_output != 0) {
            noise_output[0] = psg_noise_on[0];
            noise_output[1] = psg_noise_on[1];
            noise_output[2] = psg_noise_on[2];
        } else {
            noise_output[0] = 0;
            noise_output[1] = 0;
            noise_output[2] = 0;
        }

    // Run Envelope

        envelope_volume = 0;

        switch (psg_register[13] & 0xf) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 9:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 0;
            }
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 15:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 0;
            }
            break;
        case 8:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 31;
            }
            break;
        case 10:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter
                    < psg_envelope_interval * 64) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval - 32;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 31;
            }
            break;
        case 11:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = 31
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 31;
            }
            break;
        case 12:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 32;
                envelope_volume = 0;
            }
            break;
        case 13:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                envelope_volume = 31;
            }
            break;
        case 14:
            if (psg_envelope_counter < psg_envelope_interval * 32) {
                envelope_volume = psg_envelope_counter
                        / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter
                    < psg_envelope_interval * 64) {
                envelope_volume = 63
                        - psg_envelope_counter / psg_envelope_interval;
                psg_envelope_counter += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter -= psg_envelope_interval * 64;
                envelope_volume = 0;
            }
            break;
        }


    // Run Oscillator

    for (int i = 0; i < 4 ; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) {
            tone_output[i] = psg_tone_on[i];
        } else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
            tone_output[i] = psg_tone_on[i];
        } else {
            tone_output[i] = 0;
        }
    }

    // Mixer

    master_volume = 0;

        for (int j = 0; j < 3; j++) {
            if ((tone_output[j] + noise_output[j]) > 0) {
                if ((psg_register[j + 8] & 0x10) == 0) {
                    master_volume += psg_volume[(psg_register[j + 8 ]
                            & 0xf) * 2 + 1];
                } else {
                    master_volume += psg_volume[envelope_volume];
                }
            }
        }

    psg_master_volume = master_volume / 4 + beep_enable*63 ;    // Add beep

    if (psg_master_volume > 255)
        psg_master_volume = 255;
#endif
    return true;
}

// PSG virtual registers

void psg_write(uint32_t data) {

    uint32_t channel,freqdiv,freq;

    psg_register_number=ioport[0xa0];

    if(psg_register_number>15) return;

    psg_register[psg_register_number]=data;


    // printf("[PSG:%x,%x]",psg_register_number,data);

    switch(psg_register_number&0xf) {
        case 0:
        case 1:
            if((psg_register[0]==0)&&(psg_register[1]==0)) {
                psg_osc_interval[0]=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[0] + ((psg_register[1]&0x0f)<<8) );
            freq >>= 4;
            if(freq!=0) {
                psg_osc_interval[0] = TIME_UNIT / freq;
                psg_osc_counter[0]=0;
            } else {
                psg_osc_interval[0]=UINT32_MAX;
            }
            break;
        case 2:
        case 3:
            if((psg_register[2]==0)&&(psg_register[3]==0)) {
                psg_osc_interval[1]=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[2] + ((psg_register[3]&0x0f)<<8) );
            freq >>= 4;
            if(freq!=0) {
                psg_osc_interval[1] = TIME_UNIT / freq;
                psg_osc_counter[1]=0;
            } else {
                psg_osc_interval[1]=UINT32_MAX;
            }
            break;
        case 4:
        case 5:
            if((psg_register[4]==0)&&(psg_register[5]==0)) {
                psg_osc_interval[2]=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[4] + ((psg_register[5]&0x0f)<<8) );
            freq >>= 4;
            if(freq!=0) {
                psg_osc_interval[2] = TIME_UNIT / freq;
                psg_osc_counter[2]=0;
                } else {
                    psg_osc_interval[2]=UINT32_MAX;
                }
            break;
        case 6:
            if((psg_register[6]==0)&&(psg_register[7]==0)) {
                psg_noise_interval=UINT32_MAX;
                break;
            }
            freq = psg_master_clock / ( psg_register[6] & 0x1f );
            freq >>= 4;
            if(freq!=0) {
                psg_noise_interval = TIME_UNIT / freq;
                psg_noise_counter = 0;
            } else {
                psg_noise_interval=UINT32_MAX;
            }
            break;
        case 7:
            psg_tone_on[0]=((psg_register[7]&1)==0?1:0);
            psg_tone_on[1]=((psg_register[7]&2)==0?1:0);
            psg_tone_on[2]=((psg_register[7]&4)==0?1:0);
            psg_noise_on[0]=((psg_register[7]&8)==0?1:0);
            psg_noise_on[1]=((psg_register[7]&16)==0?1:0);
            psg_noise_on[2]=((psg_register[7]&32)==0?1:0);
            break;
        case 0xb:
        case 0xc:
            freq = psg_master_clock / ( psg_register[0xb] + (psg_register[0xc]<<8) );
            if(freq!=0) {
                psg_envelope_interval= TIME_UNIT / freq;
                psg_envelope_interval<<=5;
            } else {
                psg_envelope_interval=UINT32_MAX/2-1;
            }
            break;
        case 0xd:
            psg_envelope_counter=0;
            break;
//                        case 0xf:
//                        psg_reset(1,psg_no);
    }
}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}

uint8_t tapein() {

static uint32_t tape_diff_cycles;
static uint8_t tape_bits,tape_file_data;
static uint8_t tape_half_bit,tape_signal;
static uint16_t tape_data;
static uint16_t tape_byte;
static uint32_t tape_header_bits;
static uint8_t tape_baud;
static uint8_t tape_last_bits;

    if(tape_ready==0) {
        return 0;
    }

    if(load_enabled==0) {
        return 0;
    }

    load_enabled=2;

    tape_diff_cycles=cpu_cycles-tape_cycles;
//    tape_cycles=cpu_cycles;
//    tape_last_bits=data;

    if(tape_phase%2) {

        if(tape_diff_cycles<tape_waits[(tape_last_bits+1)%2]) {
            return tape_signal;
        }

//    printf("[D:%d,%d,%d,%d,%x]",tape_diff_cycles,tape_signal,tape_last_bits,tape_bits,tape_file_data);

        tape_cycles=cpu_cycles;

        if(tape_bits==0) { // start bit
            if(tape_signal) {   // 1 -> 0
                tape_signal=0;
                tape_bits++;
                tape_half_bit=0;
                tape_last_bits=(tape_file_data&1);
                return 0;
            } else {            // 0 -> 1
                tape_signal=1;
                return 1;
            }
        }
        if(tape_bits<9) {
            if(tape_signal) {   // 1 -> 0
                tape_signal=0;
                if(tape_last_bits) {
                    if(tape_half_bit==0) {
                        tape_half_bit++;
                        return 0;
                    }
                }
                if(tape_bits<8) {
                    tape_last_bits=tape_file_data>>tape_bits;
                    tape_last_bits&=1;
                    tape_bits++;
                    tape_half_bit=0;
                    return 0;
                } else {
                    tape_last_bits=1;
                    tape_bits++;
                    tape_half_bit=0;
                    return 0;                    
                }
                return 0;
            } else {            // 0 -> 1
                tape_signal=1;
                return 1;
            }
        }

            if(tape_signal) {   // 1 -> 0

                if(tape_last_bits) {
                    if(tape_half_bit==0) {
                        tape_half_bit++;
                        tape_signal=0;
                        return 0;
                    }
                }
                if(tape_bits==9) {
                    tape_last_bits=1;
                    tape_bits++;
                    tape_signal=0;
                    tape_half_bit=0;
                    return 0;
                } else {
                    tape_last_bits=0;
                    tape_bits=0;
                    tape_signal=0;
                    tape_half_bit=0;
                    lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
                    tape_data=tape_file_data;
                    tape_ptr++;

                    return 0;                    
                }
            } else {            // 0 -> 1
                tape_signal=1;
                return 1;
            }
        
    } else {
        // Header 
        // Return '1' 2400baud

        if((tape_diff_cycles)>10000L) { // First 'h'
            tape_cycles=cpu_cycles;
            tape_signal=1;
            tape_header_bits=0;
            return 1;
        }
        if(tape_diff_cycles>tape_waits[0]) {

//    printf("[H:%d,%d,%d,%d]",tape_diff_cycles,tape_signal,tape_header_bits,tape_phase);

            tape_cycles=cpu_cycles;
            if(tape_signal) {
                tape_signal=0;
                tape_header_bits++;
                if(tape_header_bits>8000) {
                    // Skip CAS Header
                    for(int i=0;i<9;i++) {
                        lfs_file_read(&lfs,&lfs_file,&tape_file_data,1);
                    }
                    tape_bits=0;
                    tape_ptr++;
                    tape_last_bits=0;
                    tape_phase++;
                    tape_header_bits=0;
                }
                return 0;
            } else {
                tape_signal=1;
                return 1;
            }
        } else {
            return tape_signal;
        }
    }


#if 0
    static uint8_t tapebyte;



    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
    tape_ptr++;

    if(tapebyte==0xd3) {
        tape_leader++;
    } else if (tape_leader) {
        tape_leader++;
        if(tape_leader>0x20) {
            tape_leader=0;
        }
    }

//    printf("(%02x)",tapebyte);

    return tapebyte;
#endif
    return 0;

}

void tapeout(uint8_t data) {

static uint32_t tape_diff_cycles;
static uint8_t tape_bits,tape_file_data;
static uint8_t tape_half_bit;
static uint16_t tape_data;
static uint16_t tape_byte;
static uint8_t tape_baud;
static uint8_t tape_last_bits;

    if(tape_ready) {

        if(tape_last_bits!=data) {

            tape_diff_cycles=cpu_cycles-tape_cycles;
            tape_cycles=cpu_cycles;
            tape_last_bits=data;

//            printf("[%d:%d]",data,tape_diff_cycles);

            // Skip headers

            if(tape_phase%2) {
                if(data==0) {
                    if((tape_diff_cycles>tape_waits[tape_baud]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[tape_baud]+TAPE_WAIT_WIDTH)) {
                        tape_data=0;
                        tape_half_bit=0; 
  //                      printf("0");
                    } else if((tape_diff_cycles>tape_waits[tape_baud-1]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[tape_baud-1]+TAPE_WAIT_WIDTH)) {
                        if(tape_half_bit) {
                            tape_data=0x8000;
                            tape_half_bit=0;
   //                         printf("1");
                        } else {
                            tape_half_bit=1;
                            return;
                        }
                    } 
                    tape_byte=(tape_byte>>1)|tape_data;
                    tape_bits++;
                    if(tape_bits==11) {
                        if(save_enabled) {
                            save_enabled=2;
                            tape_file_data=(tape_byte>>6)&0xff;
                            tape_ptr++;
                            lfs_file_write(&lfs,&lfs_file,&tape_file_data,1);
                        } else {
                            printf("[%02x]",(tape_byte>>6)&0xff);
                        }
                        tape_bits=0;
                        tape_byte=0;
                    }
                }                
            } else {

                if(data!=0) {
                    if(tape_diff_cycles>10000L) {  // It is first H bit
                        tape_baud=0;
                        tape_bits=0;
                        tape_byte=0;
                    }
                } 
                if(data==0) {
                    if(tape_baud==0) {  // Baud rate check  (header bit is always '1')
                        if((tape_diff_cycles>tape_waits[0]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[0]+TAPE_WAIT_WIDTH)) {
                            tape_baud=1;
//printf("[1200]\n");
                        } else {
                            tape_baud=3;
//printf("[2400]\n");                            
                        }
                    } else {
                        if((tape_diff_cycles>tape_waits[tape_baud]-TAPE_WAIT_WIDTH)&&(tape_diff_cycles<tape_waits[tape_baud]+TAPE_WAIT_WIDTH)) {
                        // first '0' bit = Statbit of Data section
                            tape_bits=1;
                            tape_byte=0;
                            tape_phase++;
                        // Write CAS Header

                            lfs_file_write(&lfs,&lfs_file,tape_cas_header,8);

                        }
                    } 
                }
            }
        }

//        if(save_enabled) {
//
//        }
    } 
    
}

void menuinit(void) {

    // Initialize VDP

    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_0, 0b00000000);
    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_1, 0b01110000);
    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_2, 0b00001000);
    vrEmuTms9918WriteRegisterValue(menuscreen, TMS_REG_3, 0b00001000);
 
    vrEmuTms9918SetNameTableAddr(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918SetPatternTableAddr(menuscreen, TMS_DEFAULT_VRAM_PATT_ADDRESS);
    vrEmuTms9918SetFgBgColor(menuscreen, TMS_WHITE,TMS_BLACK);
    // Copy CG rom

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_PATT_ADDRESS);
    vrEmuTms9918WriteBytes(menuscreen, font, sizeof(font));

    // Clear screen 

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918WriteByteRpt(menuscreen,0,960);

    return;

}

static inline void video_cls() {
    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS);
    vrEmuTms9918WriteByteRpt(menuscreen,0,960);
}

// static inline void video_scroll() {

//     memmove(vga_data_array, vga_data_array + VGA_PIXELS_X*10, (VGA_PIXELS_X*(VGA_PIXELS_X-10)));
//     memset(vga_data_array + (VGA_CHARS_X*(VGA_PIXELS_X-10)), 0, VGA_PIXELS_X*10);

// }

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;
    uint32_t vramindex;

    vrEmuTms9918SetAddressWrite(menuscreen, TMS_DEFAULT_VRAM_NAME_ADDRESS + cursor_x + cursor_y*VGA_CHARS_X);

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        vrEmuTms9918WriteData(menuscreen,string[i]);

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
//                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=2;
    cursor_y=2;
    fbcolor=7;
      video_print("                                    ");
    for(int i=3;i<19;i++) {
        cursor_x=2;
        cursor_y=i;
        video_print("                                    ");
    }

    cursor_x=2;
    cursor_y=19;
    fbcolor=7;
    video_print("                                    ");

}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    for(int i=0;i<LFS_LS_FILES;i++) {
        cursor_x=20;
        cursor_y=i+3;
        fbcolor=7;
//        video_print("                    ");
                video_print("  ");
    }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {
            
            if(num_entry>=LFS_LS_FILES*(page+1)) {
                break;
            }

            if((num_entry%LFS_LS_FILES)!=(LFS_LS_FILES-1)) {
                for(int i=num_entry%LFS_LS_FILES;i<LFS_LS_FILES;i++) {
                    cursor_x=22;
                    cursor_y=i+3;
                    fbcolor=7;
                    video_print("                  ");                    
                }
            }

            break;
        }

        cursor_x=28;
        cursor_y=23;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=23;
                    cursor_y=num_entry%LFS_LS_FILES+3;

                    if(num_entry==num_selected) {
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    snprintf(str,16,"%s            ",lfs_dir_info.name);
                    video_print(str);
//                    video_print(lfs_dir_info.name);

                }

                if(num_selected>=0) {
                    cursor_x=20;
                    cursor_y=(num_selected%LFS_LS_FILES)+3;
                    video_print("->");
                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

// SD card file browser — lists *.rom files from the SD root directory.
// Returns number of matching files found; copies selected filename to out_name.
static int sd_draw_files(int num_selected, int page, char *out_name) {
    FILINFO fno;
    DIR dir;
    unsigned char str[22];
    int num_entry = 0;

    // Clear the file list area
    for (int i = 0; i < LFS_LS_FILES; i++) {
        cursor_x = 20;
        cursor_y = i + 3;
        fbcolor = 7;
        video_print("  ");
    }

    if (f_findfirst(&dir, &fno, "/msx", "*.rom") != FR_OK &&
        f_findfirst(&dir, &fno, "/msx", "*.ROM") != FR_OK) {
        f_closedir(&dir);
        return 0;
    }

    while (fno.fname[0]) {
        if (num_entry >= LFS_LS_FILES * page &&
            num_entry <  LFS_LS_FILES * (page + 1)) {

            cursor_x = 23;
            cursor_y = num_entry % LFS_LS_FILES + 3;

            if (num_entry == num_selected) {
                if (out_name) strncpy(out_name, fno.fname, 64);
                fbcolor = 0;  // highlight selected
            } else {
                fbcolor = 7;
            }

            snprintf((char *)str, sizeof(str), "%-17s", fno.fname);
            video_print(str);

            if (num_selected >= 0) {
                cursor_x = 20;
                cursor_y = (num_selected % LFS_LS_FILES) + 3;
                video_print("->");
            }
        }

        num_entry++;

        if (f_findnext(&dir, &fno) != FR_OK) break;
    }

    f_closedir(&dir);

    // Page indicator
    cursor_x = 28;
    cursor_y = 23;
    fbcolor = 7;
    snprintf((char *)str, sizeof(str), "SD Pg%02d", page + 1);
    video_print(str);

    return num_entry;
}

// Browse SD card .rom files and select one.
// Returns 0 on selection (path written to out_path), -1 on cancel/empty.
static int sd_file_selector(char *out_path, int path_size) {
    uint32_t num_selected = 0;
    int num_files = sd_draw_files(-1, 0, NULL);

    if (num_files <= 0) return -1;

    while (1) {
        while (video_vsync == 0) ;
        video_vsync = 0;
        dvi_adapter_set_screen(menuscreen);

        sd_draw_files(num_selected, num_selected / LFS_LS_FILES, out_path);

        tuh_task();

        if (keypressed == 0x52) {  // up
            keypressed = 0;
            if (num_selected > 0) num_selected--;
        }
        if (keypressed == 0x51) {  // down
            keypressed = 0;
            if (num_selected < (uint32_t)(num_files - 1)) num_selected++;
        }
        if (keypressed == 0x4b) {  // page up
            keypressed = 0;
            if (num_selected >= LFS_LS_FILES) num_selected -= LFS_LS_FILES;
        }
        if (keypressed == 0x4e) {  // page down
            keypressed = 0;
            if (num_selected < (uint32_t)(num_files - LFS_LS_FILES))
                num_selected += LFS_LS_FILES;
        }
        if (keypressed == 0x28) {  // Enter — confirm
            keypressed = 0;
            return 0;
        }
        if (keypressed == 0x29) {  // ESC — cancel
            keypressed = 0;
            return -1;
        }
    }
}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;
        dvi_adapter_set_screen(menuscreen);

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x4b) { // Pageup
            keypressed=0;
            if(num_selected>=LFS_LS_FILES) {
                num_selected-=LFS_LS_FILES;
            }
        }

        if(keypressed==0x4e) { // Pagedown
            keypressed=0;
            if(num_selected<num_files-LFS_LS_FILES) {
                num_selected+=LFS_LS_FILES;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=3;
        cursor_y=18;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;
        dvi_adapter_set_screen(menuscreen);

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=3;
                cursor_y=18;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }


        }
    }

}

//----------------------------------------------------------------------------------------------

void psg_reset(int flag) {


    psg_noise_seed = 12345;

    if (flag == 0) {
        for (int i = 0; i < 16; i++) {
            psg_register[i] = 0;
        }
    } else {
        for (int i = 0; i < 15; i++) {
            psg_register[i] = 0;
        }
    }
    psg_register[7] = 0xff;

    psg_noise_interval = UINT32_MAX;
    psg_envelope_interval = UINT32_MAX / 2 - 1;

    for (int i = 0; i < 3; i++) {
        psg_osc_interval[i] = UINT32_MAX;
        psg_tone_on[i] = 0;
        psg_noise_on[i] = 0;
    }


}

//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(addr, (const uint8_t *)buffer, size);
    restore_interrupts(ints);

    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);

//    printf("[FS] ERASE: %p, %d\n", offset, block);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(offset, c->block_size);
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
// Static LFS cache buffers — avoids lfs_malloc calls at mount time.
static uint8_t _lfs_read_buf[FLASH_PAGE_SIZE];
static uint8_t _lfs_prog_buf[FLASH_PAGE_SIZE];
static uint8_t _lfs_lookahead_buf[FLASH_PAGE_SIZE];

const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256

    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?

    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256

    // Static buffers — no malloc needed at mount
    .read_buffer      = _lfs_read_buf,
    .prog_buffer      = _lfs_prog_buf,
    .lookahead_buffer = _lfs_lookahead_buf,
};

// Keyboard

static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

void process_kbd_leds(void) {

    hid_led=0;

    if(key_caps) hid_led+=KEYBOARD_LED_CAPSLOCK;          // CAPS Lock
    if(key_kana) hid_led+=KEYBOARD_LED_NUMLOCK;           // KANA -> Numlock
    if((hid_dev_addr!=255)&&(hid_instance!=255)) {
        tuh_hid_set_report(hid_dev_addr, hid_instance, 0, HID_REPORT_TYPE_OUTPUT, &hid_led, sizeof(hid_led));
    }

}


void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;

    if(menumode==0) { // Emulator mode

        for(int i=0;i<11;i++) {

            keymap[i]=0xff;

        }

        if(report->modifier&0x22) {  // SHIFT
            keymap[6]&=0xfe;
        }

        if(report->modifier&0x11) {  // CTRL
            keymap[6]&=0xfd;
        }

        if(report->modifier&0x44) {  // ALT
            keymap[6]&=0xfb;
        }

        for(int i=0;i<6;i++) {

            if ( report->keycode[i] ) {

                usbkey=report->keycode[i];
                if(msxusbcode[usbkey*2]) {
                    keymap[msxusbcode[usbkey*2+1]] &= ~msxusbcode[usbkey*2];
                }

            // Enter Menu
                if(usbkey==0x45) {
                    prev_report=*report;
                    menumode=1;
                    keypressed=0;
                }  
            }
        }

    prev_report=*report;

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}

// cart slots 

void cart_type_checker(uint8_t cartno, uint32_t cartsize) {

    // check cart type on flash
    printf("[CTC] cartno=%d cartsize=%lu\n", (int)cartno, (unsigned long)cartsize);

    uint32_t hists[8],candidate_count;
    uint8_t param,candidate;

//    printf("[Cart %d type %d]",cartno,cartsize);

    for(int i=0;i<8;i++) {
        hists[i]=0;
    }

    for(int i=0;i<cartsize;i++) {

        param=0;

        if((cartno==0)&&(cartrom1[i]==0x32)&&(cartrom1[i+1]==0)) {
            param=cartrom1[i+2];
        }
        if((cartno==1)&&(cartrom2[i]==0x32)&&(cartrom2[i+1]==0)) { 
            param=cartrom2[i+2];
        }

        switch(param) {

            case 0x50:
                hists[4]++;
                break;

            case 0x60:
                hists[1]++;
                hists[2]++;
                hists[3]++;
                break;

            case 0x68:
                hists[1]++;
                break;

            case 0x70:
                hists[1]++;
                hists[2]++;
                hists[4]++;
                break;

            case 0x78:
                hists[1]++;
                break;

            case 0x80:
                hists[3]++;
                break;

            case 0x90:
                hists[4]++;
                break;

            case 0x98:
                hists[4]++;
                break;

            case 0xa0:
                hists[3]++;
                break;

            case 0xb0:
                hists[4]++;
                break;

            default:
                break;            

        }
    }

    // Guesser

    //  ASCII 8  :0x6000-0x7fff  
    //  ASCII16  :0x6000-0x67ff,0x7000-0x77ff
    //  Konami   :0x6000-0xbfff
    //  KonamiSCC:0x5000-0xb7ff


    candidate=0;
    if(cartsize>=0x10000) {
        candidate_count=0;
        for(int i=1;i<5;i++) {
//        printf("[cart:%d,%d]\n",i,hists[i]);
            if(hists[i]>candidate_count) {
                candidate=i;
                candidate_count=hists[i];
            }
        }
    }

    carttype[cartno]=candidate;

    return;

}


int32_t cart_size_check(uint32_t cartno) {

    int32_t filesize;

    if(cartno==0) {
        filesize=lfs_file_size(&lfs,&lfs_cart1);
//        printf("[Cart1 check %d bytes]\n",filesize);
        if(filesize>262144) {
            return -1;
        }
        if((cart_enable[1])&&(filesize>131072)) {
            return -1;
        }
    } else {
        filesize=lfs_file_size(&lfs,&lfs_cart2);
        if(filesize>131072) {
            return -1;
        }
    }

    return 0;

}

int32_t cart_compare(uint32_t cartno) {

    int32_t filesize;
    uint8_t cartdata;
    int32_t match;

    if(cartno==0) {
        filesize=lfs_file_size(&lfs,&lfs_cart1);
        printf("[CMP0] %ld bytes vs flash@%08lx\n", (long)filesize, (unsigned long)CART1BASE);
        uart_tx_wait_blocking(uart1);
        lfs_file_rewind(&lfs,&lfs_cart1);
        match=0;
        for(int i=0;i<filesize;i++) {
            lfs_file_read(&lfs,&lfs_cart1,&cartdata,1);
            if(cartrom1[i]!=cartdata) {
                match=-1;
            }
        }
        printf("[CMP0] result=%ld (%s)\n", (long)match, match ? "need write" : "already ok");
        uart_tx_wait_blocking(uart1);
        return match;

    } else {
        filesize=lfs_file_size(&lfs,&lfs_cart2);
        printf("[CMP1] %ld bytes vs flash@%08lx\n", (long)filesize, (unsigned long)CART2BASE);
        uart_tx_wait_blocking(uart1);
        lfs_file_rewind(&lfs,&lfs_cart2);
        match=0;
        for(int i=0;i<filesize;i++) {
            lfs_file_read(&lfs,&lfs_cart2,&cartdata,1);
            if(cartrom2[i]!=cartdata) {
                match=-1;
            }
        }
        printf("[CMP1] result=%ld (%s)\n", (long)match, match ? "need write" : "already ok");
        uart_tx_wait_blocking(uart1);
        return match;
    }

}

// Flash a ROM file from SD directly to a cart slot, before core1 starts.
// sd_full_path: full FatFs path e.g. "/msx/GAME.ROM"
// cart_base_addr: XIP address of the cart slot (CART1BASE or CART2BASE)
// Returns ROM byte size on success, 0 on failure.
static uint32_t flash_rom_from_sd(const char *sd_full_path, uint32_t cart_base_addr) {
    FIL fil;
    if (f_open(&fil, sd_full_path, FA_READ) != FR_OK) {
        printf("[BOOT] cannot open %s\n", sd_full_path);
        uart_tx_wait_blocking(uart1);
        return 0;
    }
    FSIZE_t filesize = f_size(&fil);
    if (filesize == 0 || filesize > 262144u) {
        printf("[BOOT] bad ROM size %lu\n", (unsigned long)filesize);
        uart_tx_wait_blocking(uart1);
        f_close(&fil);
        return 0;
    }

    uint32_t flash_off = cart_base_addr - XIP_BASE;
    uint32_t erase_size = ((uint32_t)filesize + 65535u) & ~65535u;
    printf("[BOOT] %s → 0x%08lx (%lu B)\n",
           sd_full_path, (unsigned long)cart_base_addr, (unsigned long)filesize);
    uart_tx_wait_blocking(uart1);
    {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(flash_off, erase_size);
        restore_interrupts(ints);
    }

    UINT br;
    watchdog_enable(1000, false);
    uint32_t ofs = flash_off;
    for (;;) {
        memset(flash_buffer, 0xFF, 4096);
        f_read(&fil, flash_buffer, 4096, &br);
        if (br == 0) break;
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(ofs, flash_buffer, 4096);
        restore_interrupts(ints);
        watchdog_update();
        printf(".");
        uart_tx_wait_blocking(uart1);
        ofs += 4096;
    }
    watchdog_disable();
    printf(" done\n");
    uart_tx_wait_blocking(uart1);
    f_close(&fil);
    return (uint32_t)filesize;
}

void cart_write(uint32_t cartno) {

    int32_t filesize;
    uint32_t flash_off;

    if (cartno == 0) {
        filesize  = lfs_file_size(&lfs, &lfs_cart1);
        lfs_file_rewind(&lfs, &lfs_cart1);
        flash_off = CART1BASE - XIP_BASE;
    } else {
        filesize  = lfs_file_size(&lfs, &lfs_cart2);
        lfs_file_rewind(&lfs, &lfs_cart2);
        flash_off = CART2BASE - XIP_BASE;
    }

    if (filesize <= 0) return;

    // Bulk 64KB block erase — avoids per-sector 4KB erase hangs on marginal flash cells.
    uint32_t erase_size = ((uint32_t)filesize + 65535u) & ~65535u;
    printf("[CW%lu] erase 0x%06lx+0x%05lx\n",
           (unsigned long)cartno, (unsigned long)flash_off, (unsigned long)erase_size);
    uart_tx_wait_blocking(uart1);
    {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(flash_off, erase_size);
        restore_interrupts(ints);
    }
    printf("[CW%lu] erase done, prog ", (unsigned long)cartno);
    uart_tx_wait_blocking(uart1);

    // Watchdog: reboot if flash_range_program stalls on a defective page.
    watchdog_enable(1000, false);
    for (int i = 0; i < filesize; i += 4096) {
        if (cartno == 0)
            lfs_file_read(&lfs, &lfs_cart1, &flash_buffer, 4096);
        else
            lfs_file_read(&lfs, &lfs_cart2, &flash_buffer, 4096);
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(flash_off + i, (const uint8_t *)flash_buffer, 4096);
        restore_interrupts(ints);
        watchdog_update();
        printf(".");
        uart_tx_wait_blocking(uart1);
    }
    watchdog_disable();
    printf(" done\n");
    uart_tx_wait_blocking(uart1);

}



//

static uint8_t mem_read(void *context,uint16_t address)
{

    uint8_t b;
    uint8_t slot,extslotno;
    uint8_t bank;
    uint8_t bankno;

    slot=ioport[0xa8];

    bank=(address>>14);
    slot>>=bank*2;
    slot&=3;

   if(address==0xffff) {
 //       printf("[ES:%x]",extslot);
 //       return ~extslot[slot];        
        if(slot==3) {
            return ~extslot[3];
        } else {
            return 0xff;
        }
    }


    switch (slot)
    {
    case 0:  // BASIC ROM
    
        return basicrom[address&0x7fff];

    case 1:  // CART ROM 1

        if(cart_enable[0]) {

            switch(carttype[0]) {

                case 0: // Plain
                    return cartrom1[(address-0x4000)&0xffff];

                case 1:  // ASCII8

                    bank=(address-0x4000)>>13;
                    bank&=3;
                    bankno=megarom[bank];

                    return cartrom1[(address&0x1fffL)+bankno*0x2000L];

                case 2:  // ASCII16

                    bank=(address-0x4000)>>14;
                    bank&=1;
                    bankno=megarom[bank];

                    return cartrom1[(address&0x3fffL)+bankno*0x4000L];

                case 3:  // KONAMI8

                    bank=(address-0x4000)>>13;
                    bank&=3;
                    bankno=megarom[bank];

                    if(bank==0) bankno=0;

                    return cartrom1[(address&0x1fffL)+bankno*0x2000L];

                case 4:  // KONAMI SCC

                    bank=(address-0x4000)>>13;
                    bank&=3;
                    bankno=megarom[bank];

                    if(bankno==63) {
                        if((address>=0x9800)&&(address<0xa000)) {   // SCC

#ifdef USE_I2S
//printf("[SCR:%x:%x]",address,SCC_read(msxscc1,address));
                        return SCC_read(msxscc1,address);

#endif
                            return 0xff;
                        }
                    }

                    return cartrom1[(address&0x1fffL)+bankno*0x2000L];

                default:

                    return 0xff;
            }

        } else {
            return 0xff;
        }

    case 2:  // Cart ROM2

        if(cart_enable[1]) {

            switch(carttype[1]) {

                case 0: // Plain
                    return cartrom2[(address-0x4000)&0xffff];

                case 1:  // ASCII8

                    bank=(address-0x4000)>>13;
                    bank&=3;
                    bankno=megarom[bank+4];

                    return cartrom2[(address&0x1fffL)+bankno*0x2000L];

                case 2:  // ASCII16

                    bank=(address-0x4000)>>14;
                    bank&=1;
                    bankno=megarom[bank+4];

                    return cartrom2[(address&0x3fffL)+bankno*0x4000L];

                case 3:  // KONAMI8

                    bank=(address-0x4000)>>13;
                    bank&=3;
                    bankno=megarom[bank+4];

                    if(bank==0) bankno=0;

                    return cartrom2[(address&0x1fffL)+bankno*0x2000L];

                case 4:  // KONAMI SCC

                    bank=(address-0x4000)>>13;
                    bank&=3;
                    bankno=megarom[bank+4];

                    if(bankno==63) {
                        if((address>=0x9800)&&(address<0xa000)) {   // SCC

#ifdef USE_I2S
//printf("[SCR:%x:%x]",address,SCC_read(msxscc,address));
                        return SCC_read(msxscc2,address);

#endif
                            return 0xff;
                        }
                    }

                    return cartrom2[(address&0x1fffL)+bankno*0x2000L];

                default:

                    return 0xff;
            }

        } else {
            return 0xff;
        }

    case 3:  // RAM

        extslotno=extslot[3];
        extslotno>>=bank*2;
        extslotno&=3;

        switch(extslotno) {
            case 0:

                return mainram[(address&0x3fff)+memmap[bank]*0x4000];
//                return mainram[address];

            case 1:  // FDD

#ifdef USE_FDC

                if((address>=0x4000)&&(address<0x7ff8)) {
                    return extrom1[address&0x3fff];
                }

                if(address==0x7ff8) { // status
// if(Z80_PC(cpu)!=0x7703) {
// printf("[FS:%x/%x]",fdc_read_status(),Z80_PC(cpu));
// }
                return fdc_read_status();
                }
                if(address==0x7ff9) { // Track
                    return fdc_read_track();
                }
                if(address==0x7ffa) { // Sector
                    return fdc_read_sector();
                }               
                if(address==0x7ffb) { // data
                    return fdc_read();
                }
                if(address==0x7ffc) { // control1
                    return fdc_read_control1();
                }
                if(address==0x7ffd) { // control2
                    return fdc_read_control2();
                }
                if(address==0x7fff) { // status flag
                    return fdc_read_status_flag();
                }
#endif
                return 0xff;

#ifdef  USE_OPLL
            case 2:

                if((address>=0x4000)&&(address<0x8000)) {
                    return extrom2[address&0x3fff];
                }

                return 0xff;
#endif


            default:
                return 0xff;

        }

    default:
        break;
    }

    return 0;

}

static void mem_write(void *context,uint16_t address, uint8_t data)
{

    uint8_t slot,extslotno,bank;

    slot=ioport[0xa8];

    bank=(address>>14);
    slot>>=bank*2;
    slot&=3;

    if(address==0xffff) {
        extslot[slot]=data;
        return;        
    }

    switch (slot)
    {
    case 0:  // BASIC ROM
    
        return;

    case 1:  // CART ROM

        // MEGAROM
        if(cart_enable[0]) {
            switch(carttype[0]) {
                
                case 1:  // ASCII 8
                    if((address>=0x6000)&&(address<0x6800)) {
                        megarom[0]=data;
                    } else if((address>=0x6800)&&(address<0x7000)) {
                        megarom[1]=data;
                    } else if((address>=0x7000)&&(address<0x7800)) {
                        megarom[2]=data;
                    } else if((address>=0x7800)&&(address<0x8000)) {
                        megarom[3]=data;
                    } 
                    return;

                case 2:  // ASCII 16

                    if((address>=0x6000)&&(address<0x6800)) {
                        megarom[0]=data;
                    } else if((address>=0x7000)&&(address<0x7800)) {
                        megarom[1]=data;
                    } 
                    return;

                case 3:  // Konami 8

                    if((address>=0x6000)&&(address<0x8000)) {
                        megarom[1]=data&0x3f;
                    } else if((address>=0x8000)&&(address<0xa000)) {
                        megarom[2]=data&0x3f;
                    } else if((address>=0xa000)&&(address<0xc000)) {
                        megarom[3]=data&0x3f;
                    } 

                    return;

                case 4:  // Konami SCC
                
                    if((address>=0x5000)&&(address<0x5800)) {
                        megarom[0]=data&0x3f;
                    } else if((address>=0x7000)&&(address<0x7800)) {
                        megarom[1]=data&0x3f;
                    } else if((address>=0x9000)&&(address<0x9800)) {
                        megarom[2]=data&0x3f;
                    } else if((address>=0xb000)&&(address<0xb800)) {
                        megarom[3]=data&0x3f;
                    } 


#ifdef USE_I2S
                    SCC_write(msxscc1,address,data);
#endif
                    return;

                default:

            }
        }

        return;
 
    case 2:  // Cart ROM 2
        // MEGAROM
        if(cart_enable[1]) {
            switch(carttype[1]) {
                
                case 1:  // ASCII 8
                    if((address>=0x6000)&&(address<0x6800)) {
                        megarom[4]=data;
                    } else if((address>=0x6800)&&(address<0x7000)) {
                        megarom[5]=data;
                    } else if((address>=0x7000)&&(address<0x7800)) {
                        megarom[6]=data;
                    } else if((address>=0x7800)&&(address<0x8000)) {
                        megarom[7]=data;
                    } 
                    return;

                case 2:  // ASCII 16

                    if((address>=0x6000)&&(address<0x6800)) {
                        megarom[4]=data;
                    } else if((address>=0x7000)&&(address<0x7800)) {
                        megarom[5]=data;
                    } 
                    return;

                case 3:  // Konami 8

                    if((address>=0x6000)&&(address<0x8000)) {
                        megarom[5]=data&0x3f;
                    } else if((address>=0x8000)&&(address<0xa000)) {
                        megarom[6]=data&0x3f;
                    } else if((address>=0xa000)&&(address<0xc000)) {
                        megarom[7]=data&0x3f;
                    } 

                    return;

                case 4:  // Konami SCC
                
                    if((address>=0x5000)&&(address<0x5800)) {
                        megarom[4]=data&0x3f;
                    } else if((address>=0x7000)&&(address<0x7800)) {
                        megarom[5]=data&0x3f;
                    } else if((address>=0x9000)&&(address<0x9800)) {
                        megarom[6]=data&0x3f;
                    } else if((address>=0xb000)&&(address<0xb800)) {
                        megarom[7]=data&0x3f;
                    } 

#ifdef USE_I2S
                    SCC_write(msxscc2,address,data);
#endif
                    return;

                default:

            }
        }

        return;

    case 3:  // Extslot

        extslotno=extslot[3];
        extslotno>>=bank*2;
        extslotno&=3;

        switch(extslotno) {
            case 0:

//                mainram[address]=data;
                mainram[(address&0x3fff)+memmap[bank]*0x4000]=data;
                return;

            case 1: // FDC
#ifdef USE_FDC
                if(address==0x7ff8) { // command
                    fdc_command_write(data);
                    return;
                }
                if(address==0x7ff9) { // Track
                    fdc_write_track(data);
                    return;
                }
                if(address==0x7ffa) { // Sector                
                    fdc_write_sector(data);
                    return;
                }               
                if(address==0x7ffb) { // data
                    fdc_write(data);
                    return;
                }
                if(address==0x7ffc) { // control1
                    fdc_write_control1(data);
                    return;
                }
                if(address==0x7ffd) { // control2
                    fdc_write_control2(data);
                }
#endif
                return;

            default:
                return;

        }

    default:
        break;
    }

    return;

}

static uint8_t io_read(void *context, uint16_t address)
{
    uint8_t data = ioport[address&0xff];
    uint8_t b;
    uint32_t kanji_addr;

    address&=0xff;

// if((address&0xf0)!=0x90) {
// if(address!=0xa8) {
//     printf("[R:%x:%x]",Z80_PC(cpu),  address);

// }
// }

    switch(address) {

        case 0x98:  // VDP Read

            return vrEmuTms9918ReadData(mainscreen);

        case 0x99:  // VDP Status

            b=vrEmuTms9918ReadStatus(mainscreen);

            if(video_vsync==2) {
                b|=0x80;
                video_vsync=0;
            }

            return b;

        case 0xa2: // PSG READ
            if(ioport[0xa0]<0x0e) {
#ifdef USE_I2S
                return PSG_readReg(msxpsg,ioport[0xa0]&0xf);
#else
                return psg_register[ioport[0xa0]&0xf];
#endif
            } else if(ioport[0xa0]==0xe) {  // joystick
//                    b=0x3f;
                    if(key_kana_jis) b|=0x40;
                    if((tape_ready)&&(load_enabled)) {
                        if(tapein()) b|=0x80;
                    }
                    if(!gamepad_select) {
                        uint32_t nc = nunchuck_joy_bits | nes_joy_bits;
                        uint8_t joy = gamepad_info;
                        // Active-high bits (nunchuck/NES) → PSG active-low bits
                        if (nc & 0x0004) joy &= ~0x01u;  // up    → bit0
                        if (nc & 0x0008) joy &= ~0x02u;  // down  → bit1
                        if (nc & 0x0001) joy &= ~0x04u;  // left  → bit2
                        if (nc & 0x0002) joy &= ~0x08u;  // right → bit3
                        if (nc & 0x0010) joy &= ~0x10u;  // fire A → bit4
                        if (nc & 0x0040) joy &= ~0x20u;  // fire B → bit5
                        b |= joy;
                    } else {
                        b|=0x3f;
                    }
                    return b;
            } else if(ioport[0xa0]==0xf) {
                    b=0x7f;
                    if(key_kana==0) b|=0x80;
                    return b;
            }

            return 0xff;

        case 0xa3:
            return 0xff;

        case 0xa9:  // i8s55 Port B (Keyboard)

//            printf("[Key%x:%x]",ioport[0xaa]&0xf,keymap[ioport[0xaa]&0xf]);

            return keymap[ioport[0xaa]&0xf];

        default:
            break;

    }

    return ioport[address&0xff];

}

static void io_write(void *context, uint16_t address, uint8_t data)
{

//    printf("[%02x:%04x]",address&0xff,Z80_PC(cpu));
//    printf("[%02x:%02x]",address&0xff,data);


    uint8_t b;

    address&=0xff;

    switch(address) {

// DEBUG

        // case 0xa8:
        //     ioport[0xa8]=data;
        //     return;

// DEBUG

#ifdef USE_OPLL

        case 0x7d:
            OPLL_writeReg(msxopll,ioport[0x7c],data);            
            return;
#endif

        case 0x98:  // VDP Write
            vrEmuTms9918WriteData(mainscreen,data);
            return;

        case 0x99:  // VDP control
            vrEmuTms9918WriteAddr(mainscreen,data);
            return;

        case 0xa0:  // PSG register
            ioport[0xa0]=data;
            return;

        case 0xa1:  // PSG data
#ifdef USE_I2S
            PSG_writeReg(msxpsg,ioport[0xa0],data);
#else
            psg_write(data);
#endif
            if(ioport[0xa0]==0xf) {
                if(data&0x80) {
                    key_kana=0;
                } else {
                    key_kana=1;
                }
                if(data&0x40) {
                    gamepad_select=1;
                } else {
                    gamepad_select=0;
                }
            }

            return;

        case 0xaa: // i8255 Port C

            if(data&0x10) {  // CMT Remote
                tape_ready=0;
            } else {
                tape_ready=1;
                if(ioport[0xaa]&0x10) {
                    tape_phase=0;
                }
            }

            if(data&0x40) {    // Key Caps LED
                key_caps=0;
            } else {              
                key_caps=1;
            }

            if(data&0x80) {    // Beep
                beep_enable=1;
            } else {
                beep_enable=0;
            }

            ioport[0xaa]=data;
            return;

        case 0xab:  // i8255 Control

            if((data&0x80)==0) { // Bit operation

                b=(data&0x0e)>>1;

                if(b==4) {  // CMT Remote
                    if(data&1) {
//printf("[CMT OFF]");
                        tape_ready=0;

                    } else {
//printf("[CMT ON]");
                        tape_ready=1;
                        tape_phase=0;
                    }
                }


                if(b==5) {  // Tape Out
                    tapeout(data&1);
                }

                if(b==6) {
                    if(data&1) {
                        key_caps=0;
                    } else {
                        key_caps=1;
                    }
                }

                if(b==7) {  // Beep
                    if(data&1) {
                        beep_enable=1;
                    } else {
                        beep_enable=0;
                    }
                }

                if(data&1) {
                    ioport[0xaa]|= 1<<b;
                } else {
                    ioport[0xaa]&= ~(1<<b);
                }

            }


            return;

        case 0xfc:      // Memory mapper
        case 0xfd:
        case 0xfe:
        case 0xff:
            ioport[address&0xff]=data;
            memmap[address&3]=data&7;
            return;

        default:
            break;
    }

    ioport[address&0xff]=data;

    return;

}

static uint8_t ird_read(void *context,uint16_t address) {

    // MSX Use Interrupt Mode 1

    z80_int(&cpu,FALSE);

    return 0xff;        // RST38

}

static void reti_callback(void *context) {

//    printf("RETI");

    // subcpu_enable_irq=0;
    // timer_enable_irq=0;
    // subcpu_irq_processing=0;
    // subcpu_command_processing=0;
    // subcpu_ird=0xff;

    // z80_int(&cpu,FALSE);

}


#ifdef USE_I2S
void i2s_init(void){

    PIO audio_pio=pio1;

    gpio_set_function(14, GPIO_FUNC_PIO1);
    gpio_set_function(15, GPIO_FUNC_PIO1);
    gpio_set_function(15 + 1, GPIO_FUNC_PIO1);

    uint offset = pio_add_program(audio_pio, &audio_i2s_program);

    audio_i2s_program_init(audio_pio, 0, offset, 14, 15);

    // clock 

    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    uint32_t divider = system_clock_frequency * 4 / SAMPLING_FREQ; // avoid arithmetic overflow
    pio_sm_set_clkdiv_int_frac(audio_pio, 0 , divider >> 8u, divider & 0xffu);

    printf("I2S clock divider %d\n",divider);

    for(int i=0;i<I2S_NUMSAMPLES*2;i++) {
        i2s_buffer0[i]=0;
        i2s_buffer1[i]=0;
    }

    // Initialize sound chips

    // Static instances — initialise in place instead of malloc + New()
    PSG_init(msxpsg, 3579545/2, SAMPLING_FREQ);
    PSG_setVolumeMode(msxpsg, 2);
    PSG_reset(msxpsg);

    SCC_init(msxscc1, 3579545, SAMPLING_FREQ);
    SCC_reset(msxscc1);

    SCC_init(msxscc2, 3579545, SAMPLING_FREQ);
    SCC_reset(msxscc2);

#ifdef USE_OPLL
//    msxopll = OPLL_new(3579545, SAMPLING_FREQ);
      msxopll = OPLL_new(3600000, 3600000/72);
    OPLL_setChipType(msxopll, 0);
    OPLL_reset(msxopll);
    // printf("[OPLL:%x]",msxopll);
#endif

}

void i2s_dma_init(void) {

    dma_channel_config c0 = dma_channel_get_default_config(i2s_chan_0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, DREQ_PIO1_TX0) ;
    channel_config_set_chain_to(&c0, i2s_chan_1);
    channel_config_set_ring(&c0, false, 5);                               // Set ring buffer to 3 bits depth (8 words) 

    dma_channel_configure(
        i2s_chan_0,                 
        &c0,                        
        &pio1->txf[0],          
        &i2s_buffer0,            
        I2S_NUMSAMPLES,                    
        false
    );

    dma_channel_config c1 = dma_channel_get_default_config(i2s_chan_1); 
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true); 
    channel_config_set_write_increment(&c1, false);
    channel_config_set_dreq(&c1, DREQ_PIO1_TX0);
    channel_config_set_chain_to(&c1, i2s_chan_0);
    channel_config_set_ring(&c1, false, 5);                               // Set ring buffer to 3 bits depth (8 words) 

    dma_channel_configure(
        i2s_chan_1,                         
        &c1,                                
        &pio1->txf[0],  
        &i2s_buffer1,                   
        I2S_NUMSAMPLES,                     
        false                               
    );

    dma_channel_start(i2s_chan_0);
//    dma_channel_start(i2s_chan_1);

}

static inline void i2s_process(void) {

    int16_t wave,wave2;

    if(dma_channel_is_busy(i2s_chan_0)==true) {
        if(i2s_active_dma!=i2s_chan_0) {
            i2s_active_dma=i2s_chan_0;
            for(int i=0;i<7;i++) {
#ifndef USE_OPLL
               wave=PSG_calc(msxpsg);
               if(cart_enable[0]) { wave+=SCC_calc(msxscc1); }
               if(cart_enable[1]) { wave+=SCC_calc(msxscc2); }
               if(beep_enable) wave+=0x1000;
#else
                wave=wave_buffer[wave_out++];
                wave_out&=0x7;
                wave+=OPLL_calc(msxopll);
#endif
                i2s_buffer1[i]=wave;
            }
        }
    } else if (dma_channel_is_busy(i2s_chan_1)==true) {
        if(i2s_active_dma!=i2s_chan_1) {
            i2s_active_dma=i2s_chan_1;
            for(int i=0;i<7;i++) {
#ifndef USE_OPLL
               wave=PSG_calc(msxpsg);
               if(cart_enable[0]) { wave+=SCC_calc(msxscc1); }
               if(cart_enable[1]) { wave+=SCC_calc(msxscc2); }
               if(beep_enable) wave+=0x1000;
#else
                wave=wave_buffer[wave_out++];
                wave_out&=0x7;
                wave+=OPLL_calc(msxopll);
#endif
                i2s_buffer0[i]=wave;
            }
        }
    }

}
#endif

void init_emulator(void) {
//  setup emulator 

    // Initial Bank selection
    ioport[0xa8]=0;

    for(int i=0;i<11;i++) {
        keymap[i]=0xff;
    }

    key_kana=0;
    key_caps=0;

    for(int i=0;i<8;i++) {
        megarom[i]=0;
    }

#ifdef USE_I2S
    PSG_reset(msxpsg);
    SCC_reset(msxscc1);
    SCC_reset(msxscc2);
#ifdef USE_OPLL
    OPLL_reset(msxopll);
#endif
#else
    psg_reset(0);
#endif

    tape_ready=0;
    tape_leader=0;

    fdc_init();

    gamepad_info=0x3f;

    memmap[0]=3;
    memmap[1]=2;
    memmap[2]=1;
    memmap[3]=0;

}

// main_core1 removed — core1 is now used by dvi_adapter for PicoDVI scan loop.

int main() {

    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

    static uint32_t hsync_wait,vsync_wait;

    // ROM sizes discovered pre-core1 (>0 means a ROM was found / flashed).
    uint32_t boot_rom_size[2] = {0, 0};

    // 1. Core voltage + clock — must be done before stdio/DVI init.
    //    board_init() is intentionally NOT called (it resets clock to 120MHz).
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(SYSCLOCK_KHZ, true);
    sleep_ms(10);

    stdio_init_all();

    // 2. UART on GPIO4/5 via uart1 (RP2040-PiZero; was GPIO12/13 uart0)
    uart_init(uart1, 9600);
    gpio_set_function(4, GPIO_FUNC_UART);   // TX
    gpio_set_function(5, GPIO_FUNC_UART);   // RX

    // 3. SD card — init before DVI so we can flash ROMs before core1 starts.
    sd_init();

    // Pre-core1 ROM flash: core1 (DVI) is not running yet, so XIP is safe to
    // disable for flash_range_erase / flash_range_program.
    // SELROM1 / SELROM2 on the SD card hold the full path of the last ROM
    // selected in the menu (e.g. "/msx/GAME.ROM").
    // On watchdog reboot (triggered by menu): stream ROM from SD to flash.
    // On hardware / power-on reset: ROM already in flash — just read size.
    {
        static const char * const selrom_files[2] = { "/msx/SELROM1", "/msx/SELROM2" };
        static const uint32_t cart_bases[2] = { CART1BASE, CART2BASE };
        bool watchdog_boot = watchdog_enable_caused_reboot();

        for (int slot = 0; slot < 2; slot++) {
            char rompath[80] = {0};
            UINT br;
            FIL f;
            if (f_open(&f, selrom_files[slot], FA_READ) == FR_OK) {
                f_read(&f, rompath, sizeof(rompath) - 1, &br);
                f_close(&f);
                rompath[br] = '\0';
                while (br > 0 && (rompath[br-1] == '\n' || rompath[br-1] == '\r'))
                    rompath[--br] = '\0';
            }
            if (rompath[0] == '\0') continue;

            printf("[BOOT] slot%d path=%s watchdog=%d\n", slot, rompath, (int)watchdog_boot);
            uart_tx_wait_blocking(uart1);

            if (watchdog_boot) {
                boot_rom_size[slot] = flash_rom_from_sd(rompath, cart_bases[slot]);
            } else {
                // ROM already in flash — open SD file only to get its size.
                FIL sf;
                if (f_open(&sf, rompath, FA_READ) == FR_OK) {
                    boot_rom_size[slot] = (uint32_t)f_size(&sf);
                    f_close(&sf);
                }
            }
        }
    }

    // 4. DVI init — launches core1, claims pio0 and DMA channels 0-5
    dvi_adapter_init();

    // 4b. Nunchuck I2C joystick on I2C1/GPIO2+3
    nunchuck_init();

    // 4b2. NES controller on GPIO10/11/12
    nes_init();

    // 4. PWM audio on GPIO6 (RP2040-PiZero; was GPIO10)
    gpio_set_function(6, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(6);

    pwm_set_wrap(pwm_slice_num, 256);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // 5. PSG/sound timer
    add_repeating_timer_us(1000000/SAMPLING_FREQ, sound_handler, NULL, &timer2);

    // 6. USB host (hardware USB on GPIO15/16, no PIO-USB needed)
    tuh_init(0);


//    video_cls();

    video_hsync=0;
    video_vsync=0;

    // video_mode=0;
    // fbcolor=0x7;

// uart handler

    // irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    // irq_set_enabled(UART0_IRQ,true);
    // uart_set_irq_enables(uart0,true,false);

    // HSync repeating timer — replaces the old VGA PIO0_IRQ_0 handler.
    // Fires every 64µs to pace Z80 execution; accumulates into VSync at 262 ticks.
    add_repeating_timer_us(-64, hsync_cb, NULL, &timer);

// mount littlefs
    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
       cursor_x=0;
       cursor_y=0;
       fbcolor=7;
       video_print("Initializing LittleFS...");
       // format
       lfs_format(&lfs,&PICO_FLASH_CFG);
       lfs_mount(&lfs,&PICO_FLASH_CFG);
   }

    mainscreen = vrEmuTms9918New();
    menuscreen = vrEmuTms9918New();
    // Enable DVI display now that mainscreen is valid.
    dvi_adapter_set_screen(mainscreen);

    menuinit();

    // Apply cart state for any ROM that was already in flash at boot time.
    for (int slot = 0; slot < 2; slot++) {
        if (boot_rom_size[slot] > 0) {
            cart_type_checker(slot, boot_rom_size[slot]);
            cart_loaded[slot] = 1;
            cart_enable[slot] = 1;
        }
    }

    menumode=1;  // Pause emulator

    init_emulator();

    cpu.read = mem_read;
    cpu.write = mem_write;
    cpu.in = io_read;
    cpu.out = io_write;
	cpu.fetch = mem_read;
    cpu.fetch_opcode = mem_read;
    cpu.reti = reti_callback;
    cpu.inta = ird_read;

    z80_power(&cpu,true);
    z80_instant_reset(&cpu);

    cpu_hsync=0;
    cpu_cycles=0;


    lfs_handler=lfs;
//    fdc_init(diskbuffer);

    fd_drive_status[0]=0;

    // start emulator
    
    menumode=0;

    while(1) {

#ifdef USE_OPLL
        // Fill sound data on core 0
        while(wave_in!=wave_out) {
               int16_t wave=PSG_calc(msxpsg);
               if(cart_enable[0]) { wave+=SCC_calc(msxscc1); }
               if(cart_enable[1]) { wave+=SCC_calc(msxscc2); }
               if(beep_enable) wave+=0x1000;
               wave_buffer[wave_in++]=wave;
               wave_in&=0x7;
        }
#endif

        if(menumode==0) { // Emulator mode

        cpu_cycles += z80_run(&cpu,1);
        cpu_clocks++;

        // Wait

//        if((cpu_cycles-cpu_hsync)>1 ) { // 63us * 3.58MHz = 227
//#ifndef USE_OPLL
        if((!cpu_boost)&&(cpu_cycles-cpu_hsync)>198 ) { // 63us * 3.58MHz = 227

            while(video_hsync==0) ;
            cpu_hsync=cpu_cycles;
            video_hsync=0;
        }
//#endif

        if((video_vsync==2)&&(cpu.iff1)) {
            if(vrEmuTms9918RegValue(mainscreen,TMS_REG_1)&0x20) { // VDP Enable interrupt
                z80_int(&cpu,true);
            }
        }

        if(video_vsync==1) { // VBlank
            // Tell core1 which screen to render (non-blocking).
            VrEmuTms9918 *screen = (menumode == 0) ? mainscreen : menuscreen;
            dvi_adapter_set_screen(screen);

            nunchuck_poll();
            nes_poll();
            tuh_task();
            process_kbd_leds();
            video_vsync=2;
            vsync_scanline=_hsync_count;

            if((tape_autoclose)&&(save_enabled==2)) {
                if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                    save_enabled=0;
                    lfs_file_close(&lfs,&lfs_file);
                }
            }

            if((tape_autoclose)&&(load_enabled==2)) {
                if((cpu_cycles-tape_cycles)>TAPE_THRESHOLD) {
                    load_enabled=0;
                    lfs_file_close(&lfs,&lfs_file);
                }
            }


        }

        } else { // Menu Mode


            unsigned char str[80];
            
            if(menuprint==0) {

                video_cls();
//                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=0;
            cursor_y=0;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=0;
            cursor_y=1;
            video_print(str);

            sprintf(str,"TAPE:%x",tape_ptr);
            cursor_x=0;
            cursor_y=2;
            video_print(str);


            cursor_x=3;            
            cursor_y=6;
            if(save_enabled==0) {
                video_print("SAVE: empty");
            } else {
                sprintf(str,"SAVE: %8s",tape_filename);
                video_print(str);
            }
            cursor_x=3;
            cursor_y=7;

            if(load_enabled==0) {
                video_print("LOAD: empty");
            } else {
                sprintf(str,"LOAD: %8s",tape_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=8;

            if(cart_loaded[0]==0) {
                video_print("Slot1: empty");
            } else {
                sprintf(str,"Slot1: %8s",cart1_filename);
                video_print(str);
            }

            cursor_x=4;
            cursor_y=9;

            if(cart_enable[0]) {
                 video_print("Cart:Enable");
            } else {
                 video_print("Cart:Disable");
            }

            cursor_x=4;
            cursor_y=10;

            switch(carttype[0]) {
                case 0:
                    video_print("Type:Plain");
                    break;  
                case 1:
                    video_print("Type:ASCII 8K");
                    break;  
                case 2:
                    video_print("Type:ASCII 16K");
                    break;  
                case 3:
                    video_print("Type:Konami");
                    break;  
                case 4:
                    video_print("Type:KonamiSCC");
                    break;  
                default:
                    video_print("Type:Unknown");
                    break;  
            }

            cursor_x=3;
            cursor_y=11;

            if(cart_loaded[1]==0) {
                video_print("Slot2: empty");
            } else {
                sprintf(str,"Slot2: %8s",cart2_filename);
                video_print(str);
            }

            cursor_x=4;
            cursor_y=12;

            if(cart_enable[1]) {
                 video_print("Cart:Enable");
            } else {
                 video_print("Cart:Disable");
            }

            cursor_x=4;
            cursor_y=13;

            switch(carttype[1]) {
                case 0:
                    video_print("Type:Plain");
                    break;  
                case 1:
                    video_print("Type:ASCII 8K");
                    break;  
                case 2:
                    video_print("Type:ASCII 16K");
                    break;  
                case 3:
                    video_print("Type:Konami");
                    break;  
                case 4:
                    video_print("Type:KonamiSCC");
                    break;  
                default:
                    video_print("Type:Unknown");
                    break;  
            }

            cursor_x=3;
            cursor_y=14;
#ifdef USE_FDC
            if(fd_drive_status[0]==0) {
                video_print("FD: empty");
            } else {
                sprintf(str,"FD: %8s",fd_filename);
                video_print(str);
            }
#endif
//  TODO:
//    CHANGE Kana type


            cursor_x=3;
            cursor_y=15;

            video_print("DELETE File");

            cursor_x=3;
            cursor_y=16;

            if(cpu_boost) {
                 video_print("CPU:Fast");
            } else {
                 video_print("CPU:Normal");
            }

            cursor_x=3;
            cursor_y=17;

            if(key_kana_jis) {
                 video_print("KANA:JIS");
            } else {
                 video_print("KANA:aiueo");
            }


            cursor_x=3;
            cursor_y=18;

            video_print("Reset");

            cursor_x=3;
            cursor_y=19;

            video_print("PowerCycle");

            cursor_x=0;
            cursor_y=menuitem+6;
            video_print("->");

   // for DEBUG ...

//            cursor_x=0;
//             cursor_y=23;
//                  sprintf(str,"%04x %04x %04x %04x %04x",Z80_PC(cpu),Z80_AF(cpu),Z80_BC(cpu),Z80_DE(cpu),Z80_HL(cpu));
// //                 sprintf(str,"%04x",Z80_PC(cpu));
//                  video_print(str);

            static int dbg_vsync = 0;
            if(filelist==0) {
                printf("[MENU] draw_files start\n");
                draw_files(-1,0);
                printf("[MENU] draw_files done\n");
                filelist=1;
                dbg_vsync = 5;  // print next 5 vsync iterations
            }

            if(dbg_vsync) {
                printf("[MENU] pre-vsync vsync=%d mode=%d key=%d stall=%lu\n",
                       (int)video_vsync, (int)menumode, (int)keypressed,
                       (unsigned long)dvi_stall_count);
                uart_tx_wait_blocking(uart1);
            }
            // Timeout-guarded vsync wait: if vsync doesn't fire within ~100ms,
            // report it so we can detect a stopped timer.
            {
                uint32_t t0 = time_us_32();
                while(video_vsync==0) {
                    if ((time_us_32() - t0) > 100000u) {
                        printf("[MENU] vsync TIMEOUT stall=%lu\n",
                               (unsigned long)dvi_stall_count);
                        uart_tx_wait_blocking(uart1);
                        t0 = time_us_32();  // reset to keep printing if still stuck
                    }
                }
            }
            if(dbg_vsync) {
                printf("[MENU] post-vsync stall=%lu\n",
                       (unsigned long)dvi_stall_count);
                if(dbg_vsync == 1) {
                    // On last debug frame: dump TMS9918 state so we can check
                    // whether the display is actually enabled.
                    uint8_t r0 = vrEmuTms9918RegValue(menuscreen, TMS_REG_0);
                    uint8_t r1 = vrEmuTms9918RegValue(menuscreen, TMS_REG_1);
                    bool disp = vrEmuTms9918DisplayEnabled(menuscreen);
                    printf("[MENU] TMS REG0=%02x REG1=%02x disp=%d screen=%p\n",
                           r0, r1, (int)disp, (void*)menuscreen);
                }
                uart_tx_wait_blocking(uart1);
                dbg_vsync--;
            }

            video_vsync=0;
            dvi_adapter_set_screen(menuscreen);

                tuh_task();

                if(keypressed==0x52) { // Up
                    cursor_x=0;
                    cursor_y=menuitem+6;
                    video_print("  ");
                    keypressed=0;
                    if(menuitem>0) menuitem--;
#ifndef USE_FDC
                    if(menuitem==8) menuitem--;
#endif
                }

                if(keypressed==0x51) { // Down
                    cursor_x=0;
                    cursor_y=menuitem+6;
                    video_print("  ");
                    keypressed=0;
                    if(menuitem<13) menuitem++; 
#ifndef USE_FDC
                    if(menuitem==8) menuitem++;
#endif
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE
                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                // tape_phase=0;
                                tape_ptr=0;
                                // tape_count=0;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD
                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDONLY);
                                load_enabled=1;
                                // tape_phase=0;
                                tape_ptr=0;
                                // tape_count=0;
//                                file_cycle=cpu.PC;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { // Slot Load

                        if (sd_available()) {
                            // SD card present — save ROM path and reboot.
                            // flash_rom_from_sd() runs pre-core1 on the next boot.
                            char sd_path[64] = {0};
                            if (sd_file_selector(sd_path, sizeof(sd_path)) == 0) {
                                char sd_full_path[80];
                                snprintf(sd_full_path, sizeof(sd_full_path), "/msx/%s", sd_path);
                                FIL fil;
                                if (f_open(&fil, "/msx/SELROM1", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
                                    UINT bw;
                                    f_write(&fil, sd_full_path, strlen(sd_full_path), &bw);
                                    f_close(&fil);
                                }
                                printf("[MENU] slot0 SELROM1=%s rebooting\n", sd_full_path);
                                uart_tx_wait_blocking(uart1);
                                watchdog_enable(1, 1);
                                while (1);
                            }
                        } else {
                            // No SD card — fall back to LittleFS
                            uint32_t res=file_selector();
                            if(res==0) {
                                memcpy(cart1_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_cart1,cart1_filename,LFS_O_RDONLY);
                                if(cart_size_check(0)==0) {
                                    if(cart_compare(0)!=0) {
                                        cart_write(0);
                                    }
                                    cart_type_checker(0, (uint32_t)lfs_file_size(&lfs,&lfs_cart1));
                                    cart_loaded[0]=1;
                                    cart_enable[0]=1;
                                } else {
                                    cart_loaded[0]=0;
                                }
                                lfs_file_close(&lfs,&lfs_cart1);
                            }
                        }

                        // Reinitialise menuscreen: the SD file selector and flash
                        // operations may have trashed the TMS9918 VRAM.
                        menuinit();
                        menuprint=0;
                        // Drain USB events and force display back to menu.
                        // Flash ops hold interrupts off for up to 400ms; stale events
                        // (e.g. a buffered F12) must be consumed before the menu loop
                        // checks keypressed, otherwise menumode becomes 0 → black screen.
                        tuh_task();
                        keypressed = 0;
                        // Belt-and-suspenders: force menu state regardless of what
                        // any USB event or race may have done during flash ops.
                        menumode = 1;
                        dvi_adapter_set_screen(menuscreen);
                    }

                    if(menuitem==3) { // Cart enable/disable
                        cart_enable[0]++;
                        if(cart_enable[0]>1) cart_enable[0]=0;
                        menuprint=0;
                    }

                    if(menuitem==4) { // Cart enable/disable
                        carttype[0]++;
                        if(carttype[0]>4) carttype[0]=0;
                        menuprint=0;
                    }

                    if(menuitem==5) { // Slot2 Load

                        if (sd_available()) {
                            // SD card present — save ROM path and reboot.
                            // flash_rom_from_sd() runs pre-core1 on the next boot.
                            char sd_path[64] = {0};
                            if (sd_file_selector(sd_path, sizeof(sd_path)) == 0) {
                                char sd_full_path[80];
                                snprintf(sd_full_path, sizeof(sd_full_path), "/msx/%s", sd_path);
                                FIL fil;
                                if (f_open(&fil, "/msx/SELROM2", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
                                    UINT bw;
                                    f_write(&fil, sd_full_path, strlen(sd_full_path), &bw);
                                    f_close(&fil);
                                }
                                printf("[MENU] slot1 SELROM2=%s rebooting\n", sd_full_path);
                                uart_tx_wait_blocking(uart1);
                                watchdog_enable(1, 1);
                                while (1);
                            }
                        } else {
                            // No SD card — fall back to LittleFS
                            uint32_t res=file_selector();
                            if(res==0) {
                                memcpy(cart2_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_cart2,cart2_filename,LFS_O_RDONLY);
                                if(cart_size_check(1)==0) {
                                    if(cart_compare(1)!=0) {
                                        cart_write(1);
                                    }
                                    cart_type_checker(1, (uint32_t)lfs_file_size(&lfs,&lfs_cart2));
                                    cart_loaded[1]=1;
                                    cart_enable[1]=1;
                                } else {
                                    cart_loaded[1]=0;
                                }
                                lfs_file_close(&lfs,&lfs_cart2);
                            }
                        }

                        menuinit();
                        menuprint=0;
                        tuh_task();
                        keypressed = 0;
                        menumode = 1;
                        dvi_adapter_set_screen(menuscreen);
                    }

                    if(menuitem==6) { // Cart enable/disable
                        cart_enable[1]++;
                        if(cart_enable[1]>1) cart_enable[1]=0;
                        menuprint=0;
                    }

                    if(menuitem==7) { // Cart enable/disable
                        carttype[1]++;
                        if(carttype[1]>4) carttype[1]=0;
                        menuprint=0;
                    }

                    if(menuitem==8) {  // FD
                        if(fd_drive_status[0]==0) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(fd_filename,filename,16);
                                lfs_file_open(&lfs,&fd_drive[0],fd_filename,LFS_O_RDONLY);
                                fdc_check(0);
                            }
                        } else {
                            lfs_file_close(&lfs,&fd_drive[0]);
                            fd_drive_status[0]=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==9) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuinit();
                        menuprint=0;

                    }

                    if(menuitem==10) { 
                        cpu_boost++;
                        if(cpu_boost>1) cpu_boost=0;
                        menuprint=0;
                    }


                    if(menuitem==11) { 
                        key_kana_jis++;
                        if(key_kana_jis>1) key_kana_jis=0;
                        menuprint=0;
                    }

                    if(menuitem==12) { // Reset
                        menumode=0;
                        menuprint=0;
                    
                        init_emulator();
                        z80_power(&cpu,true);

                    }

                    if(menuitem==13) { // PowerCycle
                        menumode=0;
                        menuprint=0;

                        memset(mainram,0,0x10000);
                        memset(ioport,0,0x100);

                        init_emulator();

//                        z80_instant_reset(&cpu);
                        z80_power(&cpu,true);

                    }

                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                //  break;     // escape from menu
                }

        }


    }

}
