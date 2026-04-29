#pragma once
#include <stdint.h>

// NES controller driver — bit-bang on GPIO10/11/12
// nes_joy_bits uses the same active-high bitmask as nunchuck_joy_bits:
//   0x0001 = left, 0x0002 = right, 0x0004 = up, 0x0008 = down
//   0x0010 = A button (fire A), 0x0040 = B button (fire B)

#ifdef __cplusplus
extern "C" {
#endif

void nes_init(void);
void nes_poll(void);

extern volatile uint32_t nes_joy_bits;

#ifdef __cplusplus
}
#endif
