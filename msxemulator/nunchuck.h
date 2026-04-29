#pragma once
#include <stdint.h>

// Wii Nunchuck driver over I2C1 (SDA=GPIO2, SCL=GPIO3)
// nunchuck_joy_bits uses the same active-high bitmask as kbd_joy_bits:
//   0x0001 = left, 0x0002 = right, 0x0004 = up, 0x0008 = down
//   0x0010 = Z button (fire/A), 0x0020 = C button (B)

#ifdef __cplusplus
extern "C" {
#endif

void nunchuck_init(void);
void nunchuck_poll(void);

extern volatile uint32_t nunchuck_joy_bits;

#ifdef __cplusplus
}
#endif
