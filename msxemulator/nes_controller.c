// nes_controller.c — NES gamepad driver for msxemulator (RP2040-PiZero)
//
// Protocol: pulse LATCH high for 12 µs to latch all buttons into the shift
// register, then clock out 8 bits on DATA (active low) by toggling CLK.
// Button order: A, B, Select, Start, Up, Down, Left, Right.
//
// Wiring:
//   NES CLK   → GPIO10  (output)
//   NES LATCH → GPIO11  (output)
//   NES DATA  → GPIO12  (input, pull-up; controller pulls low when pressed)
//   NES VCC   → 3.3 V
//   NES GND   → GND

#include "nes_controller.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define NES_CLK   10
#define NES_LATCH 11
#define NES_DATA  12

// Pulse width in microseconds — NES spec minimum is 12 µs.
#define NES_PULSE_US 12

volatile uint32_t nes_joy_bits = 0;

void nes_init(void) {
    gpio_init(NES_CLK);
    gpio_init(NES_LATCH);
    gpio_init(NES_DATA);

    gpio_set_dir(NES_CLK,   GPIO_OUT);
    gpio_set_dir(NES_LATCH, GPIO_OUT);
    gpio_set_dir(NES_DATA,  GPIO_IN);

    gpio_pull_up(NES_DATA);   // controller pulls low when button pressed

    gpio_put(NES_CLK,   0);
    gpio_put(NES_LATCH, 0);
}

void nes_poll(void) {
    // Pulse LATCH to snapshot all button states into the shift register.
    gpio_put(NES_LATCH, 1);
    sleep_us(NES_PULSE_US);
    gpio_put(NES_LATCH, 0);
    sleep_us(NES_PULSE_US);

    // Clock out 8 bits. DATA is valid after the latch falls (bit 0 = A),
    // then after each rising CLK edge the next bit is presented.
    // Bit order: A, B, Select, Start, Up, Down, Left, Right.
    uint8_t raw = 0;
    for (int i = 0; i < 8; i++) {
        if (!gpio_get(NES_DATA))      // active low → pressed
            raw |= (1u << i);
        gpio_put(NES_CLK, 1);
        sleep_us(NES_PULSE_US);
        gpio_put(NES_CLK, 0);
        sleep_us(NES_PULSE_US);
    }

    // raw bit positions after shift-out:
    //   0=A  1=B  2=Select  3=Start  4=Up  5=Down  6=Left  7=Right
    uint32_t joy = 0;
    if (raw & (1u << 4)) joy |= 0x0004;  // Up
    if (raw & (1u << 5)) joy |= 0x0008;  // Down
    if (raw & (1u << 6)) joy |= 0x0001;  // Left
    if (raw & (1u << 7)) joy |= 0x0002;  // Right
    if (raw & (1u << 0)) joy |= 0x0010;  // A → Fire A
    if (raw & (1u << 1)) joy |= 0x0040;  // B → Fire B

    nes_joy_bits = joy;
}
