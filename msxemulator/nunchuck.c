#include "nunchuck.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define NUNCHUCK_I2C     i2c1
#define NUNCHUCK_SDA     2
#define NUNCHUCK_SCL     3
#define NUNCHUCK_ADDR    0x52
#define NUNCHUCK_FREQ    100000

// Joystick dead zone: values within THRESHOLD of center (128) are ignored
#define JOY_THRESHOLD    48

volatile uint32_t nunchuck_joy_bits = 0;

void nunchuck_init(void) {
    i2c_init(NUNCHUCK_I2C, NUNCHUCK_FREQ);
    gpio_set_function(NUNCHUCK_SDA, GPIO_FUNC_I2C);
    gpio_set_function(NUNCHUCK_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(NUNCHUCK_SDA);
    gpio_pull_up(NUNCHUCK_SCL);

    // Disable encryption (classic init sequence)
    uint8_t buf[2];
    buf[0] = 0xF0; buf[1] = 0x55;
    i2c_write_blocking(NUNCHUCK_I2C, NUNCHUCK_ADDR, buf, 2, false);
    sleep_ms(1);
    buf[0] = 0xFB; buf[1] = 0x00;
    i2c_write_blocking(NUNCHUCK_I2C, NUNCHUCK_ADDR, buf, 2, false);
    sleep_ms(1);
}

void nunchuck_poll(void) {
    // Request a new data sample
    uint8_t req = 0x00;
    if (i2c_write_blocking(NUNCHUCK_I2C, NUNCHUCK_ADDR, &req, 1, false) < 0) {
        nunchuck_joy_bits = 0;
        return;
    }

    sleep_us(200);

    // Read 6 bytes: jx, jy, ax, ay, az, buttons
    uint8_t data[6];
    if (i2c_read_blocking(NUNCHUCK_I2C, NUNCHUCK_ADDR, data, 6, false) < 0) {
        nunchuck_joy_bits = 0;
        return;
    }

    uint8_t jx   = data[0];   // 0-255, center ~128
    uint8_t jy   = data[1];   // 0-255, center ~128
    uint8_t btns = data[5];
    bool btn_z = !(btns & 0x01);  // Z button, active low
    bool btn_c = !(btns & 0x02);  // C button, active low

    uint32_t joy = 0;
    if (jy > 128 + JOY_THRESHOLD) joy |= 0x0004;  // up
    if (jy < 128 - JOY_THRESHOLD) joy |= 0x0008;  // down
    if (jx < 128 - JOY_THRESHOLD) joy |= 0x0001;  // left
    if (jx > 128 + JOY_THRESHOLD) joy |= 0x0002;  // right
    if (btn_z)                     joy |= 0x0010;  // Z = fire / Button A
    if (btn_c)                     joy |= 0x0040;  // C = Button B (MASK_KEY_USER2)

    nunchuck_joy_bits = joy;
}
