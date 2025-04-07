#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>

#define SEESAW_I2C_ADDR 0x49
#define SEESAW_ENCODER_BASE 0x11
#define SEESAW_ENCODER_POSITION 0x30
#define SEESAW_ENCODER_DELTA 0x40
#define SEESAW_ENCODER_INTENSET 0x10
#define SEESAW_ENCODER_INTENCLR 0x20
#define SEESAW_ENCODER_STATUS 0x00

#define SEESAW_GPIO_BASE 0x01
#define SEESAW_GPIO_BULK 0x04

#define BUTTON_UP_PIN 2
#define BUTTON_DOWN_PIN 4
#define BUTTON_LEFT_PIN 3
#define BUTTON_RIGHT_PIN 5
#define BUTTON_SELECT_PIN 1
#define SEESAW_GPIO_PULLENSET 0x0B
#define SEESAW_GPIO_DIRCLR_BULK 0x03

#define SEESAW_GPIO_INTENSET 0x08 // GPIO interrupt enable register
#define SEESAW_GPIO_INTENCLR 0x09 // GPIO interrupt clear register
#define SEESAW_GPIO_INTFLAG  0x0A

#define interrupt_buton_alias DT_ALIAS(interruptpin)
static const struct gpio_dt_spec ROTARY_ENC_INTERRUPT_PIN = GPIO_DT_SPEC_GET(interrupt_buton_alias, gpios); /*Calibration Button*/
static struct gpio_callback int_callback;                                                                   /* Callback function of Tare Button Interrupt*/

extern struct k_sem menu_sem; /* Seamaphore for Tare Task Synchronization*/

// uint8_t button_pressed =0;

static int seesaw_write_register(uint8_t regHigh, uint8_t regLow, const uint8_t *data, size_t length);
static int seesaw_read_register(uint8_t regHigh, uint8_t regLow, uint8_t *data, size_t length);
int32_t get_encoder_position(void);
static int32_t get_encoder_delta(void);
// static int enable_encoder_interrupts(void);
static int disable_encoder_interrupts(void);
int rot_encoder_init();
bool is_button_pressed(uint8_t pin);
uint32_t read_button_states(uint32_t pins);
static int enable_pullups(uint32_t pins);
static int configure_as_inputs(uint32_t pins);
static void log_button_states(uint32_t gpio_states);
int read_gpio_states(uint32_t *gpio_states);
static int seesaw_write8(uint8_t regHigh, uint8_t regLow, uint8_t value);
int enable_encoder_interrupt(uint8_t encoder);
static int enable_gpio_interrupts(uint32_t pins, bool enable);
int enable_button_interrupts(bool enable);
static int seesaw_read(uint8_t reg, uint8_t *value);
static int seesaw_write(uint8_t reg, uint8_t value);
 int clear_encoder_interrupt(void);

#endif