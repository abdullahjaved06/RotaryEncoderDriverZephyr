/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "rotary_encoder.h"

LOG_MODULE_REGISTER(ENCODER_DRIVER);
#define interrupt_buton_alias DT_ALIAS(interruptpin)
static const struct gpio_dt_spec ROTARY_ENC_INTERRUPT_PIN = GPIO_DT_SPEC_GET(interrupt_buton_alias, gpios); /*Calibration Button*/
static struct gpio_callback int_callback;

void main()
{
	int ret;
	ret = rot_encoder_init();
	if (ret != 0)
	{
		LOG_ERR("Failed to initialize rotary encoder");
		// return;
	}
	LOG_INF("Rotary encoder is ready.\n");
		/*enable Interrupts for Encoder*/
	ret = enable_encoder_interrupt(0); // Enable interrupt for encoder 0
	if (ret < 0)
	{
		LOG_ERR("Failed to enable encoder interrupt");
	}
	else
	{
		LOG_INF("Encoder interrupt enabled successfully");
	}
	k_msleep(500);
	/*enable Interrupts for GPIO's*/
	if (enable_button_interrupts(true) < 0)
	{
		LOG_ERR("Failed to enable button interrupts");
	}
}

void gpios_interrupt_init()
{
	int ret;
	/*INT button initialized as input pin*/
	ret = gpio_pin_configure_dt(&ROTARY_ENC_INTERRUPT_PIN, GPIO_INPUT);
	if (!device_is_ready(ROTARY_ENC_INTERRUPT_PIN.port))
	{
		printk("Error: gpio device %s is not ready\n", ROTARY_ENC_INTERRUPT_PIN.port->name);
		return;
	}
	if (ret != 0)
	{
		LOG_ERR("Error: failed to configure %s pin %d\n", ROTARY_ENC_INTERRUPT_PIN.port->name, ROTARY_ENC_INTERRUPT_PIN.pin);
		return;
	}

	/*Interrupt configured on Weight button pin*/
	ret = gpio_pin_interrupt_configure_dt(&ROTARY_ENC_INTERRUPT_PIN, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0)
	{
		printk("Error: failed to configure interrupt on %s pin %d\n", ROTARY_ENC_INTERRUPT_PIN.port->name, ROTARY_ENC_INTERRUPT_PIN.pin);
		return;
	}

	gpio_init_callback(&int_callback, interrupt_handler, BIT(ROTARY_ENC_INTERRUPT_PIN.pin));
	gpio_add_callback(ROTARY_ENC_INTERRUPT_PIN.port, &int_callback);
}