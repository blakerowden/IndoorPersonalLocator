/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/printk.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define LED0_NODE DT_ALIAS(led0)
uint8_t trig = 10;
uint8_t echo = 8;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void thread_ultra_read(void)
{
	const struct device *ultra = device_get_binding("GPIO_1");
	if (ultra == NULL)
	{
		// TODO: DO SOMETHING
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(ultra, trig, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(ultra, echo, GPIO_INPUT);

	printk("configured\n");

	while (1) {
		gpio_pin_toggle_dt(&led);

		gpio_pin_set_raw(ultra, (gpio_pin_t) trig, 1);
		//printk("trig on\n");
		k_usleep(10);
		gpio_pin_set_raw(ultra, trig, 0);
		//printk("trig off\n");

		while (!gpio_pin_get_raw(ultra, echo))
		{
			//printk("no echo\n");
			//do nothing
		}

		int64_t now = k_uptime_get();

		while (gpio_pin_get_raw(ultra, echo))
		{
			//printk("echo echo\n");
			//do nothing
		}

		printk("return pulse: %lld\n", k_uptime_delta(&now));
		float dist = (0.5 * 34 * k_uptime_delta(&now));
		//printk("dist: %d\n", dist);

		k_msleep(SLEEP_TIME_MS);

	}
}

K_THREAD_DEFINE(ultra_read, 1024, thread_ultra_read, NULL, NULL, NULL, 20, 0, 0);