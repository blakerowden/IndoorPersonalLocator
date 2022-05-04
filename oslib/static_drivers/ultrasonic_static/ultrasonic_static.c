/**
 * @file ultrasonic.c
 * @author Liana van Teijlingen - 45802616
 * @version 
 * @date 2022-04-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>   
#include <sys_clock.h>
#include <timing/timing.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <zephyr/types.h>
#include <irq.h>
#include <stddef.h>
#include <logging/log.h>
#include <sys/printk.h>

#include "ultrasonic_static.h"

#define SLEEP_TIME_MS   1000

#define LED0_NODE DT_ALIAS(led0)

uint8_t trig = 10; //D5
uint8_t echo = 8; //D4

//static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

K_MSGQ_DEFINE(ultra_msgq, sizeof(uint32_t), 10, 4);

void thread_ultra_read(void) 
{
	uint32_t cm;
	uint64_t cycles;
	uint64_t ns;
	timing_t pulse, return_echo;
	uint8_t echo_read = 0;
	//struct data_ultra_t send_distance;

	const struct device *dev = device_get_binding("GPIO_1");

	gpio_pin_configure(dev, trig, GPIO_OUTPUT);
	gpio_pin_configure(dev, echo, GPIO_INPUT | GPIO_ACTIVE_HIGH | GPIO_INT_EDGE | GPIO_INT_DEBOUNCE);

	timing_init();

	uint64_t lock;

	while(1)
	{
		timing_start();
		lock = irq_lock();
		gpio_pin_set(dev, trig, 1);
		k_sleep(K_USEC(10));
		//printk("one: %d\n", gpio_pin_get(dev, trig));
		gpio_pin_set(dev, trig, 0);
		//printk("two: %d\n", gpio_pin_get(dev, trig));

		while (echo_read == 0)
		{
			echo_read = gpio_pin_get(dev, echo); 
		}
		pulse = timing_counter_get();

		while (echo_read == 1)
		{
			echo_read = gpio_pin_get(dev, echo); 
		}
		return_echo = timing_counter_get();
		irq_unlock(lock);


		cycles = timing_cycles_get(&pulse, &return_echo);
		ns = timing_cycles_to_ns(cycles);
		timing_stop();
		cm = 0.0000174816*ns; // + 2.7493855422; somehow the +c is bad, wild
		//printk("time: %lld,     dist: %d\n", ns, cm);
		
		if (k_msgq_put(&ultra_msgq, &cm, K_NO_WAIT) != 0)
		{
			k_msgq_purge(&ultra_msgq);
		}

		k_sleep(K_MSEC(50));

	}
}
