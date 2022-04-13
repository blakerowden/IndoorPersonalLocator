/**
 ************************************************************************
 * @file scu_io.h
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains required definitions of scu_io.c
 **********************************************************************
 **/

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>
#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <shell/shell.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include "kernel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/pwm.h>
#include <drivers/regulator.h>

#ifndef SCU_BLE_H
#define SCU_BLE_H

#define LEDR_NODE DT_ALIAS(led0)
#define LEDR DT_GPIO_LABEL(LEDR_NODE, gpios)
#define PINR DT_GPIO_PIN(LEDR_NODE, gpios)
#define FLAGSR DT_GPIO_FLAGS(LEDR_NODE, gpios)

#define LEDG_NODE DT_ALIAS(led1)
#define LEDG DT_GPIO_LABEL(LEDG_NODE, gpios)
#define PING DT_GPIO_PIN(LEDG_NODE, gpios)
#define FLAGSG DT_GPIO_FLAGS(LEDG_NODE, gpios)

#define LEDB_NODE DT_ALIAS(led2)
#define LEDB DT_GPIO_LABEL(LEDB_NODE, gpios)
#define PINB DT_GPIO_PIN(LEDB_NODE, gpios)
#define FLAGSB DT_GPIO_FLAGS(LEDB_NODE, gpios)

#define RGB_RED 0
#define RGB_GREEN 1
#define RGB_BLUE 2

void led_gpio_enable(void);
int led_rgb_set(int rgb, int mode);
int led_rgb_get(int rgb);
int toggle_rgb(int rgb);

#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
void init_button(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
int button_get(void);
void init_buzzer(void);
void process_buzzer(int freq);

#endif