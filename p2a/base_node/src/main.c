/**
 * @file main.c
 * @brief Main.c file for the base node.
 *  Based on the AHU
 *  Built to run on the NRF52840 USB Dongle.
 * @version 0.1
 * @date 2022-04-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "main.h"