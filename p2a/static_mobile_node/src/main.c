/**
 * @file main.c
 * @brief 
 *  Based on the SCU
 *  Mobile: Built to run on the Thingy:52
 *  Static: Built to run on the Argon (Connects to Ultrasonic ranges)
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

// Mode Select Settings =======================================================

__mode_t mode = MODE_MOBILE;
