/**
 * @file main.h
 * @brief
 * @version 0.1
 * @date 2022-04-13
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include <device.h>
#include <devicetree.h>
#include <errno.h>
#include <logging/log.h>
#include <stddef.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "ble_mobile_scan.h"
#include "mobile_ble.h"

// Define Thread Settings ======================================================
#define BLE_CONNECT_THREAD_STACK    8192
#define BLE_DISCOVER_THREAD_STACK   8192
#define BLE_SCAN_THREAD_STACK       8192

#define BLE_CONNECT_THREAD_PRIORITY     2
#define BLE_DISCOVER_THREAD_PRIORITY    2
#define BLE_SCAN_THREAD_PRIORITY        2

typedef enum {
    MODE_MOBILE = 0,
    MODE_STATIC = 1,
} mode_t;

#endif //__MAIN_H__
