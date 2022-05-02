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
#include "static_ble.h"
#include "shell_base.h"
#include "ultrasonic_static.h"
#include "hal_imu.h"

// Define Thread Settings ======================================================
#define BLE_CONNECT_THREAD_STACK    4096
#define BLE_DISCOVER_THREAD_STACK   4096
#define BLE_SCAN_THREAD_STACK       4096
#define IMU_THREAD_STACK            4096

#define BLE_CONNECT_THREAD_PRIORITY     2
#define BLE_DISCOVER_THREAD_PRIORITY    2
#define BLE_SCAN_THREAD_PRIORITY        2
#define IMU_THREAD_PRIORITY             2

#define MODE_MOBILE  0
#define MODE_STATIC  1


#endif //__MAIN_H__
