/**
 * @file main.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au) - s4427634
 * @brief Weather Station - Application Host Unit
 * @version 0.2
 * @date 2022-03-15
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

#include "shell_base.h"
#include "mobile_ble.h"
#include "ble_mobile_scan.h"

// Debug Settings ==============================================================
#define DEBUG_BLE_LED 0

// Logging Module ==============================================================
LOG_MODULE_REGISTER(log_main);

// Functions ===================================================================

/**
 * @brief Initialises the hardware and shell
 *
 */
void initialise(void)
{
        begin_shell();
}

void main(void)
{

        initialise();
}

// Thread Defines ==============================================================

// START BLE BASE entry thread : Delayed Start (Wait for USB to be ready)

K_THREAD_DEFINE(ble_mobile, THREAD_BLE_CONNECT_STACK, thread_ble_connect, NULL, NULL, NULL, THREAD_PRIORITY_BLE_CONNECT_THREAD, 0, 0);
K_THREAD_DEFINE(ble_mobile_discover, THREAD_BLE_CONNECT_STACK, thread_ble_discover, NULL, NULL, NULL, THREAD_PRIORITY_BLE_CONNECT_THREAD, 0, 0);
K_THREAD_DEFINE(ble_mobile_scan, THREAD_BLE_SCAN_STACK, thread_ble_mobile_scan, NULL, NULL, NULL, THREAD_PRIORITY_BLE_SCAN, 100, 0);
