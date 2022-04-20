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

#include "main.h"

#define MODE MODE_MOBILE // MODE_MOBILE or MODE_STATIC

LOG_MODULE_REGISTER(log_main); // Logging Module

// Threads =====================================================================
#if MODE == MODE_MOBILE
K_THREAD_DEFINE(ble_mobile, BLE_CONNECT_THREAD_STACK, thread_ble_connect, NULL,
                NULL, NULL, BLE_CONNECT_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_mobile_discover, BLE_DISCOVER_THREAD_STACK,
                thread_ble_discover, NULL, NULL, NULL,
                BLE_DISCOVER_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_mobile_scan, BLE_SCAN_THREAD_STACK, thread_ble_mobile_scan,
                NULL, NULL, NULL, BLE_SCAN_THREAD_PRIORITY, 0, 0);
#endif

#if MODE == MODE_STATIC
// INSERT STATIC MODULE THREADS HERE
#endif