/**
 * @file main.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au) - s4427634
 * @brief Mobile/Static Node
 * @version 0.2
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "main.h"

//#define MODE MODE_MOBILE  // MODE_MOBILE or MODE_STATIC
#define MODE MODE_MOBILE

LOG_MODULE_REGISTER(log_main);  // Logging Module

// Threads =====================================================================
#if MODE == MODE_MOBILE
K_THREAD_DEFINE(ble_mobile, BLE_CONNECT_THREAD_STACK, thread_ble_connect, NULL,
                NULL, NULL, BLE_CONNECT_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_mobile_scan, BLE_SCAN_THREAD_STACK, thread_ble_mobile_scan,
                NULL, NULL, NULL, BLE_SCAN_THREAD_PRIORITY, 0, 500);
K_THREAD_DEFINE(read_imu, IMU_THREAD_STACK, thread_imu_rw,
                NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 500);
K_THREAD_DEFINE(rssi_monitor, BLE_SCAN_THREAD_STACK, rssi_monitor_thread,
                NULL, NULL, NULL, BLE_SCAN_THREAD_PRIORITY, 0, 500);
// MPU9250 Thread
//K_THREAD_DEFINE(imu, THREAD_IMU_RW_STACK, thread_read_imu, NULL, NULL, NULL, THREAD_PRIORITY_IMU, 0, 100);

#elif MODE == MODE_STATIC
    // INSERT STATIC MODULE SPECIFIC THREADS HERE
    K_THREAD_DEFINE(ultra_read, 2048, thread_ultra_read, NULL, NULL, NULL, 20, 0, 0);
    K_THREAD_DEFINE(ble_static, 8192, thread_ble_adv, NULL, NULL, NULL, 20, 0, 0);
#endif