/**
 * @file ble_base.h
 * @author Boston O'Neill
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef BLE_MOBILE_SCAN_H
#define BLE_MOBILE_SCAN_H

/* Debug Thread Stack size */
#define THREAD_BLE_SCAN_STACK 4096
/* Debug Thread Priority */
#define THREAD_PRIORITY_BLE_SCAN -3

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000

/**
 * @brief Base thread to start the BLE stack
 *
 */

void thread_ble_mobile_scan(void);

void rssi_monitor_thread(void);

/**
 * @brief Write data to the SCU TX GATT attribute.
 *
 */
void scu_write(void);

#endif  // BLE_MOBILE_SCAN_H