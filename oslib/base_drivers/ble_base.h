/**
 * @file ble_base.h
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef BLE_BASE_H
#define BLE_BASE_H

#define THREAD_BLE_BASE_STACK 8192

#define THREAD_PRIORITY_BLE_BASE 2
#define THREAD_PRIORITY_DATA_PROCESS -20

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000

extern struct k_sem sem_data_arrived;

/**
 * @brief Base thread to start the BLE stack
 *
 */
void thread_ble_base(void);

/**
 * @brief Write data to the SCU TX GATT attribute.
 *
 */
void scu_write(void);

#endif  // BLE_BASE_H