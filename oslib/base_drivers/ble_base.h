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

/* Debug Thread Stack size */
#define THREAD_BLE_BASE_STACK 4094
/* Debug Thread Priority */
#define THREAD_PRIORITY_BLE_BASE -2
#define THREAD_PRIORITY_READ_BASE -10

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000

/**
 * @brief Base thread to start the BLE stack
 *
 */
extern void thread_ble_base(void *, void *, void *);

/**
 * @brief Thread to print the BLE data to the terminal
 * 
 */
extern void thread_ble_terminal_print(void *, void *, void *);

#endif  // BLE_BASE_H