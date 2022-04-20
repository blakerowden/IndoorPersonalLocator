/**
 ************************************************************************
 * @file scu_ble.h
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains required definitions of scu_ble.c
 **********************************************************************
 **/

#ifndef SCU_BLE_H
#define SCU_BLE_H

#define THREAD_BLE_CONNECT_STACK 2048
#define THREAD_PRIORITY_BLE_LED_THREAD 20
#define THREAD_PRIORITY_BLE_CONNECT_THREAD -3

#define RGB_RED 0
#define RGB_GREEN 1
#define RGB_BLUE 2

extern struct k_sem sem_data_arrived;

void thread_ble_connect(void);
void thread_ble_discover(void *arg1, void *arg2, void *arg3);
void thread_ble_write(void *arg1, void *arg2, void *arg3);

#endif