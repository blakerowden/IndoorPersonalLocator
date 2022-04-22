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

#define THREAD_BLE_CONNECT_STACK 4096
#define THREAD_PRIORITY_BLE_CONNECT_THREAD -3

extern struct k_sem sem_data_arrived;

void thread_ble_connect(void);
void thread_ble_discover(void *arg1, void *arg2, void *arg3);
void thread_ble_write(void *arg1, void *arg2, void *arg3);

#endif