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

#define THREAD_BLE_CONNECT_STACK2 4096
#define THREAD_PRIORITY_BLE_CONNECT_THREAD2 -3

void thread_ble_adv(void);

#endif