/**
 ************************************************************************
 * @file static_ble.h
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains required definitions of static_ble.c
 **********************************************************************
 **/

#ifndef STATIC_BLE_H
#define STATIC_BLE_H

#define THREAD_BLE_CONNECT_STACK2 4096
#define THREAD_PRIORITY_BLE_CONNECT_THREAD2 -3

void thread_ble_adv(void);

extern struct k_msgq ultra_msgq;

#endif /* STATIC_BLE_H */