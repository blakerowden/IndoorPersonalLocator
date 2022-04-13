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

#define THREAD_BLE_LED_THREAD_STACK 1024
#define THREAD_BLE_CONNECT_STACK 2048
#define THREAD_PRIORITY_BLE_LED_THREAD 20
#define THREAD_PRIORITY_BLE_CONNECT_THREAD -3

#define RGB_RED 0
#define RGB_GREEN 1
#define RGB_BLUE 2

void thread_ble_connect(void);
void thread_ble_discover(void *arg1, void *arg2, void *arg3);

void thread_send_message(void);
void thread_get_message(void);
void thread_read_button(void);

#endif