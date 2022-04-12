/*
 ************************************************************************
 * @file bluetooth_driver.h
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Blutetooth driver include file for thread definitions in main.
 **********************************************************************
 */

#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H

/*Define Stack Sizes for threads*/
#define BLE_CLIENT_STACK_SIZE 4096
#define BLE_READ_STACK_SIZE 4096
#define BLE_COMMAND_STACK_SIZE 1042
#define SAMP_COMMAND_STACK_SIZE 1042
#define BUTTON_BLE_STACK_SIZE 512

/*Define all threads for main.c*/
void ble_client(void *arg1, void *arg2, void *arg3);

void thread_ble_read(void *arg1, void *arg2, void *arg3);

void thread_ble_commands(void *arg1, void *arg2, void *arg3);

void thread_button_ble(void *arg1, void *arg2, void *arg3);

void thread_samp_commands(void *arg1, void *arg2, void *arg3);

#endif /*BLUETOOTH_DRIVER_H*/