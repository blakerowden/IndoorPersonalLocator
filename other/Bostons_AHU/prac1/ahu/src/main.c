
/*
 ************************************************************************
 * @file main.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Main source code to start required threads for prac1.
 **********************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include "rgb_driver.h"
#include "bluetooth_driver.h"
#include "time_driver.h"
#include "shell_driver.h"
#include "button_driver.h"

void shell_entry(void *, void *, void *);

K_THREAD_DEFINE(shell_id, SHELL_STACK_SIZE, shell_entry, NULL, NULL, NULL, 3, 0, 0);

K_THREAD_DEFINE(ble_client_id, BLE_CLIENT_STACK_SIZE, ble_client, NULL, NULL, NULL, 3, 0, 0);

K_THREAD_DEFINE(ble_read, BLE_READ_STACK_SIZE, thread_ble_read, NULL, NULL, NULL, 3, 0, 0);

K_THREAD_DEFINE(ble_commands, BLE_COMMAND_STACK_SIZE, thread_ble_commands, NULL, NULL, NULL, 3, 0, 0);

K_THREAD_DEFINE(samp_commands, SAMP_COMMAND_STACK_SIZE, thread_samp_commands, NULL, NULL, NULL, 3, 0, 0);

K_THREAD_DEFINE(button_ble, BUTTON_BLE_STACK_SIZE, thread_button_ble, NULL, NULL, NULL, 3, 0, 0);

void main(void)
{

	init_button_driver();

	init_rgb_led();

}
