/**
 * @file main.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au) - s4427634
 * @brief Base Node
 * @version 0.2
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "main.h"

// Debug Settings ==============================================================
#define DEBUG_BLE_LED 0

// Logging Module ==============================================================
LOG_MODULE_REGISTER(log_main);

// Functions ===================================================================

K_THREAD_STACK_DEFINE(ble_base_stack, THREAD_BLE_BASE_STACK);
K_THREAD_STACK_DEFINE(ble_terminal_stack, THREAD_BLE_BASE_STACK);

/**
 * @brief Enable USB Driver.
 *
 */
void main(void) {
    /* Setup DTR */
    const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    
    /* Enable the USB Driver */
    if (usb_enable(NULL)) return;
    /* Wait on DTR - 'Data Terminal Ready'*/
    while (!dtr) {
        uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    struct k_thread ble_base;
    struct k_thread ble_terminal;

   k_thread_create(
       &ble_base, ble_base_stack, K_THREAD_STACK_SIZEOF(ble_base_stack),
       thread_ble_base, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0,
       K_MSEC(100));

    k_thread_create(
        &ble_terminal, ble_terminal_stack, K_THREAD_STACK_SIZEOF(ble_terminal_stack),
        thread_ble_terminal_print, NULL, NULL, NULL, THREAD_PRIORITY_READ_BASE, 0,
        K_MSEC(100));
}