/**
 * @file main.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au) - s4427634
 * @brief Weather Station - Application Host Unit
 * @version 0.2
 * @date 2022-03-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "led_driver.h"
#include "shell_base.h"
#include "shell_time.h"
#include "shell_led.h"
#include "shell_scu.h"
#include "ble_base.h"
#include "pb_driver.h"
#include "ahu_data.h"

// Debug Settings ==============================================================
#define DEBUG_BLE_LED 0

// Logging Module ==============================================================
LOG_MODULE_REGISTER(log_main);

// Functions ===================================================================

/**
 * @brief Initialises the hardware and shell
 *
 */
void initialise(void)
{

        init_leds();
        setup_pb();
        begin_shell();
}

void main(void)
{

        initialise();
}

// Thread Defines ==============================================================

// START BLE BASE entry thread : Delayed Start (Wait for USB to be ready)
K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_ble_base, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(rx_data, THREAD_BLE_BASE_STACK, process_rx_data, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
K_THREAD_DEFINE(ahu_data_thread, THREAD_BLE_BASE_STACK, JSON_thread, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
// Start BLE LED Thread
#if DEBUG_BLE_LED
K_THREAD_DEFINE(ble_led, THREAD_BLE_LED_STACK, thread_ble_led, NULL, NULL, NULL, THREAD_PRIORITY_BLE_LED, 0, 0);
#endif