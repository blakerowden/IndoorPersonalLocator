/**
 * @file ahu_data.h
 * @author Blake Rowden
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef AHU_DATA_H
#define AHU_DATA_H

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <errno.h>
#include <stddef.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <usb/usb_device.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "ble_base.h"
#include "hci_driver.h"
#include "log_driver.h"

// Stores the Index for the data access
enum {
    Ultrasonic_1,
    Ultrasonic_2,
    Ultrasonic_3,
    Ultrasonic_4,
    Delta,
    Heading,
    Time,
    Node_A,
    Node_B,
    Node_C,
    Node_D,
    Node_E,
    Node_F,
    Node_G,
    Node_H,
    Node_I,
    Node_J,
    Node_K,
    Node_L
};

/**
 * @brief Process the RAW data into the data struct
 *
 */
void process_rx_data(void);

#endif  // AHU_DATA_H