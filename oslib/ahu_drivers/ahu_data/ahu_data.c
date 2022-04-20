/**
 * @file ahu_data.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <errno.h>
#include <stddef.h>
#include <sys/printk.h>
#include <usb/usb_device.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include <sys/byteorder.h>

#include "ahu_data.h"
#include "ble_base.h"
#include "hci_driver.h"
#include "led_driver.h"
#include "log_driver.h"

typedef struct
{
    uint8_t temp;
    uint8_t humidity;
    uint8_t air_pressure;
    uint8_t VOC;
    uint8_t X;
    uint8_t Y;
    uint8_t Z;
    uint8_t rgb[3];
    uint8_t buzzer;
    bool push_button;
} ahu_data_t;

// Logging Module
LOG_MODULE_REGISTER(DATA, INITIAL_DATA_LOG_LEVEL);

ahu_data_t ahu_data;

/**
 * @brief Process incoming data from the SCU RX GATT attribute.
 *
 */
void process_rx_data(void)
{

    while (1)
    {

        if (!k_sem_take(&sem_data_arrived, K_FOREVER))
        {
            printk("[RAW RX]: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", rx_buff[0],
                    rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4], rx_buff[5]);
        }
    }
}