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
            printk("Ultrasonic 1: %d \n"
                    "Ultrasonic 2: %d \n"
                    "Ultrasonic 3: %d \n"
                    "Ultrasonic 3: %d \n"
                    "Delta Movement: %d \n" 
                    "Current Heading: %d \n" 
                    "Current Time: %d \n"
                    "4011-A: %d \n"
                    "4011-B: %d \n"
                    "4011-C: %d \n"
                    "4011-D: %d \n"
                    "4011-E: %d \n"
                    "4011-F: %d \n"
                    "4011-G: %d \n"
                    "4011-H: %d \n"
                    "4011-I: %d \n"
                    "4011-J: %d \n"
                    "4011-K: %d \n"
                    "4011-L: %d \n",
                    rx_buff[US_1], rx_buff[US_2], rx_buff[US_3], rx_buff[US_4],
                    rx_buff[delta], rx_buff[head], rx_buff[time],
                    rx_buff[A], rx_buff[B], rx_buff[C], rx_buff[D],
                    rx_buff[E], rx_buff[F], rx_buff[G], rx_buff[H],
                    rx_buff[I], rx_buff[J], rx_buff[K], rx_buff[L]);
        }
    }
}