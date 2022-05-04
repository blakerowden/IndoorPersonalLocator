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

#include "data_base.h"

#include "led_driver.h"

// Logging Module
LOG_MODULE_REGISTER(DATA, INITIAL_DATA_LOG_LEVEL);

/**
 * @brief Process incoming data from the SCU RX GATT attribute.
 *
 */
void process_rx_data(void) {
    while (true) {
        printk("\n");
        if (!k_sem_take(&sem_data_arrived, K_FOREVER)) {
            printk(
                "{ "
                "\"Ultrasonic_1\": %d, "
                "\"Ultrasonic_2\": %d, "
                "\"Ultrasonic_3\": %d, "
                "\"Ultrasonic_4\": %d, "
                "\"Delta\": %d, "
                "\"Heading\": %d, "
                "\"Time\": %d, "
                "\"4011-A\": %d, "
                "\"4011-B\": %d, "
                "\"4011-C\": %d, "
                "\"4011-D\": %d, "
                "\"4011-E\": %d, "
                "\"4011-F\": %d, "
                "\"4011-G\": %d, "
                "\"4011-H\": %d, "
                "\"4011-I\": %d, "
                "\"4011-J\": %d, "
                "\"4011-K\": %d, "
                "\"4011-L\": %d }\n",
                rx_buff[Ultrasonic_1], rx_buff[Ultrasonic_2],
                rx_buff[Ultrasonic_3], rx_buff[Ultrasonic_4], rx_buff[Delta],
                rx_buff[Heading], rx_buff[Time], rx_buff[Node_A],
                rx_buff[Node_B], rx_buff[Node_C], rx_buff[Node_D],
                rx_buff[Node_E], rx_buff[Node_F], rx_buff[Node_G],
                rx_buff[Node_H], rx_buff[Node_I], rx_buff[Node_J],
                rx_buff[Node_K], rx_buff[Node_L]);
        }
    }
}