/**
 ************************************************************************
 * @file main.c
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Initialises scu threads
 **********************************************************************
 * */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <data/json.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include "scu_ble.h"
//#include "scu_io.h"

K_THREAD_DEFINE(ble_connect, 1024, thread_ble_connect, NULL, NULL, NULL, THREAD_PRIORITY_BLE_CONNECT_THREAD, 0, 0);
K_THREAD_DEFINE(ble_discover, 1024, thread_ble_discover, NULL, NULL, NULL, THREAD_PRIORITY_BLE_CONNECT_THREAD, 0, 0);
K_THREAD_DEFINE(send_message, 1024, thread_send_message, NULL, NULL, NULL, 20, 0, 0);
// K_THREAD_DEFINE(get_message, 4096, thread_get_message, NULL, NULL, NULL, 20, 0, 0);
// K_THREAD_DEFINE(read_button, 512, thread_read_button, NULL, NULL, NULL, 20, 0, 0);

void main(void)
{

    // led_gpio_enable();
    // init_buzzer();
    // init_hts();
    // init_lps();
    // init_ccs();
    // init_lis();
}