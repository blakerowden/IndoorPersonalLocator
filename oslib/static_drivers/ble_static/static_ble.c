/**
 ************************************************************************
 * @file scu_ble.c
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains code pertaining to bluetooth operation
 **********************************************************************
 **/

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>
#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <shell/shell.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include "kernel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/pwm.h>
#include <drivers/regulator.h>

#include "ble_uuid.h"
#include "hci_driver.h"
#include "static_ble.h"

/**
 * @brief Initialises bluetooth, and begins advertising data
 *            on BLE.
 *
 */
static void bt_ready(void)
{
    int err;

    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                        0xd8, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                        0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb), 
        BT_DATA_BYTES(0x00, 0x00)
    };

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    // bt_passkey_set(0xAA289);
    printk("Advertising successfully started\n");
}

extern struct k_msgq ultra_msgq; 

/**
 * @brief Enabled bluetooth, and sets connection callback handler, awaits
 *          central to connect to peripheral (mobile)
 *
 */
void thread_ble_adv(void)
{

    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_ready();

    while (1) {

        uint16_t read;

        if (k_msgq_get(&ultra_msgq, &read, K_FOREVER) == 0)
        {
            struct bt_data ad[] = {
                BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
                BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                    0xd8, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb), 
                BT_DATA_BYTES(read)
            };

            bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        }

    }

}