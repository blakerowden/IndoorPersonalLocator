/**
 * @file ble_base.c
 * @author Boston O'Neill
 * @brief
 * @version 0.1
 * @date 2022-03-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "ble_mobile_scan.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <usb/usb_device.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "hci_driver.h"
#include "kernel.h"
#include "mobile_ble.h"

static void start_scan(void);

static struct bt_conn *default_conn;

uint16_t static_uuid[] = {0xd8, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                          0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};

int staticFound = 0;

/* Custom UUIDs For Mobile and it's GATT Attributes */
#define UUID_BUFFER_SIZE 16

int currentRSSI;

char currentString[BT_ADDR_LE_STR_LEN];

static bool parse_device(struct bt_data *data, void *user_data) {
    int i;
    int matchedCount = 0;

    if (data->type == BT_DATA_UUID128_ALL) {
        uint16_t temp = 0;
        for (i = 0; i < data->data_len; i++) {
            temp = data->data[i];
            if (temp == static_uuid[i]) {
                matchedCount++;
            }
        }

        if (matchedCount == UUID_BUFFER_SIZE) {
            staticFound = 1;
            printk("Found US");
            return true;
        } else {
            if (data->data_len == 3 && staticFound == 1) {
                uint8_t ultra_id = data->data[2];
                node_ultra[ultra_id] = (data->data[0] << 8) + data->data[1];
                printk("%d - US%d from %d and %d\n", node_ultra[ultra_id],
                       ultra_id, data->data[0], data->data[1]);
            }
            staticFound = 0;

            return false;
        }
    }

    return true;
}

/**
 * @brief Callback function for when scan detects device, scanned devices
 *          are filtered by their connectibilty and scan data is parsed.
 *
 * @param addr Device Address
 * @param rssi RSSI
 * @param type Device Type
 * @param ad Adv Data
 */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad) {
    bt_addr_le_to_str(addr, currentString, 18);

    for (int i = 0; i < 12; i++) {
        // printk("Expected:%s got:%s\n", static_nodes[i].address,
        // currentString);
        if (strcmp(currentString, static_nodes[i].address) == 0) {
            node_rssi[i] = rssi;
            node_timestamp[i] = k_uptime_get();
            // printk("RSSI of Device \"%s\" is:%d\n", currentString, rssi);
            // printk("Sending in node %d: %d\n", i, tx_buff[i+7]);
        }
    }

    bt_data_parse(ad, parse_device, (void *)addr);

    bt_le_scan_stop();
    k_msleep(2);
    start_scan();
    return;
}

/**
 * @brief Starts passive BLE scanning for nearby
 *          devices.
 */
static void start_scan(void) {
    int err;

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err) {
        printk("Scanning failed to start (err %d)\n", err);
        return;
    }
}

void thread_ble_mobile_scan(void) {
    /*default_conn = NULL;
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }*/

    start_scan();
}

void rssi_monitor_thread(void) {
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            if (node_timestamp[i] <= node_timestamp[j] - 500) {
                node_rssi[i] = 0;
            }
        }
    }

    k_msleep(5);
}