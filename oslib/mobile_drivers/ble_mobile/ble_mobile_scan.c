/**
 * @file ble_base.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
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
#include "mobile_ble.h"

static void start_scan(void);

static struct bt_conn *default_conn;

int j;

uint16_t static_uuid[] = {0xd8, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91, 
                       0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};

/* Custom UUIDs For Mobile and it's GATT Attributes */
#define UUID_BUFFER_SIZE 16

int currentRSSI;

char currentString[BT_ADDR_LE_STR_LEN];

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

    int temp = 0;
    int matchedCount = 0;

    for (int i = 0; i < 12; i++) {
        // printk("Expected:%s got:%s\n", static_nodes[i].address,
        // currentString);
        if (strcmp(currentString, static_nodes[i].address) == 0) {
            node_rssi[i] = rssi + 256;
            // printk("RSSI of Device \"%s\" is:%d\n", currentString, rssi);
            // printk("Sending in node %d: %d\n", i, tx_buff[i+7]);
        }
    }

    for (int i = 0; i < ad->len; i++) {
        temp = ad->data[i];
        if (temp == static_uuid[i]) {
            printk("Found match bit %d\n", i);
            matchedCount++;
        } else {
            break;
        }
    }

    if (matchedCount == UUID_BUFFER_SIZE) {
        // Do Nothing for now
    }

    bt_le_scan_stop();
    k_msleep(10);
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
    default_conn = NULL;

    j = 7;

    start_scan();
}