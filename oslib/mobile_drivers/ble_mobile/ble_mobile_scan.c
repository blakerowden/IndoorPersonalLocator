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
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <errno.h>
#include <stddef.h>
#include <sys/printk.h>
#include <usb/usb_device.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <sys/byteorder.h>

#include "ble_mobile_scan.h"
#include "log_driver.h"
#include "hci_driver.h"
#include "shell_scu.h"

static void start_scan(void);

static struct bt_conn *default_conn;

uint16_t corner_uuid[] = {0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                       0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};

/* Custom UUIDs For Mobile and it's GATT Attributes */
#define UUID_BUFFER_SIZE 16

int currentRSSI;

// Logging Module
LOG_MODULE_REGISTER(BLE_SCAN, INITIAL_BLE_LOG_LEVEL);


/**
 * @brief Used to parse the advertisement data
 *        in order to find the UUID of the service we are looking for.
 *
 */
static bool parse_device(struct bt_data *data, void *user_data)
{
  int i;
  int matchedCount = 0;

  LOG_DBG("[AD]: %u data_len %u\n", data->type, data->data_len);

  if (data->type == BT_DATA_UUID128_ALL)
  {

    uint16_t temp = 0;
    for (i = 0; i < data->data_len; i++)
    {
      temp = data->data[i];
      if (temp == corner_uuid[i])
      {
        matchedCount++;
      }
    }

    if (matchedCount == UUID_BUFFER_SIZE)
    {
        tx_buff[0] = currentRSSI;
        
        k_msleep(60);
        
        bt_le_scan_stop();
        start_scan();

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
                         struct net_buf_simple *ad)
{

  currentRSSI = rssi;

  if (default_conn)
  {
    return;
  }

  /* We're only interested in connectable events */
  if (type == BT_GAP_ADV_TYPE_ADV_IND ||
      type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
  {
    bt_data_parse(ad, parse_device, (void *)addr);
  }
}

/**
 * @brief Starts passive BLE scanning for nearby
 *          devices.
 */
static void start_scan(void)
{
  int err;

  err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
  if (err)
  {
    LOG_ERR("Scanning failed to start (err %d)\n", err);
    return;
  }
}


void thread_ble_mobile_scan(void)
{

  default_conn = NULL;

  LOG_INF("Bluetooth rssi scanning initialized\n");

  start_scan();
}