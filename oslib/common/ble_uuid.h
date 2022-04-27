/**
 * @file ble_uuid.h
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief Contains the struct for the UUIDs
 * @version 0.1
 * @date 2022-03-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __BLE_ATT_H__
#define __BLE_ATT_H__

#include <bluetooth/uuid.h>

/* Custom UUIDs For Mobile and it's GATT Attributes */
#define UUID_BUFFER_SIZE 16

// Used to as a key to test against scanned UUIDs
uint16_t ble_uuid[] = {0xd5, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91, 
                       0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};
                       
//uint16_t scan_uuid[] = {0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
//                       0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};                       

static struct bt_uuid_128 node_scu = BT_UUID_INIT_128(
    0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_ahu = BT_UUID_INIT_128(
    0xe3, 0x68, 0x4d, 0xe0, 0xa9, 0xcf, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_tx = BT_UUID_INIT_128(
    0x2a, 0xae, 0xfc, 0xda, 0xa9, 0xd0, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

static struct bt_uuid_128 node_rx = BT_UUID_INIT_128(
    0x41, 0xbf, 0xbd, 0xba, 0xa9, 0xd0, 0x11, 0xec,
    0xb9, 0x09, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02);

#endif //__BLE_ATT_H__
