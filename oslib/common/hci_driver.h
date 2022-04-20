/**
 * @file hci_driver.h
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HCI_DRIVER_H
#define HCI_DRIVER_H

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#define PREAMBLE 0xAA
#define REQUEST 0x01
#define RESPONSE 0x02

// Stores the DID's for the different devices
typedef enum
{
    NO_DEV,
    HTS221_T,
    HTS221_H,
    LPS22_AP,
    CCS811_VOC,
    LIS2DH_X_ACC,
    LIS2DH_Y_ACC,
    LIS2DH_Z_ACC,
    RGB_LED,
    BUZ,
    PB,
    DC,
    SAMPLE,
    ALL
} device_id;

// Stores the name and address of a static node
typedef struct
{
    char name[6];
    uint8_t address[6];
    uint16_t rssi
} static_node;

static_node[] static_nodes = {
    {"4011-A", {0xF5, 0x75, 0xFE, 0x85, 0x34, 0x67}, 0x0000},
    {"4011-B", {0xE5, 0x73, 0x87, 0x06, 0x1E, 0x86}, 0x0000},
    {"4011-C", {0xCA, 0x99, 0x9E, 0xFD, 0x98, 0xB1}, 0x0000},
    {"4011-D", {0xCB, 0x1B, 0x89, 0x82, 0xFF, 0xFE}, 0x0000},
    {"4011-E", {0xF9, 0xBD, 0x57, 0xFF, 0x25, 0x04}, 0x0000},
    {"4011-F", {0xC1, 0x13, 0x27, 0xE9, 0xB7, 0x7C}, 0x0000},
    {"4011-G", {0xF1, 0x04, 0x48, 0x06, 0x39, 0xA0}, 0x0000},
    {"4011-H", {0xCA, 0x0C, 0xE0, 0xDB, 0xCE, 0x60}, 0x0000},
    {"4011-I", {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x0000},
    {"4011-J", {0xF7, 0x0B, 0x21, 0xF1, 0xC8, 0xE1}, 0x0000},
    {"4011-K", {0xFD, 0xE0, 0x8D, 0xFA, 0x3E, 0x4A}, 0x0000},
    {"4011-L", {0xEE, 0x32, 0xF7, 0x28, 0xFA, 0xAC}, 0x0000},
}

extern uint16_t tx_buff[17]; // Stores the data to be sent to the AHU
extern uint16_t rx_buff[17]; // Stores the data received to the AHU

extern uint8_t stream_freq; // Stores the frequency of the data stream
extern uint8_t all_active;  // Stores the state of the all_active flag

/**
 * @brief Get the data length object
 *
 * @param data1
 * @param data2
 * @param data3
 * @param data4
 * @return uint8_t The length of the data in bytes
 */
uint8_t get_data_length(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);

/**
 * @brief Pages up the data into the HCI format for tx and rx
 *
 * @param type
 * @param data1
 * @param data2
 * @param data3
 * @param data4
 * @param data5
 * @return int
 */
int package_hci_message(uint8_t type, uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4, uint16_t data5);

/**
 * @brief Clears the TX buffer
 *
 */
void clear_tx(void);

/**
 * @brief Clears the RX buffer
 *
 */
void clear_rx(void);

#endif // HCI_DRIVER_H