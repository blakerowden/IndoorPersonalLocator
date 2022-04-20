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
typedef enum {
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

// Stores the Index for the data access
typedef enum {
    US_1,
    US_2,
    US_3,
    US_4,
    delta,
    head,
    time,
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L
} data_access;

//Stores the name and address of a static node
typedef struct
{
    char name[6];
    char address[17];
    uint16_t rssi;
} static_node;

extern static_node static_nodes[12];

extern uint16_t tx_buff[19]; // Stores the data to be sent to the AHU
extern uint16_t rx_buff[19]; // Stores the data received to the AHU

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