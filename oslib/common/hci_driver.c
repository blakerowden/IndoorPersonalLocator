/**
 * @file hci_driver.c
 * @author Blake Rowden (b.rowden@uqconnect.edu.au)
 * @brief
 * @version 0.1
 * @date 2022-03-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "hci_driver.h"

// Logging Module
LOG_MODULE_REGISTER(HCI, LOG_LEVEL_ERR);

uint8_t tx_buff[] = {
    0x0000, 0x0000, 0x0000, 0x0000,                  // US1, US2, US3, US4,
    0x0000, 0x0000, 0x0000,                          // Delta, Heading, Time
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  // Nodes 1-6
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000   // Nodes 7-12
};
uint8_t rx_buff[] = {
    0x0000, 0x0000, 0x0000, 0x0000,                  // US1, US2, US3, US4,
    0x0000, 0x0000, 0x0000,                          // Delta, Heading, Time
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  // Nodes 1-6
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000   // Nodes 7-12
};

static_node static_nodes[12] = {
    {"4011-A", {"F5:75:FE:85:34:67"}, 0x0000},
    {"4011-B", {"E5:73:87:06:1E:86"}, 0x0000},
    {"4011-C", {"CA:99:9E:FD:98:B1"}, 0x0000},
    {"4011-D", {"CB:1B:89:82:FF:FE"}, 0x0000},
    {"4011-E", {"F9:BD:57:FF:25:04"}, 0x0000},
    {"4011-F", {"C1:13:27:E9:B7:7C"}, 0x0000},
    {"4011-G", {"F1:04:48:06:39:A0"}, 0x0000},
    {"4011-H", {"CA:0C:E0:DB:CE:60"}, 0x0000},
    {"4011-I", {"D4:7F:D4:7C:20:13"}, 0x0000},
    {"4011-J", {"F7:0B:21:F1:C8:E1"}, 0x0000},
    {"4011-K", {"FD:E0:8D:FA:3E:4A"}, 0x0000},
    {"4011-L", {"EE:32:F7:28:FA:AC"}, 0x0000},
};

void clear_tx(void) {
    for (int i = 0; i < 19; i++) {
        tx_buff[i] = 0x0000;
    }
}

void clear_rx(void) {
    for (int i = 0; i < 19; i++) {
        rx_buff[i] = 0x0000;
    }
}
