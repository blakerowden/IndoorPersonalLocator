# SPDX-License-Identifier: Apache-2.0

set(BOARD thingy52_nrf52832)
set(BOARD_FLASH_RUNNER jlink)
set(CONF_FILE prj.conf bt.conf)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ahu)

#Add Include Directories
include_directories(
                        ../../oslib/mobile_drivers/ble_mobile
                        ../../oslib/common
                        )

#Add Target Sources
target_sources(app PRIVATE
                        ../../oslib/mobile_drivers/ble_mobile/ble_mobile_scan.c
                        ../../oslib/mobile_drivers/ble_mobile/mobile_ble.c
                        ../../oslib/common/hci_driver.c
                        src/main.c
                        )

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)