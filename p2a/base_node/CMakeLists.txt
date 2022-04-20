# SPDX-License-Identifier: Apache-2.0

set(BOARD nrf52840dongle_nrf52840)
set(CONF_FILE prj.conf bt.conf usb.conf shell.conf)
set(DTC_OVERLAY_FILE dtc_shell.overlay)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ahu)

#Add Include Directories
include_directories(
                        ../../oslib/base_drivers/
                        ../../oslib/common
                        )

#Add Target Sources
target_sources(app PRIVATE
                        ../../oslib/base_drivers/ble_base.c
                        ../../oslib/base_drivers/data_base.c
                        ../../oslib/base_drivers/shell_base.c
                        ../../oslib/common/led_driver.c
                        ../../oslib/common/hci_driver.c
                        ../../oslib/common/pb_driver.c
                        src/main.c
                        )

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)