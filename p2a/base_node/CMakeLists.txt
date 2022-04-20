# SPDX-License-Identifier: Apache-2.0

set(BOARD nrf52840dongle_nrf52840)
set(CONF_FILE prj.conf bt.conf usb.conf shell.conf)
set(DTC_OVERLAY_FILE dtc_shell.overlay)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ahu)

#Add Include Directories
include_directories(
<<<<<<< HEAD
                        ../../oslib/mobile_drivers/ble_mobile
                        ../../oslib/ahu_drivers/ahu_shell
=======
                        ../../oslib/ahu_drivers/ahu_ble
                        ../../oslib/ahu_drivers/ahu_shell
                        ../../oslib/ahu_drivers/ahu_data
>>>>>>> bdddf4748301772ad4c769c9e287dad8cb56c3b6
                        ../../oslib/common
                        )

#Add Target Sources
target_sources(app PRIVATE
<<<<<<< HEAD
                        ../../oslib/mobile_drivers/ble_mobile/ble_mobile_scan.c
                        ../../oslib/mobile_drivers/ble_mobile/ble_mobile.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_base.c
                        ../../oslib/common/hci_driver.c
=======
                        ../../oslib/ahu_drivers/ahu_ble/ble_base.c
                        ../../oslib/ahu_drivers/ahu_data/ahu_data.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_base.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_led.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_time.c
                        ../../oslib/ahu_drivers/ahu_shell/shell_scu.c
                        ../../oslib/common/led_driver.c
                        ../../oslib/common/hci_driver.c
                        ../../oslib/common/pb_driver.c
>>>>>>> bdddf4748301772ad4c769c9e287dad8cb56c3b6
                        src/main.c
                        )

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)