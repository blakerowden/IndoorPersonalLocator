# SPDX-License-Identifier: Apache-2.0

set(CONF_FILE prj.conf)
set(DTC_OVERLAY_FILE dtc_shell.overlay)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prac1)


target_sources(app PRIVATE src/main.c
    ../../oslib/ahu_drivers/ahu_rgb/rgb_driver.c
    ../../oslib/ahu_drivers/ahu_ble/bluetooth_driver.c
    ../../oslib/ahu_drivers/ahu_timing/time_driver.c
    ../../oslib/ahu_drivers/ahu_ble/hci.c
    ../../oslib/ahu_drivers/ahu_ble/ble_commands.c
    ../../oslib/ahu_drivers/ahu_shell/shell_driver.c
    ../../oslib/ahu_drivers/ahu_button/button_driver.c)

include_directories(app PUBLIC ../../oslib/ahu_drivers/ahu_ble
                    app PUBLIC ../../oslib/ahu_drivers/ahu_rgb
                    app PUBLIC ../../oslib/ahu_drivers/ahu_timing
                    app PUBLIC ../../oslib/ahu_drivers/ahu_shell
                    app PUBLIC ../../oslib/ahu_drivers/ahu_button)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)