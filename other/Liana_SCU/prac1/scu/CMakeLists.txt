# SPDX-License-Identifier: Apache-2.0
#Setting to exclusively build for the Thingy
set(BOARD thingy52_nrf52832)
set(BOARD_FLASH_RUNNER jlink)
#set(DTC_OVERLAY_FILE boards/mpu9250.overlay)
set(CONF_FILE prj.conf)

cmake_minimum_required(VERSION 3.13.1)
#set(DTC_OVERLAY_FILE app.overlay)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scu)

target_sources(app PRIVATE src/main.c 
        ../../oslib/scu_drivers/scu_ble/scu_ble.c
        #../../oslib/scu_drivers/scu_sensors/scu_sensors.c
        #../../oslib/scu_drivers/scu_io/scu_io.c
        )

#Add include_directories for libraries, path starts from this files location.
include_directories(app PUBLIC ../../oslib/scu_drivers/scu_ble
                    #app PUBLIC ../../oslib/scu_drivers/scu_sensors
                    #app PUBLIC ../../oslib/scu_drivers/scu_io
                    )


zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)