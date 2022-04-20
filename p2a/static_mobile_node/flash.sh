rm build -r

west build -p auto -b nrf52840dongle_nrf52840

nrfutil pkg generate --hw-version 52 --sd-req=0x00         --application build/zephyr/zephyr.hex         --application-version 1 prac1.zip

nrfutil dfu usb-serial -pkg prac1.zip -p /dev/ttyACM0

putty
