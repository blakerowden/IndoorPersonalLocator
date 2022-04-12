s4579873 Assingment 1 AHU

Boston, O'Neill - 45798737

4.1.1. AHU Command Line Interface Shell:
The command line shell is fully functional and incorporates a shell like overlay to write messages to the AHU. To enter a command simply type the command followed by the enter key.

4.1.2. System Timer Read Task
The shell will print the time to standard output if the user command is either "time" or "time f". "time f" formats the time output to seconds, minutes, and hours.

4.1.3. AHU LED Control Task
The shell commands "led o {r,g,b}" "led t {r,g,b}" and "led f {r,g,b}" will turn on, toggle or turn off the respective colour input into the command.

4.1.4. Log Mesage Task
The system logs all errors, warnings, info, and debug messages to the logs, and outputs these to the terminal. They can be disabled and filtered using "log -h".

4.1.5. SCU Command Interface
All SCU commands are functional and recieve responses from the SCU.

4.1.6. Communications
The SCU and AHU communicate entirely through bluetooth using bt_gatt_write().

4.1.7. Host Cotroller Interface
The HCI is entirely implemented in hci.h

4.1.8. Zephyr OS Libraries
The code runs entirely on zephyr using threads and if threads are in seperate files they communicate using message queues.

4.1.9. JSON Interface
The all command is functioning and enables json ouptut of data to the shell.

4.1.10 MQTT Publisher
The MQTT Publisher is not implemented.

4.1.11 Challenge
The challenge is not implemented.



The folder structure of the AHU for this project is:

```bash
"repo"
├── "oslib"
│   └── "ahu_drivers"
│       ├── "ahu_ble"
│       │   ├── "ble_commands.c"
│       │   ├── "ble_commands.h"
│       │   ├── "bluetooth_driver.c"
│       │   ├── "bluetooth_driver.h"
│       │   ├── "hci.c"
│       │   └── "hci.h"
│       ├── "ahu_button"
│       │   ├── "button_driver.c"
│       │   └── "button_driver.h"
│       ├── "ahu_rgb"
│       │   ├── "rgb_driver.c"
│       │   └── "rgb_driver.h"
│       ├── "ahu_shell"
│       │   ├── "shell_driver.c"
│       │   └── "shell_driver.h"
│       └── "ahu_timing"
│           ├── "time_driver.c"
│           └── "time_driver.h"
├── "prac1"
│   └── "ahu"
│       ├── "CMakeLists.txt"
│       ├── "dtc_shell.overlay"
│       ├── "prj.conf"
|       ├── "mqtt"
|       |   └── "mqttPub.py"
│       └── "src"
│           └── "main.c"
│   
└── "README.md"
```

The following sources were used to develop the ahu

BLE Sample:
```
https://github.com/uqembeddedsys/zephyr-examples/tree/main/ble_connect_sample
```
Zephyr Files:
```
https://github.com/zephyrproject-rtos/zephyr 
```

build and flash using the following command in the terminal:

```bash
rm build -r

west build -p auto -b nrf52840dongle_nrf52840

nrfutil pkg generate --hw-version 52 --sd-req=0x00 --application build/zephyr/zephyr.hex         --application-version 1 prac1.zip

nrfutil dfu usb-serial -pkg prac1.zip -p /dev/ttyACM0
```

To access the tool install putty and use /dev/ttyACM0 at 115200 baud rate.

Tool:
```
https://www.digikey.com/en/products/detail/nordic-semiconductor-asa/NRF52840-DONGLE/9491124
```