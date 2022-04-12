/*
 ************************************************************************
 * @file hci.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Host controller interface driver to allow for printing of
 * Data recieved from SCU in HCI format. Also defines HCI definitions.
 **********************************************************************
 */

#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <shell/shell.h>
#include <sys/byteorder.h>

/*Define packet variables*/
#define HTS221_Temp_ID 0x01
#define HTS221_Hum_ID 0x02
#define LPS22_AirP_ID 0x03
#define CCS811_Dev_ID 0x04
#define LIS2DH_X_ID 0x05
#define LIS2DH_Y_ID 0x06
#define LIS2DH_Z_ID 0x07
#define RGB_LED_ID 0x08
#define BUZZ_Dev_ID 0x09
#define PB_Dev_ID 0x0A

/* Define logging module */
LOG_MODULE_REGISTER(sensor_module, LOG_LEVEL_DBG);

void print_reading(uint8_t data[14])
{
    /*Print reading in regular format using a switch on the device ID*/
    uint8_t reading[11];
    for (int i = 0; i < 10; i++)
    {
        if (data[i + 3] != ' ')
        {
            reading[i] = data[i + 3];
        }
        else
        {
            reading[i] = '\0';
        }
    }

    switch (data[2])
    {
    case HTS221_Temp_ID:
        printk("Temperature : %s°C\n", reading);
        break;
    case HTS221_Hum_ID:
        printk("Humidity : %s percent rh\n", reading);
        break;
    case LPS22_AirP_ID:
        printk("Air Pressure : %shPa\n", reading);
        break;
    case CCS811_Dev_ID:
        printk("Air Quality : %sppm\n", reading);
        break;
    case LIS2DH_X_ID:
        printk("X Accel : %s m/s^2\n", reading);
        break;
    case LIS2DH_Y_ID:
        printk("Y Accel : %s m/s^2\n", reading);
        break;
    case LIS2DH_Z_ID:
        printk("Z Accel : %s m/s^2\n", reading);
        break;
    case PB_Dev_ID:
        printk("PB State : %s\n", reading);
        break;
    }
    return;
}

void print_JSON(struct shell *shell, uint8_t data[14])
{

    /*Print reading in JSON Format*/
    uint8_t reading[11];
    for (int i = 0; i < 10; i++)
    {
        if (data[i + 3] != ' ')
        {
            reading[i] = data[i + 3];
        }
        else
        {
            reading[i] = '\0';
        }
    }

    if (data[2] == 0x0A)
    {
        shell_print(shell, "0x%02x,[%s]}\n", data[2], reading);
    }
    else if (data[2] == 0x01)
    {
        shell_print(shell, "{0x%02x,[%s],", data[2], reading);
    }
    else
    {
        shell_print(shell, "0x%02x,[%s],", data[2], reading);
    }
    return;
}