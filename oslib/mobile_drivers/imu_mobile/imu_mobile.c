/**
 ************************************************************************
 * @file imu_mobile.c
 * @author Liana van Teijlingen
 * @date 20.04.2021
 * @brief Contains code pertaining to imu operation
 **********************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include "kernel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu_mobile.h"

struct device *lis;

void init_lis(void)
{
    lis = DEVICE_DT_GET_ANY(st_lis2dh);
    if (lis == NULL)
    {
        printk("No lis2dh found\n");
        return;
    }
    if (!device_is_ready(lis))
    {
        printk("Device %s is not ready\n", lis->name);
        return;
    }
}

double read_lis(int axis)
{
    if (lis == NULL)
    {
        printk("No lis2dh found\n");
    }
    struct sensor_value accel[3];
    const char *overrun = "";
    int rc = sensor_sample_fetch(lis);
    if (rc == -EBADMSG)
    {
        /* Sample overrun.  Ignore in polled mode. */
        if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER))
        {
            overrun = "[OVERRUN] ";
        }
        rc = 0;
    }
    if (rc == 0)
    {
        rc = sensor_channel_get(lis,
                                SENSOR_CHAN_ACCEL_XYZ,
                                accel);
    }

    double x_value = sensor_value_to_double(&accel[0]);
    double y_value = sensor_value_to_double(&accel[1]);
    double z_value = sensor_value_to_double(&accel[2]);
    if (axis == 0x00)
    {
        return x_value;
    }
    else if (axis == 0x01)
    {
        return y_value;
    }
}

void thread_read_imu(void)
{

    init_lis();

    while (1) {
        double x = read_lis(0x00);
        double y = read_lis(0x01);

        int xint = (int) x;
        int yint = (int) y;
        int xfl = (int) (x - xint)*110;
        int xfl = (int) (x - xint)*110;

        printk("imu reading x: %d.%d, imu reading y: %d.%d" xint, xfl, yint, yfl);

        k_msleep(SLEEP_TIME_MS);
    }

}

