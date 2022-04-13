/**
 ************************************************************************
 * @file scu_sensors.c
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains code pertaining to sensor operation
 **********************************************************************
 **/

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>
#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <shell/shell.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include "kernel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/pwm.h>
#include <drivers/regulator.h>

#include "scu_sensors.h"

struct device *hts;
struct device *lps;
struct device *ccs;
struct device *lis;

static void hts221_handler(const struct device *dev,
                           const struct sensor_trigger *trig)
{
    process_hts221(dev, 0x01);
}

void init_hts(void)
{
    hts = device_get_binding("HTS221");
    // const struct device *hts221 = device_get_binding("HTS221");
    if (IS_ENABLED(CONFIG_HTS221_TRIGGER))
    {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ALL,
        };
        if (sensor_trigger_set(hts, &trig, hts221_handler) < 0)
        {
            printk("Cannot configure trigger\n");
            return;
        }
    }
}

double read_hts(int sensor)
{
    struct sensor_value temp, hum;
    if (sensor_sample_fetch(hts) < 0)
    {
        printk("Sensor sample update error\n");
        return;
    }
    if (sensor_channel_get(hts, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0)
    {
        printk("Cannot read HTS221 temperature channel\n");
        return;
    }
    if (sensor_channel_get(hts, SENSOR_CHAN_HUMIDITY, &hum) < 0)
    {
        printk("Cannot read HTS221 humidity channel\n");
        return;
    }
    double temp_value = sensor_value_to_double(&temp);
    double hum_value = sensor_value_to_double(&hum);
    if (sensor == 0x01)
    {
        return temp_value;
    }
    else if (sensor == 0x02)
    {
        return hum_value;
    }
}

void init_lps(void)
{
    lps = device_get_binding(DT_LABEL(DT_INST(0, st_lps22hb_press)));
}

double read_lps(void)
{
    struct sensor_value pressure, temp;

    if (sensor_sample_fetch(lps) < 0)
    {
        printk("Sensor sample update error\n");
        return;
    }

    if (sensor_channel_get(lps, SENSOR_CHAN_PRESS, &pressure) < 0)
    {
        printk("Cannot read LPS22HB pressure channel\n");
        return;
    }

    if (sensor_channel_get(lps, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0)
    {
        printk("Cannot read LPS22HB temperature channel\n");
        return;
    }

    double pressure_value = sensor_value_to_double(&pressure);
    return pressure_value;
}

void init_ccs(void)
{
    ccs = device_get_binding(DT_LABEL(DT_INST(0, ams_ccs811)));
    struct ccs811_configver_type cfgver;
    int rc;

    rc = ccs811_configver_fetch(ccs, &cfgver);
    if (rc == 0)
    {
        printk("HW %02x; FW Boot %04x App %04x ; mode %02x\n",
                cfgver.hw_version, cfgver.fw_boot_version,
                cfgver.fw_app_version, cfgver.mode);
        app_fw_2 = (cfgver.fw_app_version >> 8) > 0x11;
    }
}

double read_ccs(void)
{
    struct sensor_value tvoc;
    int rc = 0;
    if (rc == 0)
    {
        rc = sensor_sample_fetch(ccs);
    }

    const struct ccs811_result_type *rp = ccs811_result(ccs);

    sensor_channel_get(ccs, SENSOR_CHAN_VOC, &tvoc);

    double voc_value = sensor_value_to_double(&tvoc);
    return voc_value;
}

void init_lis(void)
{
    lis = DEVICE_DT_GET_ANY(st_lis2dh);
    if (lis == NULL)
    {
        printk("No device found\n");
        return;
    }
    if (!device_is_ready(lis))
    {
        printk("Device %s is not ready\n", lis->name);
        return;
    }
}

double read_lis(int sensor)
{
    printk("readin\n");
    struct sensor_value accel[3];
    const char *overrun = "";
    int rc = sensor_sample_fetch(lis);
    printk("oop");
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
    if (sensor == 0x05)
    {
        printk("am x\n");
        return x_value;
    }
    else if (sensor == 0x06)
    {
        return y_value;
    }
    else if (sensor == 0x07)
    {
        return z_value;
    }
}