/**
 ************************************************************************
 * @file scu_sensors.h
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains required definitions of scu_sensors.c
 **********************************************************************
 **/

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/sensor/ccs811.h>
#include <zephyr.h>
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

#ifndef SCU_SENSORS_H
#define SCU_SENSORS_H

static bool app_fw_2;

static void hts221_handler(const struct device *dev,
						   const struct sensor_trigger *trig);
void init_hts(void);
double read_hts(int sensor);
void init_lps(void);
double read_lps(void);
void init_ccs(void);
double read_ccs(void);
void init_lis(void);
double read_lis(int sensor);

#endif