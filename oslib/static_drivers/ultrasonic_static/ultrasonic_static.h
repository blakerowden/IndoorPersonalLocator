/**
 * @file ultrasonic.c
 * @author Liana van Teijlingen - 45802616
 * @version 
 * @date 2022-04-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ULTRASONIC_STATIC_H
#define ULTRASONIC_STATIC_H

#include <zephyr.h>

extern struct k_msgq ultra_msgq;

void thread_ultra_read(void);

#endif /* ULTRASONIC_STATIC_H */