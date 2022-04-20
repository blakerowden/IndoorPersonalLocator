/**
 ************************************************************************
 * @file scu_io.c
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains pertaining to IO device operation
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

#include "scu_io.h"

static atomic_t flag_rgb_red = (atomic_t) false;
static atomic_t flag_rgb_green = (atomic_t) false;
static atomic_t flag_rgb_blue = (atomic_t) false;
static atomic_t flag_button = (atomic_t) false;

void led_gpio_enable(void)
{
    //gpio_pin_configure(device_get_binding(LEDR), PINR, GPIO_OUTPUT_ACTIVE | FLAGSR);
    //gpio_pin_configure(device_get_binding(LEDG), PING, GPIO_OUTPUT_ACTIVE | FLAGSG);
    //gpio_pin_configure(device_get_binding(LEDB), PINB, GPIO_OUTPUT_ACTIVE | FLAGSB);
    //gpio_pin_set(device_get_binding(LEDR), PINR, 0);
    //gpio_pin_set(device_get_binding(LEDG), PING, 0);
    //gpio_pin_set(device_get_binding(LEDB), PINB, 0);
}

int led_rgb_set(int rgb, int mode)
{
    //int ret = 1;
    //if (rgb == RGB_RED)
    //{
    //    if (gpio_pin_set(device_get_binding(LEDR), PINR, mode) == 0)
    //    {
    //        atomic_set(&flag_rgb_red, (atomic_t)mode);
    //    }
    //}
    //else if (rgb == RGB_GREEN)
    //{
    //    if (gpio_pin_set(device_get_binding(LEDG), PING, mode) == 0)
    //    {
    //        atomic_set(&flag_rgb_green, (atomic_t)mode);
    //    }
    //}
    //else if (rgb == RGB_BLUE)
    //{
    //    if (gpio_pin_set(device_get_binding(LEDB), PINB, mode) == 0)
    //    {
    //        atomic_set(&flag_rgb_blue, (atomic_t)mode);
    //    }
    //}
//
    //return ret;
}

int led_rgb_get(int rgb)
{
    //if (rgb == RGB_RED)
    //{
    //    return atomic_get(&flag_rgb_red);
    //}
    //else if (rgb == RGB_GREEN)
    //{
    //    return atomic_get(&flag_rgb_green);
    //}
    //else if (rgb == RGB_BLUE)
    //{
    //    return atomic_get(&flag_rgb_blue);
    //}
}

int toggle_rgb(int rgb)
{
    //int ret = 0;
    //if (rgb == RGB_RED)
    //{
    //    int val = led_rgb_get(RGB_RED);
    //    if (led_rgb_set(rgb, !val) != 0)
    //    {
    //        ret = 1;
    //    }
    //}
    //else if (rgb == RGB_GREEN)
    //{
    //    int val = led_rgb_get(RGB_GREEN);
    //    if (led_rgb_set(rgb, !val) != 0)
    //    {
    //        ret = 1;
    //    }
    //}
    //else
    //{
    //    int val = led_rgb_get(RGB_BLUE);
    //    if (led_rgb_set(rgb, !val) != 0)
    //    {
    //        ret = 1;
    //    }
    //}
//
    //return ret;
}

// #define SW0_NODE	DT_ALIAS(sw0)
// static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
// static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void init_button(void)
{
    int ret = 0;
    if (!device_is_ready(button.port))
    {
        printk("Error: button device %s is not ready\n",
                button.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0)
    {
        printk("Error %d: failed to configure %s pin %d\n",
                ret, button.port->name, button.pin);
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&button,
                                            GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0)
    {
        printk("Error %d: failed to configure interrupt on %s pin %d\n",
                ret, button.port->name, button.pin);
        return;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
}

void thread_read_button(void)
{

    init_button();

    while (1)
    {
        /* If we have an LED, match its state to the button's. */
        int val = gpio_pin_get_dt(&button);

        if (val >= 0)
        {
            // gpio_pin_set(device_get_binding(LEDR), PINR, val);
            atomic_set(&flag_button, val);
            // printk("%ld\n", atomic_get(&flag_button));
        }
        k_msleep(5);
    }
}

int button_get(void)
{
    return atomic_get(&flag_button);
}

#define PWM_CTLR DT_INVALID_NODE
#define PWM_FLAGS 0

#define MIN_PERIOD_USEC (USEC_PER_SEC / 128U)
#define MAX_PERIOD_USEC USEC_PER_SEC

#define PWM_DRIVER DT_PWMS_LABEL(DT_ALIAS(pwmsound))
#define PWM_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwmsound))

struct device *pwm_dev;

void init_buzzer(void)
{
    // struct device *pwm_dev;
    uint64_t cycles;
    pwm_dev = device_get_binding("PWM_0");
    if (!pwm_dev)
    {
        printk("Cannot find %s!\n", "PWM_0");
        return;
    }
    pwm_get_cycles_per_sec(pwm_dev, PWM_CHANNEL, &cycles);

    const struct device *speakerPower = device_get_binding("spk-pwr-ctrl");

    if (speakerPower == NULL)
    {
        return;
    }
    struct onoff_client *clientChecker;

    int pwr = regulator_enable(speakerPower, clientChecker);
}

void process_buzzer(int freq)
{
    int period = (int)(1000000 / (freq + 1));
    pwm_pin_set_usec(pwm_dev, PWM_CHANNEL, period, 500 / 2U, PWM_FLAGS);
}
