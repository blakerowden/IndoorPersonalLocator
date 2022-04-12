/*
 ************************************************************************
 * @file button_driver.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Button driver for intialising button and storing callback action.
 **********************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>
#include <logging/log.h>

/*Define logging module*/
LOG_MODULE_REGISTER(button_module, LOG_LEVEL_DBG);

/*Setup Devicetree*/
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
/*Setup gpio info for button switch*/
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															  {0});
static struct gpio_callback button_cb_data;

/*Create data item to send in queue*/
struct data_item_t
{
	uint8_t buttonState;
} buttonObj;

/*Define message queue for button press*/
K_MSGQ_DEFINE(buttonQueue, 8, 10, 4);

/*Callback*/
void button_pressed(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins)
{

	/*Set button flag in struct to on or off (0 or 1)*/
	buttonObj.buttonState = buttonObj.buttonState ^ 1;

	/*Load button state into queue for ble thread*/
	if (k_msgq_put(&buttonQueue, &buttonObj, K_NO_WAIT) != 0)
	{
		/* Queue is full, we could purge it, a loop can be
		 * implemented here to keep trying after a purge.
		 */
		k_msgq_purge(&buttonQueue);
	}
}

void init_button_driver(void)
{

	/*Initialise ret value for errors*/
	int ret;

	/*Check button is ready*/
	if (!device_is_ready(button.port))
	{
		printk("Error: button device %s is not ready\n",
			   button.port->name);
		return;
	}

	/*Configure gpio for button*/
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button.port->name, button.pin);
		return;
	}

	/*Configure interrupt calback to the function above*/
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

	/*Set initial Button State*/
	buttonObj.buttonState = 0;

	/*Send initial button state in queue*/
	if (k_msgq_put(&buttonQueue, &buttonObj, K_NO_WAIT) != 0)
	{
		/* Queue is full, we could purge it, a loop can be
		 * implemented here to keep trying after a purge.
		 */
		k_msgq_purge(&buttonQueue);
	}

	/*Inform of succesful startup*/
	LOG_DBG("Button configured succesfully.");
}