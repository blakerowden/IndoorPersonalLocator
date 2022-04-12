/*
 ************************************************************************
 * @file rgb_driver.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief RGB driver. Allows for initialisation of rgb led and shell
 * commands to control it.
 **********************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/pwm.h>
#include <shell/shell.h>
#include <logging/log.h>

/* Define logging module */
LOG_MODULE_REGISTER(led_module, LOG_LEVEL_DBG);

/*Setup Device Tree Configs*/
#define RED_NODE DT_ALIAS(red_pwm_led)
#define GREEN_NODE DT_ALIAS(green_pwm_led)
#define BLUE_NODE DT_ALIAS(blue_pwm_led)

#if DT_NODE_HAS_STATUS(RED_NODE, okay)
#define RED_CTLR_NODE DT_PWMS_CTLR(RED_NODE)
#define RED_CHANNEL DT_PWMS_CHANNEL(RED_NODE)
#define RED_FLAGS DT_PWMS_FLAGS(RED_NODE)
#else
#error "Unsupported board: red-pwm-led devicetree alias is not defined"
#define RED_CTLR_NODE DT_INVALID_NODE
#define RED_CHANNEL 0
#define RED_FLAGS 0
#endif

#if DT_NODE_HAS_STATUS(GREEN_NODE, okay)
#define GREEN_CTLR_NODE DT_PWMS_CTLR(GREEN_NODE)
#define GREEN_CHANNEL DT_PWMS_CHANNEL(GREEN_NODE)
#define GREEN_FLAGS DT_PWMS_FLAGS(GREEN_NODE)
#else
#error "Unsupported board: green-pwm-led devicetree alias is not defined"
#define GREEN_CTLR_NODE DT_INVALID_NODE
#define GREEN_CHANNEL 0
#define GREEN_FLAGS 0
#endif

#if DT_NODE_HAS_STATUS(BLUE_NODE, okay)
#define BLUE_CTLR_NODE DT_PWMS_CTLR(BLUE_NODE)
#define BLUE_CHANNEL DT_PWMS_CHANNEL(BLUE_NODE)
#define BLUE_FLAGS DT_PWMS_FLAGS(BLUE_NODE)
#else
#error "Unsupported board: blue-pwm-led devicetree alias is not defined"
#define BLUE_CTLR_NODE DT_INVALID_NODE
#define BLUE_CHANNEL 0
#define BLUE_FLAGS 0
#endif

/*Define stepsize and microsends per second for pwm*/

#define PERIOD_USEC (USEC_PER_SEC / 50U)
#define STEPSIZE_USEC 2000

/*Setup colour enumerators*/

enum
{
	RED,
	GREEN,
	BLUE
};

/*Setup array of device structs for each pwm led colour*/
const struct device *pwm_dev[3];

/*Initialise global flags for red status, blue status and green status*/
bool redOn;
bool blueOn;
bool greenOn;

/* Declare command handler prototypes */
static int cmd_led_ctrl_o(const struct shell *, size_t, char **);
static int cmd_led_ctrl_f(const struct shell *, size_t, char **);
static int cmd_led_ctrl_t(const struct shell *, size_t, char **);

/* Specify Shell Commands for rgb control */
SHELL_STATIC_SUBCMD_SET_CREATE(led_ctrl,
							   SHELL_CMD(o, NULL, "Turn led on.", cmd_led_ctrl_o),
							   SHELL_CMD(f, NULL, "Turn led off.", cmd_led_ctrl_f),
							   SHELL_CMD(t, NULL, "Toggle led.", cmd_led_ctrl_t),
							   SHELL_SUBCMD_SET_END);

/*Register shell commands for rgb*/
SHELL_CMD_REGISTER(led, &led_ctrl, "Led Control (Red/Blue)", NULL);

int pwm_set(const struct device *pwm_dev, uint32_t pwm_pin,
			uint32_t pulse_width, pwm_flags_t flags)
{
	/*Set pwm channel for input pwm options*/
	return pwm_pin_set_usec(pwm_dev, pwm_pin, PERIOD_USEC,
							pulse_width, flags);
}

void set_led_clr(const struct shell *shell, char colour)
{

	/*Initialise return value for errors*/
	int ret = 0;

	/*switch to check input colour and set accordingly*/
	switch (colour)
	{
	case 'r':
		if (redOn == 1)
		{
			LOG_WRN("red led already on");
			break;
		}
		ret = pwm_set(pwm_dev[RED], RED_CHANNEL,
					  10000U, RED_FLAGS);
		redOn = 1;
		LOG_INF("red led is on");
		break;
	case 'b':
		if (blueOn == 1)
		{
			LOG_WRN("blue led already on");
			break;
		}
		ret = pwm_set(pwm_dev[BLUE], BLUE_CHANNEL,
					  10000U, BLUE_FLAGS);
		blueOn = 1;
		LOG_INF("blue led is on");
		break;
	case 'g':
		if (greenOn == 1)
		{
			LOG_WRN("green led already on");
			break;
		}
		ret = pwm_set(pwm_dev[GREEN], GREEN_CHANNEL,
					  10000U, GREEN_FLAGS);
		greenOn = 1;
		LOG_INF("green led is on");
		break;
	default:
		LOG_ERR("invalid command");
		break;
	}

	/*Check if ret was set and if it was throw error*/
	if (ret != 0)
	{
		LOG_ERR("led write failed");
		return;
	}

	return;
}

void clear_led_clr(const struct shell *shell, char colour)
{

	/*Initialise return value for errors*/
	int ret = 0;

	/*switch to check input colour and turn off accordingly*/
	switch (colour)
	{
	case 'r':
		if (redOn == 0)
		{
			LOG_WRN("red led already off");
			break;
		}
		else
		{
			LOG_INF("red led is off");
		}
		ret = pwm_set(pwm_dev[RED], RED_CHANNEL,
					  0U, RED_FLAGS);
		redOn = 0;
		break;
	case 'b':
		if (blueOn == 0)
		{
			LOG_WRN("blue led already off");
			break;
		}
		else
		{
			LOG_INF("blue led is off");
		}
		ret = pwm_set(pwm_dev[BLUE], BLUE_CHANNEL,
					  0U, BLUE_FLAGS);
		blueOn = 0;
		break;
	case 'g':
		if (greenOn == 0)
		{
			LOG_WRN("green led already off");
			break;
		}
		else
		{
			LOG_INF("green led is off");
		}
		ret = pwm_set(pwm_dev[GREEN], GREEN_CHANNEL,
					  0U, GREEN_FLAGS);
		greenOn = 0;
		break;
	default:
		LOG_ERR("invalid command");
		break;
	}

	/*Check if ret was set and if it was throw error*/
	if (ret != 0)
	{
		LOG_ERR("led write failed");
		return;
	}

	return;
}

void toggle_led(const struct shell *shell, char colour)
{

	/*Switch to toggle matching colour that is input*/
	switch (colour)
	{
	case 'r':
		if (redOn == 1)
		{
			clear_led_clr(shell, 'r');
			redOn = 0;
		}
		else
		{
			set_led_clr(shell, 'r');
			redOn = 1;
		};
		break;
	case 'b':
		if (blueOn == 1)
		{
			clear_led_clr(shell, 'b');
			blueOn = 0;
		}
		else
		{
			set_led_clr(shell, 'b');
			blueOn = 1;
		}
		break;
	case 'g':
		if (greenOn == 1)
		{
			clear_led_clr(shell, 'g');
			greenOn = 0;
		}
		else
		{
			set_led_clr(shell, 'g');
			greenOn = 1;
		}
		break;
	/*If colour not recognised*/
	default:
		LOG_ERR("invalid command");
		break;
	}

	return;
}

int cmd_led_ctrl_o(const struct shell *shell, size_t argc,
				   char **argv)
{
	/*Defined argc as unused*/
	ARG_UNUSED(argc);

	/*Check colour is not 2 characters or more*/
	if (argv[1][1])
	{
		LOG_ERR("Incorrect Arguments");
		return 0;
	}

	/*extract colour from shell command*/
	char colour = argv[1][0];

	/*set clr that is input*/
	set_led_clr(shell, colour);

	return 0;
}

int cmd_led_ctrl_f(const struct shell *shell, size_t argc,
				   char **argv)
{

	/*Defined argc as unused*/
	ARG_UNUSED(argc);

	/*Check colour is not 2 characters or more*/
	if (argv[1][1])
	{
		LOG_ERR("Incorrect Arguments");
		return 0;
	}

	/*extract colour from shell command*/
	char colour = argv[1][0];

	/*Clear matching colour*/
	clear_led_clr(shell, colour);
	return 0;
}

int cmd_led_ctrl_t(const struct shell *shell, size_t argc,
				   char **argv)
{

	/*Define argc as unused*/
	ARG_UNUSED(argc);

	/*Check colour is not 2 characters or more*/
	if (argv[1][1])
	{
		LOG_ERR("Incorrect Arguments");
		return 0;
	}

	/*extract colour from shell command*/
	char inputColour = argv[1][0];

	/*toggle matching colour*/
	toggle_led(shell, inputColour);

	return 0;
}

void init_rgb_led(void)
{

	/*Set pwm devices for colours*/
	pwm_dev[RED] = DEVICE_DT_GET(RED_CTLR_NODE);
	pwm_dev[GREEN] = DEVICE_DT_GET(GREEN_CTLR_NODE);
	pwm_dev[BLUE] = DEVICE_DT_GET(BLUE_CTLR_NODE);

	/*Check all colour devices are correct and ready*/
	if (!device_is_ready(pwm_dev[RED]))
	{
		LOG_ERR("led init failed");
		return;
	}

	if (!device_is_ready(pwm_dev[GREEN]))
	{
		LOG_ERR("led init failed");
		return;
	}
	if (!device_is_ready(pwm_dev[BLUE]))
	{
		LOG_ERR("led init failed");
		return;
	}

	/*Set starting state of flags*/
	redOn = 0;
	greenOn = 0;
	blueOn = 0;

	/*Inform once initialisation complete*/
	LOG_DBG("led init OK");
}