/*
 ************************************************************************
 * @file time_driver.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Time driver with shell command for returning time
 ************************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <usb/usb_device.h>
#include <shell/shell.h>
#include <drivers/uart.h>
#include <string.h>

#include "time_driver.h"

/* Register Logging Module */
LOG_MODULE_REGISTER(time_module, LOG_LEVEL_DBG);

/* Declare command handler prototypes */
int cmd_time_f(const struct shell *, size_t, char **);
int cmd_time_null(const struct shell *, size_t, char **);

/* Specify Shell Commands for time */
SHELL_STATIC_SUBCMD_SET_CREATE(time,
							   SHELL_CMD(f, NULL, "Show formatted time.", cmd_time_f),
							   SHELL_SUBCMD_SET_END);

/* Register Command Into Shell */
SHELL_CMD_REGISTER(time, &time, "Display Time", cmd_time_null);

int cmd_time_f(const struct shell *shell, size_t argc,
			   char **argv)
{
	/*Check for incorrect args or incorrect number of args*/
	if (strcmp(argv[0], "f") != 0)
	{
		LOG_ERR("Invalid Arguments");
		return 0;
	}
	else if (argc > 1)
	{
		LOG_ERR("Invalid Arguments");
		return 0;
	}

	/* Set current time in seconds*/
	int currentTime = k_uptime_get() / 1000;

	/*Set current time in hours*/
	int hours = currentTime / 3600;

	/*Correct Negative Hours*/
	if (hours < 0)
		hours = 0;

	/*Initialise and calculate minutes*/
	int minutes = (currentTime - (3600 * hours)) / 60;

	/*Correct Negative Minutes*/
	if (minutes < 0)
		minutes = 0;

	/*Initialise and calculate seconds*/
	int seconds = (currentTime - (3600 * hours) - (minutes * 60));

	/*Print final formatted time*/
	shell_print(shell, "Current runtime is %d hrs, %d mins, and %d seconds", hours, minutes, seconds);

	return 0;
}

int cmd_time_null(const struct shell *shell, size_t argc,
				  char **argv)
{
	/*Define args as unused*/
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/*Check for incorrect number of args*/
	if (argc > 1)
	{
		LOG_ERR("Invalid Arguments");
		return 0;
	}

	/* Set current time in seconds*/
	int currentTime = k_uptime_get() / 1000;

	/* Print unformatted time to shell*/
	shell_print(shell, "Current runtime is %d seconds", currentTime);

	return 0;
}
