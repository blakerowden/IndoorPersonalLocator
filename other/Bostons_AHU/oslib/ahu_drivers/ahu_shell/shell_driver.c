/*
 ************************************************************************
 * @file shell_driver.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Driver for shell thread.
 **********************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

void shell_entry(void *arg1, void *arg2, void *arg3)
{
	/* Setup DTR */
	const struct device *shell_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	uint32_t dtr = 0;

	/*Enable USB*/
	if (usb_enable(NULL))
		return;

	/*Wait on DTR*/
	while (!dtr)
	{
		uart_line_ctrl_get(shell_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	/*DTR OK, Continue*/
}