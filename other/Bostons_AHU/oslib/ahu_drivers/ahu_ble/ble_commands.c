/*
 ************************************************************************
 * @file ble_commands.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief BLE commands file for all of the shell commands related
 * to BLE.
 **********************************************************************
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <shell/shell.h>
#include <logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "hci.h"

/* Define logging module */
LOG_MODULE_REGISTER(ble_command_module, LOG_LEVEL_DBG);

struct data_item_t
{
    uint8_t devID;
    uint8_t packet[13];
} fifoSend;

struct data_item_t2
{
    uint8_t sampleTime;
    uint8_t JSONFormat;
    struct shell *shellPrint;
} samplerThreadObj;

K_MSGQ_DEFINE(bluetoothQueue, 104, 10, 4);

K_MSGQ_DEFINE(samplerQueue, sizeof(samplerThreadObj), 10, 4);

/*Define Shellcommands relatiing to bluetooth*/
int cmd_hts221_r(const struct shell *, size_t, char **);
int cmd_lps22_r(const struct shell *, size_t, char **);
int cmd_ccs811_r(const struct shell *, size_t, char **);
int cmd_lis2dh_r(const struct shell *, size_t, char **);
int cmd_buzzer_w(const struct shell *, size_t, char **);
int cmd_rgb_w(const struct shell *, size_t, char **);
int cmd_pb_r(const struct shell *, size_t, char **);
int cmd_dc_w(const struct shell *, size_t, char **);
int cmd_all_on(struct shell *, size_t, char **);
int cmd_all_off(const struct shell *, size_t, char **);
int cmd_sample_w(const struct shell *, size_t, char **);

/*Register Shell commands relatiing to bluetooth*/
SHELL_STATIC_SUBCMD_SET_CREATE(all,
                               SHELL_CMD(o, NULL, "read hts221 sensor", cmd_all_on),
                               SHELL_CMD(f, NULL, "read hts221 sensor", cmd_all_off),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(all, &all, "Enable JSON Output", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(hts221,
                               SHELL_CMD(r, NULL, "read hts221 sensor", cmd_hts221_r),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(hts221, &hts221, "Read from hts221", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(lps22,
                               SHELL_CMD(r, NULL, "read lps22 sensor", cmd_lps22_r),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lps22, &lps22, "Read from lps22", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(ccs811,
                               SHELL_CMD(r, NULL, "read css811 sensor", cmd_ccs811_r),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ccs811, &ccs811, "Read from ccs811", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(lis2dh,
                               SHELL_CMD(r, NULL, "read lis2dh sensor", cmd_lis2dh_r),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lis2dh, &lis2dh, "Read from lis2dh", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(buzzer,
                               SHELL_CMD(w, NULL, "set buzzer pwm", cmd_buzzer_w),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(buzzer, &buzzer, "Write to buzzer", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(rgb,
                               SHELL_CMD(w, NULL, "set rgb state", cmd_rgb_w),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(rgb, &rgb, "Write to rgb led", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(pb,
                               SHELL_CMD(r, NULL, "Set pb state", cmd_pb_r),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pb, &pb, "Read push-button state", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(dc,
                               SHELL_CMD(w, NULL, "Set dc", cmd_dc_w),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dc, &dc, "Write duty cycle", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(sample,
                               SHELL_CMD(w, NULL, "Set sample.", cmd_sample_w),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sample, &sample, "Write sample", NULL);

int cmd_hts221_r(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 2 || argv[1][1] || argv[0][0] != 'r')
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    switch (argv[1][0])
    {
    case 't':
        fifoSend.devID = HTS221_Temp_ID;
        break;
    case 'h':
        fifoSend.devID = HTS221_Hum_ID;
        break;
    }

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_lps22_r(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 1 || argv[0][1] || argv[0][0] != 'r')
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    fifoSend.devID = LPS22_AirP_ID;

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}
int cmd_ccs811_r(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 1 || argv[0][1] || argv[0][0] != 'r')
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    fifoSend.devID = CCS811_Dev_ID;

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_lis2dh_r(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 2)
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    if (argv[1][1])
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    switch (argv[1][0])
    {
    case 'x':
        fifoSend.devID = LIS2DH_X_ID;
        break;
    case 'y':
        fifoSend.devID = LIS2DH_Y_ID;
        break;
    case 'z':
        fifoSend.devID = LIS2DH_Z_ID;
        break;
    default:
        LOG_ERR("Invalid Argument");
        return 0;
    }

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_buzzer_w(const struct shell *shell, size_t argc, char **argv)
{

    fifoSend.devID = BUZZ_Dev_ID;
    int i;

    /*Check for arg errors*/
    if (argc > 3)
    {
        LOG_ERR("Invalid Argument argc");
        return 0;
    }

    for (int i = 0; i < 6; i++)
    {
        if (argv[1][i])
        {
            if (!isdigit(argv[1][i]))
            {
                LOG_ERR("Invalid Argument argv");
                return 0;
            }
        }
    }

    for (i = 0; i < 12; i++)
    {
        fifoSend.packet[i] = argv[1][i];
    }

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_rgb_w(const struct shell *shell, size_t argc, char **argv)
{

    fifoSend.devID = RGB_LED_ID;
    fifoSend.packet[0] = 0x00;
    fifoSend.packet[1] = 0x00;
    fifoSend.packet[2] = 0x00;
    fifoSend.packet[3] = '\0';

    /*Check for arg errors*/
    if (argc > 3)
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    if (argv[1][1])
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    switch (argv[1][0])
    {
    case 'r':
        fifoSend.packet[0] = 0x01;
        break;
    case 'g':
        fifoSend.packet[1] = 0x01;
        break;
    case 'b':
        fifoSend.packet[2] = 0x01;
        break;
    default:
        LOG_ERR("Invalid Argument");
        return 0;
    }
    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_pb_r(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 1 || argv[0][1] || argv[0][0] != 'r')
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    fifoSend.devID = PB_Dev_ID;

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_dc_w(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 3)
    {
        LOG_ERR("Invalid Argument argc");
        return 0;
    }

    for (int i = 0; i < 6; i++)
    {
        if (argv[1][i])
        {
            if (!isdigit(argv[1][i]))
            {
                LOG_ERR("Invalid Argument argv");
                return 0;
            }
        }
    }

    fifoSend.devID = 0x0B;
    int i;

    for (i = 0; i < 12; i++)
    {
        fifoSend.packet[i] = argv[1][i];
    }

    if (k_msgq_put(&bluetoothQueue, &fifoSend, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&bluetoothQueue);
    }

    return 0;
}

int cmd_sample_w(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 3)
    {
        LOG_ERR("Invalid Argument argc");
        return 0;
    }

    for (int i = 0; i < 6; i++)
    {
        if (argv[1][i])
        {
            if (!isdigit(argv[1][i]))
            {
                LOG_ERR("Invalid Argument argv");
                return 0;
            }
        }
    }

    samplerThreadObj.sampleTime = atoi(argv[1]);

    if (k_msgq_put(&samplerQueue, &samplerThreadObj, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&samplerQueue);
    }

    return 0;
}

int cmd_all_on(struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 1 || argv[0][1] || argv[0][0] != 'o')
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    samplerThreadObj.JSONFormat = 1;

    samplerThreadObj.shellPrint = shell;

    if (!samplerThreadObj.sampleTime)
    {
        samplerThreadObj.sampleTime = 2;
    }

    shell_print(shell, "Sampling is on and sample interval is: %d seconds", samplerThreadObj.sampleTime);

    if (k_msgq_put(&samplerQueue, &samplerThreadObj, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&samplerQueue);
    }

    return 0;
}
int cmd_all_off(const struct shell *shell, size_t argc, char **argv)
{

    /*Check for arg errors*/
    if (argc > 1 || argv[0][1] || argv[0][0] != 'f')
    {
        LOG_ERR("Invalid Argument");
        return 0;
    }

    samplerThreadObj.JSONFormat = 0;

    shell_print(shell, "Sampling is off and sample interval is: %d seconds", samplerThreadObj.sampleTime);

    if (k_msgq_put(&samplerQueue, &samplerThreadObj, K_NO_WAIT) != 0)
    {
        /* Queue is full, we could purge it, a loop can be
         * implemented here to keep trying after a purge.
         */
        k_msgq_purge(&samplerQueue);
    }

    return 0;
}
