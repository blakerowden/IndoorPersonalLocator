/**
 ************************************************************************
 * @file scu_ble.c
 * @author Liana van Teijlingen
 * @date 04.04.2021
 * @brief Contains code pertaining to bluetooth operation
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

#include "scu_ble.h"
//#include "scu_sensors.h"
//#include "scu_io.h"

#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t) false
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t) true)
#define WAIT_FOR_FLAG(flag)          \
    while (!(bool)atomic_get(&flag)) \
    {                                \
        (void)k_sleep(K_MSEC(1));    \
    }

struct received_packet
{
    uint8_t preamble;
    uint8_t typelen;
    uint8_t id;
    uint8_t data[11];
};

struct return_packet
{
    uint8_t preamble;
    uint8_t typelen;
    uint8_t id;
    double data;
};

K_MSGQ_DEFINE(tosend_msgq, sizeof(struct return_packet), 10, 4);
K_MSGQ_DEFINE(command_msgq, sizeof(struct received_packet), 10, 4);

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

// SCU UUID
#define SCU_UUID                                                        \
    BT_UUID_DECLARE_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

// AHU UUID
#define AHU_UUID                                                        \
    BT_UUID_DECLARE_128(0x02, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

// SCU CHARACTERISTC UUID
#define SCU_CH_UUID                                                     \
    BT_UUID_DECLARE_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00)

// AHU CHARACTERISTIC UUID
#define ACU_CH_UUID                                                     \
    BT_UUID_DECLARE_128(0x02, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                  0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb),
};

static struct bt_uuid *scu_svc_uuid = AHU_UUID;

static atomic_t flag_discover_complete = (atomic_t) false;

// Keeps Track of BLE connection within APP
bool ble_connected = false;

// GATT CHARACTERISTIC VALUES
static struct bt_conn *g_conn;

/**
 * @brief Initialises bluetooth, and begins advertising data
 *            on BLE.
 *
 */
static void bt_ready(void)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        // settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    // bt_passkey_set(0xAA289);
    printk("Advertising successfully started\n");
}

/**
 * @brief Connection call back
 *
 * @param conn conenction handler
 * @param err err val
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("Connection failed (err 0x%02x)\n", err);
        ble_connected = false;
    }
    else
    {
        printk("BLE Connected to Device\n");
        ble_connected = true;
        struct bt_le_conn_param *param = BT_LE_CONN_PARAM(6, 6, 0, 400);
        g_conn = bt_conn_ref(conn);
        if (bt_conn_le_param_update(conn, param) < 0)
        {
            while (1)
            {
                printk("Connection Update Error\n");
                k_msleep(10);
            }
        }
    }
}

/**
 * @brief Disconnect Callback, used to keep track of connection status
 *          in the application layer
 *
 * @param conn connection handler
 * @param reason disconnect reason.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
    ble_connected = false;
    bt_ready();

    bt_conn_unref(g_conn);
    // UNSET_FLAG(flag_is_connected);
}

/**
 * @brief Passcode handler for accessing encrypted data
 *
 * @param conn connection handler
 * @param passkey passkey
 */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}
/**
 * @brief Conn callback data structs, holds
 *          function pointers.
 *
 */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};
/**
 * @brief Conn AUTH callback data structs, holds
 *          function pointers.
 *
 */
static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    //    .cancel = auth_cancel,
};

static uint16_t ahu_handle;
// static uint16_t long_ahu_handle;

static uint8_t ahu_packet[14];

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    int err;

    if (attr == NULL)
    {
        if (ahu_handle == 0)
        {
            printk("Did not discover ahu (%x)", ahu_handle);
        }

        (void)memset(params, 0, sizeof(*params));

        return BT_GATT_ITER_STOP;
    }

    SET_FLAG(flag_discover_complete);

    printk("[ATTRIBUTE] handle %u\n", attr->handle);

    if (params->type == BT_GATT_DISCOVER_PRIMARY &&
        bt_uuid_cmp(params->uuid, AHU_UUID) == 0)
    {
        printk("Found test service\n");
        params->uuid = NULL;
        params->start_handle = attr->handle + 1;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, params);
        if (err != 0)
        {
            printk("Discover failed (err %d)\n", err);
        }

        return BT_GATT_ITER_STOP;
    }
    else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC)
    {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

        if (bt_uuid_cmp(chrc->uuid, ACU_CH_UUID) == 0)
        {
            printk("Found chrc\n");
            ahu_handle = chrc->value_handle;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static void gatt_discover(void)
{
    static struct bt_gatt_discover_params discover_params;
    int err;

    printk("Discovering services and characteristics\n");

    discover_params.uuid = scu_svc_uuid;

    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(g_conn, &discover_params);
    if (err != 0)
    {
        printk("Discover failed(err %d)\n", err);
    }

    WAIT_FOR_FLAG(flag_discover_complete);

    printk("Discover complete\n");
    printk("%x\n", ahu_handle);
}

static ssize_t read_ahu_ch(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             (void *)ahu_packet, sizeof(ahu_packet));
}

static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_write_params *params)
{
    if (err != BT_ATT_ERR_SUCCESS)
    {
        printk("Write failed: 0x%02X\n", err);
    }

    (void)memset(params, 0, sizeof(*params));

    // SET_FLAG(flag_write_complete);
}

static void gatt_write(uint16_t handle, char packet[14])
{
    static struct bt_gatt_write_params write_params;
    // int err;

    printk("%u,%u,%u,%c%c%c%c%c%c\n", packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8]);

    // printk("Writing to ahu\n");
    write_params.data = packet;
    write_params.length = 14 * sizeof(char);

    write_params.func = gatt_write_cb;
    write_params.handle = handle;

    // UNSET_FLAG(flag_write_complete);

    if (bt_gatt_write(g_conn, &write_params) != 0)
    {
        printk("fail\n");
    }
}

static ssize_t receive_ahu(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           const void *buf, uint16_t len,
                           uint16_t offset, uint8_t flags)
{
    printk("FIRST\n");
    if (len > sizeof(ahu_packet))
    {
        printk("Invalid chrc length\n");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    else if (offset + len > sizeof(ahu_packet))
    {
        printk("Invalid chrc offset and length\n");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (flags != 0)
    {
        printk("Invalid flags");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    (void)memcpy(ahu_packet + offset, buf, len);

    struct received_packet data;
    data.preamble = ahu_packet[0];
    data.typelen = ahu_packet[1];
    data.id = ahu_packet[2];

    memcpy(data.data, &ahu_packet[3], 11 * sizeof(char));

    printk("%s\n", data.data);

    if (k_msgq_put(&command_msgq, &data, K_NO_WAIT) != 0)
    {
        k_msgq_purge(&command_msgq);
    }
    else
    {
        printk("add to queue %u\n", data.id);
    }

    return len;
}

void thread_get_message(void)
{
    struct received_packet data;

    while (1)
    {

        if (k_msgq_get(&command_msgq, &data, K_FOREVER) == 0)
        {
            printk("got a thing");
        }
    }
}

void thread_send_message(void)
{
    struct return_packet data;

    while (1)
    {
        if (k_msgq_get(&tosend_msgq, &data, K_FOREVER) == 0)
        {

            char scu_packet[14] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '\0'};
            scu_packet[0] = data.preamble;
            scu_packet[1] = data.typelen;
            scu_packet[2] = data.id;
            char reading[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            if (data.id != 0x08 && data.id != 0x09 && data.id != 0x0A)
            {
                sprintf(reading, "%-9.1f", data.data);
            }
            else
            {
                int value = (uint8_t)data.data;
                sprintf(reading, "%-9d", value);
            }
            memcpy(&scu_packet[3], &reading, 10 * sizeof(char));

            gatt_write(ahu_handle, scu_packet);

            memset(&data, 0, sizeof(struct return_packet));
        }
        k_msleep(100);
    }
}

BT_GATT_SERVICE_DEFINE(test_svc,
                       BT_GATT_PRIMARY_SERVICE(SCU_UUID),
                       BT_GATT_CHARACTERISTIC(SCU_CH_UUID,
                                              BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                                              read_ahu_ch, receive_ahu, NULL));

/**
 * @brief Enabled bluetooth, and sets connection callback handler, awaits
 *          central to connect to peripheral (mobile)
 *
 */
void thread_ble_connect(void)
{

    bt_ready();

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    while (1)
    {
        k_msleep(SHORT_SLEEP_MS);
    }
}

void thread_ble_discover(void *arg1, void *arg2, void *arg3)
{
    int flag = 0;
    while (1)
    {
        while (!ble_connected)
        {
            flag = 0;
            k_msleep(1000);
        }
        if (flag == 0)
        {
            gatt_discover();
            flag = 1;
        }
        k_msleep(100);
    }
}
