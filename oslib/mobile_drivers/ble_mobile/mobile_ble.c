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

#include "ble_uuid.h"
#include "hci_driver.h"
#include "mobile_ble.h"

K_SEM_DEFINE(sem_data_arrived, 0, 1);

#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t) false
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t) true)
#define WAIT_FOR_FLAG(flag)          \
    while (!(bool)atomic_get(&flag)) \
    {                                \
        (void)k_sleep(K_MSEC(1));    \
    }

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xd5, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                  0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb),
};


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

static uint16_t base_handle;

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{

    int err;

    if (attr == NULL)
    {
        if (base_handle == 0)
        {
            printk("Did not discover ahu (%x)", base_handle);
        }

        (void)memset(params, 0, sizeof(*params));

        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_PRIMARY &&
        bt_uuid_cmp(params->uuid, &node_ahu.uuid) == 0)
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

        if (bt_uuid_cmp(chrc->uuid, &node_rx.uuid) == 0)
        {
            printk("Found chrc\n");
            base_handle = chrc->value_handle;
        }
        SET_FLAG(flag_discover_complete);
    }

    return BT_GATT_ITER_CONTINUE;
}

static void gatt_discover(void)
{
    static struct bt_gatt_discover_params discover_params;
    int err;

    printk("Discovering services and characteristics\n");

    discover_params.uuid = &node_ahu.uuid;

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

    //k_sem_give(&sem_data_arrived);
}

static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_write_params *params)
{
    if (err != BT_ATT_ERR_SUCCESS)
    {
        printk("Write failed: 0x%02X\n", err);
    }

    (void)memset(params, 0, sizeof(*params));

}

static void gatt_write(uint16_t handle)
{
    static struct bt_gatt_write_params write_params;

    //printk("Writing to base\n");
    write_params.data = tx_buff;
    write_params.length = 19 * sizeof(uint8_t);

    write_params.func = gatt_write_cb;
    write_params.handle = handle;

    if (bt_gatt_write(g_conn, &write_params) != 0)
    {
        printk("fail\n");
    }
}

BT_GATT_SERVICE_DEFINE(test_svc,
                       BT_GATT_PRIMARY_SERVICE(&node_scu),
                       BT_GATT_CHARACTERISTIC(&node_tx.uuid,
                                              BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                                              NULL, NULL, NULL));

/**
 * @brief Enabled bluetooth, and sets connection callback handler, awaits
 *          central to connect to peripheral (mobile)
 *
 */
void thread_ble_connect(void)
{

    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_ready();

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

}

void thread_ble_discover(void *arg1, void *arg2, void *arg3)
{
    int flag = 0;
    uint8_t lastNum = 0;
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
        if (flag == 1){
            k_msleep(300);
            //if(lastNum != tx_buff[7]){
                printk("Writing: \n{");
                for(int i = 0; i < 19; i++){
                    printk("%d, ", tx_buff[i]-256);
                }
                printk("}\n");
                lastNum = tx_buff[7];
                gatt_write(base_handle);
           // }
            
        }
    }
}