/*
 ************************************************************************
 * @file bluetooth_driver.c
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Includes all of the necessary actions for connecting and communicating
 * over bluetooth.
 **********************************************************************
 */

#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <shell/shell.h>
#include <sys/byteorder.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <logging/log.h>
#include <errno.h>

#include "kernel.h"


#include "bluetooth_driver.h"
#include "hci.h"

/*Register Bluetooth Shel Logger*/
LOG_MODULE_REGISTER(bluetooth_module, LOG_LEVEL_DBG);

/*Define and create flags and flag functions for discover func*/
#define CREATE_FLAG(flag) static atomic_t flag = (atomic_t) false
#define SET_FLAG(flag) (void)atomic_set(&flag, (atomic_t) true)
#define WAIT_FOR_FLAG(flag)          \
    while (!(bool)atomic_get(&flag)) \
    {                                \
        (void)k_sleep(K_MSEC(1));    \
    }

CREATE_FLAG(flag_discover_complete);

/*Include external structs from other files*/
extern struct k_msgq bluetoothQueue;

extern struct k_msgq buttonQueue;

extern struct k_msgq samplerQueue;

/*Define data structs for each queue*/
struct data_item_t
{
    uint8_t devID;
    uint8_t packet[13];
};

struct data_item_t2
{
    uint8_t sampleTime;
    uint8_t JSONFormat;
    struct shell *shellPrint;
} samplerObj;

struct data_item_t3
{
    uint8_t buttonState;
} buttonObject;

struct data_item_t4
{
    uint8_t rssi;
    uint8_t USData;
} currentDevice;

static void start_scan(void);

/*Define all initial variables for hci packets, handles, etc.*/
struct bt_conn *default_conn;

bool ble_connected;

#define UUID_BUFFER_SIZE 16

static uint8_t chrc_data[14] = {DEV_PREAMBLE, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t chrc2_data[14] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint16_t chrc_handle = 0;

/*Charecteristic UUIDs for this devices (CH2) and SCU (CH)*/
#define TEST_CH_UUID                                                    \
    BT_UUID_DECLARE_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

#define TEST_CH2_UUID                                                   \
    BT_UUID_DECLARE_128(0x02, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

uint16_t mobile_uuid[] = {0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                          0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};

static struct bt_uuid *test_svc_uuid = TEST_CH_UUID;

#define TEST_CHRC_UUID                                                  \
    BT_UUID_DECLARE_128(0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00)

#define TEST_CHRC2_UUID                                                 \
    BT_UUID_DECLARE_128(0x02, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x03, \
                        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0x00)

//RSSI RX BUFFER
int16_t rx_rssi[] = {0x00, 0x00, 0x00, 0x00};

uint8_t read_rssi_from_mobile(struct bt_conn *conn, uint8_t err,
                              struct bt_gatt_read_params *params,
                              const void *data, uint16_t length)
{
    memcpy(&rx_rssi, data, sizeof(rx_rssi));
    printk("RSSI: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_rssi[0], rx_rssi[1], rx_rssi[2], rx_rssi[3]);
    return 0;
}

/*Found Device Function to establish connection with device*/
static bool parse_device(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *addr = user_data;
    int i;
    int matchedCount = 0;

    if (data->type == BT_DATA_UUID128_ALL)
    {

        uint16_t temp = 0;
        for (i = 0; i < data->data_len; i++)
        {
            temp = data->data[i];
            if (temp == mobile_uuid[i])
            {
                matchedCount++;
            }
        }

        if (matchedCount == UUID_BUFFER_SIZE)
        {
            LOG_INF("RSSI of Node is: %d", currentDevice.rssi);
            // MOBILE UUID MATCHED
            
            LOG_INF("Mobile UUID Found, attempting to connect");

            int err = bt_le_scan_stop();
            k_msleep(10);

            if (err)
            {
                LOG_ERR("Stop LE scan failed (err %d)", err);
                return true;
            }
            
            /*
            struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;

            err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                    param, &default_conn);
            if (err)
            {
                LOG_ERR("Create conn failed (err %d)", err);
                
            }
            */
            start_scan();
            
            return false;
        }
    }
    return true;
}

/*Write callback for when devices is written to*/
static ssize_t write_test_chrc(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{
    if (len > sizeof(chrc2_data))
    {
        LOG_ERR("Invalid chrc length");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    else if (offset + len > sizeof(chrc2_data))
    {
        LOG_ERR("Invalid chrc offset and length");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (flags != 0)
    {
        LOG_ERR("Invalid flags %u", flags);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    (void)memcpy(chrc2_data + offset, buf, len);

    /*Check if it is a dc packet*/
    if (chrc2_data[2] == 0x0B)
    {
        LOG_INF("Recieved Confirmation of DC W");
        return len;
    }

    /*Print json format or regular*/
    if (samplerObj.JSONFormat == 1)
    {
        print_JSON(samplerObj.shellPrint, chrc2_data);
    }
    else
    {
        print_reading(chrc2_data);
    }
    return len;
}

/*Second gatt write functions*/
static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_write_params *params)
{
    if (err != BT_ATT_ERR_SUCCESS)
    {
        LOG_ERR("Write cb failed: 0x%02X", err);
        return;
    }

    return;

    (void)memset(params, 0, sizeof(*params));
}

/*Main gatt write function pulls write_parambs for gatt_writ_cb*/
static void gatt_write(uint16_t handle)
{
    static struct bt_gatt_write_params write_params;
    int err;

    write_params.data = chrc_data;
    write_params.length = sizeof(chrc_data);

    write_params.func = gatt_write_cb;
    write_params.handle = handle;

    err = bt_gatt_write(default_conn, &write_params);

    if (err != 0)
    {
        LOG_ERR("bt_gatt_write failed: %d", err);
        return;
    }
    else
    {
        return;
    }
}

/*Call this to dsicover handle of device*/
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    int err;

    if (attr == NULL)
    {
        if (chrc_handle == 0)
        {
            LOG_ERR("Did not discover chrc (%x)",
                    chrc_handle);
        }

        (void)memset(params, 0, sizeof(*params));

        return BT_GATT_ITER_STOP;
    }

    SET_FLAG(flag_discover_complete);

    if (params->type == BT_GATT_DISCOVER_PRIMARY &&
        bt_uuid_cmp(params->uuid, TEST_CH_UUID) == 0)
    {

        params->uuid = NULL;
        params->start_handle = attr->handle + 1;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, params);
        if (err != 0)
        {
            LOG_ERR("Discover failed (err %d)", err);
        }

        return BT_GATT_ITER_STOP;
    }
    else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC)
    {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

        if (bt_uuid_cmp(chrc->uuid, TEST_CHRC_UUID) == 0)
        {
            chrc_handle = chrc->value_handle;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/*This is used to discover the device UUID's*/
static void gatt_discover(void)
{
    static struct bt_gatt_discover_params discover_params;
    int err;

    LOG_INF("Discovering services and characteristics");

    discover_params.uuid = test_svc_uuid;

    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(default_conn, &discover_params);
    if (err != 0)
    {
        LOG_ERR("Discover failed(err %d)", err);
    }

    WAIT_FOR_FLAG(flag_discover_complete);
}

/*When a device is found this is called and in turne callse parse_device*/
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{

    currentDevice.rssi = rssi;

    if (default_conn)
    {
        return;
    }

    if (type == BT_GAP_ADV_TYPE_ADV_IND ||
        type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
    {
        bt_data_parse(ad, parse_device, (void *)addr);
    }
}

/*Start scanning for SCU's*/
static void start_scan(void)
{
    int err;

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err)
    {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return;
    }

    LOG_INF("Scanning successfully started");
}

/*Once connected*/
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        LOG_ERR("Failed to connect to %s (%u)", addr, err);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        start_scan();
        return;
    }

    if (conn != default_conn)
    {
        return;
    }
    ble_connected = true;
    LOG_INF("Connected: %s", addr);
}

/*Once disconnected*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (conn != default_conn)
    {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;
    ble_connected = false;
    start_scan();
}

/*Define callbacks and characteristics and UUIDs for AHU*/
BT_GATT_SERVICE_DEFINE(test_svc,
                       BT_GATT_PRIMARY_SERVICE(TEST_CH2_UUID),
                       BT_GATT_CHARACTERISTIC(TEST_CHRC2_UUID,
                                              BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                                              NULL, write_test_chrc, NULL), );

/*Define connected state pcallbacks to above functions*/
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/*Thread for reading SCU UUIDs and gatt_writing to it*/
void thread_ble_read(void *arg1, void *arg2, void *arg3)
{
    /*flag for if device is discovered or not*/
    int flag = 0;
    while (1)
    {
        while (!ble_connected)
        {
            flag = 0;
            k_msleep(500);
        }
        /*While connected and jason format = 1 print json format*/
        while (ble_connected)
        {
            if (flag == 0)
            {
                gatt_discover();
                flag = 1;
            }
            k_msleep(100);
            if (samplerObj.JSONFormat == 1)
            {
                chrc_data[2] = 8;
            }
            while (samplerObj.JSONFormat == 1)
            {
                if (buttonObject.buttonState == 0)
                {
                    chrc_data[3] = '\0';
                    chrc_data[2] = 0x00;

                    while (chrc_data[2] < LIS2DH_Z_ID)
                    {
                        chrc_data[2]++;
                        gatt_write(chrc_handle);
                        k_msleep(20);
                    }

                    chrc_data[2] = PB_Dev_ID;
                    gatt_write(chrc_handle);
                    k_msleep(20);
                    k_msleep(samplerObj.sampleTime * 1000);
                }
                k_msleep(20);
            }
        }
    }
}

/*Thread to handle button queue*/
void thread_button_ble(void *arg1, void *arg2, void *arg3)
{
    /*While Loop for reading queue*/
    while (1)
    {
        k_msgq_get(&buttonQueue, &buttonObject, K_FOREVER);
        k_msleep(100);
    }
}

/*Thread for processing commands*/
void thread_ble_commands(void *arg1, void *arg2, void *arg3)
{
    /*Initialise Queue Data*/
    struct data_item_t dataReceived;
    uint8_t reading[11];
    uint8_t length = 0;

    /*While Loop for reading queue*/
    while (1)
    {
        if (k_msgq_get(&bluetoothQueue, &dataReceived, K_FOREVER) == 0)
        {
            length = 0;
            chrc_data[2] = dataReceived.devID;
            for (int i = 0; i < 10; i++)
            {
                chrc_data[i + 3] = dataReceived.packet[i];
            }
            for (int i = 0; i < 11; i++)
            {
                if (chrc_data[i + 3])
                {
                    length++;
                }
                reading[i] = chrc_data[i + 3];
            }
            chrc_data[1] = (0x10 + length);
            gatt_write(chrc_handle);
        }
    }
}

/*Thread for processing sampler w command*/
void thread_samp_commands(void *arg1, void *arg2, void *arg3)
{
    /*Set Default JSON Format*/
    samplerObj.JSONFormat = 0;
    while (1)
    {
        k_msgq_get(&samplerQueue, &samplerObj, K_FOREVER);
    }
}

/*Bt initialiser client*/
void ble_client(void *arg1, void *arg2, void *arg3)
{
    int err;

    /*Enable Bluetooth*/
    err = bt_enable(NULL);
    default_conn = NULL;

    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    /*DBG message to state bluetooth is finished initialising*/
    LOG_DBG("Bluetooth initialized");

    /*setup blutooth callbacks for connected and disconnected*/
    bt_conn_cb_register(&conn_callbacks);

    /*Start scanning for device*/
    start_scan();
}
