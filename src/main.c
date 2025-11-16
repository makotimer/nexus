/**
 * @file main.c
 * @brief BLE Central: Scan, Connect, Read Device Name, Log with 10s Debounce
 *
 * This application scans for nearby BLE devices, connects to new ones (or those
 * not seen in the last 10 seconds), reads the Device Name via GATT (if not in
 * advertising data), and logs results. Designed for Zephyr on Nordic nRF DK.
 *
 * Features:
 * - Passive scanning with RSSI filtering
 * - 10-second re-log debounce using history tracking
 * - Safe connection/disconnect via workqueue
 * - GATT discovery of GAP Device Name (0x2A00)
 * - Robust error handling and scan restart logic
 *
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

/* ==========================================================================
 * 1. Device History Tracking (10-second re-log prevention)
 * ========================================================================== */
#define MAX_HISTORY_ENTRIES     8
#define RELOG_INTERVAL_MS       10000  // 10 seconds

struct history_entry {
    bt_addr_le_t addr;      // Device address
    int64_t      last_seen; // Timestamp from k_uptime_get()
};

static struct history_entry device_history[MAX_HISTORY_ENTRIES];
static size_t               history_count = 0;

/**
 * @brief Check if a device address exists in history
 */
static bool history_contains(const bt_addr_le_t *addr)
{
    for (size_t i = 0; i < history_count; i++) {
        if (bt_addr_le_eq(&device_history[i].addr, addr)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Update or add device to history with current timestamp
 */
static void history_update(const bt_addr_le_t *addr)
{
    int64_t now = k_uptime_get();

    // Update existing entry
    for (size_t i = 0; i < history_count; i++) {
        if (bt_addr_le_eq(&device_history[i].addr, addr)) {
            device_history[i].last_seen = now;
            return;
        }
    }

    // Add new entry
    if (history_count < MAX_HISTORY_ENTRIES) {
        bt_addr_le_copy(&device_history[history_count].addr, addr);
        device_history[history_count].last_seen = now;
        history_count++;
    } else {
        // Replace oldest entry
        size_t oldest_idx = 0;
        for (size_t i = 1; i < MAX_HISTORY_ENTRIES; i++) {
            if (device_history[i].last_seen < device_history[oldest_idx].last_seen) {
                oldest_idx = i;
            }
        }
        bt_addr_le_copy(&device_history[oldest_idx].addr, addr);
        device_history[oldest_idx].last_seen = now;
    }
}

/**
 * @brief Check if enough time has passed to re-log this device
 */
static bool history_allow_log(const bt_addr_le_t *addr)
{
    int64_t now = k_uptime_get();

    for (size_t i = 0; i < history_count; i++) {
        if (bt_addr_le_eq(&device_history[i].addr, addr)) {
            return (now - device_history[i].last_seen) >= RELOG_INTERVAL_MS;
        }
    }
    return true; // Not seen before
}

/* ==========================================================================
 * 2. Workqueue Handlers (Safe async operations)
 * ========================================================================== */
static struct bt_conn *active_conn = NULL;

static void restart_scan_work_handler(struct k_work *work);
static void disconnect_work_handler(struct k_work *work);

static void device_found(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t adv_type, struct net_buf_simple *ad);

K_WORK_DELAYABLE_DEFINE(restart_scan_work, restart_scan_work_handler);
K_WORK_DELAYABLE_DEFINE(disconnect_work, disconnect_work_handler);

/**
 * @brief Restart BLE scanning when safe (not connected)
 */
static void restart_scan_work_handler(struct k_work *work)
{
    if (active_conn) {
        // Still connected — retry soon
        k_work_reschedule(&restart_scan_work, K_MSEC(500));
        return;
    }

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err == 0) {
        return; // Success
    }

    if (err == -EALREADY) {
        printk("Scan stopping in progress, retrying in 100ms...\n");
        k_work_reschedule(&restart_scan_work, K_MSEC(100));
        return;
    }

    printk("Failed to start scan (err %d), retrying in 1s\n", err);
    k_work_reschedule(&restart_scan_work, K_SECONDS(1));
}

/**
 * @brief Safely disconnect the active connection
 */
static void disconnect_work_handler(struct k_work *work)
{
    if (!active_conn) {
        return;
    }

    bt_conn_disconnect(active_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

/* ==========================================================================
 * 3. GATT Client: Discover and Read Device Name (0x2A00)
 * ========================================================================== */
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_read_params    read_params;
static uint16_t                      name_chrc_handle;
static char                          device_name[64] = {0};

static struct bt_uuid_16 gap_name_uuid = BT_UUID_INIT_16(BT_UUID_GAP_DEVICE_NAME_VAL);

/**
 * @brief GATT read callback — called when Device Name is read
 */
static uint8_t gatt_read_name_cb(struct bt_conn *conn, uint8_t err,
                                 struct bt_gatt_read_params *params,
                                 const void *data, uint16_t length)
{
    if (err) {
        printk("GATT read failed (err %u)\n", err);
    } else if (data && length > 0) {
        size_t copy_len = MIN(length, sizeof(device_name) - 1);
        memcpy(device_name, data, copy_len);
        device_name[copy_len] = '\0';
        printk("GATT Device Name: \"%s\"\n", device_name);
    } else {
        printk("GATT Device Name: (empty)\n");
    }

    // Schedule disconnect (never disconnect in callback)
    k_work_reschedule(&disconnect_work, K_NO_WAIT);
    return BT_GATT_ITER_STOP;
}

/**
 * @brief GATT discovery callback — find Device Name characteristic
 */
static uint8_t gatt_discover_cb(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Device Name characteristic not found\n");
        k_work_reschedule(&disconnect_work, K_NO_WAIT);
        return BT_GATT_ITER_STOP;
    }

    if (bt_uuid_cmp(params->uuid, &gap_name_uuid.uuid) == 0) {
        name_chrc_handle = attr->handle + 1; // Value handle

        // Read the value
        memset(&read_params, 0, sizeof(read_params));
        read_params.func = gatt_read_name_cb;
        read_params.handle_count = 1;
        read_params.single.handle = name_chrc_handle;
        read_params.single.offset = 0;

        int err = bt_gatt_read(conn, &read_params);
        if (err) {
            printk("Failed to start GATT read (%d)\n", err);
            k_work_reschedule(&disconnect_work, K_NO_WAIT);
        }
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;
}

/* ==========================================================================
 * 4. Connection Management
 * ========================================================================== */
static void conn_connected(struct bt_conn *conn, uint8_t err)
{
    if (err || conn != active_conn) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    // Use name from advertising if available
    if (device_name[0] != '\0') {
        printk("Connected: %s — Name from adv: \"%s\"\n", addr_str, device_name);
        k_work_reschedule(&disconnect_work, K_NO_WAIT);
        return;
    }

    // Discover Device Name via GATT
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = &gap_name_uuid.uuid;
    discover_params.func = gatt_discover_cb;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int rc = bt_gatt_discover(conn, &discover_params);
    if (rc) {
        printk("Failed to start discovery (%d)\n", rc);
        k_work_reschedule(&disconnect_work, K_NO_WAIT);
    }
}

static void conn_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (conn != active_conn) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("Disconnected from %s (reason 0x%02x)\n", addr_str, reason);

    bt_conn_unref(active_conn);
    active_conn = NULL;

    // Restart scanning after short delay
    k_work_reschedule(&restart_scan_work, K_MSEC(300));
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = conn_connected,
    .disconnected = conn_disconnected,
};

/* ==========================================================================
 * 5. Advertising Data Parsing
 * ========================================================================== */
/**
 * @brief Parse advertising data for device name
 * @return true if name found (stop parsing), false to continue
 */
static bool adv_parse_name(struct bt_data *data, void *user_data)
{
    char *name_buf = user_data;

    if (data->type != BT_DATA_NAME_SHORTENED && data->type != BT_DATA_NAME_COMPLETE) {
        return false;
    }

    size_t len = MIN(data->data_len, sizeof(device_name) - 1);
    memcpy(name_buf, data->data, len);
    name_buf[len] = '\0';
    return true; // Stop parsing
}

/* ==========================================================================
 * 6. Scan Callback — Device Found
 * ========================================================================== */
static void device_found(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t adv_type, struct net_buf_simple *ad)
{
    // Ignore if already connected
    if (active_conn) {
        return;
    }

    // Filter non-connectable or weak signals
    if (adv_type != BT_GAP_ADV_TYPE_ADV_IND &&
        adv_type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }
    if (rssi < -75) {
        return;
    }

    // Debounce: skip if logged recently
    if (history_contains(addr) && !history_allow_log(addr)) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    // Parse name from advertising data
    device_name[0] = '\0';
    bt_data_parse(ad, adv_parse_name, device_name);

    // Log discovery
    if (device_name[0] != '\0') {
        if (strncmp(device_name, "S24", 3) == 0) {
            printk("Found phone --> %s (RSSI %d dBm): %s\n", addr_str, rssi, device_name);
        } else {
            printk("Found device --> %s (RSSI %d dBm): \"%s\"\n", addr_str, rssi, device_name);
        }
    } else {
        printk("Found device --> %s (RSSI %d dBm)\n", addr_str, rssi);
    }

    // Update history *before* connecting
    history_update(addr);

    // Stop scan and attempt connection
    bt_le_scan_stop();

    struct bt_le_conn_param *conn_params = BT_LE_CONN_PARAM(
        BT_GAP_INIT_CONN_INT_MIN,
        BT_GAP_INIT_CONN_INT_MAX,
        0,   // Latency
        600  // Timeout (x10ms)
    );

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, conn_params, &active_conn);
    if (err) {
        printk("Connection failed (%d), restarting scan...\n", err);
        k_work_reschedule(&restart_scan_work, K_MSEC(500));
    }
}

/* ==========================================================================
 * 7. Main Entry Point
 * ========================================================================== */
int main(void)
{
    printk("Starting BLE Central Scanner...\n");

    if (bt_enable(NULL)) {
        printk("Bluetooth initialization failed\n");
        return 0;
    }

    printk("Bluetooth initialized successfully\n");

    // Register connection callbacks
    bt_conn_cb_register(&conn_callbacks);

    // Start scanning immediately
    k_work_schedule(&restart_scan_work, K_NO_WAIT);

    // Main thread sleeps — all work is async
    return 0;
}