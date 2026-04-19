#include "ble_link.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "ble_link";

#define DEVICE_NAME           "RescueVision"
#define MAX_NOTIFY_PAYLOAD    240   // safe ceiling under MTU 247 minus 3 ATT bytes
#define MAX_POINTS_PER_NOTIFY ((MAX_NOTIFY_PAYLOAD - BLE_LINK_HEADER_BYTES) / BLE_LINK_POINT_BYTES)

// Custom 128-bit UUIDs (randomly generated, no semantic meaning).
//   service:      6e400d00-b5a3-f393-e0a9-e50e24dc4a01
//   frame_data:   6e400d01-b5a3-f393-e0a9-e50e24dc4a01
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
    0x01, 0x4a, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x00, 0x0d, 0x40, 0x6e);

    
static const ble_uuid128_t CHR_UUID = BLE_UUID128_INIT(
    0x01, 0x4a, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x0d, 0x40, 0x6e);

static uint8_t  s_own_addr_type;
static uint16_t s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_val_handle = 0;
static volatile bool s_subscribed = false;
static volatile uint16_t s_negotiated_mtu = 23;   // BLE default until exchange

static SemaphoreHandle_t s_publish_lock;

// ── GATT ────────────────────────────────────────────────────────────────────

static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // Frame data is notify-only; reads return empty.
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return 0;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid       = &CHR_UUID.u,
                .access_cb  = chr_access,
                .flags      = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
                .val_handle = &s_chr_val_handle,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gap_event(struct ble_gap_event *event, void *arg);

// ── advertising ─────────────────────────────────────────────────────────────

static void start_advertising(void) {
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields  fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.uuids128 = (ble_uuid128_t *)&SVC_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed; rc=%d", rc);
        return;
    }

    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising as \"%s\"", DEVICE_NAME);
}

// ── GAP events ──────────────────────────────────────────────────────────────

static int gap_event(struct ble_gap_event *event, void *arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "peer connected (handle=%u)", s_conn_handle);
            // Central initiates MTU exchange; we accept up to PREFERRED_MTU (247).
        } else {
            ESP_LOGW(TAG, "connect failed; status=%d", event->connect.status);
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "peer disconnected (reason=0x%04x)", event->disconnect.reason);
        s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed     = false;
        s_negotiated_mtu = 23;
        start_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        s_negotiated_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU negotiated: %u", s_negotiated_mtu);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_chr_val_handle) {
            s_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "subscriber %s", s_subscribed ? "ON" : "OFF");
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

// ── host callbacks ──────────────────────────────────────────────────────────

static void on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "addr infer failed; rc=%d", rc);
        return;
    }
    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "BLE ready, addr=%02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    start_advertising();
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "host reset; reason=%d", reason);
}

static void host_task(void *param) {
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ── public API ──────────────────────────────────────────────────────────────

bool ble_link_is_subscribed(void) {
    return s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

static int notify_chunk(const uint8_t *buf, size_t len) {
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) return BLE_HS_ENOMEM;
    return ble_gatts_notify_custom(s_conn_handle, s_chr_val_handle, om);
}

bool ble_link_publish_frame(uint32_t frame_num,
                            uint32_t timestamp_ms,
                            uint16_t dwm_heading_cdeg,
                            const ble_link_point_t *points,
                            uint16_t num_points) {
    if (!ble_link_is_subscribed()) return false;
    if (xSemaphoreTake(s_publish_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "publish lock timeout for frame %u", (unsigned)frame_num);
        return false;
    }

    // Per-notify capacity: keep some headroom under MTU.
    size_t mtu_payload = (s_negotiated_mtu > 3) ? (s_negotiated_mtu - 3) : 20;
    if (mtu_payload > MAX_NOTIFY_PAYLOAD) mtu_payload = MAX_NOTIFY_PAYLOAD;
    if (mtu_payload < BLE_LINK_HEADER_BYTES + BLE_LINK_POINT_BYTES) {
        mtu_payload = BLE_LINK_HEADER_BYTES + BLE_LINK_POINT_BYTES;
    }
    size_t pts_per_notify = (mtu_payload - BLE_LINK_HEADER_BYTES) / BLE_LINK_POINT_BYTES;

    uint8_t buf[MAX_NOTIFY_PAYLOAD];
    uint16_t sent = 0;
    int notifies = 0;
    bool ok = true;

    while (sent < num_points || (num_points == 0 && notifies == 0)) {
        uint16_t this_n = num_points - sent;
        if (this_n > pts_per_notify) this_n = (uint16_t)pts_per_notify;

        size_t off = 0;
        memcpy(buf + off, &frame_num,        4); off += 4;
        memcpy(buf + off, &timestamp_ms,     4); off += 4;
        memcpy(buf + off, &dwm_heading_cdeg, 2); off += 2;
        memcpy(buf + off, &this_n,           2); off += 2;

        if (this_n > 0) {
            memcpy(buf + off, &points[sent], (size_t)this_n * BLE_LINK_POINT_BYTES);
            off += (size_t)this_n * BLE_LINK_POINT_BYTES;
        }

        int rc = notify_chunk(buf, off);
        if (rc != 0) {
            ESP_LOGW(TAG, "notify failed frame=%u rc=%d (sent %u of %u pts)",
                     (unsigned)frame_num, rc, sent, num_points);
            ok = false;
            break;
        }
        sent += this_n;
        notifies++;
        if (num_points == 0) break;   // single zero-point notify
    }

    ESP_LOGI(TAG, "notified frame=%u pts=%u notifies=%d hdg=%.2f°",
             (unsigned)frame_num, num_points, notifies,
             (float)dwm_heading_cdeg / 100.0f);

    xSemaphoreGive(s_publish_lock);
    return ok;
}

void ble_link_init(void) {
    s_publish_lock = xSemaphoreCreateMutex();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc); return; }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc); return; }

    ble_svc_gap_device_name_set(DEVICE_NAME);

    // GAP event callback is registered per-connection by the stack via
    // ble_gap_adv_start's cb arg — we pass NULL there and rely on the host's
    // global event delivery: re-register via ble_gap_adv_start in connect mode.
    // Simplest: bind callback through ble_gap_event by setting it on adv_start.
    // We already pass it in start_advertising via the cb arg.
    // (NimBLE delivers all GAP events through that callback once advertising.)

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "NimBLE host started; service & char registered");
}
