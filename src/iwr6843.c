#include "iwr6843.h"
#include "dwm_geom.h"
#include "ble_link.h"
#include "vital_signs_cfg.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_cp210x.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// IWR6843AOPEVM CP2105 dual-UART bridge.
// Per TI mmWave SDK AOP demo: interface 0 = Standard = CLI  @ 115200;
//                             interface 1 = Enhanced = DATA @ 921600.
#define IWR_IFACE_DATA   1
#define IWR_IFACE_CLI    0

#define CLI_BAUD         115200
#define DATA_BAUD        921600

#define DATA_IN_BUF      8192
#define CLI_IN_BUF       512
#define STREAM_BUF_SIZE  32768
#define MAX_FRAME_BYTES  16384

static const char *TAG = "iwr6843";

static const uint8_t MAGIC[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

// Embedded config file — declared as a string literal in vital_signs_cfg.h.
#define CFG_TEXT        VITAL_SIGNS_CFG
#define CFG_TEXT_LEN    (sizeof(VITAL_SIGNS_CFG) - 1)

static StreamBufferHandle_t s_data_stream = NULL;
static cdc_acm_dev_hdl_t    s_cli_dev     = NULL;
static cdc_acm_dev_hdl_t    s_data_dev    = NULL;

// CLI ack channel.
static SemaphoreHandle_t s_cli_ack_sem = NULL;
static volatile int      s_cli_ack_type = 0;      // 0=none, 1=Done, 2=Error
static char              s_cli_line[256];
static size_t            s_cli_line_len = 0;

// Bring-up: hex-dump the first N raw bytes after sensorStart so we can verify
// the byte stream matches TI visualizer capture (Phase 8 step 5 of impl plan).
#define RAW_HEX_DUMP_BYTES 256
static volatile size_t s_raw_dumped = 0;

// ── helpers ──────────────────────────────────────────────────────────────────

static bool stream_read_exact(void *dst, size_t n, TickType_t timeout) {
    uint8_t *p = dst;
    size_t got = 0;
    TickType_t deadline = xTaskGetTickCount() + timeout;
    while (got < n) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) return false;
        size_t r = xStreamBufferReceive(s_data_stream, p + got, n - got, deadline - now);
        if (r == 0) return false;
        got += r;
    }
    return true;
}

static void hex_dump_line_tag(const char *tag, const uint8_t *buf, size_t n, size_t offset) {
    char line[80];
    size_t pos = 0;
    pos += snprintf(line + pos, sizeof(line) - pos, "%04x:", (unsigned)offset);
    for (size_t i = 0; i < n && pos < sizeof(line) - 4; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, " %02x", buf[i]);
    }
    ESP_LOGI(tag, "%s", line);
}

static void hex_dump_line(const uint8_t *buf, size_t n, size_t offset) {
    hex_dump_line_tag("iwr_raw", buf, n, offset);
}

// ── TLV handlers ─────────────────────────────────────────────────────────────

// Survival filter: keep only confident detections within useful range.
//   SNR_MIN_DB     — below this it's almost certainly noise / sidelobes
//   RANGE_MIN_M    — IWR's near-field is dominated by leakage / ringdown
//   RANGE_MAX_M    — beyond this distance_mm overflows uint16 (and the radar's
//                    SNR floor would have killed it anyway)
#define SNR_MIN_DB    12.0f
#define RANGE_MIN_M   0.30f
#define RANGE_MAX_M   6.00f
#define MAX_PTS_OUT   256

static void handle_compressed_points(uint32_t frame_num,
                                     const uint8_t *payload, uint32_t len) {
    if (len < sizeof(iwr_point_unit_t)) return;
    const iwr_point_unit_t *pu = (const iwr_point_unit_t *)payload;
    const iwr_compressed_point_t *pts =
        (const iwr_compressed_point_t *)(payload + sizeof(iwr_point_unit_t));
    uint32_t n = (len - sizeof(iwr_point_unit_t)) / sizeof(iwr_compressed_point_t);

    float assembly_hdg = dwm_get_assembly_world_heading_deg();
    ESP_LOGI(TAG, "pc: %u raw pts  assemblyHdg=%.1f°", (unsigned)n, assembly_hdg);

    static ble_link_point_t out[MAX_PTS_OUT];
    uint16_t kept = 0;
    uint32_t max_print = n < 3 ? n : 3;

    for (uint32_t i = 0; i < n && kept < MAX_PTS_OUT; i++) {
        float range = pts[i].range     * pu->rangeUnit;
        float az    = pts[i].azimuth   * pu->azUnit;
        float elev  = pts[i].elevation * pu->elevUnit;
        float snr   = pts[i].snr       * pu->snrUnit;

        if (snr < SNR_MIN_DB)      continue;
        if (range < RANGE_MIN_M)   continue;
        if (range > RANGE_MAX_M)   continue;

        dwm_point_t dp;
        dwm_transform_iwr_spherical(range, az, elev, &dp);

        if (dp.distance_mm > 65535.0f) continue;

        // Quantize to wire format.
        float bearing = dp.world_bearing_deg;
        if (bearing < 0.0f) bearing += 360.0f;
        if (bearing >= 360.0f) bearing -= 360.0f;

        float elev_cdeg_f = dp.world_elevation_deg * 100.0f;
        if (elev_cdeg_f >  9000.0f) elev_cdeg_f =  9000.0f;
        if (elev_cdeg_f < -9000.0f) elev_cdeg_f = -9000.0f;

        out[kept].distance_mm    = (uint16_t)(dp.distance_mm + 0.5f);
        out[kept].bearing_cdeg   = (uint16_t)(bearing * 100.0f + 0.5f);
        out[kept].elevation_cdeg = (int16_t)lroundf(elev_cdeg_f);

        if (i < max_print) {
            float dop = pts[i].doppler * pu->dopplerUnit;
            ESP_LOGI(TAG,
                     "   [%u] iwr(r=%.2fm az=%+.2f el=%+.2f) → "
                     "distDWM=%umm wbear=%u.%02u° wel=%+.1f°  dop=%.2f snr=%.1f",
                     (unsigned)i, range, az, elev,
                     out[kept].distance_mm,
                     out[kept].bearing_cdeg / 100u, out[kept].bearing_cdeg % 100u,
                     out[kept].elevation_cdeg / 100.0f,
                     dop, snr);
        }
        kept++;
    }

    if (kept == 0) {
        ESP_LOGI(TAG, "   no points survived filter (SNR≥%.0fdB, %.2f–%.2fm) — frame dropped",
                 SNR_MIN_DB, RANGE_MIN_M, RANGE_MAX_M);
        return;
    }

    if (!ble_link_is_subscribed()) return;

    float hdg = assembly_hdg;
    if (hdg < 0.0f)    hdg += 360.0f;
    if (hdg >= 360.0f) hdg -= 360.0f;
    uint16_t hdg_cdeg = (uint16_t)(hdg * 100.0f + 0.5f);
    uint32_t ts_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    ble_link_publish_frame(frame_num, ts_ms, hdg_cdeg, out, kept);
}

static void handle_target_list(const uint8_t *payload, uint32_t len) {
    uint32_t n = len / sizeof(iwr_target_t);
    const iwr_target_t *tgts = (const iwr_target_t *)payload;
    ESP_LOGI(TAG, "tracks: %u", (unsigned)n);
    for (uint32_t i = 0; i < n; i++) {
        ESP_LOGI(TAG, "   t%u pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) conf=%.2f",
                 (unsigned)tgts[i].tid,
                 tgts[i].posX, tgts[i].posY, tgts[i].posZ,
                 tgts[i].velX, tgts[i].velY, tgts[i].velZ,
                 tgts[i].confidenceLevel);
    }
}

static void handle_vital_signs(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(iwr_vital_signs_t)) return;
    const iwr_vital_signs_t *vs = (const iwr_vital_signs_t *)payload;
    ESP_LOGI(TAG, "vitals: id=%u bin=%u hr=%.1f br=%.1f breathDev=%.3f",
             vs->id, vs->rangeBin, vs->heartRate, vs->breathRate,
             vs->breathDeviation);
}

static void parse_tlvs(uint32_t frame_num,
                       const uint8_t *payload, uint32_t payload_len, uint32_t num_tlvs) {
    size_t off = 0;
    for (uint32_t i = 0; i < num_tlvs; i++) {
        if (sizeof(iwr_tlv_header_t) > payload_len - off) {
            ESP_LOGW(TAG, "TLV header overrun at i=%u off=%u", (unsigned)i, (unsigned)off);
            return;
        }
        iwr_tlv_header_t hdr;
        memcpy(&hdr, payload + off, sizeof(hdr));
        off += sizeof(iwr_tlv_header_t);

        if (hdr.length > payload_len - off) {
            ESP_LOGW(TAG, "TLV payload overrun (type=%u len=%u off=%u pl=%u) — dropping frame",
                     (unsigned)hdr.type, (unsigned)hdr.length,
                     (unsigned)off, (unsigned)payload_len);
            return;
        }

        switch (hdr.type) {
            case IWR_TLV_COMPRESSED_POINTS:
                handle_compressed_points(frame_num, payload + off, hdr.length);
                break;
            //case IWR_TLV_TARGET_LIST_3D:
                //handle_target_list(payload + off, hdr.length);
               // break;
            case IWR_TLV_VITAL_SIGNS:
                handle_vital_signs(payload + off, hdr.length);
                break;
            case IWR_TLV_TARGET_INDEX:
            case IWR_TLV_TARGET_HEIGHT:
            case IWR_TLV_PRESENCE:
                // recognised but not logged in bring-up
                break;
            default:
                ESP_LOGI(TAG, "   unhandled TLV type=%u len=%u",
                         (unsigned)hdr.type, (unsigned)hdr.length);
                break;
        }
        uint32_t aligned_length = (hdr.length + 3) & ~(uint32_t)3;
        off += aligned_length;
    }
}

// ── parser task ──────────────────────────────────────────────────────────────

static void parser_task(void *arg) {
    uint8_t window[8] = {0};

    for (;;) {
        // SYNC: Find magic word efficiently
        uint8_t b;
        if (!stream_read_exact(&b, 1, pdMS_TO_TICKS(500))) continue;
        if (b != MAGIC[0]) continue;
        
        window[0] = b;
        if (!stream_read_exact(&window[1], 7, pdMS_TO_TICKS(50))) continue;
        if (memcmp(window, MAGIC, 8) != 0) continue;

        // Read remaining 32 bytes of the 40-byte header.
        uint8_t hdr_rest[32];
        if (!stream_read_exact(hdr_rest, sizeof(hdr_rest), pdMS_TO_TICKS(200))) {
            ESP_LOGW(TAG, "header timeout after magic — resyncing");
            continue;
        }

        uint32_t version, totalPacketLen, platform, frameNumber, timeCpuCycles;
        uint32_t numDetectedObj, numTLVs, subFrameNumber;
        memcpy(&version,        hdr_rest +  0, 4);
        memcpy(&totalPacketLen, hdr_rest +  4, 4);
        memcpy(&platform,       hdr_rest +  8, 4);
        memcpy(&frameNumber,    hdr_rest + 12, 4);
        memcpy(&timeCpuCycles,  hdr_rest + 16, 4);
        memcpy(&numDetectedObj, hdr_rest + 20, 4);
        memcpy(&numTLVs,        hdr_rest + 24, 4);
        memcpy(&subFrameNumber, hdr_rest + 28, 4);

        if (totalPacketLen < sizeof(iwr_frame_header_t) ||
            totalPacketLen > MAX_FRAME_BYTES) {
            ESP_LOGW(TAG, "bad totalPacketLen=%u — resyncing", (unsigned)totalPacketLen);
            continue;
        }

        uint32_t remaining = totalPacketLen - sizeof(iwr_frame_header_t);
        uint8_t *payload = malloc(remaining);
        if (!payload) {
            ESP_LOGE(TAG, "OOM: could not alloc %u-byte payload", (unsigned)remaining);
            continue;
        }
        if (!stream_read_exact(payload, remaining, pdMS_TO_TICKS(500))) {
            ESP_LOGW(TAG, "payload timeout (wanted %u) — resyncing", (unsigned)remaining);
            free(payload);
            continue;
        }

        ESP_LOGI(TAG, "── frame %u: nTLV=%u nObj=%u totalLen=%u",
                 (unsigned)frameNumber, (unsigned)numTLVs,
                 (unsigned)numDetectedObj, (unsigned)totalPacketLen);

        parse_tlvs(frameNumber, payload, remaining, numTLVs);
        free(payload);
    }
}

// ── CDC callbacks ─────────────────────────────────────────────────────────────

static bool data_rx_cb(const uint8_t *data, size_t len, void *arg) {
    // Hex-dump the first RAW_HEX_DUMP_BYTES bytes of raw stream for bring-up.
    if (s_raw_dumped < RAW_HEX_DUMP_BYTES) {
        size_t remaining = RAW_HEX_DUMP_BYTES - s_raw_dumped;
        size_t take = len < remaining ? len : remaining;
        for (size_t off = 0; off < take; off += 16) {
            size_t chunk = (take - off) < 16 ? (take - off) : 16;
            hex_dump_line(data + off, chunk, s_raw_dumped + off);
        }
        s_raw_dumped += take;
    }
    xStreamBufferSend(s_data_stream, data, len, 0);
    return true;
}

// Bring-up: log the first CLI_RAW_DUMP_BYTES bytes received on the CLI port
// as hex, so we can see partial / garbled replies that never terminate a line.
#define CLI_RAW_DUMP_BYTES 256
static volatile size_t s_cli_raw_dumped = 0;

static bool cli_rx_cb(const uint8_t *data, size_t len, void *arg) {
    if (s_cli_raw_dumped < CLI_RAW_DUMP_BYTES) {
        size_t remaining = CLI_RAW_DUMP_BYTES - s_cli_raw_dumped;
        size_t take = len < remaining ? len : remaining;
        for (size_t off = 0; off < take; off += 16) {
            size_t chunk = (take - off) < 16 ? (take - off) : 16;
            hex_dump_line_tag("iwr_cli_raw", data + off, chunk, s_cli_raw_dumped + off);
        }
        s_cli_raw_dumped += take;
    }
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r') continue;
        if (c == '\n') {
            s_cli_line[s_cli_line_len] = '\0';
            if (s_cli_line_len > 0) {
                ESP_LOGI("iwr_cli", "<< %s", s_cli_line);
                if (strstr(s_cli_line, "Done")) {
                    s_cli_ack_type = 1;
                    xSemaphoreGive(s_cli_ack_sem);
                } else if (strstr(s_cli_line, "Error") ||
                           strstr(s_cli_line, "error")) {
                    s_cli_ack_type = 2;
                    xSemaphoreGive(s_cli_ack_sem);
                }
            }
            s_cli_line_len = 0;
        } else if (s_cli_line_len < sizeof(s_cli_line) - 1) {
            s_cli_line[s_cli_line_len++] = c;
        } else {
            s_cli_line_len = 0; // overflow, reset
        }
    }
    return true;
}

static void dev_event_cb(const cdc_acm_host_dev_event_data_t *event, void *user_ctx) {
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "IWR disconnected");
        cdc_acm_host_close(event->data.cdc_hdl);
        if (event->data.cdc_hdl == s_cli_dev)  s_cli_dev  = NULL;
        if (event->data.cdc_hdl == s_data_dev) s_data_dev = NULL;
    }
}

// ── CLI send + config ────────────────────────────────────────────────────────

static int send_cli_command(const char *cmd) {
    if (!s_cli_dev) return -1;

    // Drain any stale ack.
    xSemaphoreTake(s_cli_ack_sem, 0);
    s_cli_ack_type = 0;

    size_t n = strlen(cmd);
    ESP_LOGI("iwr_cli", ">> %s", cmd);

    uint8_t *buf = malloc(n + 1);
    if (!buf) return -2;
    memcpy(buf, cmd, n);
    buf[n] = '\n';
    esp_err_t err = cdc_acm_host_data_tx_blocking(s_cli_dev, buf, n + 1, 1000);
    free(buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tx failed: %s", esp_err_to_name(err));
        return -3;
    }

    if (xSemaphoreTake(s_cli_ack_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGW(TAG, "no ack within 2s for: %s", cmd);
        return -4;
    }
    return (s_cli_ack_type == 1) ? 0 : -5;
}

static void send_config_task(void *arg) {
    // Wait until both USB endpoints are open.
    for (int i = 0; i < 200 && (!s_cli_dev || !s_data_dev); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!s_cli_dev) {
        ESP_LOGE(TAG, "CLI port never opened — aborting config");
        vTaskDelete(NULL);
    }

    // Let the radar fully boot after DTR release (reset line may have been
    // pulsed during enumeration). 2s is generous; TI's own visualizer waits
    // a similar time before the first CLI send.
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "sending config (%u bytes)", (unsigned)CFG_TEXT_LEN);

    const char *p = CFG_TEXT;
    const char *cfg_end = CFG_TEXT + CFG_TEXT_LEN;
    char line[256];

    while (p < cfg_end) {
        const char *eol = p;
        while (eol < cfg_end && *eol != '\n' && *eol != '\r') eol++;
        size_t len = eol - p;
        if (len > 0 && len < sizeof(line)) {
            memcpy(line, p, len);
            line[len] = '\0';

            char *s = line;
            while (*s == ' ' || *s == '\t') s++;
            size_t slen = strlen(s);
            while (slen > 0 && (s[slen - 1] == ' ' || s[slen - 1] == '\t')) {
                s[--slen] = '\0';
            }

            if (*s && *s != '%') {
                int r = send_cli_command(s);
                if (r != 0) {
                    ESP_LOGW(TAG, "   (cmd result %d, continuing)", r);
                }
                vTaskDelay(pdMS_TO_TICKS(30));
            }
        }
        p = eol;
        while (p < cfg_end && (*p == '\n' || *p == '\r')) p++;
    }

    ESP_LOGI(TAG, "config complete — waiting for data frames");
    vTaskDelete(NULL);
}

// ── port open tasks ──────────────────────────────────────────────────────────

static void open_data_port_task(void *arg) {
    cdc_acm_host_device_config_t cfg = {
        .connection_timeout_ms = 30000,
        .out_buffer_size       = 0,
        .in_buffer_size        = DATA_IN_BUF,
        .event_cb              = dev_event_cb,
        .data_cb               = data_rx_cb,
        .user_arg              = NULL,
    };
    cdc_acm_dev_hdl_t dev = NULL;
    esp_err_t err = cp210x_vcp_open(CP2105_PID, IWR_IFACE_DATA, &cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "data port open failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    s_data_dev = dev;

    cdc_acm_line_coding_t lc = {
        .dwDTERate = DATA_BAUD, .bCharFormat = 0, .bParityType = 0, .bDataBits = 8,
    };
    cdc_acm_host_line_coding_set(s_data_dev, &lc);
    // DTR/RTS on CP2105 are wired to IWR boot/reset GPIOs on the AOPEVM.
    // Keep both DEASSERTED so the radar stays in Functional Mode, not reset.
    cdc_acm_host_set_control_line_state(s_data_dev, false, false);
    ESP_LOGI(TAG, "DATA port open @ %u baud (iface %u)", DATA_BAUD, IWR_IFACE_DATA);
    vTaskDelete(NULL);
}

static void open_cli_port_task(void *arg) {
    cdc_acm_host_device_config_t cfg = {
        .connection_timeout_ms = 30000,
        .out_buffer_size       = 256,
        .in_buffer_size        = CLI_IN_BUF,
        .event_cb              = dev_event_cb,
        .data_cb               = cli_rx_cb,
        .user_arg              = NULL,
    };
    cdc_acm_dev_hdl_t dev = NULL;
    esp_err_t err = cp210x_vcp_open(CP2105_PID, IWR_IFACE_CLI, &cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CLI port open failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    s_cli_dev = dev;

    cdc_acm_line_coding_t lc = {
        .dwDTERate = CLI_BAUD, .bCharFormat = 0, .bParityType = 0, .bDataBits = 8,
    };
    cdc_acm_host_line_coding_set(s_cli_dev, &lc);
    // DTR/RTS on CP2105 are wired to IWR boot/reset GPIOs on the AOPEVM.
    // Keep both DEASSERTED so the radar stays in Functional Mode, not reset.
    cdc_acm_host_set_control_line_state(s_cli_dev, false, false);
    ESP_LOGI(TAG, "CLI port open @ %u baud (iface %u)", CLI_BAUD, IWR_IFACE_CLI);
    vTaskDelete(NULL);
}

// ── public API ────────────────────────────────────────────────────────────────

void iwr6843_init(void) {
    _Static_assert(sizeof(iwr_frame_header_t) == 40,  "frame header must be 40B");
    _Static_assert(sizeof(iwr_tlv_header_t)   == 8,   "tlv header must be 8B");
    _Static_assert(sizeof(iwr_point_unit_t)   == 20,  "pUnit must be 20B");
    _Static_assert(sizeof(iwr_compressed_point_t) == 8, "point must be 8B");
    _Static_assert(sizeof(iwr_target_t)       == 112, "target must be 112B");
    _Static_assert(sizeof(iwr_vital_signs_t)  == 136, "vitals must be 136B");

    s_data_stream  = xStreamBufferCreate(STREAM_BUF_SIZE, 1);
    s_cli_ack_sem  = xSemaphoreCreateBinary();

    xTaskCreate(parser_task,         "iwr_parser",   8192, NULL, 6, NULL);
    xTaskCreate(open_cli_port_task,  "iwr_cli_open", 4096, NULL, 5, NULL);
    xTaskCreate(open_data_port_task, "iwr_data_open",4096, NULL, 5, NULL);
    xTaskCreate(send_config_task,    "iwr_cfg",      4096, NULL, 4, NULL);
}
