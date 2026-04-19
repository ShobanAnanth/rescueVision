#include "iwr6843.h"
#include "vital_signs_cfg.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_cp210x.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "esp_system.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ble_link.h"
#include "dwm_geom.h"

#if HEURISTIC_GATES == 2
#include "rescue_vision_model.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/Micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#endif

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

#if HEURISTIC_GATES == 2
// TFLite Globals
static const tflite::Model* s_model = nullptr;
static tflite::MicroInterpreter* s_interpreter = nullptr;
static TfLiteTensor* s_input = nullptr;
static TfLiteTensor* s_output = nullptr;

constexpr int kTensorArenaSize = 15 * 1024; // 15KB Arena
static uint8_t s_tensor_arena[kTensorArenaSize];

static void init_tflite_model() {
    tflite::InitializeTarget();
    s_model = tflite::GetModel(rescue_vision_model);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model provided is schema version %ld not equal to supported version %d.",
                 s_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    static tflite::MicroMutableOpResolver<5> resolver;
    resolver.AddAdd();
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddSoftmax();
    resolver.AddMul();

    static tflite::MicroInterpreter static_interpreter(
        s_model, resolver, s_tensor_arena, kTensorArenaSize);
    s_interpreter = &static_interpreter;

    TfLiteStatus allocate_status = s_interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return;
    }

    s_input = s_interpreter->input(0);
    s_output = s_interpreter->output(0);
    ESP_LOGI(TAG, "TFLite Model Initialized successfully.");
}
#endif

EventGroupHandle_t g_iwr_state_group = NULL;
volatile bool g_pause_iwr_listening = false;
static bool s_config_done = false;

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
    uint8_t *p = (uint8_t*)dst;
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

#define MAX_TARGETS 20
static iwr_target_t s_last_targets[MAX_TARGETS];
static uint32_t s_num_targets = 0;

static iwr_target_t s_prev_targets[MAX_TARGETS];
static uint32_t s_num_prev_targets = 0;

static iwr_vital_signs_t s_last_vitals[MAX_TARGETS];
static uint32_t s_num_vitals = 0;

static iwr_vital_signs_t s_prev_vitals[MAX_TARGETS];
static uint32_t s_num_prev_vitals = 0;

static uint32_t s_last_active_track_tick = 0;

bool iwr6843_has_active_targets(void) {
    if (s_last_active_track_tick == 0) return false;
    // Considered active if valid target seen within last 2 seconds
    return (xTaskGetTickCount() - s_last_active_track_tick) < pdMS_TO_TICKS(2000);
}

#define MAX_POINTS 500
static iwr_point_unit_t s_pc_unit;
static iwr_compressed_point_t s_last_points[MAX_POINTS];
static uint32_t s_num_points = 0;

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

static void handle_compressed_points(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(iwr_point_unit_t)) return;
    const iwr_point_unit_t *pu = (const iwr_point_unit_t *)payload;
    const iwr_compressed_point_t *pts =
        (const iwr_compressed_point_t *)(payload + sizeof(iwr_point_unit_t));
    uint32_t n = (len - sizeof(iwr_point_unit_t)) / sizeof(iwr_compressed_point_t);
    // Save points for Centroid Gating.
    s_pc_unit = *pu;
    s_num_points = (n < MAX_POINTS) ? n : MAX_POINTS;
    if (s_num_points > 0) {
        memcpy(s_last_points, pts, s_num_points * sizeof(iwr_compressed_point_t));
    }
}

static void handle_target_list(const uint8_t *payload, uint32_t len) {
    uint32_t n = len / sizeof(iwr_target_t);
    const iwr_target_t *tgts = (const iwr_target_t *)payload;
    s_num_targets = (n < MAX_TARGETS) ? n : MAX_TARGETS;
    for (uint32_t i = 0; i < s_num_targets; i++) {
        memcpy(&s_last_targets[i], &tgts[i], sizeof(iwr_target_t));
    }
}

static void handle_vital_signs(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(iwr_vital_signs_t)) return;
    const iwr_vital_signs_t *vs = (const iwr_vital_signs_t *)payload;
    if (s_num_vitals < MAX_TARGETS) {
        memcpy(&s_last_vitals[s_num_vitals], vs, sizeof(iwr_vital_signs_t));
        s_num_vitals++;
    }
}

static void parse_tlvs(uint32_t frame_num, const uint8_t *payload, uint32_t payload_len, uint32_t num_tlvs) {
    size_t off = 0;
    
    // Save last frame data before overwriting
    memcpy(s_prev_targets, s_last_targets, s_num_targets * sizeof(iwr_target_t));
    s_num_prev_targets = s_num_targets;
    memcpy(s_prev_vitals, s_last_vitals, s_num_vitals * sizeof(iwr_vital_signs_t));
    s_num_prev_vitals = s_num_vitals;

    // Clear out tracked info accumulated from the prior frame
    s_num_vitals = 0;
    s_num_targets = 0;
    s_num_points = 0;

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
            break;
        }

        switch (hdr.type) {
            case IWR_TLV_COMPRESSED_POINTS:
                handle_compressed_points(payload + off, hdr.length);
                break;
            case IWR_TLV_TARGET_LIST_3D:
                handle_target_list(payload + off, hdr.length);
                break;
            case IWR_TLV_VITAL_SIGNS:
                handle_vital_signs(payload + off, hdr.length);
                break;
            case IWR_TLV_TARGET_INDEX:
            case IWR_TLV_TARGET_HEIGHT:
            case IWR_TLV_PRESENCE:
                // recognised but not logged in bring-up
                break;
            default:
                // Silent default
                break;
        }
        off += hdr.length;
    }

    if (s_num_targets > 0) {
#ifdef CSV_OUTPUT
        // Dumps raw tabular data to build an ML dataset:
        // [TID, posX, posY, posZ, velX, velY, velZ, conf, HR, BR, BR_dev, (Label if gates on)]
        for (uint32_t i = 0; i < s_num_targets; i++) {
            const iwr_target_t *t = &s_last_targets[i];
            const iwr_vital_signs_t *v = NULL;
            for (uint32_t j = 0; j < s_num_vitals; j++) {
                if (s_last_vitals[j].id == t->tid) { v = &s_last_vitals[j]; break; }
            }

            int label = 1; // Default to Valid
#if HEURISTIC_GATES == 1
            // Gate 1: Teleportation checking
            for (uint32_t pt = 0; pt < s_num_prev_targets; pt++) {
                if (s_prev_targets[pt].tid == t->tid) {
                    float dx = t->posX - s_prev_targets[pt].posX;
                    float dy = t->posY - s_prev_targets[pt].posY;
                    float dz = t->posZ - s_prev_targets[pt].posZ;
                    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
                    if (dist > 0.5f) label = 0; // Hijacked bounding box
                    break;
                }
            }
            // Gate 2: Biological Variance
            if (v) {
                if (v->breathDeviation > 1.0f) label = 0; // Extremely noisy breathing phase
                for (uint32_t pv = 0; pv < s_num_prev_vitals; pv++) {
                    if (s_prev_vitals[pv].id == t->tid) {
                        float dhr = fabsf(v->heartRate - s_prev_vitals[pv].heartRate);
                        if (dhr > 15.0f && s_prev_vitals[pv].heartRate > 0.01f) label = 0; // Impossible HR jump
                        break;
                    }
                }
            }
            // Gate 3: Point Cloud Centroid Gating (Ultimate Filter)
            if (label != 0) {
                int points_in_radius = 0;
                for (uint32_t p_idx = 0; p_idx < s_num_points; p_idx++) {
                    float range = s_last_points[p_idx].range * s_pc_unit.rangeUnit;
                    float az = s_last_points[p_idx].azimuth * s_pc_unit.azUnit;
                    float el = s_last_points[p_idx].elevation * s_pc_unit.elevUnit;
                    
                    float py_pc = range * cosf(el) * cosf(az);
                    float px_pc = range * cosf(el) * sinf(az);
                    float pz_pc = range * sinf(el);

                    float dx = t->posX - px_pc;
                    float dy = t->posY - py_pc;
                    float dz = t->posZ - pz_pc;
                    if (sqrtf(dx*dx + dy*dy + dz*dz) < 0.4f) {
                        points_in_radius++;
                    }
                }
                if (points_in_radius < 3) label = 0; // Ghost: No actual matter inside tracker bounds
            }
            printf("CSV_TRACK,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%d\n",
                   (unsigned)t->tid, t->posX, t->posY, t->posZ, t->velX, t->velY, t->velZ, t->confidenceLevel,
                   v ? v->heartRate : 0.0, v ? v->breathRate : 0.0, v ? v->breathDeviation : 0.0, label);
#elif HEURISTIC_GATES == 2
            if (s_interpreter && s_input && s_output) {
                // Apply the exact StandardScaler that was used in train_ai_model.py
                float raw_features[10] = {
                    t->posX, t->posY, t->posZ, 
                    t->velX, t->velY, t->velZ, 
                    t->confidenceLevel, 
                    (v ? v->heartRate : 0.0f), 
                    (v ? v->breathRate : 0.0f), 
                    (v ? v->breathDeviation : 0.0f)
                };

                for (int f = 0; f < 10; f++) {
                    s_input->data.f[f] = (raw_features[f] - feature_means[f]) / feature_scales[f];
                }

                TfLiteStatus invoke_status = s_interpreter->Invoke();
                if (invoke_status == kTfLiteOk) {
                    float max_val = s_output->data.f[0];
                    label = 0;
                    for (int c = 1; c < 3; c++) {
                        if (s_output->data.f[c] > max_val) {
                            max_val = s_output->data.f[c];
                            label = c;
                        }
                    }
                } else {
                    ESP_LOGE(TAG, "TFLite Invoke Failed");
                }
            }
            // To make sure CSV doesn't print when NN is used:
            // The prompt says: "The output should then be only a classification, as well as the x,y,z of the classified track. This means the csv toggle should probably be only in the branches where the nn is not used."
            #if CSV_OUTPUT == 1
            printf("CSV_TRACK,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%d\n",
                   (unsigned)t->tid, t->posX, t->posY, t->posZ, t->velX, t->velY, t->velZ, t->confidenceLevel,
                   v ? v->heartRate : 0.0, v ? v->breathRate : 0.0, v ? v->breathDeviation : 0.0, label);
            #endif
#else
            printf("CSV_TRACK,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f\n",
                   (unsigned)t->tid, t->posX, t->posY, t->posZ, t->velX, t->velY, t->velZ, t->confidenceLevel,
                   v ? v->heartRate : 0.0, v ? v->breathRate : 0.0, v ? v->breathDeviation : 0.0);
#endif
        }
#else
#if HEURISTIC_GATES == 2
        printf("\n\n[INF] ~ %u Tracks Detected ~\n", (unsigned)s_num_targets);
#else
        printf("\n\n[FRAME SYNTHESIS] ~ %u Tracks Detected ~\n", (unsigned)s_num_targets);
#endif
        ble_link_point_t* ble_out = (ble_link_point_t*) malloc(s_num_targets * sizeof(ble_link_point_t));
        uint16_t valid_ble_pts = 0;

        for (uint32_t i = 0; i < s_num_targets; i++) {
            const iwr_target_t *t = &s_last_targets[i];
            
            const iwr_vital_signs_t *v = NULL;
            for (uint32_t j = 0; j < s_num_vitals; j++) {
                if (s_last_vitals[j].id == t->tid) {
                    v = &s_last_vitals[j];
                    break;
                }
            }

            int label = 1;
#if HEURISTIC_GATES == 1
            for (uint32_t pt = 0; pt < s_num_prev_targets; pt++) {
                if (s_prev_targets[pt].tid == t->tid) {
                    float dx = t->posX - s_prev_targets[pt].posX;
                    float dy = t->posY - s_prev_targets[pt].posY;
                    float dz = t->posZ - s_prev_targets[pt].posZ;
                    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
                    if (dist > 0.5f) label = 0;
                    break;
                }
            }
            if (v) {
                if (v->breathDeviation > 1.0f) label = 0;
                for (uint32_t pv = 0; pv < s_num_prev_vitals; pv++) {
                    if (s_prev_vitals[pv].id == t->tid) {
                        float dhr = fabsf(v->heartRate - s_prev_vitals[pv].heartRate);
                        if (dhr > 15.0f && s_prev_vitals[pv].heartRate > 0.01f) label = 0;
                        break;
                    }
                }
            }
            if (label != 0) {
                int points_in_radius = 0;
                for (uint32_t p_idx = 0; p_idx < s_num_points; p_idx++) {
                    float range = s_last_points[p_idx].range * s_pc_unit.rangeUnit;
                    float az = s_last_points[p_idx].azimuth * s_pc_unit.azUnit;
                    float el = s_last_points[p_idx].elevation * s_pc_unit.elevUnit;
                    
                    float py_pc = range * cosf(el) * cosf(az);
                    float px_pc = range * cosf(el) * sinf(az);
                    float pz_pc = range * sinf(el);

                    float dx = t->posX - px_pc;
                    float dy = t->posY - py_pc;
                    float dz = t->posZ - pz_pc;
                    if (sqrtf(dx*dx + dy*dy + dz*dz) < 0.4f) {
                        points_in_radius++;
                    }
                }
                if (points_in_radius < 3) label = 0; 
            }
#elif HEURISTIC_GATES == 2
            if (s_interpreter && s_input && s_output) {
                float raw_features[10] = {
                    t->posX, t->posY, t->posZ, 
                    t->velX, t->velY, t->velZ, 
                    t->confidenceLevel, 
                    (v ? v->heartRate : 0.0f), 
                    (v ? v->breathRate : 0.0f), 
                    (v ? v->breathDeviation : 0.0f)
                };

                for (int f = 0; f < 10; f++) {
                    s_input->data.f[f] = (raw_features[f] - feature_means[f]) / feature_scales[f];
                }

                TfLiteStatus invoke_status = s_interpreter->Invoke();
                if (invoke_status == kTfLiteOk) {
                    float max_val = s_output->data.f[0];
                    label = 0;
                    for (int c = 1; c < 3; c++) {
                        if (s_output->data.f[c] > max_val) {
                            max_val = s_output->data.f[c];
                            label = c;
                        }
                    }
                }
            }
#endif

            // Extra constant-velocity/jitter filtering
            if (label != 0) {
                float speed = sqrtf(t->velX * t->velX + t->velY * t->velY + t->velZ * t->velZ);
                if (speed > 4.0f) label = 0; // Way too fast for biological tracking

                for (uint32_t pt = 0; pt < s_num_prev_targets; pt++) {
                    if (s_prev_targets[pt].tid == t->tid) {
                        float dx = t->posX - s_prev_targets[pt].posX;
                        float dy = t->posY - s_prev_targets[pt].posY;
                        float dz = t->posZ - s_prev_targets[pt].posZ;
                        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
                        if (dist > 1.0f) label = 0; // Aggressive teleportation checking
                        break;
                    }
                }
            }

            // Mark tracking status if biological target exists
            if (label > 0) {
                s_last_active_track_tick = xTaskGetTickCount();
            }

            // Publish valid humans (and ghosts for debugging) to BLE
            if (ble_link_is_subscribed() && ble_out) {
                dwm_point_t dp;
                // Standard DWM geometry transform
                dwm_transform_iwr_xyz(t->posX, t->posY, t->posZ, &dp);
                
                if (dp.distance_mm <= 65535.0f) {
                    float bearing = dp.world_bearing_deg;
                    float elev_cdeg_f = dp.world_elevation_deg * 100.0f;
                    if (elev_cdeg_f >  9000.0f) elev_cdeg_f =  9000.0f;
                    if (elev_cdeg_f < -9000.0f) elev_cdeg_f = -9000.0f;

                    ble_out[valid_ble_pts].distance_mm    = (uint16_t)(dp.distance_mm + 0.5f);
                    ble_out[valid_ble_pts].bearing_cdeg   = (uint16_t)(bearing * 100.0f + 0.5f);
                    ble_out[valid_ble_pts].elevation_cdeg = (int16_t)lroundf(elev_cdeg_f);
                    ble_out[valid_ble_pts].class_id       = (uint16_t)label;
                    valid_ble_pts++;
                }
            }

            printf("========================================================================\n");
#if HEURISTIC_GATES == 2
            const char* class_str = "👻 GHOST";
            if (label == 1) class_str = "🚶 ACTIVE";
            if (label == 2) class_str = "💤 UNCONSCIOUS";
            printf("[%s] TRACK ID: %-3u | 📍 POS: (%+5.2f, %+5.2f, %+5.2f)   [X, Y, Z meters]\n",
                   class_str, (unsigned)t->tid, t->posX, t->posY, t->posZ);
#else
            printf("%s TRACK ID: %-3u | 📍 POS: (%+5.2f, %+5.2f, %+5.2f)   [X, Y, Z meters]\n",
                   label == 1 ? "🚶" : "👻", (unsigned)t->tid, t->posX, t->posY, t->posZ);
            printf("                | 🏃 VEL: (%+5.2f, %+5.2f, %+5.2f)  m/s | Conf: %3.0f%%\n",
                   t->velX, t->velY, t->velZ, t->confidenceLevel);
            
            if (v) {
                printf("❤️  VITALS     | HR: %3.1f bpm | BR: %3.1f bpm | BR_Dev: %1.3f | Bin: %u\n", 
                       v->heartRate, v->breathRate, v->breathDeviation, (unsigned)v->rangeBin);
            } else {
                printf("❤️  VITALS     | [Awaiting lock... Target must be stationary]\n");
            }
#endif
        }
        printf("========================================================================\n");
        if (ble_out) {
            if (ble_link_is_subscribed()) {
                float hdg = dwm_get_assembly_world_heading_deg();
                if (hdg < 0.0f)    hdg += 360.0f;
                if (hdg >= 360.0f) hdg -= 360.0f;
                uint16_t hdg_cdeg = (uint16_t)(hdg * 100.0f + 0.5f);
                uint32_t ts_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

                ble_link_publish_frame(frame_num, ts_ms, hdg_cdeg, ble_out, valid_ble_pts);
            }
            free(ble_out);
        }
#endif
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
        uint8_t *payload = (uint8_t*)malloc(remaining);
        if (!payload) {
            ESP_LOGE(TAG, "OOM: could not alloc %u-byte payload", (unsigned)remaining);
            continue;
        }
        if (!stream_read_exact(payload, remaining, pdMS_TO_TICKS(500))) {
            ESP_LOGW(TAG, "payload timeout (wanted %u) — resyncing", (unsigned)remaining);
            free(payload);
            continue;
        }

        // ESP_LOGI(TAG, "── frame %u: nTLV=%u nObj=%u totalLen=%u",
        //         (unsigned)frameNumber, (unsigned)numTLVs,
        //         (unsigned)numDetectedObj, (unsigned)totalPacketLen);

        if (!s_config_done) {
            free(payload);
            continue;
        }
        if (g_pause_iwr_listening) {
            free(payload);
            continue;
        }
        
        xEventGroupSetBits(g_iwr_state_group, IWR_STATE_FRAME_RECV);

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

    uint8_t *buf = (uint8_t*)malloc(n + 1);
    if (!buf) return -2;
    memcpy(buf, cmd, n);
    buf[n] = '\n';
    
    // The ESP-IDF CDC-ACM driver needs an extra moment after freeing the semaphore
    // so the physical PHY can settle and the CP2105 is ready to ingest.
    // vTaskDelay(pdMS_TO_TICKS(0));
    
    esp_err_t err = cdc_acm_host_data_tx_blocking(s_cli_dev, buf, n + 1, 2000); // 2000ms TX timeout instead of 1000ms
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

    bool config_success = false;
    int reset_count = 0;

    while (!config_success && reset_count < 10) {
        reset_count++;
        
        ESP_LOGW(TAG, "==========================================================");
        ESP_LOGW(TAG, "  PLEASE PRESS THE PHYSICAL 'RESET' BUTTON ON THE IWR6843 ");
        ESP_LOGW(TAG, "  IF THE BOARD DOES NOT AUTOMATICALLY START SENDING DATA. ");
        ESP_LOGW(TAG, "==========================================================\n");
        // No longer pulsing DTR/RTS here. The CP2105 is known to lock its bulk endpoints
        // when its control lines are manipulated via ESP-IDF CDC-ACM host. Wait 3s instead.
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // Purge the queue/semaphore of any startup trash
        xSemaphoreTake(s_cli_ack_sem, 0);
        int sync_r = send_cli_command(""); // send newline to sync prompt
        if (sync_r == -3) {
            ESP_LOGE(TAG, "Fatal USB hardware collapse detecting prompt. Rebooting ESP32!");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_LOGI(TAG, "sending config (%u bytes)", (unsigned)CFG_TEXT_LEN);

        const char *p = CFG_TEXT;
        const char *cfg_end = CFG_TEXT + CFG_TEXT_LEN;
        char line[256];
        bool failed_this_cycle = false;
        int timeouts = 0;

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
                    if (r == -4) {
                        timeouts++;
                        if (timeouts >= 3) {
                            ESP_LOGE(TAG, "   (excessive timeouts, hardware looks dead. resetting)");
                            failed_this_cycle = true;
                            break;
                        }
                    } else if (r == -5) {
                        ESP_LOGE(TAG, "   (cmd error %d on %s, ignoring but noting)", r, s);
                    } else if (r == -3) {
                        // ESP_ERR_INVALID_STATE or Bulk OUT Error! The FTDI/CP210x endpoint mathematically collapsed.
                        // We must break the cycle immediately and force a full physical reset.
                        ESP_LOGE(TAG, "   (fatal USB hardware collapse detected. Rebooting ESP32!)");
                        vTaskDelay(pdMS_TO_TICKS(100));
                        esp_restart();
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            p = eol;
            while (p < cfg_end && (*p == '\n' || *p == '\r')) p++;
        }

        if (!failed_this_cycle) {
            config_success = true;
        }
    }

    if (config_success) {
        ESP_LOGI(TAG, "config complete — waiting for data frames");
        s_config_done = true;
        xEventGroupSetBits(g_iwr_state_group, IWR_STATE_CONFIG_DONE);
    } else {
        ESP_LOGE(TAG, "config permanently failed after multiple reboot attempts!");
    }
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
        .out_buffer_size       = 4096,
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
#if HEURISTIC_GATES == 2
    init_tflite_model();
#endif

    _Static_assert(sizeof(iwr_frame_header_t) == 40,  "frame header must be 40B");
    _Static_assert(sizeof(iwr_tlv_header_t)   == 8,   "tlv header must be 8B");
    _Static_assert(sizeof(iwr_point_unit_t)   == 20,  "pUnit must be 20B");
    _Static_assert(sizeof(iwr_compressed_point_t) == 8, "point must be 8B");
    _Static_assert(sizeof(iwr_target_t)       == 112, "target must be 112B");
    _Static_assert(sizeof(iwr_vital_signs_t)  == 136, "vitals must be 136B");

    s_data_stream  = xStreamBufferCreate(STREAM_BUF_SIZE, 1);
    s_cli_ack_sem  = xSemaphoreCreateBinary();
    g_iwr_state_group = xEventGroupCreate();

    xTaskCreate(parser_task,         "iwr_parser",   8192, NULL, 6, NULL);
    xTaskCreate(open_cli_port_task,  "iwr_cli_open", 4096, NULL, 5, NULL);
    xTaskCreate(open_data_port_task, "iwr_data_open",4096, NULL, 5, NULL);
    xTaskCreate(send_config_task,    "iwr_cfg",      4096, NULL, 8, NULL);
}
void iwr6843_pause_listening(bool pause) { g_pause_iwr_listening = pause; }