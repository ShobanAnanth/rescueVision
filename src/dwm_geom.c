#include "dwm_geom.h"
#include "compass.h"
#include "stepper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "dwm_geom";

static float s_cos_tilt = 1.0f;
static float s_sin_tilt = 0.0f;

static volatile bool s_calibrated = false;

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

static float wrap_360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

float dwm_get_assembly_world_heading_deg(void) {
    return compass_get_windowed_heading_deg();
}

bool dwm_geom_is_calibrated(void) {
    return s_calibrated;
}

void dwm_transform_iwr_xyz(float x_iwr_m, float y_iwr_m, float z_iwr_m,
                           dwm_point_t *out) {
    // Work in mm to match the offset constants.
    float x_iwr_mm = x_iwr_m * 1000.0f;
    float y_iwr_mm = y_iwr_m * 1000.0f;
    float z_iwr_mm = z_iwr_m * 1000.0f;

    // 1. Rotate by Rx(+TILT) to bring the IWR axes parallel to the DWM axes.
    float rx = x_iwr_mm;
    float ry = s_cos_tilt * y_iwr_mm - s_sin_tilt * z_iwr_mm;
    float rz = s_sin_tilt * y_iwr_mm + s_cos_tilt * z_iwr_mm;

    // 2. Translate by the vector FROM the DWM TO the sensor (adding the offset).
    out->dwm_x_mm = rx + DWM_GEOM_OFFSET_X_MM;
    out->dwm_y_mm = ry + DWM_GEOM_OFFSET_Y_MM;
    out->dwm_z_mm = -rz + DWM_GEOM_OFFSET_Z_MM;

    // 3. Rotate DWM body → world. Heading H is CW from world +Y (north),
    //    world axes are +X = east, +Y = north, +Z = up. The matrix that takes
    //    a body vector into world is:
    //        [  cos H   sin H ]
    //        [ -sin H   cos H ]
    float h_rad = deg2rad(wrap_360(compass_get_windowed_heading_deg() + angle));
    float ch = cosf(h_rad), sh = sinf(h_rad);
    out->world_x_mm =  ch * out->dwm_x_mm + sh * out->dwm_y_mm;
    out->world_y_mm = -sh * out->dwm_x_mm + ch * out->dwm_y_mm;
    out->world_z_mm = out->dwm_z_mm;

    // Distance from DWM (frame-invariant).
    out->distance_mm = sqrtf(out->dwm_x_mm * out->dwm_x_mm +
                             out->dwm_y_mm * out->dwm_y_mm +
                             out->dwm_z_mm * out->dwm_z_mm);

    // Cardinal bearing in world frame: 0=N, 90=E. atan2(east, north).
    out->world_bearing_deg = wrap_360(rad2deg(atan2f(out->world_x_mm, out->world_y_mm)));
    float horiz = sqrtf(out->world_x_mm * out->world_x_mm +
                        out->world_y_mm * out->world_y_mm);
    out->world_elevation_deg = rad2deg(atan2f(out->world_z_mm, horiz));
}

void dwm_transform_iwr_spherical(float range_m, float az_rad, float el_rad,
                                 dwm_point_t *out) {
    float ce = cosf(el_rad);
    dwm_transform_iwr_xyz(range_m * ce * sinf(az_rad),
                          range_m * ce * cosf(az_rad),
                          range_m * sinf(el_rad),
                          out);
}

void dwm_geom_calibrate_zero(void) {
    // Heading is now a live windowed average from the compass — nothing to
    // snapshot. This call simply opens the calibration gate so downstream
    // consumers (stepper, BLE) know the compass has completed its sweep.
    s_calibrated = true;
    ESP_LOGI(TAG, "calibration gate open: assembly heading is now live windowed average");
}

// Waits for the compass's hard-iron calibration to converge (all 8 × 45°
// sectors confirmed), then opens the downstream gate. Blocks indefinitely.
static void dwm_calibrate_task(void *arg) {
    const TickType_t POLL_PERIOD = pdMS_TO_TICKS(100);
    const int LOG_PERIOD_MS = 3000;
    int waited_ms = 0;
    int last_log_ms = -LOG_PERIOD_MS;

    while (!compass_is_calibrated()) {
        if (waited_ms - last_log_ms >= LOG_PERIOD_MS) {
            ESP_LOGW(TAG,
                     "waiting on compass calibration — rotate the device through "
                     "a full 360° sweep to converge hard-iron offsets");
            last_log_ms = waited_ms;
        }
        vTaskDelay(POLL_PERIOD);
        waited_ms += 100;
    }

    dwm_geom_calibrate_zero();
    vTaskDelete(NULL);
}

// Periodic visibility into the heading pipeline so it's debuggable without
// needing the IWR feed running.
static void dwm_geom_log_task(void *arg) {
    for (;;) {
        float raw = compass_get_heading_deg();
        float win = compass_get_windowed_heading_deg();
        if (s_calibrated) {
            ESP_LOGI(TAG,
                     "raw=%6.1f°  windowed=%6.1f°  stepper=%7.2f°  →  assemblyHdg=%6.1f°",
                     raw, win, angle, dwm_get_assembly_world_heading_deg());
        } else {
            ESP_LOGI(TAG,
                     "compass=%s  stepper=%7.2f°  (waiting on compass for calibration)",
                     isnan(raw) ? "  NaN " : "ready", angle);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void dwm_geom_init(void) {
    s_cos_tilt = cosf(deg2rad(DWM_GEOM_TILT_X_DEG));
    s_sin_tilt = sinf(deg2rad(DWM_GEOM_TILT_X_DEG));
    ESP_LOGI(TAG, "IWR→DWM offset (mm): (%.2f, %.2f, %.2f)  tilt_X=%.2f°",
             DWM_GEOM_OFFSET_X_MM, DWM_GEOM_OFFSET_Y_MM,
             DWM_GEOM_OFFSET_Z_MM, DWM_GEOM_TILT_X_DEG);
    xTaskCreate(dwm_calibrate_task, "dwm_geom_cal", 3072, NULL, 3, NULL);
    xTaskCreate(dwm_geom_log_task,  "dwm_geom_log", 3072, NULL, 2, NULL);
}
