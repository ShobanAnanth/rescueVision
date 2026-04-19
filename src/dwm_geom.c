#include "dwm_geom.h"
#include "compass.h"
#include "stepper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "dwm_geom";

static float s_stepper_ref_deg       = 0.0f;
static float s_world_heading_ref_deg = 0.0f;
static float s_cos_tilt = 1.0f;
static float s_sin_tilt = 0.0f;

// Set true once we've captured a real (non-NaN) compass reading. Until then,
// dwm_get_assembly_world_heading_deg() returns just the stepper delta.
static volatile bool s_calibrated = false;

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

static float wrap_360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

float dwm_get_assembly_world_heading_deg(void) {
    // Cardinal heading is CW-positive (0=N, 90=E). Stepper `angle` is
    // CCW-positive. CCW body rotation = lower compass bearing, so subtract.
    float delta_ccw = angle - s_stepper_ref_deg;
    return wrap_360(s_world_heading_ref_deg - delta_ccw);
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
    out->dwm_z_mm = rz + DWM_GEOM_OFFSET_Z_MM;

    // 3. Rotate DWM body → world. Heading H is CW from world +Y (north),
    //    world axes are +X = east, +Y = north, +Z = up. The matrix that takes
    //    a body vector into world is:
    //        [  cos H   sin H ]
    //        [ -sin H   cos H ]
    float h_rad = deg2rad(dwm_get_assembly_world_heading_deg());
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
    float compass = compass_get_heading_deg();
    if (isnan(compass)) {
        ESP_LOGW(TAG, "calibrate skipped: compass has no reading yet");
        return;
    }
    s_stepper_ref_deg       = angle;
    s_world_heading_ref_deg = compass;
    s_calibrated = true;
    ESP_LOGI(TAG, "calibrated zero: stepper_ref=%.2f°  world_heading_ref=%.2f° (cardinal)",
             s_stepper_ref_deg, s_world_heading_ref_deg);
}

// Polls compass at startup until it produces a real reading, then captures
// the world-heading reference. Falls back to whatever the compass returned
// after MAX_WAIT_MS even if we hit timeout.
static void dwm_calibrate_task(void *arg) {
    const TickType_t POLL_PERIOD = pdMS_TO_TICKS(100);
    const int MAX_WAIT_MS = 5000;
    int waited_ms = 0;

    while (waited_ms < MAX_WAIT_MS) {
        if (!isnan(compass_get_heading_deg())) break;
        vTaskDelay(POLL_PERIOD);
        waited_ms += 100;
    }

    if (isnan(compass_get_heading_deg())) {
        ESP_LOGW(TAG, "compass never reported a heading after %dms — "
                      "world reference left at 0°; rotations will be relative only",
                 MAX_WAIT_MS);
        s_stepper_ref_deg       = angle;
        s_world_heading_ref_deg = 0.0f;
        s_calibrated = true;
    } else {
        // Give the chip a couple more samples for the value to stabilise.
        vTaskDelay(pdMS_TO_TICKS(300));
        dwm_geom_calibrate_zero();
    }
    vTaskDelete(NULL);
}

// Periodic visibility into the heading pipeline so it's debuggable without
// needing the IWR feed running.
static void dwm_geom_log_task(void *arg) {
    for (;;) {
        float compass = compass_get_heading_deg();
        float stepper = angle;
        float delta   = stepper - s_stepper_ref_deg;
        if (s_calibrated) {
            ESP_LOGI(TAG,
                     "compass=%6.1f°  stepper=%7.2f° (Δ%+7.2f°)  →  assemblyHdg=%6.1f° (cardinal)",
                     compass, stepper, delta,
                     dwm_get_assembly_world_heading_deg());
        } else {
            ESP_LOGI(TAG,
                     "compass=%s  stepper=%7.2f°  (waiting on compass for calibration)",
                     isnan(compass) ? "  NaN " : "ready", stepper);
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
