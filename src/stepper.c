#include "stepper.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "dwm_geom.h"
#include "esp_log.h"

static const char *TAG = "stepper";

float angle = 0.0f;

float stepper_get_angle_deg(void) {
    return angle;
}

static const uint8_t step_sequence[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1},
};

static void enable_pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = 20000,   // 20 kHz — above audible range, no whine
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch_a = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = ENA_PIN,
        .duty       = ENABLE_DUTY,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch_a);

    ledc_channel_config_t ch_b = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = ENB_PIN,
        .duty       = ENABLE_DUTY,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch_b);
}

static void apply_step(int idx)
{
    const uint8_t *s = step_sequence[idx & 3];
    gpio_set_level(IN1_PIN, s[0]);
    gpio_set_level(IN2_PIN, s[1]);
    gpio_set_level(IN3_PIN, s[2]);
    gpio_set_level(IN4_PIN, s[3]);
}

// Normalize angle to [0, 360)
static float wrap_360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

// Calculate shortest angular distance between two angles (handles wraparound)
static float angle_delta(float from_deg, float to_deg) {
    float delta = to_deg - from_deg;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

// Turn the stepper to an absolute angle (in degrees, CCW-positive)
static void turn_to_angle(int *step_idx, float target_deg)
{
    const float deg_per_step = 360.0f / STEPS_PER_REV;

    // Calculate shortest path to target
    target_deg = wrap_360(target_deg);
    float current = wrap_360(angle);
    float delta = angle_delta(current, target_deg);

    int direction = (delta > 0) ? CCW : CW;
    int steps_needed = (int)(fabsf(delta) / deg_per_step + 0.5f);

    for (int i = 0; i < steps_needed; i++) {
        apply_step(*step_idx);
        *step_idx = (*step_idx + 4 + direction) % 4;
        angle += direction * deg_per_step;
        if (angle >= 360.0f) angle -= 360.0f;
        else if (angle < 0.0f) angle += 360.0f;
        vTaskDelay(pdMS_TO_TICKS(STEP_PERIOD_MS));
    }
}

void initStepper()
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) |
                        (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    gpio_config(&io_conf);
    enable_pwm_init();
}

void stepperTask()
{
    // 0°=North, 90°=East, 180°=South, 270°=West — forward then backtrack
    static const float world_headings[] = { 0.0f, 90.0f, 180.0f, 270.0f, 360.0f, 270.0f, 180.0f, 90.0f };
    static const int num_targets = 8;
    int step_idx = 0;

    // Block until compass + dwm_geom calibration are both complete
    while (!dwm_geom_is_calibrated()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    while (1) {
        for (int i = 0; i < num_targets; i++) {
            float target = dwm_get_stepper_angle_for_heading(world_headings[i]);
            ESP_LOGI(TAG, "rotating to %.0f° (world heading)", world_headings[i]);
            turn_to_angle(&step_idx, target);
            ESP_LOGI(TAG, "arrived at %.0f° — holding for 3s", world_headings[i]);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}
