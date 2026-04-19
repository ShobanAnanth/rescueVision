#include "stepper.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

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

static SemaphoreHandle_t turn_sem;

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

static void run_quarter_turn(int *step_idx, int direction)
{
    const float deg_per_step = 360.0f / STEPS_PER_REV;
    for (int i = 0; i < STEPS_PER_90; i++) {
        apply_step(*step_idx);
        *step_idx = (*step_idx + 4 + direction) % 4;
        angle += direction * deg_per_step;
        if (angle >= 360.0f) angle -= 360.0f;
        else if (angle < 0.0f) angle += 360.0f;
        vTaskDelay(pdMS_TO_TICKS(STEP_PERIOD_MS));
    }
}

static void dummy_timer_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        xSemaphoreGive(turn_sem);
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

    turn_sem = xSemaphoreCreateBinary();
    xTaskCreate(dummy_timer_task, "timer", 2048, NULL, 5, NULL);
}

void stepperTask()
{
    static const int turns[8] = { CW, CW, CW, CW, CCW, CCW, CCW, CCW };
    int step_idx = 0;

    while (1) {
        for (int i = 0; i < 8; i++) {
            printf("%s 90° (angle=%.1f)\n", turns[i] == CW ? "CW" : "CCW", angle);
            run_quarter_turn(&step_idx, turns[i]);
            xSemaphoreTake(turn_sem, portMAX_DELAY);
        }
    }
}
