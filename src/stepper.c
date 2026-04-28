#include "stepper.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "iwr6843.h"
#include "dwm_geom.h"

float angle = 0.0f;

static portMUX_TYPE step_mux = portMUX_INITIALIZER_UNLOCKED;

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
    taskENTER_CRITICAL(&step_mux);
    gpio_set_level(IN1_PIN, s[0]);
    gpio_set_level(IN2_PIN, s[1]);
    gpio_set_level(IN3_PIN, s[2]);
    gpio_set_level(IN4_PIN, s[3]);
    taskEXIT_CRITICAL(&step_mux);
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
    static const int turns[8] = { CW, CW, CW, CW, CCW, CCW, CCW, CCW };
    int step_idx = 0;

    printf("\n======================================================\n");
    printf("[Stepper] ACTION REQUIRED: Please properly position the\n");
    printf("          stepper mount manually now. The motor is free.\n");
    printf("[Stepper] You have 10 seconds before the motor locks!\n");
    printf("======================================================\n\n");
    
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    // Apply the first sequence step to actively energize the coils through the L298N.
    // This locks the motor and generates holding torque!
    apply_step(step_idx);
    printf("[Stepper] Motor is now LOCKED.\n");

    printf("[Stepper] Waiting for Compass Calibration...\n");
    while (!dwm_geom_is_calibrated()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Block until config is completely finished and we successfully hit a frame
    printf("[Stepper] Waiting for IWR6843 configuration & first valid frame...\n");
    if (g_iwr_state_group != NULL) {
        xEventGroupWaitBits(g_iwr_state_group, 
                            IWR_STATE_CONFIG_DONE | IWR_STATE_FRAME_RECV, 
                            pdFALSE, pdTRUE, portMAX_DELAY);
    }
    
#ifdef CSV_OUTPUT
    printf("[Stepper] ML CSV Recording Mode Active: Locking Stepper Mount to Static Target.\n");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#else
    printf("[Stepper] IWR6843 is actively tracking! Starting sweep pattern.\n");

    while (1) {
        for (int i = 0; i < 8; i++) {
            // Wait while there are actively tracked humans, do not move the stepper
            while (iwr6843_has_active_targets()) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            printf("%s 90° (angle=%.1f)\n", turns[i] == CW ? "CW" : "CCW", angle);
            
            // Pause IWR listening to ignore artifacts/motion from rotating the sensor
            iwr6843_pause_listening(true);
            
            run_quarter_turn(&step_idx, turns[i]);
            
            // Post movement stabilization
            vTaskDelay(pdMS_TO_TICKS(1000));
            iwr6843_pause_listening(false);

            // Fixed dwell: 10s from end of each move, not from a free-running timer
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
#endif
}
