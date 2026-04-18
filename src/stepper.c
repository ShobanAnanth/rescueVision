#include "stepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

void initStepper() {

    // Define the DIR pin configuration
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DIR_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Set initial direction
    gpio_set_level(DIR_PIN, 1);

    // Setup LEDC Timer for the STEP pin
    // We use a hardware timer to achieve smooth stepping without blocking the CPU.
    // "Half power" is interpreted as continuous stepping at a moderate frequency
    // with a 50% duty cycle pulse train.
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 500,  // 500 steps/sec
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Setup LEDC Channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = STEP_PIN,
        .duty           = 0, // Starts off (0% duty cycle)
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void stepperTask() {
    while (1) {
        printf("Motor running for 5 seconds...\n");
        // Start continuous steps: 50% duty cycle (512 out of 1024)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        
        // Wait 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));

        printf("Motor stopped for 5 seconds...\n");
        // Stop stepping: 0% duty cycle
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        
        // Wait 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Optionally swap direction here for a nice demo effect
        // gpio_set_level(DIR_PIN, !gpio_get_level(DIR_PIN));
    }
}