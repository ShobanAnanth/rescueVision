#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#define LED_GPIO  48  // v1.0 = 48, v1.1 = 38

void app_main(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds       = 1,
        .led_model      = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    led_strip_handle_t strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));

    while (1) {
        led_strip_set_pixel(strip, 0, 32, 0, 0);
        led_strip_refresh(strip);
        vTaskDelay(pdMS_TO_TICKS(500));

        led_strip_clear(strip);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
