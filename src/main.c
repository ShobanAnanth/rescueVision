#include "ble_link.h"
#include "compass.h"
#include "dwm_geom.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iwr6843.h"
#include "stepper.h"
#include "usb_manager.h"

static const char *TAG = "main";

TaskHandle_t poop;

void realmain() {
  compass_init();
  ESP_LOGI(TAG, "app_main start");
  usb_manager_init();
  iwr6843_init();

  // while (1) vTaskDelay(pdMS_TO_TICKS(1000));

  initStepper();
  xTaskCreate(stepperTask, "stepper", 4096, NULL, 10, &poop);

  dwm_geom_init();
  ble_link_init();
}

void compassDemo() {
  compass_init();

  while (1) {
    float heading = compass_get_heading_deg();
    ESP_LOGI(TAG, "Demo Heading: %.1f deg", heading);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void app_main(void) { realmain(); }
