#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "compass.h"
#include "stepper.h"
#include "usb_manager.h"
#include "iwr6843.h"

static const char *TAG = "main";

TaskHandle_t poop;
void app_main(void) {
    ESP_LOGI(TAG, "app_main start");
    //usb_manager_init();
    //iwr6843_init();
    compass_init();

    // while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    
    initStepper();
    xTaskCreate(stepperTask, "stepper", 4096, NULL, 3, &poop);
}
