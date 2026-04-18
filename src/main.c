#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_manager.h"
#include "iwr6843.h"
#include "stepper.h"

#define LED_GPIO  38  // v1.0 = 48, v1.1 = 38

TaskHandle_t *poop;

void app_main(void) {
    usb_manager_init();
    iwr6843_init();

    // while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    
    initStepper();
    xTaskCreate(stepperTask, "stepper", 4096, NULL, 3, poop);
}
