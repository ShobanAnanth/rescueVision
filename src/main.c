#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepper.h"

#define LED_GPIO  38  // v1.0 = 48, v1.1 = 38

TaskHandle_t *poop;

void app_main(void) {
    initStepper();
    xTaskCreate(stepperTask, "stepper", 1024, NULL, 5, poop);
}
