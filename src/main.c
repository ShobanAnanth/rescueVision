#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_manager.h"
#include "iwr6843.h"

void app_main(void) {
    usb_manager_init();
    iwr6843_init();

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    
}
