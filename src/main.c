#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_bridge.h"

void app_main(void) {
    uart_bridge_init();
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
