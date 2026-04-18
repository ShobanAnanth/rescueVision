#include "usb_manager.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "usb_manager";

static void usb_host_task(void *arg) {
    while (1) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

// Periodically list all USB devices the host sees. If this reports 0 devices
// forever, the CP2105 isn't enumerating — check cable, power, and SOP switches.
static void usb_probe_task(void *arg) {
    uint8_t addr_list[16];
    int last_count = -1;
    while (1) {
        int n = 0;
        if (usb_host_device_addr_list_fill(sizeof(addr_list), addr_list, &n) == ESP_OK) {
            if (n != last_count) {
                ESP_LOGI(TAG, "USB devices on bus: %d", n);
                last_count = n;
            }
            for (int i = 0; i < n; i++) {
                usb_device_handle_t dev;
                if (usb_host_device_open(0, addr_list[i], &dev) == ESP_OK) {
                    const usb_device_desc_t *desc;
                    if (usb_host_get_device_descriptor(dev, &desc) == ESP_OK) {
                        ESP_LOGI(TAG, "   addr=%u VID=0x%04X PID=0x%04X class=0x%02X",
                                 addr_list[i], desc->idVendor, desc->idProduct,
                                 desc->bDeviceClass);
                    }
                    usb_host_device_close(0, dev);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void usb_manager_init(void) {
    usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_host_task,  "usb_host",  4096, NULL, 2, NULL);

    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    ESP_LOGI(TAG, "USB host + CDC-ACM ready");

    // register_client=0 is fine; addr_list_fill doesn't need a client.
    xTaskCreate(usb_probe_task, "usb_probe", 4096, NULL, 3, NULL);
}
