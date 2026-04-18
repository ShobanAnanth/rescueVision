#include "uart_bridge.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"cl
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

#define BAUD_RATE       115200
#define IN_BUF_SIZE     4096    // per USB transfer; increase for radar TLV frames
#define LINE_BUF_SIZE   1024    // max assembled line length per device
#define QUEUE_DEPTH     32      // messages buffered between rx_cb and print task

static const char *TAG = "uart_bridge";

typedef struct {
    char    data[LINE_BUF_SIZE];
    size_t  len;
} line_msg_t;

typedef struct {
    char         line_buf[LINE_BUF_SIZE];
    size_t       line_pos;
    QueueHandle_t queue;
    const char   *tag;
} dev_ctx_t;

// Called from CDC driver task — must not block
static bool rx_cb(const uint8_t *data, size_t len, void *arg) {
    dev_ctx_t *ctx = arg;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            // Strip trailing \r
            if (ctx->line_pos > 0 && ctx->line_buf[ctx->line_pos - 1] == '\r') {
                ctx->line_pos--;
            }
            if (ctx->line_pos > 0) {
                line_msg_t *msg = malloc(sizeof(line_msg_t));
                if (msg) {
                    memcpy(msg->data, ctx->line_buf, ctx->line_pos);
                    msg->data[ctx->line_pos] = '\0';
                    msg->len = ctx->line_pos;
                    if (xQueueSend(ctx->queue, &msg, 0) != pdTRUE) {
                        free(msg);  // drop if queue full rather than block
                    }
                }
            }
            ctx->line_pos = 0;
        } else if (ctx->line_pos < LINE_BUF_SIZE - 1) {
            ctx->line_buf[ctx->line_pos++] = c;
        }
        // silently drop if line overflows — prevents runaway binary data from crashing
    }
    return true;
}

static void dev_event_cb(const cdc_acm_host_dev_event_data_t *event, void *user_ctx) {
    if (event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "device disconnected");
        cdc_acm_host_close(event->data.cdc_hdl);
    }
}

static void print_task(void *arg) {
    dev_ctx_t *ctx = arg;
    line_msg_t *msg;
    while (1) {
        if (xQueueReceive(ctx->queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(ctx->tag, "%s", msg->data);
            free(msg);
        }
    }
}

static void usb_host_task(void *arg) {
    while (1) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void cdc_open_task(void *arg) {
    dev_ctx_t *ctx = arg;
    cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 10000,
        .out_buffer_size       = 0,
        .in_buffer_size        = IN_BUF_SIZE,
        .event_cb              = dev_event_cb,
        .data_cb               = rx_cb,
        .user_arg              = ctx,
    };
    cdc_acm_dev_hdl_t dev;
    ESP_ERROR_CHECK(cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0, &dev_cfg, &dev));
    ESP_LOGI(TAG, "device connected");

    cdc_acm_line_coding_t lc = {
        .dwDTERate   = BAUD_RATE,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits   = 8,
    };
    cdc_acm_host_line_coding_set(dev, &lc);
    cdc_acm_host_set_control_line_state(dev, true, false);
    vTaskDelete(NULL);
}

void uart_bridge_init(void) {
    dev_ctx_t *ctx = calloc(1, sizeof(dev_ctx_t));
    ctx->tag   = "arduino";
    ctx->queue = xQueueCreate(QUEUE_DEPTH, sizeof(line_msg_t *));

    xTaskCreate(print_task, "arduino_print", 4096, ctx, 4, NULL);

    usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_host_task, "usb_host", 4096, NULL, 2, NULL);

    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    xTaskCreate(cdc_open_task, "cdc_open", 4096, ctx, 5, NULL);
}
