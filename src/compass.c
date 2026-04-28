#include "compass.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define COMPASS_DRDY_GPIO GPIO_NUM_4
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
// 100 kHz is a lot more forgiving with the ESP32's internal ~45k pull-ups.
// Bump back to 400k once you've confirmed external 4.7k pull-ups are present.
#define I2C_FREQ_HZ 100000

// ---------------- QMC5883L (addr 0x0D) ----------------
#define QMC5883L_ADDR 0x0D
#define QMC5883L_DATA 0x00   // X_L, X_H, Y_L, Y_H, Z_L, Z_H
#define QMC5883L_STATUS 0x06 // bit0 DRDY, bit1 OVL, bit2 DOR
#define QMC5883L_CTRL1 0x09
#define QMC5883L_CTRL2 0x0A
#define QMC5883L_SET_RST 0x0B
#define QMC5883L_CHIP_ID_REG 0x0D
#define QMC5883L_CHIP_ID_VAL 0xFF
// OSR=512 | RNG=2G | ODR=200Hz | MODE=Continuous
// 2G range => 12000 LSB/Gauss = 120 LSB/µT (far more sensitive for a ~50 µT
// field).
#define QMC5883L_CTRL1_VAL 0x0D
#define QMC5883L_LSB_PER_UT 120.0f

// ---------------- QMC5883P (addr 0x2C) — new QST replacement part
// ----------------
#define QMC5883P_ADDR 0x2C
#define QMC5883P_CHIP_ID_REG 0x00
#define QMC5883P_CHIP_ID_VAL 0x80
#define QMC5883P_DATA 0x01 // X_L, X_H, Y_L, Y_H, Z_L, Z_H
#define QMC5883P_STATUS 0x09
#define QMC5883P_CTRL1 0x0A
#define QMC5883P_CTRL2 0x0B
// CTRL1: mode=Normal(11) | ODR=200Hz(11) | OSR1=8(01) | OSR2=8(01) => 0x5F
#define QMC5883P_CTRL1_VAL 0x5F
// CTRL2: SET/RESET ON, range = 2 Gauss (bits 3:2 = 11)
#define QMC5883P_CTRL2_VAL 0x0C
// 2G range => 15000 LSB/G = 150 LSB/µT (more sensitive, matches L chip intent)
#define QMC5883P_LSB_PER_UT 150.0f

typedef enum { CHIP_NONE, CHIP_QMC5883L, CHIP_QMC5883P } chip_kind_t;

static const char *TAG = "compass";
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static chip_kind_t s_chip = CHIP_NONE;
static uint8_t s_data_reg = 0;
static uint8_t s_status_reg = 0;
static float s_lsb_per_ut = 1.0f;
static TaskHandle_t s_compass_task_handle = NULL;

static void IRAM_ATTR drdy_isr_handler(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(s_compass_task_handle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// NaN until the chip produces its first reading — lets dwm_geom's delayed
// calibration distinguish "no sample yet" from "actually pointing at 0°".
static volatile float s_heading_deg = NAN;

float compass_get_heading_deg(void) { return s_heading_deg; }

static esp_err_t reg_write(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *dst, size_t n) {
  return i2c_master_transmit_receive(s_dev, &reg, 1, dst, n, 100);
}

static void bus_scan(void) {
  ESP_LOGI(TAG, "I2C scan on SDA=GPIO%d SCL=GPIO%d:", I2C_SDA_GPIO,
           I2C_SCL_GPIO);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (i2c_master_probe(s_bus, a, 20) == ESP_OK) {
      ESP_LOGI(TAG, "  device ACKed at 0x%02X", a);
      found++;
    }
  }
  if (!found)
    ESP_LOGE(TAG, "  no devices found — check wiring, power (3V3), pull-ups");
}

static chip_kind_t identify_chip(void) {
  uint8_t id = 0;
  i2c_device_config_t cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .scl_speed_hz = I2C_FREQ_HZ,
  };

  // Try QMC5883L first.
  cfg.device_address = QMC5883L_ADDR;
  if (i2c_master_bus_add_device(s_bus, &cfg, &s_dev) == ESP_OK) {
    if (reg_read(QMC5883L_CHIP_ID_REG, &id, 1) == ESP_OK) {
      ESP_LOGI(TAG, "probe 0x%02X reg 0x0D = 0x%02X (expect 0xFF for QMC5883L)",
               QMC5883L_ADDR, id);
      if (id == QMC5883L_CHIP_ID_VAL)
        return CHIP_QMC5883L;
    }
    i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
  }

  // Fall back to QMC5883P.
  cfg.device_address = QMC5883P_ADDR;
  if (i2c_master_bus_add_device(s_bus, &cfg, &s_dev) == ESP_OK) {
    if (reg_read(QMC5883P_CHIP_ID_REG, &id, 1) == ESP_OK) {
      ESP_LOGI(TAG, "probe 0x%02X reg 0x00 = 0x%02X (expect 0x80 for QMC5883P)",
               QMC5883P_ADDR, id);
      if (id == QMC5883P_CHIP_ID_VAL)
        return CHIP_QMC5883P;
    }
    i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
  }

  return CHIP_NONE;
}

static esp_err_t configure_chip(void) {
  switch (s_chip) {
  case CHIP_QMC5883L:
    // Datasheet §7.2.2: soft-reset, SET/RESET period = 0x01, then continuous
    // mode.
    reg_write(QMC5883L_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    reg_write(QMC5883L_CTRL2, 0x00);
    reg_write(QMC5883L_SET_RST, 0x01);
    reg_write(QMC5883L_CTRL1, QMC5883L_CTRL1_VAL);
    s_data_reg = QMC5883L_DATA;
    s_status_reg = QMC5883L_STATUS;
    s_lsb_per_ut = QMC5883L_LSB_PER_UT;
    return ESP_OK;
  case CHIP_QMC5883P:
    reg_write(QMC5883P_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10)); // soft reset
    reg_write(QMC5883P_CTRL2, QMC5883P_CTRL2_VAL);
    reg_write(QMC5883P_CTRL1, QMC5883P_CTRL1_VAL);
    s_data_reg = QMC5883P_DATA;
    s_status_reg = QMC5883P_STATUS;
    s_lsb_per_ut = QMC5883P_LSB_PER_UT;
    return ESP_OK;
  default:
    return ESP_FAIL;
  }
}

static void compass_task(void *arg) {
  uint32_t good = 0, stale = 0;
  TickType_t last_report = xTaskGetTickCount();

  for (;;) {
    // Wait for the ISR to signal that new data is ready.
    // We use a timeout (10ms) instead of portMAX_DELAY. If DRDY was ALREADY high 
    // before the ISR was registered, or if we miss an edge, we will time out, 
    // manually read the data (which clears DRDY to low), and subsequent edges will work.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

    uint8_t status = 0;
    if (reg_read(s_status_reg, &status, 1) != ESP_OK) {
      ESP_LOGW(TAG, "status read failed");
      continue;
    }
    if (!(status & 0x01)) {
      stale++;
      continue;
    }

    uint8_t d[6];
    if (reg_read(s_data_reg, d, 6) != ESP_OK) {
      ESP_LOGW(TAG, "data read failed");
      continue;
    }

    int16_t x = (int16_t)((d[1] << 8) | d[0]);
    int16_t y = (int16_t)((d[3] << 8) | d[2]);
    int16_t z = (int16_t)((d[5] << 8) | d[4]);

    float x_uT = x / s_lsb_per_ut;
    float y_uT = y / s_lsb_per_ut;
    float z_uT = z / s_lsb_per_ut;
    float mag = sqrtf(x_uT * x_uT + y_uT * y_uT + z_uT * z_uT);

    // --- Hard-Iron Calibration (Running Min/Max) ---
    // If your heading only wiggles a few degrees (e.g. 200-230), it means the Earth's 
    // magnetic vector is riding on top of a massive static offset (from a nearby 
    // magnet, like your stepper motor). This traces a circle that does not enclose 0,0.
    static int16_t x_min = 32767, x_max = -32768;
    static int16_t y_min = 32767, y_max = -32768;

    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (y < y_min) y_min = y;
    if (y > y_max) y_max = y;

    // Calculate the center of the circle (the static magnetic offset)
    float x_offset = (x_max + x_min) / 2.0f;
    float y_offset = (y_max + y_min) / 2.0f;

    // Subtract the offset to center the circle at 0,0
    float x_cal = (float)x - x_offset;
    float y_cal = (float)y - y_offset;

    // Cardinal heading using CALIBRATED values
    float heading = atan2f(x_cal, y_cal) * 180.0f / (float)M_PI;
    if (heading < 0.0f)
      heading += 360.0f;
    s_heading_deg = heading;

    // Print raw vs calibrated data so the user can literally see the offset!
    if (good % 20 == 0) {
      ESP_LOGI(TAG, "Raw: X=%6d Y=%6d | Off: X=%6.0f Y=%6.0f | Hdg: %.1f", 
               x, y, x_offset, y_offset, heading);
    }
    
    good++;

    // if ((xTaskGetTickCount() - last_report) > pdMS_TO_TICKS(5000)) {
    //   ESP_LOGI(
    //       TAG,
    //       "stats: %u good, %u stale polls (|B| should be 25–65 µT on Earth)",
    //       good, stale);
    //   good = stale = 0;
    //   last_report = xTaskGetTickCount();
    // }
  }
}

void compass_init(void) {
  i2c_master_bus_config_t bus_cfg = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .sda_io_num = I2C_SDA_GPIO,
      .scl_io_num = I2C_SCL_GPIO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

  bus_scan();

  s_chip = identify_chip();
  if (s_chip == CHIP_NONE) {
    ESP_LOGE(TAG, "no supported magnetometer found (QMC5883L @ 0x0D or "
                  "QMC5883P @ 0x2C).");
    ESP_LOGE(TAG,
             "check: module powered from 3V3, SDA=GPIO%d, SCL=GPIO%d, external "
             "4.7k pull-ups.",
             I2C_SDA_GPIO, I2C_SCL_GPIO);
    return;
  }
  ESP_LOGI(TAG, "detected %s",
           s_chip == CHIP_QMC5883L ? "QMC5883L" : "QMC5883P");

  if (configure_chip() != ESP_OK) {
    ESP_LOGE(TAG, "chip configure failed");
    return;
  }

  // Configure DRDY GPIO
  gpio_config_t io_conf = {.intr_type = GPIO_INTR_POSEDGE,
                           .pin_bit_mask = (1ULL << COMPASS_DRDY_GPIO),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = 0,
                           .pull_down_en = 1};
  gpio_config(&io_conf);

  // 1. Create the task first so s_compass_task_handle is valid!
  xTaskCreate(compass_task, "compass", 4096, NULL, 3, &s_compass_task_handle);

  // 2. Install ISR service and add handler AFTER the task is created
  esp_err_t isr_err = gpio_install_isr_service(0);
  if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "gpio_install_isr_service returned %d", isr_err);
  }
  gpio_isr_handler_add(COMPASS_DRDY_GPIO, drdy_isr_handler, NULL);

  ESP_LOGI(TAG, "running @ SDA=GPIO%d SCL=GPIO%d %d Hz, DRDY=GPIO%d",
           I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ, COMPASS_DRDY_GPIO);
}
