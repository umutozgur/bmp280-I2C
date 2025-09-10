# BMP280 Component for ESP-IDF

This is a simple BMP280 I2C component for ESP-IDF projects.  
It supports both default and custom SDA/SCL pins.

---

# Türkçe Açıklama

Bu, ESP-IDF projeleri için basit bir BMP280 I2C component’idir.  
Hem varsayılan hem de özel SDA/SCL pinlerini destekler.

---

## Usage / Kullanım

### 1. Default pins / Varsayılan pinler
```c
#include "bmp280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void app_main(void) {
    if (bmp280_init() != ESP_OK) {
        ESP_LOGE("MAIN", "BMP280 initialization failed!");
        return;
    }

    bmp280_data_t data;
    while (1) {
        if (bmp280_read(&data) == ESP_OK) {
            ESP_LOGI("MAIN", "Temp: %.2f °C, Pressure: %.2f hPa, Altitude: %.2f m",
                     data.temperature, data.pressure, data.altitude);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
