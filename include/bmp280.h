#pragma once
#include "esp_err.h"

// I2C pin ve adres ayarları
#define BMP280_I2C_ADDR 0x76

typedef struct {
    float temperature;
    float pressure;
    float altitude;
} bmp280_data_t;

esp_err_t bmp280_init(void);
esp_err_t bmp280_init_pins(gpio_num_t sda_io, gpio_num_t scl_io);
esp_err_t bmp280_read(bmp280_data_t *data);
