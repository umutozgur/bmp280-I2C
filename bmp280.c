#include "bmp280.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"  // gpio_num_t tipini kullanabilmek için

#define TAG "BMP280_COMPONENT"

// I2C ayarları / deneme
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

// Register adresleri
#define REG_ID         0xD0
#define REG_CALIB      0x88
#define REG_CTRL_MEAS  0xF4
#define REG_PRESS_MSB  0xF7
#define REG_TEMP_MSB   0xFA

#define SEA_LEVEL_PRESSURE 1013.25f // hPa

// Kalibrasyon değerleri
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

static bmp280_calib_t calib;
static int32_t t_fine;

/**
 * @brief Write a single byte to BMP280 register
 * @param reg: Register address
 * @param val: Value to write
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t i2c_write_byte(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_I2C_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Write a single byte to BMP280 register
 * @param reg: Register address
 * @param val: Value to write
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDR, &reg, 1, buf, len, 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Read BMP280 calibration data from the sensor
 */
static void read_calibration(void) {
    uint8_t buf[24];
    i2c_read_bytes(REG_CALIB, buf, 24);

    calib.dig_T1 = (buf[1]<<8)|buf[0];
    calib.dig_T2 = (buf[3]<<8)|buf[2];
    calib.dig_T3 = (buf[5]<<8)|buf[4];
    calib.dig_P1 = (buf[7]<<8)|buf[6];
    calib.dig_P2 = (buf[9]<<8)|buf[8];
    calib.dig_P3 = (buf[11]<<8)|buf[10];
    calib.dig_P4 = (buf[13]<<8)|buf[12];
    calib.dig_P5 = (buf[15]<<8)|buf[14];
    calib.dig_P6 = (buf[17]<<8)|buf[16];
    calib.dig_P7 = (buf[19]<<8)|buf[18];
    calib.dig_P8 = (buf[21]<<8)|buf[20];
    calib.dig_P9 = (buf[23]<<8)|buf[22];
}

/**
 * @brief Read BMP280 calibration data from the sensor
 */
static float compensate_temp(int32_t adc_T) {
    int32_t var1 = ((((adc_T>>3) - ((int32_t)calib.dig_T1 <<1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T>>4) - ((int32_t)calib.dig_T1)) * ((adc_T>>4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

/**
 * @brief Compensate raw pressure reading from BMP280
 * @param adc_P: Raw pressure value from sensor
 * @return Pressure in hPa
 */
static float compensate_press(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3)>>8) + ((var1 * (int64_t)calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib.dig_P1)>>33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125)/var1;
    var1 = (((int64_t)calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7)<<4);
    return p / 25600.0f;
}

/**
 * @brief Calculate altitude from pressure using the barometric formula
 * @param pressure_hpa: Pressure in hPa
 * @return Altitude in meters
 */
static float calc_altitude(float pressure_hpa) {
    return 44330.0f * (1.0f - powf(pressure_hpa / SEA_LEVEL_PRESSURE, 0.1903f));
}


/**
 * @brief Initialize BMP280 with default I2C pins (SDA=21, SCL=22)
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t bmp280_init(void) {
    return bmp280_init_pins(GPIO_NUM_21, GPIO_NUM_22); // default SDA/SCL
}

/**
 * @brief Initialize BMP280 with custom I2C pins
 * @param sda_io: GPIO number for SDA
 * @param scl_io: GPIO number for SCL
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t bmp280_init_pins(gpio_num_t sda_io, gpio_num_t scl_io) {
    // I2C master init
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // Chip ID kontrol
    uint8_t id;
    if (i2c_read_bytes(REG_ID, &id, 1) != ESP_OK) return ESP_FAIL;
    if (id != 0x58 && id != 0x60) return ESP_FAIL;

    read_calibration();

    // Normal mode
    uint8_t ctrl_meas = (1<<5)|(1<<2)|3;
    return i2c_write_byte(REG_CTRL_MEAS, ctrl_meas);
}

/**
 * @brief Read temperature, pressure and calculate altitude
 * @param data: Pointer to bmp280_data_t struct to store results
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bmp280_read(bmp280_data_t *data) {
    uint8_t buf[6];
    if (i2c_read_bytes(REG_PRESS_MSB, buf, 6) != ESP_OK) return ESP_FAIL;

    int32_t adc_P = (buf[0]<<12)|(buf[1]<<4)|(buf[2]>>4);
    int32_t adc_T = (buf[3]<<12)|(buf[4]<<4)|(buf[5]>>4);

    data->temperature = compensate_temp(adc_T);
    data->pressure = compensate_press(adc_P);
    data->altitude = calc_altitude(data->pressure);
    return ESP_OK;
}
