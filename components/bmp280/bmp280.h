#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

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
} bmp280_calib_data_t;

typedef struct __attribute__((packed)) {
    int32_t temperature;
    uint32_t pressure;
} bmp280_data_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    bmp280_calib_data_t calib; 
} bmp280_handle_t;

esp_err_t bmp280_init(i2c_master_bus_handle_t bus_handle, bmp280_handle_t *dev, uint8_t address);
esp_err_t bmp280_read(bmp280_handle_t *dev, bmp280_data_t *data);

#endif // BMP280_H
