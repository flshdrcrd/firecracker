#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#define MPU6050_ADDR 0x68

typedef struct __attribute__((packed)) {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_data_t;

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle);
esp_err_t mpu6050_read(i2c_master_dev_handle_t dev_handle, mpu6050_data_t *data);
#endif // MPU6050_H