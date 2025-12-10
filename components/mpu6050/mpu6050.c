#include "mpu6050.h"

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) return ret;

    uint8_t init_data[][2] = {{0x6B, 0x00}, {0x1B, 0x18}, {0x1C, 0x18}};

    for (int i = 0; i < 3; i++) {
        ret = i2c_master_transmit(*dev_handle, init_data[i], 2, -1);
        if (ret != ESP_OK) return ret;
    }

    return ESP_OK;
}

esp_err_t mpu6050_read(i2c_master_dev_handle_t dev_handle, mpu6050_data_t *data) {
    uint8_t reg[1];
    uint8_t raw_data[14];
    reg[0] = 0x3B;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, reg, 1, raw_data, 14, -1);
    if (ret != ESP_OK) return ret;
    
    data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    // raw_data[6-7] is temperature
    data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    
    return ESP_OK;
}