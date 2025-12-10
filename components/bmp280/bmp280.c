#include "bmp280.h"

static esp_err_t read_calibration(bmp280_handle_t *dev) {
    uint8_t reg = 0x88;
    uint8_t calib_data[24];

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, calib_data, sizeof(calib_data), -1);
    if (ret != ESP_OK) return ret;

    dev->calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dev->calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dev->calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    dev->calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dev->calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dev->calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dev->calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dev->calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dev->calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dev->calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dev->calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dev->calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    return ESP_OK;
}

static int32_t calculate_t_fine(bmp280_handle_t *dev, int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * ((int32_t)dev->calib.dig_T3)) >> 14;
    
    return var1 + var2;
}

static uint32_t compensate_pressure(bmp280_handle_t *dev, int32_t adc_P, int32_t t_fine) {
    int64_t var1, var2, p;
    
    // Use the passed t_fine
    var1 = ((int64_t)t_fine) - 128000;
    
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;
    
    if (var1 == 0) return 0;
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);
    
    return (uint32_t)p;
}

esp_err_t bmp280_init(i2c_master_bus_handle_t bus_handle, bmp280_handle_t *dev, uint8_t address){
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) return ret;

    // Configure sensor: Normal mode, 16x oversampling
    uint8_t config[][2] = {{0xF4, 0x57}, {0xF5, 0x14}};
    for (int i = 0; i < 2; i++) {
        ret = i2c_master_transmit(dev->i2c_dev, config[i], 2, -1);
        if (ret != ESP_OK) return ret;
    }

    return read_calibration(dev);
}

esp_err_t bmp280_read(bmp280_handle_t *dev, bmp280_data_t *data) {
    uint8_t reg = 0xF7;
    uint8_t raw_data[6];

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, raw_data, sizeof(raw_data), -1);
    if (ret != ESP_OK) return ret;

    int32_t adc_P = (raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2];
    int32_t adc_T = (raw_data[3] << 16) | (raw_data[4] << 8) | raw_data[5];

    adc_P >>= 4;
    adc_T >>= 4;


    int32_t t_fine = calculate_t_fine(dev, adc_T);

    data->temperature = (t_fine * 5 + 128) >> 8;
    data->pressure = compensate_pressure(dev, adc_P, t_fine);

    return ESP_OK;
}
