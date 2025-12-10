#include "bmp280.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

#define PYRO_PIN 999
#define LED_PIN 999
#define BUZZER_PIN 999
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SEA_LEVEL_PRESSURE 1013.25

#define LOOP_DELAY_MS 100
#define LOOP_DT_SEC (LOOP_DELAY_MS / 1000.0)

#define ALPHA_ALT 0.1
#define ALPHA_VEL 0.1

#define MIN_ALT_INCREASE 50.0
#define APOGEE_VELOCITY_THRESHOLD -2.0
#define CONSECUTIVE_SAMPLES 5

typedef enum { IDLE, ASCENT, DESCENT } flight_state_t;

typedef struct {
    int64_t timestamp;
    bmp280_data_t bmp_primary;
    bmp280_data_t bmp_secondary;
    mpu6050_data_t mpu;
} sensor_measurement_t;

QueueHandle_t sensor_queue;

float calculate_altitude(float pressure_hPa) {
    return 8425 * log(SEA_LEVEL_PRESSURE / (pressure_hPa / 25600.0));
}

void i2c_init(i2c_master_bus_handle_t *i2c_bus_handle) {
    gpio_config_t io_conf = {.pin_bit_mask =
                                 (1ULL << I2C_SCL_PIN) | (1ULL << I2C_SDA_PIN),
                             .mode = GPIO_MODE_INPUT_OUTPUT_OD,
                             .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);

    i2c_master_bus_config_t i2c_bus_config = {.clk_source = I2C_CLK_SRC_DEFAULT,
                                              .scl_io_num = I2C_SCL_PIN,
                                              .sda_io_num = I2C_SDA_PIN,
                                              .glitch_ignore_cnt = 7,
                                              .flags.enable_internal_pullup =
                                                  true};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle));
}

void app_main(void) {
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_init(&i2c_bus_handle);

    bmp280_handle_t dev_bmp280_primary;
    bmp280_init(i2c_bus_handle, &dev_bmp280_primary, 0x76);

    bmp280_handle_t dev_bmp280_secondary;
    bmp280_init(i2c_bus_handle, &dev_bmp280_secondary, 0x77);

    i2c_master_dev_handle_t dev_mpu6050;
    mpu6050_init(i2c_bus_handle, &dev_mpu6050);

    flight_state_t current_state = IDLE;
    int descent_check_counter = 0;

    sensor_queue = xQueueCreate(20, sizeof(sensor_measurement_t));
    sensor_measurement_t measurement;

    printf("%lld us | Starting calibration\n", esp_timer_get_time());
    float sum_alt = 0;
    for (int i = 0; i < 50; i++) {
        bmp280_read(&dev_bmp280_primary, &measurement.bmp_primary);
        float press_hPa = measurement.bmp_primary.pressure / 25600.0;
        sum_alt += calculate_altitude(press_hPa);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    float ground_altitude = sum_alt / 50.0;
    float filtered_altitude = ground_altitude;
    float prev_filtered_altitude = ground_altitude;
    float filtered_velocity = 0.0;

    printf("%lld us | Ground Altitude set at: %.2f meters\n",
           esp_timer_get_time(), ground_altitude);

    while (1) {
        measurement.timestamp = esp_timer_get_time();
        bmp280_read(&dev_bmp280_primary, &measurement.bmp_primary);
        bmp280_read(&dev_bmp280_secondary, &measurement.bmp_secondary);
        mpu6050_read(dev_mpu6050, &measurement.mpu);

        float temp_c = measurement.bmp_primary.temperature / 100.0;
        float press_hPa = measurement.bmp_primary.pressure / 25600.0;
        float raw_altitude = calculate_altitude(press_hPa);

        filtered_altitude = (ALPHA_ALT * raw_altitude) +
                            ((1.0 - ALPHA_ALT) * filtered_altitude);

        float instant_velocity =
            (filtered_altitude - prev_filtered_altitude) / LOOP_DT_SEC;

        filtered_velocity = (ALPHA_VEL * instant_velocity) +
                            ((1.0 - ALPHA_VEL) * filtered_velocity);

        prev_filtered_altitude = filtered_altitude;

        switch (current_state) {
        case IDLE:
            if (filtered_altitude > (ground_altitude + MIN_ALT_INCREASE)) {
                current_state = ASCENT;
                printf("%lld us | Launch detected\n", esp_timer_get_time());
            }
            break;

        case ASCENT:
            if (filtered_velocity < APOGEE_VELOCITY_THRESHOLD) {
                descent_check_counter++;

                if (descent_check_counter >= CONSECUTIVE_SAMPLES) {
                    current_state = DESCENT;
                    printf("%lld us | Apogee detected (Vel: %.2f m/s, Alt: "
                           "%.2f m)\n",
                           esp_timer_get_time(), filtered_velocity,
                           filtered_altitude);
                    // TODO: Fire Pyro
                }
            } else {
                descent_check_counter = 0;
            }
            break;

        case DESCENT:
            break;
        }

        printf("%lld us | Raw temp: %5.2f C | Raw press: %5.2f hPa\n",
               esp_timer_get_time(), temp_c, press_hPa);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}