#include "bmp280.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

// Config

// Loop frequency
#define LOOP_FREQ_HZ 20

// Filter settings
#define ALPHA_ALT 0.3
#define ALPHA_VEL 0.3

// Thresholds
#define MIN_ALT_INCREASE 50.0
#define APOGEE_VELOCITY_THRESHOLD -2.0
#define CONSECUTIVE_SAMPLES 5

// Queue sizes
#define MEASUREMENT_QUEUE_SIZE 100
#define EVENT_QUEUE_SIZE 100

#define CALIBRATION_SAMPLES 50
#define PYRO_DURATION_MS 500

// Pin configuration
#define PYRO_PIN 32
#define LED_PIN 2
#define BUZZER_PIN 27

#define I2C_SCL_PIN 16
#define I2C_SDA_PIN 17

#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 22
#define SPI_CLK_PIN 21
#define SPI_CS_PIN 23

#define SEA_LEVEL_PRESSURE 1013.25

typedef enum {
    IDLE,
    ASCENT,
    DESCENT,
} flight_state_t;

typedef enum {
    CALIB_START,
    CALIB_END,
    LAUNCH_DETECTED,
    APOGEE_DETECTED,
    PYRO_ON,
    PYRO_OFF,
} event_type_t;

typedef struct {
    int64_t timestamp;
    bmp280_data_t bmp;
} measurement_t;

typedef struct {
    int64_t timestamp;
    event_type_t event;
    float altitude;
    float velocity;
} event_t;

QueueHandle_t measurement_queue;
QueueHandle_t event_queue;

float calculate_altitude(float pressure_hPa) {
    return 8425 * log(SEA_LEVEL_PRESSURE / pressure_hPa);
}

void i2c_init(i2c_master_bus_handle_t *i2c_bus_handle) {
    i2c_master_bus_config_t i2c_bus_config = {.clk_source = I2C_CLK_SRC_DEFAULT,
                                              .scl_io_num = I2C_SCL_PIN,
                                              .sda_io_num = I2C_SDA_PIN,
                                              .glitch_ignore_cnt = 7,
                                              .flags.enable_internal_pullup =
                                                  true};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle));
}

void gpio_output_init(void) {
    uint64_t pin_mask =
        (1ULL << PYRO_PIN) | (1ULL << LED_PIN) | (1ULL << BUZZER_PIN);

    gpio_config_t io_conf = {.pin_bit_mask = pin_mask,
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_conf);

    gpio_set_level(PYRO_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
}

void data_logger_task(void *pvParameter) {
    measurement_t measurement;
    while (1) {
        if (xQueueReceive(measurement_queue, &measurement, portMAX_DELAY) ==
            pdTRUE) {
            printf("%lld us | Temperature: %.2f C, Pressure: %.2f hPa\n",
                   measurement.timestamp,
                   (float)measurement.bmp.temperature / 100.0,
                   (float)measurement.bmp.pressure / 25600.0);
        }
    }
}

void event_logger_task(void *pvParameter) {
    event_t event;
    while (1) {
        if (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdTRUE) {
            printf(
                "%lld us | Event: %d, Altitude: %.2f m, Velocity: %.2f m/s\n",
                event.timestamp, event.event, event.altitude, event.velocity);
        }
    }
}

void app_main(void) {
    gpio_output_init();

    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_init(&i2c_bus_handle);

    bmp280_handle_t dev_bmp280;
    bmp280_init(i2c_bus_handle, &dev_bmp280, 0x76);

    int64_t pyro_fired_time;

    flight_state_t current_state = IDLE;
    int descent_check_counter = 0;

    measurement_queue =
        xQueueCreate(MEASUREMENT_QUEUE_SIZE, sizeof(measurement_t));
    xTaskCreate(data_logger_task, "data_logger", 2048, NULL, 5, NULL);
    measurement_t measurement;

    event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(event_t));
    xTaskCreate(event_logger_task, "event_logger", 2048, NULL, 5, NULL);
    event_t event;

    event.timestamp = esp_timer_get_time();
    event.event = CALIB_START;
    event.altitude = 0.0;
    event.velocity = 0.0;
    xQueueSend(event_queue, &event, 0);

    float sum_alt = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        bmp280_read(&dev_bmp280, &measurement.bmp);
        float press_hPa = measurement.bmp.pressure / 25600.0;
        sum_alt += calculate_altitude(press_hPa);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    float ground_altitude = sum_alt / CALIBRATION_SAMPLES;
    float filtered_altitude = ground_altitude;
    float prev_filtered_altitude = ground_altitude;
    float filtered_velocity = 0.0;

    event.timestamp = esp_timer_get_time();
    event.event = CALIB_END;
    event.altitude = ground_altitude;
    event.velocity = 0.0;
    xQueueSend(event_queue, &event, 0);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / LOOP_FREQ_HZ));

        measurement.timestamp = esp_timer_get_time();
        bmp280_read(&dev_bmp280, &measurement.bmp);
        xQueueSend(measurement_queue, &measurement, 0);

        float press_hPa = measurement.bmp.pressure / 25600.0;
        float raw_altitude = calculate_altitude(press_hPa);

        filtered_altitude = (ALPHA_ALT * raw_altitude) +
                            ((1.0 - ALPHA_ALT) * filtered_altitude);

        float instant_velocity =
            (filtered_altitude - prev_filtered_altitude) / (1.0 / LOOP_FREQ_HZ);

        filtered_velocity = (ALPHA_VEL * instant_velocity) +
                            ((1.0 - ALPHA_VEL) * filtered_velocity);

        prev_filtered_altitude = filtered_altitude;

        switch (current_state) {
        case IDLE:
            if (filtered_altitude > (ground_altitude + MIN_ALT_INCREASE)) {
                current_state = ASCENT;
                event.timestamp = esp_timer_get_time();
                event.event = LAUNCH_DETECTED;
                event.altitude = filtered_altitude;
                event.velocity = filtered_velocity;
                xQueueSend(event_queue, &event, 0);
            }
            break;

        case ASCENT:
            if (filtered_velocity < APOGEE_VELOCITY_THRESHOLD) {
                descent_check_counter++;

                if (descent_check_counter >= CONSECUTIVE_SAMPLES) {
                    current_state = DESCENT;
                    event.timestamp = esp_timer_get_time();
                    event.event = APOGEE_DETECTED;
                    event.altitude = filtered_altitude;
                    event.velocity = filtered_velocity;
                    xQueueSend(event_queue, &event, 0);

                    gpio_set_level(PYRO_PIN, 1);
                    pyro_fired_time = esp_timer_get_time();

                    event.timestamp = pyro_fired_time;
                    event.event = PYRO_ON;
                    event.altitude = filtered_altitude;
                    event.velocity = filtered_velocity;
                    xQueueSend(event_queue, &event, 0);
                }
            } else {
                descent_check_counter = 0;
            }
            break;

        case DESCENT:
            if (pyro_fired_time != 0) {
                if (esp_timer_get_time() - pyro_fired_time >
                    PYRO_DURATION_MS * 1000) {
                    gpio_set_level(PYRO_PIN, 0);
                    pyro_fired_time = 0;
                    event.timestamp = esp_timer_get_time();
                    event.event = PYRO_OFF;
                    event.altitude = filtered_altitude;
                    event.velocity = filtered_velocity;
                    xQueueSend(event_queue, &event, 0);
                }
            }
            break;
        }
    }
}