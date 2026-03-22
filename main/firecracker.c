#include "bmp280.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#define MUTE_BUZZER

// Filter settings
#define ALPHA_ALT 0.3
#define ALPHA_VEL 0.3

// Thresholds
#define MIN_ALT_INCREASE 25
#define APOGEE_VELOCITY_THRESHOLD -1.0
#define APOGEE_SAMPLES 5
#define CALIBRATION_SAMPLES 50
#define PYRO_DURATION_MS 600
#define LANDING_VELOCITY_THRESHOLD 0.5
#define LANDING_SAMPLES 100
#define IDLE_DELAY_MS 120000

#define LOOP_FREQ_HZ 20
#define DATA_QUEUE_SIZE 400
#define EVENT_QUEUE_SIZE 100

#define PYRO_PIN 32
#define LED_PIN 2
#define BUZZER_PIN 27
#define I2C_SCL_PIN 16
#define I2C_SDA_PIN 17
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 22
#define SPI_CLK_PIN 21
#define SPI_CS_PIN 23

#ifdef MUTE_BUZZER
    #undef BUZZER_PIN
    #define BUZZER_PIN 2
#endif

#define MOUNT_POINT "/sdcard"
#define SPI_HOST_ID SPI2_HOST
static FILE *event_log_file = NULL;
static FILE *data_log_file = NULL;

#define SEA_LEVEL_PRESSURE 1013.25
static const char *TAG = "FIRECRACKER";

typedef enum {
    ARMED,
    ASCENT,
    DESCENT,
    LANDED,
} flight_state_t;

typedef enum {
    CALIB_START,
    CALIB_END,
    LAUNCH_DETECTED,
    APOGEE_DETECTED,
    PYRO_ON,
    PYRO_OFF,
    LANDING_DETECTED,
    MEAS_FAILED,
} event_type_t;

typedef enum {
    BUZZER_IDLE,
    BUZZER_IDLE_ERROR,
    BUZZER_ARMED,
    BUZZER_FLIGHT,
    BUZZER_LANDED
} buzzer_mode_t;

buzzer_mode_t buzzer_mode = BUZZER_IDLE;

typedef struct {
    int64_t timestamp;
    bmp280_data_t bmp;
} data_t;

typedef struct {
    int64_t timestamp;
    event_type_t event;
    float altitude;
    float velocity;
} event_t;

QueueHandle_t data_queue;
QueueHandle_t event_queue;

float calculate_altitude(float pressure_hPa) {
    return 8425 * log(SEA_LEVEL_PRESSURE / pressure_hPa);
}

void i2c_init(i2c_master_bus_handle_t *i2c_bus_handle) {
    i2c_master_bus_config_t i2c_bus_config = {.clk_source = I2C_CLK_SRC_DEFAULT,
                                              .scl_io_num = I2C_SCL_PIN,
                                              .sda_io_num = I2C_SDA_PIN,
                                              .glitch_ignore_cnt = 7,
                                              .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle));
}

esp_err_t sd_card_init(void) {
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ESP_LOGI(TAG, "Initializing SPI bus...");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SPI_DMA_CH_AUTO);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI_HOST_ID;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS_PIN;
    slot_config.host_id = host.slot;

    sdmmc_card_t *card;
    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem.");
        return ret;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    char data_file_path[64];
    char event_file_path[64];
    struct stat st;
    int file_index = 0;

    while (1) {
        sprintf(data_file_path, "%s/d_%d.csv", MOUNT_POINT, file_index);
        sprintf(event_file_path, "%s/e_%d.csv", MOUNT_POINT, file_index);

        if (stat(data_file_path, &st) == 0 || stat(event_file_path, &st) == 0) {
            ESP_LOGI(TAG, "Files exist, trying next index...");
            file_index++;
        } else {
            break;
        }
    }

    ESP_LOGI(TAG, "Writing to data file: %s", data_file_path);
    ESP_LOGI(TAG, "Writing to event file: %s", event_file_path);

    data_log_file = fopen(data_file_path, "w");
    if (data_log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open data file for writing");
        return ESP_FAIL;
    }
    fprintf(data_log_file, "timestamp_us,temperature_C,pressure_hPa\n");
    fflush(data_log_file);

    event_log_file = fopen(event_file_path, "w");
    if (event_log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open event file for writing");
        return ESP_FAIL;
    }
    fprintf(event_log_file, "timestamp_us,event_type,altitude_m,velocity_ms\n");
    fflush(event_log_file);

    return ESP_OK;
}

void gpio_output_init(void) {
    uint64_t pin_mask =
        (1ULL << PYRO_PIN) | (1ULL << LED_PIN) | (1ULL << BUZZER_PIN);

    gpio_config_t io_conf = {.pin_bit_mask = pin_mask,
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    gpio_set_level(PYRO_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
}

void data_logger_task(void *pvParameter) {
    data_t data;
    int data_counter = 0;
    while (1) {
        if (xQueueReceive(data_queue, &data, portMAX_DELAY) ==
            pdTRUE) {
            ESP_LOGI(TAG, "%lld us | Temperature: %.2f C, Pressure: %.2f hPa",
                     data.timestamp, (float)data.bmp.temperature / 100.0, (float)data.bmp.pressure / 25600.0);
            if (data_log_file != NULL) {
                fprintf(data_log_file, "%lld,%.2f,%.2f\n",
                        data.timestamp, (float)data.bmp.temperature / 100.0, (float)data.bmp.pressure / 25600.0);
                data_counter++;
                if (data_counter >= 20) {
                    fflush(data_log_file);
                    fsync(fileno(data_log_file));
                    data_counter = 0;
                }
            }
        }
    }
}

void event_logger_task(void *pvParameter) {
    event_t event;
    while (1) {
        if (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "%lld us | Event: %d, Altitude: %.2f m, Velocity: %.2f m/s",
                event.timestamp, event.event, event.altitude, event.velocity);
            if (event_log_file != NULL) {
                fprintf(event_log_file, "%lld,%d,%.2f,%.2f\n", event.timestamp,
                        event.event, event.altitude, event.velocity);
                fflush(event_log_file);
                fsync(fileno(event_log_file));
            }
        }
    }
}

void buzzer_task(void *pvParameter) {
    while (1) {
        switch (buzzer_mode) {
            case BUZZER_IDLE:
                gpio_set_level(BUZZER_PIN, 1);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(BUZZER_PIN, 0);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(1800));
                break;

            case BUZZER_IDLE_ERROR:
                for (int i = 0; i < 3; i++) {
                    gpio_set_level(BUZZER_PIN, 1);
                    gpio_set_level(LED_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_set_level(BUZZER_PIN, 0);
                    gpio_set_level(LED_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                vTaskDelay(pdMS_TO_TICKS(1400));
                break;

            case BUZZER_ARMED:
                gpio_set_level(BUZZER_PIN, 1);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(BUZZER_PIN, 0);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case BUZZER_FLIGHT:
                gpio_set_level(BUZZER_PIN, 1);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(10000));
                break;

            case BUZZER_LANDED:
                gpio_set_level(BUZZER_PIN, 1);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(BUZZER_PIN, 0);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }
}

void app_main(void) {
    gpio_output_init();

    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_init(&i2c_bus_handle);

    bmp280_handle_t dev_bmp280;
    esp_err_t err = bmp280_init(i2c_bus_handle, &dev_bmp280, 0x76);

    if (err != ESP_OK) {
    ESP_LOGE(TAG, "BMP280 init failed: %s", esp_err_to_name(err));
    buzzer_mode = BUZZER_IDLE_ERROR;

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = sd_card_init();
    if (err != ESP_OK) {
        buzzer_mode = BUZZER_IDLE_ERROR;
    }

    int64_t pyro_fired_time = 0;
    int descent_check_counter = 0;
    int landing_check_counter = 0;
    
    data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(data_t));
    xTaskCreate(data_logger_task, "data_logger", 4096, NULL, 5, NULL);
    data_t data;
    
    event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(event_t));
    xTaskCreate(event_logger_task, "event_logger", 4096, NULL, 5, NULL);
    event_t event;
    
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 7, NULL);
    
    vTaskDelay(pdMS_TO_TICKS(IDLE_DELAY_MS));
    flight_state_t current_state = ARMED;
    buzzer_mode = BUZZER_ARMED;

    event.timestamp = esp_timer_get_time();
    event.event = CALIB_START;
    event.altitude = 0.0;
    event.velocity = 0.0;
    xQueueSend(event_queue, &event, 0);

    float sum_alt = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        bmp280_read(&dev_bmp280, &data.bmp);
        float press_hPa = data.bmp.pressure / 25600.0;
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

        data.timestamp = esp_timer_get_time();

        esp_err_t ret = bmp280_read(&dev_bmp280, &data.bmp);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Sensor read failed");
            event.timestamp = esp_timer_get_time();
            event.event = MEAS_FAILED;
            event.altitude = filtered_altitude;
            event.velocity = filtered_velocity;
            xQueueSend(event_queue, &event, 0);
            continue;
        }

        xQueueSend(data_queue, &data, 0);

        float press_hPa = data.bmp.pressure / 25600.0;
        float raw_altitude = calculate_altitude(press_hPa);

        filtered_altitude = (ALPHA_ALT * raw_altitude) + ((1.0 - ALPHA_ALT) * filtered_altitude);
        float instant_velocity = (filtered_altitude - prev_filtered_altitude) / (1.0 / LOOP_FREQ_HZ);
        filtered_velocity = (ALPHA_VEL * instant_velocity) + ((1.0 - ALPHA_VEL) * filtered_velocity);

        prev_filtered_altitude = filtered_altitude;

        switch (current_state) {
        case ARMED:
            if (filtered_altitude > (ground_altitude + MIN_ALT_INCREASE)) {
                current_state = ASCENT;
                buzzer_mode = BUZZER_FLIGHT;
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

                if (descent_check_counter >= APOGEE_SAMPLES) {
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
                if (esp_timer_get_time() - pyro_fired_time > PYRO_DURATION_MS * 1000) {
                    gpio_set_level(PYRO_PIN, 0);
                    pyro_fired_time = 0;
                    event.timestamp = esp_timer_get_time();
                    event.event = PYRO_OFF;
                    event.altitude = filtered_altitude;
                    event.velocity = filtered_velocity;
                    xQueueSend(event_queue, &event, 0);
                }
            }

            if (fabs(filtered_velocity) < LANDING_VELOCITY_THRESHOLD) {
                landing_check_counter++;
                
                if (landing_check_counter >= LANDING_SAMPLES) {
                    current_state = LANDED;
                    buzzer_mode = BUZZER_LANDED;
                    event.timestamp = esp_timer_get_time();
                    event.event = LANDING_DETECTED;
                    event.altitude = filtered_altitude;
                    event.velocity = filtered_velocity;
                    xQueueSend(event_queue, &event, 0);

                    vTaskDelay(pdMS_TO_TICKS(10000));

                    if (data_log_file != NULL) {
                        fclose(data_log_file);
                        data_log_file = NULL;
                    }
                    if (event_log_file != NULL) {
                        fclose(event_log_file);
                        event_log_file = NULL;
                    }
                }
            } else {
                landing_check_counter = 0;
            }
            break;
            
        case LANDED:
            break;
        }
    }
}