#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

// --- User Pin Definitions ---
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 22
#define SPI_CLK_PIN 21
#define SPI_CS_PIN 23

static const char *TAG = "SD_NEW_FILE_TEST";

// Mount path for the partition
#define MOUNT_POINT "/sdcard"
#define SPI_HOST_ID SPI2_HOST

void app_main(void) {
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

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
        return;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI_HOST_ID;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS_PIN;
    slot_config.host_id = host.slot;

    sdmmc_card_t *card;
    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config,
                                  &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem.");
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    char file_path[64];
    struct stat st;
    int file_index = 0;

    // Loop until we find a name that doesn't exist
    while (1) {
        sprintf(file_path, "%s/data_%d.txt", MOUNT_POINT, file_index);

        // stat returns 0 if file exists, -1 if it doesn't
        if (stat(file_path, &st) == 0) {
            // File exists, try the next index
            ESP_LOGI(TAG, "File %s exists, trying next index...", file_path);
            file_index++;
        } else {
            // File does not exist, we can use this name
            break;
        }
    }

    ESP_LOGI(TAG, "Writing to new file: %s", file_path);

    FILE *f = fopen(file_path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    // Write dynamic content
    fprintf(f, "Run #%d: Hello ESP-IDF!\n", file_index);
    fclose(f);
    ESP_LOGI(TAG, "File written successfully");

    f = fopen(file_path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    char line[128];
    fgets(line, sizeof(line), f);
    fclose(f);

    // Strip newline for clean logging
    char *pos = strchr(line, '\n');
    if (pos)
        *pos = '\0';

    ESP_LOGI(TAG, "Read back content: '%s'", line);

    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    spi_bus_free(SPI_HOST_ID);
    ESP_LOGI(TAG, "Card unmounted");
}
