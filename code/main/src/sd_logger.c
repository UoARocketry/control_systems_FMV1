#include "sd_logger.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/spi_master.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"

static const char *TAG = "sd_logger";

static sdmmc_card_t *s_card = NULL;
static FILE *s_fp = NULL;

static char s_buf[SD_BUFFER_SIZE_BYTES];
static size_t s_buf_len = 0;
static bool s_ready = false;

bool sd_logger_is_ready(void)
{
    return s_ready;
}

static void buffer_reset(void)
{
    s_buf_len = 0;
}

static bool buffer_append(const char *line, size_t n)
{
    if (n > sizeof(s_buf)) return false;
    if (s_buf_len + n > sizeof(s_buf)) return false;

    memcpy(&s_buf[s_buf_len], line, n);
    s_buf_len += n;
    return true;
}

bool sd_logger_init(SemaphoreHandle_t sd_mutex)
{
    xSemaphoreTake(sd_mutex, portMAX_DELAY);

    if (s_ready) {
        xSemaphoreGive(sd_mutex);
        return true;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16 * 1024
    };

    esp_err_t err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        xSemaphoreGive(sd_mutex);
        return false;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS_GPIO;
    slot_cfg.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 16 * 1024
    };

    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sd mount failed: %s", esp_err_to_name(err));
        s_card = NULL;
        xSemaphoreGive(sd_mutex);
        return false;
    }

    s_fp = fopen(SD_LOG_FILENAME, "a");
    if (!s_fp) {
        ESP_LOGE(TAG, "failed to open log file: %s", SD_LOG_FILENAME);
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
        xSemaphoreGive(sd_mutex);
        return false;
    }

    long pos = ftell(s_fp);
    if (pos == 0) {
        fputs("t_ms,ax,ay,az,gx,gy,gz,pressure_pa,imu_ok,baro_ok\n", s_fp);
        fflush(s_fp);
    }

    buffer_reset();
    s_ready = true;

    xSemaphoreGive(sd_mutex);
    return true;
}

bool sd_logger_write_sample(const sensor_sample_t *s)
{
    if (!s_ready || !s_fp || !s) return false;

    char line[160];

    int n = snprintf(
        line, sizeof(line),
        "%lu,%d,%d,%d,%d,%d,%d,%ld,%u,%u\n",
        (unsigned long)s->t_ms,
        (int)s->ax, (int)s->ay, (int)s->az,
        (int)s->gx, (int)s->gy, (int)s->gz,
        (long)s->pressure_pa,
        (unsigned)s->imu_ok,
        (unsigned)s->baro_ok
    );

    if (n <= 0 || (size_t)n >= sizeof(line)) return false;

    if (!buffer_append(line, (size_t)n)) {
        return false; // caller decides when to flush
    }

    return true;
}

bool sd_logger_write_text(const char *text)
{
    if (!s_ready || !s_fp || !text) return false;
    return buffer_append(text, strlen(text));
}

bool sd_logger_flush(SemaphoreHandle_t sd_mutex)
{
    xSemaphoreTake(sd_mutex, portMAX_DELAY);

    if (!s_ready || !s_fp) {
        xSemaphoreGive(sd_mutex);
        return false;
    }

    if (s_buf_len == 0) {
        xSemaphoreGive(sd_mutex);
        return true;
    }

    size_t written = fwrite(s_buf, 1, s_buf_len, s_fp);
    if (written != s_buf_len) {
        ESP_LOGE(TAG, "short write: %u/%u", (unsigned)written, (unsigned)s_buf_len);
        xSemaphoreGive(sd_mutex);
        return false;
    }

    fflush(s_fp);
    buffer_reset();

    xSemaphoreGive(sd_mutex);
    return true;
}

void sd_logger_close(SemaphoreHandle_t sd_mutex)
{
    xSemaphoreTake(sd_mutex, portMAX_DELAY);

    if (s_fp) {
        if (s_buf_len) {
            fwrite(s_buf, 1, s_buf_len, s_fp);
            fflush(s_fp);
        }
        fclose(s_fp);
        s_fp = NULL;
    }

    if (s_card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
    }

    buffer_reset();
    s_ready = false;

    xSemaphoreGive(sd_mutex);
}
