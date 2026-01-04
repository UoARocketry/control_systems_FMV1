#include "logger_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "app_init.h"
#include "app_config.h"
#include "app_events.h"

#include "sd_logger.h"
#include "esp_timer.h"

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void logger_task(void *arg)
{
    uint32_t last_flush_ms = now_ms();
    uint32_t last_sd_retry_ms = 0;

    sensor_sample_t sample;

    while (1)
    {
        if (!sd_logger_is_ready())
        {
            uint32_t t = now_ms();
            if (t - last_sd_retry_ms > 2000) {
                if (sd_logger_init(sd_mutex)) {
                    xEventGroupSetBits(system_events, EVT_SD_OK | EVT_LOGGING_ACTIVE);
                } else {
                    xEventGroupClearBits(system_events, EVT_SD_OK | EVT_LOGGING_ACTIVE);
                }
                last_sd_retry_ms = t;
            }

            /* Keep draining the queue so overwrite stats remain meaningful. */
            if (xQueueReceive(sensor_queue, &sample, pdMS_TO_TICKS(50)) == pdTRUE) {
                /* discard */
            }

            continue;
        }

        if (xQueueReceive(sensor_queue, &sample, portMAX_DELAY) == pdTRUE)
        {
            /*
             * If the buffer is full, flush and retry once. If that fails, I mark SD
             * down and let the retry logic remount later.
             */
            if (!sd_logger_write_sample(&sample)) {
                if (!sd_logger_flush(sd_mutex) || !sd_logger_write_sample(&sample)) {
                    xEventGroupClearBits(system_events, EVT_SD_OK | EVT_LOGGING_ACTIVE);
                    sd_logger_close(sd_mutex);
                    continue;
                }
            }

            uint32_t t = now_ms();
            if (t - last_flush_ms > SD_FLUSH_INTERVAL_MS)
            {
                /*
                 * Stamp queue overwrite diagnostics into the log as comment lines.
                 * This avoids a separate telemetry channel for a metric I mainly
                 * care about post-flight.
                 */
                if (queue_stats.overwrite_count != 0) {
                    char diag[96];
                    int n = snprintf(
                        diag, sizeof(diag),
                        "# overwrites=%lu last_overwrite_ms=%lu\n",
                        (unsigned long)queue_stats.overwrite_count,
                        (unsigned long)queue_stats.last_overwrite_time_ms
                    );
                    if (n > 0 && (size_t)n < sizeof(diag)) {
                        if (!sd_logger_write_text(diag)) {
                            (void)sd_logger_flush(sd_mutex);
                            (void)sd_logger_write_text(diag);
                        }
                    }
                }

                if (!sd_logger_flush(sd_mutex)) {
                    xEventGroupClearBits(system_events, EVT_SD_OK | EVT_LOGGING_ACTIVE);
                    sd_logger_close(sd_mutex);
                    continue;
                }

                last_flush_ms = t;
            }
        }
    }
}
