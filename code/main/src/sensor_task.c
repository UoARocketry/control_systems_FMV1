#include "sensor_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "app_init.h"
#include "app_config.h"
#include "app_events.h"

#include "imu_driver.h"
#include "baro_driver.h"

#include "esp_timer.h"
#include "esp_compiler.h"

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void sensor_task(void *arg)
{
    bool imu_ok  = imu_init(i2c_mutex);
    bool baro_ok = baro_init(i2c_mutex);

    if (imu_ok)  xEventGroupSetBits(system_events, EVT_IMU_OK);
    else         xEventGroupClearBits(system_events, EVT_IMU_OK);

    if (baro_ok) xEventGroupSetBits(system_events, EVT_BARO_OK);
    else         xEventGroupClearBits(system_events, EVT_BARO_OK);

    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));

        sensor_sample_t sample = {0};
        sample.t_ms = now_ms();

        if (imu_ok)  sample.imu_ok  = imu_read(&sample, i2c_mutex);
        else         sample.imu_ok  = 0;

        if (baro_ok) sample.baro_ok = baro_read_pressure(&sample, i2c_mutex);
        else         sample.baro_ok = 0;

        /*
         * Sensor task does not block. If the queue is full, I drop the oldest
         * sample and insert the newest. I track overwrite count + timestamp for
         * post-flight correlation.
         */
        if (xQueueSend(sensor_queue, &sample, 0) != pdTRUE)
        {
            sensor_sample_t discarded;
            (void)xQueueReceive(sensor_queue, &discarded, 0);
            (void)xQueueSend(sensor_queue, &sample, 0);

            queue_stats.overwrite_count++;
            queue_stats.last_overwrite_time_ms = sample.t_ms;
        }
    }
}

        
        
