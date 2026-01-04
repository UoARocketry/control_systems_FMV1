#include "status_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "app_init.h"
#include "app_events.h"

void status_task(void *arg)
{
    while (1)
    {
        EventBits_t e = xEventGroupGetBits(system_events);

        bool sd_ok   = (e & EVT_SD_OK) != 0;
        bool imu_ok  = (e & EVT_IMU_OK) != 0;
        bool baro_ok = (e & EVT_BARO_OK) != 0;

        (void)sd_ok;
        (void)imu_ok;
        (void)baro_ok;

        /*
         * TODO:
         * - Map these states to LED patterns once I pick GPIO + UX.
         * - Optionally add a debug UART line here (not in sensor/logger paths).
         */

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
 