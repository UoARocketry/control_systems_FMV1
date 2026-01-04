#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "app_types.h"

extern QueueHandle_t sensor_queue;
extern SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t sd_mutex;
extern EventGroupHandle_t system_events;

extern queue_stats_t queue_stats;

void app_init(void);

