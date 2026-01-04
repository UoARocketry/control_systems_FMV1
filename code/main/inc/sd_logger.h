#pragma once

#include <stdbool.h>
#include "freertos/semphr.h"
#include "app_types.h"

bool sd_logger_init(SemaphoreHandle_t sd_mutex);
bool sd_logger_write_sample(const sensor_sample_t *s);
bool sd_logger_write_text(const char *text);
bool sd_logger_flush(SemaphoreHandle_t sd_mutex);
void sd_logger_close(SemaphoreHandle_t sd_mutex);

bool sd_logger_is_ready(void);
