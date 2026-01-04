#pragma once

#include <stdbool.h>
#include "freertos/semphr.h"
#include "app_types.h"

bool baro_init(SemaphoreHandle_t i2c_mutex);
bool baro_read_pressure(sensor_sample_t *sample, SemaphoreHandle_t i2c_mutex);
