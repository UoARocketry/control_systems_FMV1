#pragma once

#include <stdbool.h>
#include "freertos/semphr.h"
#include "app_types.h"

bool imu_init(SemaphoreHandle_t i2c_mutex);
bool imu_read(sensor_sample_t *sample, SemaphoreHandle_t i2c_mutex);


