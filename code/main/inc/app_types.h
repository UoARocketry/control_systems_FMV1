#pragma once
#include <stdint.h>

typedef struct
{
    uint32_t t_ms;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    int32_t pressure_pa;

    uint8_t imu_ok;
    uint8_t baro_ok;
} sensor_sample_t;

typedef struct
{
    uint32_t overwrite_count;
    uint32_t last_overwrite_time_ms;
} queue_stats_t;
