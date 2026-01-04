#pragma once

#include "driver/spi_common.h"


// ===== Sampling =====
#define SENSOR_SAMPLE_RATE_HZ       100
#define SENSOR_PERIOD_MS            (1000 / SENSOR_SAMPLE_RATE_HZ)

// ===== Queue =====
#define SENSOR_QUEUE_LENGTH         256

// ===== SD logging =====
#define SD_MOUNT_POINT              "/sdcard"
#define SD_LOG_FILENAME             "/sdcard/flight.csv"
#define SD_FLUSH_INTERVAL_MS        100
#define SD_BUFFER_SIZE_BYTES        4096

// ===== Tasks =====
#define SENSOR_TASK_STACK_WORDS     4096
#define LOGGER_TASK_STACK_WORDS     6144
#define STATUS_TASK_STACK_WORDS     2048

#define SENSOR_TASK_PRIORITY        10
#define LOGGER_TASK_PRIORITY        8
#define STATUS_TASK_PRIORITY        3

// ===== I2C (shared bus: MPU + BMP280) =====
#define I2C_PORT_NUM                0
#define I2C_SDA_GPIO                21
#define I2C_SCL_GPIO                22
#define I2C_FREQ_HZ                 400000

// I2C 7-bit addresses
#define MPU_I2C_ADDR                0x68
#define BMP280_I2C_ADDR             0x76   // change to 0x77 if required

// ===== SD over SPI (SDSPI) =====
#define SD_SPI_HOST                 SPI2_HOST   // VSPI on many ESP32 examples
#define SD_MOSI_GPIO                23
#define SD_MISO_GPIO                19
#define SD_SCLK_GPIO                18
#define SD_CS_GPIO                  5

