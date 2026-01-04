#include "imu_driver.h"

#include "app_config.h"
#include "driver/i2c.h"

#include <string.h>

#define MPU_REG_WHO_AM_I        0x75
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B

static bool i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT_NUM,
        addr7,
        &reg, 1,
        buf, len,
        pdMS_TO_TICKS(50)
    );
    return err == ESP_OK;
}

static bool i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t pkt[2] = { reg, val };
    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT_NUM,
        addr7,
        pkt, sizeof(pkt),
        pdMS_TO_TICKS(50)
    );
    return err == ESP_OK;
}

bool imu_init(SemaphoreHandle_t i2c_mutex)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    uint8_t who = 0;
    if (!i2c_read_reg(MPU_I2C_ADDR, MPU_REG_WHO_AM_I, &who, 1)) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    /*
     * WHO_AM_I varies across MPU variants; for MPU6050 it’s typically 0x68.
     * If this is a different MPU, I’ll widen this check later.
     */
    if (who != 0x68) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    /*
     * Bring the device out of sleep and into a known configuration.
     * I’m using conservative full-scale ranges initially (±2g, ±250dps).
     */
    if (!i2c_write_reg(MPU_I2C_ADDR, MPU_REG_PWR_MGMT_1, 0x00)) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    (void)i2c_write_reg(MPU_I2C_ADDR, MPU_REG_SMPLRT_DIV, 0x04);     // nominal divider; final rate handled at task level
    (void)i2c_write_reg(MPU_I2C_ADDR, MPU_REG_CONFIG,     0x03);     // DLPF ~44 Hz (typical)
    (void)i2c_write_reg(MPU_I2C_ADDR, MPU_REG_GYRO_CONFIG,0x00);     // ±250 dps
    (void)i2c_write_reg(MPU_I2C_ADDR, MPU_REG_ACCEL_CONFIG,0x00);    // ±2 g

    xSemaphoreGive(i2c_mutex);
    return true;
}

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

bool imu_read(sensor_sample_t *sample, SemaphoreHandle_t i2c_mutex)
{
    if (!sample) return false;

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    uint8_t raw[14] = {0};
    bool ok = i2c_read_reg(MPU_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, raw, sizeof(raw));

    xSemaphoreGive(i2c_mutex);

    if (!ok) return false;

    sample->ax = be16(&raw[0]);
    sample->ay = be16(&raw[2]);
    sample->az = be16(&raw[4]);

    /* raw[6:8] is temperature; ignored for now */
    sample->gx = be16(&raw[8]);
    sample->gy = be16(&raw[10]);
    sample->gz = be16(&raw[12]);

    return true;
}
