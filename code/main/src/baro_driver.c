#include "baro_driver.h"

#include "app_config.h"
#include "driver/i2c.h"

#include <string.h>
#include <stdint.h>

#define BMP_REG_ID          0xD0
#define BMP_REG_RESET       0xE0
#define BMP_REG_STATUS      0xF3
#define BMP_REG_CTRL_MEAS   0xF4
#define BMP_REG_CONFIG      0xF5
#define BMP_REG_PRESS_MSB   0xF7
#define BMP_REG_CALIB00     0x88

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

static bmp280_calib_t s_calib;
static int32_t s_tfine = 0;

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

static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  s16le(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

static bool read_calibration(void)
{
    uint8_t c[24] = {0};
    if (!i2c_read_reg(BMP280_I2C_ADDR, BMP_REG_CALIB00, c, sizeof(c))) {
        return false;
    }

    s_calib.dig_T1 = u16le(&c[0]);
    s_calib.dig_T2 = s16le(&c[2]);
    s_calib.dig_T3 = s16le(&c[4]);

    s_calib.dig_P1 = u16le(&c[6]);
    s_calib.dig_P2 = s16le(&c[8]);
    s_calib.dig_P3 = s16le(&c[10]);
    s_calib.dig_P4 = s16le(&c[12]);
    s_calib.dig_P5 = s16le(&c[14]);
    s_calib.dig_P6 = s16le(&c[16]);
    s_calib.dig_P7 = s16le(&c[18]);
    s_calib.dig_P8 = s16le(&c[20]);
    s_calib.dig_P9 = s16le(&c[22]);

    return true;
}

/* Bosch reference integer compensation for BMP280 */
static int32_t compensate_temp_x100(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * ((int32_t)s_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
             ((int32_t)s_calib.dig_T3)) >> 14;

    s_tfine = var1 + var2;
    T = (s_tfine * 5 + 128) >> 8;   // temperature in 0.01°C

    return T;
}

static uint32_t compensate_press_pa(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)s_tfine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid divide-by-zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

    return (uint32_t)(p >> 8); // Pa
}

bool baro_init(SemaphoreHandle_t i2c_mutex)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    uint8_t id = 0;
    if (!i2c_read_reg(BMP280_I2C_ADDR, BMP_REG_ID, &id, 1)) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    if (id != 0x58) { // BMP280 chip ID
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    (void)i2c_write_reg(BMP280_I2C_ADDR, BMP_REG_RESET, 0xB6);

    if (!read_calibration()) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    /*
     * Flight logging config:
     * - normal mode (continuous)
     * - modest oversampling to reduce noise without dragging latency
     * This is not tuned yet; it’s a stable starting point.
     */
    (void)i2c_write_reg(BMP280_I2C_ADDR, BMP_REG_CONFIG, 0x08);     // filter=2, standby=0.5ms
    (void)i2c_write_reg(BMP280_I2C_ADDR, BMP_REG_CTRL_MEAS, 0x27);  // osrs_t=1, osrs_p=1, mode=normal

    xSemaphoreGive(i2c_mutex);
    return true;
}

bool baro_read_pressure(sensor_sample_t *sample, SemaphoreHandle_t i2c_mutex)
{
    if (!sample) return false;

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    uint8_t d[6] = {0};
    bool ok = i2c_read_reg(BMP280_I2C_ADDR, BMP_REG_PRESS_MSB, d, sizeof(d));

    xSemaphoreGive(i2c_mutex);

    if (!ok) return false;

    int32_t adc_P = (int32_t)((d[0] << 12) | (d[1] << 4) | (d[2] >> 4));
    int32_t adc_T = (int32_t)((d[3] << 12) | (d[4] << 4) | (d[5] >> 4));

    (void)compensate_temp_x100(adc_T);                 // updates s_tfine
    sample->pressure_pa = (int32_t)compensate_press_pa(adc_P);

    return true;
}
