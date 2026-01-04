#include "app_init.h"

#include "app_config.h"
#include "sensor_task.h"
#include "logger_task.h"
#include "status_task.h"

#include "driver/i2c.h"

QueueHandle_t sensor_queue;
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t sd_mutex;
EventGroupHandle_t system_events;

queue_stats_t queue_stats = {0};

static void i2c_bus_init(void)
{
    /*
     * I initialise I2C once at startup. Drivers assume the bus exists.
     * If I later need hot-restart behaviour, I’ll wrap this in a bus module.
     */
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0
    };

    (void)i2c_param_config(I2C_PORT_NUM, &cfg);
    (void)i2c_driver_install(I2C_PORT_NUM, cfg.mode, 0, 0, 0);
}

void app_init(void)
{
    sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(sensor_sample_t));

    i2c_mutex = xSemaphoreCreateMutex();
    sd_mutex  = xSemaphoreCreateMutex();

    system_events = xEventGroupCreate();

    i2c_bus_init();

    xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_WORDS, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(logger_task, "logger_task", LOGGER_TASK_STACK_WORDS, NULL, LOGGER_TASK_PRIORITY, NULL);
    xTaskCreate(status_task, "status_task", STATUS_TASK_STACK_WORDS, NULL, STATUS_TASK_PRIORITY, NULL);
}
