#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "rom/crc.h"

#include "events.h"
#include "gps.h"
#include "wheel_speed.h"

#include "mma8451.h"

#define PRIORITY_TASK_TELEMETRY 10
#define PRIORITY_TASK_GPS 2
#define PRIORITY_TASK_WHEEL_SPEED 1
#define PRIORITY_TASK_STATUS_LED 0

#define STATUS_LED_GPIO GPIO_NUM_13
#define STATUS_LED_ON_TIME 100
#define STATUS_LED_OFF_TIME 1000

#define I2C_NUM I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_26
#define I2C_SCL_GPIO GPIO_NUM_25
#define I2C_FREQUENCY 400000

#define DISPLAY_UART UART_NUM_1
#define DISPLAY_TXD GPIO_NUM_2
#define DISPLAY_RXD GPIO_NUM_4
#define DISPLAY_BUFFER_SIZE 32

static void blink_status_led_task()
{
    // Configure status LED
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);

    while (true) {
        gpio_set_level(STATUS_LED_GPIO, true);
        vTaskDelay(STATUS_LED_ON_TIME / portTICK_PERIOD_MS);
        gpio_set_level(STATUS_LED_GPIO, false);
        vTaskDelay(STATUS_LED_OFF_TIME / portTICK_PERIOD_MS);
    }
}

static void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY
    };
    i2c_param_config(I2C_NUM, &conf);

    i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

void app_main(void)
{
    printf("Hello world\n");

    i2c_master_init();
    mma8451_init(MMA8451_SCALE_2G, MMA8451_DATA_RATE_50HZ);

    esp_err_t err;
    mma8451_raw_data_t acc;

    while (true) {
        err = mma8451_read_raw(&acc);

        ESP_LOGI("telemetry", "Accelerometer err:%d  x:%5d  y:%5d  z:%5d", err, acc.x, acc.y, acc.z);

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return;

    // Initiate status led task
    xTaskCreatePinnedToCore(blink_status_led_task, "blink_status_led", 64, NULL, PRIORITY_TASK_STATUS_LED, NULL, APP_CPU_NUM);

    // Initiate telemetry send task
    xTaskCreatePinnedToCore(telemetry_send_task, "telemetry_send_task", 256, NULL, PRIORITY_TASK_TELEMETRY, NULL, APP_CPU_NUM);

    // Initiate GPS read task
    xTaskCreatePinnedToCore(gps_read_task, "gps_read_task", 256, NULL, PRIORITY_TASK_GPS, NULL, APP_CPU_NUM);

    // Initiate wheel speed calculation
    xTaskCreatePinnedToCore(wheel_speed_calculation_task, "wheel_speed_calculation_task", 64, NULL, PRIORITY_TASK_WHEEL_SPEED, NULL, APP_CPU_NUM);
}

