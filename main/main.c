#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "rom/crc.h"

#include "events.h"
#include "gps.h"
#include "wheel_speed.h"

#define PRIORITY_TASK_TELEMETRY 10
#define PRIORITY_TASK_GPS 2
#define PRIORITY_TASK_WHEEL_SPEED 1
#define PRIORITY_TASK_STATUS_LED 0

#define STATUS_LED_GPIO GPIO_NUM_13
#define STATUS_LED_ON_TIME 100
#define STATUS_LED_OFF_TIME 1000

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

void app_main(void)
{
    printf("Hello world\n");
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

