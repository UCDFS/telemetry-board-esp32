#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define PRIORITY_TASK_STATUS_LED 0

#define STATUS_LED_GPIO GPIO_NUM_13
#define STATUS_LED_ON_TIME 100
#define STATUS_LED_OFF_TIME 1000

static void blink_status_led()
{
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
    // Initiate status led task
    xTaskCreatePinnedToCore(blink_status_led, "blink_status_led", 64, NULL, PRIORITY_TASK_STATUS_LED, NULL, APP_CPU_NUM);
}

