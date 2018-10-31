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
#include "wheel_speed.h"

#define WHEEL_SPEED_FL_GPIO GPIO_NUM_36
#define WHEEL_SPEED_FR_GPIO GPIO_NUM_35
#define WHEEL_SPEED_RL_GPIO GPIO_NUM_39
#define WHEEL_SPEED_RR_GPIO GPIO_NUM_34
#define WHEEL_SPEED_GPIO_SEL GPIO_SEL_36 | GPIO_SEL_35 | GPIO_SEL_39 | GPIO_SEL_34
#define WHEEL_SPEED_WAIT_TIME 100

volatile uint8_t wheel_speed_counters[4];

static void IRAM_ATTR wheel_speed_isr_handler(void* arg);

void wheel_speed_calculation_task()
{
    // Configure pins
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = WHEEL_SPEED_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = false;
    io_conf.pull_down_en = true;
    gpio_config(&io_conf);

    // Install interrupts
    gpio_install_isr_service(0);
    gpio_isr_handler_add(WHEEL_SPEED_FL_GPIO, wheel_speed_isr_handler, (void*) WHEEL_FL);
    gpio_isr_handler_add(WHEEL_SPEED_FR_GPIO, wheel_speed_isr_handler, (void*) WHEEL_FR);
    gpio_isr_handler_add(WHEEL_SPEED_RL_GPIO, wheel_speed_isr_handler, (void*) WHEEL_RL);
    gpio_isr_handler_add(WHEEL_SPEED_RR_GPIO, wheel_speed_isr_handler, (void*) WHEEL_RR);

    while (true) {
        // Allow counter to increase
        vTaskDelay(WHEEL_SPEED_WAIT_TIME / portTICK_PERIOD_MS);

        // Calculate RPMs
        uint8_t wheel_speed_rpms[4];
        for (int i = 0; i < 4; i++) {
            wheel_speed_rpms[i] = wheel_speed_counters[i] * (1000 * 60) / WHEEL_SPEED_WAIT_TIME;
            wheel_speed_counters[i] = 0;
        }

        // Write event
        write_event(EVENT_TYPE_MOTOR, EVENT_TYPE_MOTOR_WHEEL_SPEED, wheel_speed_rpms, sizeof(wheel_speed_rpms));
    }
}

static void IRAM_ATTR wheel_speed_isr_handler(void* arg)
{
    uint32_t wheel = (uint32_t) arg;
    if (wheel > 4) {
        // TODD: Error?
        return;
    }

    wheel_speed_counters[wheel]++;
}