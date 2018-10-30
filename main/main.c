#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "events.h"

#define PROTOCOL_HEADER_BIT 0xf5
#define PROTOCOL_VERSION 1
#define PROTOCOL_GIT_HASH 0
#define PROTOCOL_PACKET_BUFFER_SIZE 2048

#define PRIORITY_TASK_TELEMETRY 10
#define PRIORITY_TASK_GPS 1
#define PRIORITY_TASK_STATUS_LED 0

#define STATUS_LED_GPIO GPIO_NUM_13
#define STATUS_LED_ON_TIME 100
#define STATUS_LED_OFF_TIME 1000

#define GPS_UART UART_NUM_2
#define GPS_TXD GPIO_NUM_16
#define GPS_RXD GPIO_NUM_17
#define GPS_BUFFER_SIZE 256

typedef struct telemetry_packet_t {
    uint8_t protocol;
    uint8_t protocol_version;
    uint8_t mac_address[6];
    uint32_t session_id;
    uint32_t git_hash;
    uint64_t time_since_boot;
    uint16_t event_count;
    uint16_t data_length;
    uint32_t data_checksum;
    uint8_t data[PROTOCOL_PACKET_BUFFER_SIZE];
} telemetry_packet_t;

static SemaphoreHandle_t packet_write_sem;
static telemetry_packet_t packet = {.protocol = PROTOCOL_HEADER_BIT, .protocol_version = PROTOCOL_VERSION,
    .git_hash = PROTOCOL_GIT_HASH, .event_count = 0, .data_length = 0, .data_checksum = 0};

void write_event(event_type_t event_type, event_subtype_t event_subtype, void* data_ptr, size_t data_length)
{
    event_t event = {.type = event_type, .subtype = event_subtype, .data_length = data_length, .time_since_packet = esp_timer_get_time() - packet.time_since_boot};

    xSemaphoreTake(packet_write_sem, portMAX_DELAY);
    void* write_ptr = packet.data + packet.data_length;
    packet.data_length += sizeof(event_t) + data_length;
    xSemaphoreGive(packet_write_sem);

    memcpy(write_ptr, &event, sizeof(event_t));
    memcpy(write_ptr + sizeof(event_t), data_ptr, data_length);
}

static void telemetry_send_task()
{
    // Packet writing semaphore
    packet_write_sem = xSemaphoreCreateBinary();

    // Load mac address
    esp_efuse_mac_get_default(packet.mac_address);

    // First packet time
    packet.time_since_boot = esp_timer_get_time();

    while (true) {
        // Lock
        xSemaphoreTake(packet_write_sem, portMAX_DELAY);

        // TODO: Send packet

        // Prepare next packet
        packet.time_since_boot = esp_timer_get_time();
        packet.event_count = 0;
        packet.data_length = 0;
        packet.data_checksum = 0;

        // Unlock
        xSemaphoreGive(packet_write_sem);
    }
}

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

static void gps_read_task()
{
    // Configure GPS UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS_UART, &uart_config);
    uart_set_pin(GPS_UART, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART, GPS_BUFFER_SIZE, 0, 0, NULL, 0);

    uint8_t *data = (uint8_t *) malloc(GPS_BUFFER_SIZE);
    while (true) {
        if (uart_read_bytes(GPS_UART, data, 6, 1000 / portTICK_RATE_MS) < 6) {
            continue;
        }

        // Check for µb header
        if (data[0] != 0xb5 || data[1] != 0x62) {
            // Start over
            uart_flush_input(GPS_UART);
        }

        // Payload length + checksum size
        uint16_t remaining = (data[4] | data[5] << 8) + 2;
        if (uart_read_bytes(GPS_UART, data + 6, remaining, 10 / portTICK_RATE_MS) != remaining) {
            // TODO: Error?
            continue;
        }

        // Write event
        write_event(EVENT_TYPE_POSITION, EVENT_TYPE_POSITION_GPS_RAW, data, 6 + remaining);
    }
    free(data);
}

void app_main(void)
{
    // Initiate telemetry send task
    xTaskCreatePinnedToCore(telemetry_send_task, "telemetry_send_task", 256, NULL, PRIORITY_TASK_TELEMETRY, NULL, APP_CPU_NUM);

    // Initiate status led task
    xTaskCreatePinnedToCore(blink_status_led_task, "blink_status_led", 64, NULL, PRIORITY_TASK_STATUS_LED, NULL, APP_CPU_NUM);

    // Initiate GPS read task
    xTaskCreatePinnedToCore(gps_read_task, "gps_read_task", 256, NULL, PRIORITY_TASK_GPS, NULL, APP_CPU_NUM);
}

