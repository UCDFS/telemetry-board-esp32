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

#define PROTOCOL_HEADER_BIT 0xf5
#define PROTOCOL_VERSION 1
#define PROTOCOL_GIT_HASH 0
#define PROTOCOL_PACKET_BUFFER_SIZE 2048

static SemaphoreHandle_t packet_write_sem;
static telemetry_packet_t packet = {.protocol = PROTOCOL_HEADER_BIT, .protocol_version = PROTOCOL_VERSION,
    .git_hash = PROTOCOL_GIT_HASH, .event_count = 0, .data_length = 0, .data_checksum = 0};

void write_event(event_type_t event_type, event_subtype_t event_subtype, void* data_ptr, size_t data_length)
{
    xSemaphoreTake(packet_write_sem, portMAX_DELAY);
    event_t event = {.type = event_type, .subtype = event_subtype, .data_length = data_length, .time_since_packet = esp_timer_get_time() - packet.time_since_boot};
    void* write_ptr = packet.data + packet.data_length;
    packet.event_count++;
    packet.data_length += sizeof(event_t) + data_length;
    xSemaphoreGive(packet_write_sem);

    memcpy(write_ptr, &event, sizeof(event_t));
    memcpy(write_ptr + sizeof(event_t), data_ptr, data_length);
}

void telemetry_send_task()
{
    // Packet writing semaphore
    packet_write_sem = xSemaphoreCreateBinary();

    // Load mac address
    esp_efuse_mac_get_default(packet.mac_address);

    // Generate random session ID
    packet.session_id = esp_random();

    // First packet time
    packet.time_since_boot = esp_timer_get_time();

    while (true) {
        // Lock
        xSemaphoreTake(packet_write_sem, portMAX_DELAY);

        // Calculate checksum
        packet.data_checksum = crc32_le(0, packet.data, packet.data_length);

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