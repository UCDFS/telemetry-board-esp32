#include <string.h>

#include <esp_log.h>
#include <sys/socket.h>
#include <rom/crc.h>

#include "telemetry.h"

#define TELEMETRY_REFRESH_RATE 20

#define PROTOCOL_HEADER_BIT 0xf5
#define PROTOCOL_VERSION 1
#define PROTOCOL_GIT_HASH 0

#define TAG "Telemetry"

static SemaphoreHandle_t packet_write_sem;
static telemetry_packet_t packet = {
		.protocol = PROTOCOL_HEADER_BIT,
		.protocol_version = PROTOCOL_VERSION,
		.git_hash = PROTOCOL_GIT_HASH,
		.event_count = 0,
		.data_length = 0,
		.data_checksum = 0};

void telemetry_write_event(telemetry_event_type_t event_type, telemetry_event_subtype_t event_subtype, void *data_ptr,
						   size_t data_length)
{
	xSemaphoreTake(packet_write_sem, portMAX_DELAY);
	telemetry_event_t event = {
			.type = event_type,
			.subtype = event_subtype,
			.data_length = (uint8_t) data_length,
			.time_since_packet = (uint8_t) (esp_timer_get_time() - packet.time_since_boot)
	};
	void *write_ptr = packet.data + packet.data_length;
	packet.event_count++;
	packet.data_length += sizeof(telemetry_event_t) + data_length;
	xSemaphoreGive(packet_write_sem);

	memcpy(write_ptr, &event, sizeof(telemetry_event_t));
	memcpy(write_ptr + sizeof(telemetry_event_t), data_ptr, data_length);
}

void telemetry_send_task()
{
	// Packet writing semaphore
	packet_write_sem = xSemaphoreCreateBinary();

	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		vTaskDelete(NULL);
	}

	struct sockaddr_in dest_addr;
	dest_addr.sin_addr.s_addr = inet_addr(CONFIG_TELEMETRY_SERVER_HOST);
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(CONFIG_TELEMETRY_SERVER_PORT);

	// Load mac address
	esp_efuse_mac_get_default(packet.mac_address);

	// Generate random session ID
	packet.session_id = esp_random();

	// First packet time
	packet.time_since_boot = (uint64_t) esp_timer_get_time();

	xSemaphoreGive(packet_write_sem);

	while (true) {
		// Lock
		xSemaphoreTake(packet_write_sem, portMAX_DELAY);

		// Calculate checksum
		packet.data_checksum = crc32_le(0, packet.data, packet.data_length);

		// Send data
		int err = sendto(sock, &packet, sizeof(telemetry_packet_t) - PROTOCOL_PACKET_BUFFER_SIZE + packet.data_length,
						 0, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
		if (err < 0) {
			ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
			break;
		}
		ESP_LOGI(TAG, "Sent packet with %d events, data length %d", packet.event_count, packet.data_length);

		// Prepare next packet
		packet.time_since_boot = (uint64_t) esp_timer_get_time();
		packet.event_count = 0;
		packet.data_length = 0;
		packet.data_checksum = 0;

		// Unlock
		xSemaphoreGive(packet_write_sem);

		vTaskDelay(1000 / TELEMETRY_REFRESH_RATE / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}