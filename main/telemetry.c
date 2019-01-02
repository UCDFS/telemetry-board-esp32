#include <string.h>

#include <esp_log.h>
#include <sys/socket.h>
#include <rom/crc.h>

#include "telemetry.h"
#include "task_priorities.h"
#include "gps.h"

#define TELEMETRY_REFRESH_RATE 50

#define PROTOCOL_HEADER_BIT 0xf5
#define PROTOCOL_VERSION 1
#define PROTOCOL_GIT_HASH GIT_HASH

#define PACKET_DATA_BUFFER_SIZE 32768

typedef struct
{
	uint8_t protocol			:8;
	uint8_t protocol_version	:8;
	uint8_t mac_address[6];
	uint32_t session_id			:32;
	uint32_t git_hash			:32;
	uint64_t time_since_boot	:64;
	uint16_t event_count		:16;
	uint16_t data_length		:16;
	uint32_t data_checksum		:32;
	uint8_t data[];
} telemetry_tx_packet_t;

typedef struct
{
	telemetry_event_type_t type			:8;
	telemetry_event_subtype_t subtype	:8;
	uint16_t data_length				:16;
	uint32_t time_since_packet			:32;
	uint8_t data[];
} telemetry_event_t;

typedef struct
{
	uint8_t protocol			:8;
	uint8_t protocol_version	:8;
	uint64_t time_unix          :64;
	uint16_t message_count      :16;
	uint16_t data_length        :16;
	uint32_t data_checksum      :32;
	uint8_t data[];
} telemetry_rx_packet_t;

typedef struct
{
	telemetry_message_type_t type		:8;
	telemetry_message_subtype_t subtype	:8;
	uint16_t data_length				:16;
	uint32_t time_since_packet			:32;
	uint8_t data[];
} telemetry_message_t;

static SemaphoreHandle_t packet_write_sem;
static telemetry_tx_packet_t *tx_packet;
static telemetry_rx_packet_t *rx_packet;

static int server_socket = -1;
static struct sockaddr_in server_addr;

static TaskHandle_t telemetry_send_task_handle = NULL;
static TaskHandle_t telemetry_receive_task_handle = NULL;

static const char* TAG = "telemetry";

static void (*handler)(telemetry_event_type_t, telemetry_event_subtype_t, void *, size_t) = NULL;

void telemetry_send_task();
void telemetry_receive_task();

void telemetry_init() {
	// Server address
	server_addr = (struct sockaddr_in) {
			.sin_addr.s_addr = inet_addr(CONFIG_TELEMETRY_SERVER_HOST),
			.sin_family = AF_INET,
			.sin_port = htons(CONFIG_TELEMETRY_SERVER_PORT),
	};

	// Packet writing semaphore
	packet_write_sem = xSemaphoreCreateBinary();

	// Allocate RX packet
	rx_packet = malloc(sizeof(telemetry_rx_packet_t) + PACKET_DATA_BUFFER_SIZE);

	// Allocate TX packet
	tx_packet = malloc(sizeof(telemetry_tx_packet_t) + PACKET_DATA_BUFFER_SIZE);
	tx_packet->protocol = PROTOCOL_HEADER_BIT;
	tx_packet->protocol_version = PROTOCOL_VERSION;
	tx_packet->git_hash = PROTOCOL_GIT_HASH;
	tx_packet->event_count = 0;
	tx_packet->data_length = 0;
	tx_packet->data_checksum = 0;

	// Generate random session ID
	tx_packet->session_id = esp_random();

	// First packet time
	tx_packet->time_since_boot = (uint64_t) esp_timer_get_time();

	// Load mac address
	esp_efuse_mac_get_default(tx_packet->mac_address);

	// Allow writing events
	xSemaphoreGive(packet_write_sem);
}

bool telemetry_write_event(telemetry_event_type_t event_type, telemetry_event_subtype_t event_subtype, void *data_ptr,
						   size_t data_length) {
	if (handler != NULL) {
		// Pass event to handler
		handler(event_type, event_subtype, data_ptr, data_length);

		// Notify handler of event sent
		handler(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_TELEMETRY_EVENT_SENT, NULL, 0);
	}

	// Only process events when WiFi enabled
	#ifdef CONFIG_WIFI_ENABLED
		xSemaphoreTake(packet_write_sem, portMAX_DELAY);

		if ((PACKET_DATA_BUFFER_SIZE - tx_packet->data_length) < (sizeof(telemetry_event_t) + data_length)) {
			ESP_LOGW(TAG, "Could not write event %d subtype %d, buffer full", event_type, event_subtype);

			xSemaphoreGive(packet_write_sem);
			return false;
		}

		telemetry_event_t event = {
				.type = event_type,
				.subtype = event_subtype,
				.data_length = (uint8_t) data_length,
				.time_since_packet = (uint8_t) (esp_timer_get_time() - tx_packet->time_since_boot)
		};
		void *write_ptr = tx_packet->data + tx_packet->data_length;
		tx_packet->event_count++;
		tx_packet->data_length += sizeof(telemetry_event_t) + data_length;
		memcpy(write_ptr, &event, sizeof(telemetry_event_t));
		if (data_length > 0) {
			memcpy(write_ptr + sizeof(telemetry_event_t), data_ptr, data_length);
		}

		xSemaphoreGive(packet_write_sem);

		return true;
	#else
		return false;
	#endif
}

void telemetry_process_message(telemetry_message_type_t message_type, telemetry_message_subtype_t message_subtype, void *data_ptr,
		size_t data_length) {
	if (handler != NULL) {
		// Notify handler of message received
		handler(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_TELEMETRY_MESSAGE_RECEIVED, NULL, 0);
	}

	if (message_type == MESSAGE_TYPE_GPS) {
		if (message_subtype == MESSAGE_TYPE_GPS_UBX || message_subtype == MESSAGE_TYPE_GPS_RTCM) {
			gps_send_message((uint8_t *) (data_ptr), data_length);
		}
	}
}

void telemetry_socket_open() {
	server_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (server_socket < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		return;
	}

	// Start send/receive tasks
	xTaskCreatePinnedToCore(telemetry_send_task, "telemetry_send_task", 2048, NULL, PRIORITY_TASK_TELEMETRY, &telemetry_send_task_handle, APP_CPU_NUM);
	xTaskCreatePinnedToCore(telemetry_receive_task, "telemetry_receive_task", 2048, NULL, PRIORITY_TASK_TELEMETRY, &telemetry_receive_task_handle, APP_CPU_NUM);
}

void telemetry_socket_close() {
	closesocket(server_socket);
	server_socket = -1;

	// Stop send/receive tasks
	vTaskDelete(telemetry_send_task_handle);
	vTaskDelete(telemetry_receive_task_handle);
}

void telemetry_send_task() {
	while (true) {
		// Lock
		xSemaphoreTake(packet_write_sem, portMAX_DELAY);

		// Calculate checksum
		tx_packet->data_checksum = crc32_le(0, tx_packet->data, tx_packet->data_length);

		// Send data
		int err = sendto(server_socket, tx_packet, sizeof(telemetry_tx_packet_t) + tx_packet->data_length,
				0, (struct sockaddr *) &server_addr, sizeof(server_addr));
		if (err < 0) {
			ESP_LOGE(TAG, "Error occurred while sending: errno %d", errno);
			continue;
		}
		ESP_LOGD(TAG, "Sent packet with %d events, data length %d", tx_packet->event_count, tx_packet->data_length);

		// Prepare next packet
		tx_packet->time_since_boot = (uint64_t) esp_timer_get_time();
		tx_packet->event_count = 0;
		tx_packet->data_length = 0;
		tx_packet->data_checksum = 0;

		// Unlock
		xSemaphoreGive(packet_write_sem);

		// Indicate to handler that packet was sent
		if (handler != NULL) {
			handler(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_TELEMETRY_PACKET_SENT, NULL, 0);
		}

		do {
			vTaskDelay(1000 / TELEMETRY_REFRESH_RATE / portTICK_PERIOD_MS);
		} while (tx_packet->event_count == 0);
	}
}

void telemetry_receive_task() {
	while (true) {
		socklen_t socklen = sizeof(server_addr);
		int read = recvfrom(server_socket, rx_packet, sizeof(telemetry_rx_packet_t) + PACKET_DATA_BUFFER_SIZE,
				0, (struct sockaddr *) &server_addr, &socklen);
		if (read < 0) {
			ESP_LOGE(TAG, "Error occurred while receiving: errno %d", errno);
			continue;
		}

		if (rx_packet->protocol != PROTOCOL_HEADER_BIT || read < sizeof(telemetry_rx_packet_t)) {
			ESP_LOGW(TAG, "Unknown packet format received");
			continue;
		}

		if (rx_packet->protocol_version != PROTOCOL_VERSION) {
			ESP_LOGW(TAG, "Packet received with incorrect protocol version. Expected %d, got %d", PROTOCOL_VERSION, rx_packet->protocol_version);
			continue;
		}

		if (read < (sizeof(telemetry_rx_packet_t) + rx_packet->data_length)) {
			ESP_LOGW(TAG, "Packet received with missing data. Expected %d bytes, got %d bytes",
					(int) (sizeof(telemetry_rx_packet_t) + rx_packet->data_length), read);
			continue;
		}

		ESP_LOGI(TAG, "Received packet with %d messages, data length %d", rx_packet->message_count, rx_packet->data_length);

		// Process messages
		size_t offset = 0;
		for (int i = 0; i < rx_packet->message_count; i++) {
			// Memcpy to realign
			telemetry_message_t message;
			memcpy(&message, rx_packet + offset, sizeof(telemetry_message_t));

			offset += sizeof(telemetry_message_t);

			telemetry_process_message(message.type, message.subtype, rx_packet + offset, message.data_length);
		}

		// Indicate to handler that packet was received
		if (handler != NULL) {
			handler(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_TELEMETRY_PACKET_RECEIVED, NULL, 0);
		}
	}
}

void telemetry_set_handler(void (*h)(telemetry_event_type_t, telemetry_event_subtype_t, void *, size_t)) {
	handler = h;
}