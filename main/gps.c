#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_wifi.h>

#include <driver/uart.h>
#include <esp_log.h>

#include "telemetry.h"

#define GPS_UART UART_NUM_2
#define GPS_TXD GPIO_NUM_16
#define GPS_RXD GPIO_NUM_17
#define GPS_BUFFER_SIZE 256

#define UBX_SYNC_CHAR_1 0xb5
#define UBX_SYNC_CHAR_2 0x62

typedef struct {
	uint8_t sync_char_1;
	uint8_t sync_char_2;
	uint8_t class;
	uint8_t id;
	uint16_t length;
	uint8_t data[GPS_BUFFER_SIZE];
} ubx_message_t;

static const char* TAG = "gps";

void gps_read_task()
{
	// Configure GPS UART
	uart_config_t uart_config = {
			.baud_rate 	= 115200,
			.data_bits 	= UART_DATA_8_BITS,
			.parity 	= UART_PARITY_DISABLE,
			.stop_bits 	= UART_STOP_BITS_1,
			.flow_ctrl 	= UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(GPS_UART, &uart_config);
	uart_set_pin(GPS_UART, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(GPS_UART, GPS_BUFFER_SIZE, 0, 0, NULL, 0);

	ubx_message_t ubx_message;
	while (true) {
		if (uart_read_bytes(GPS_UART, (uint8_t *) &ubx_message, sizeof(ubx_message), 1000 / portTICK_PERIOD_MS) < 6) {
			continue;
		}

		// Check for µb header
		if (ubx_message.sync_char_1 != UBX_SYNC_CHAR_1 || ubx_message.sync_char_2 != UBX_SYNC_CHAR_2) {
			// Throw away remaining
			uart_flush_input(GPS_UART);
		}

		// Payload length + checksum size
		uint32_t remaining = ubx_message.length + 2;
		int read;
		if ((read = uart_read_bytes(GPS_UART, ubx_message.data, remaining, 10 / portTICK_PERIOD_MS)) != remaining) {
			ESP_LOGE(TAG, "Failed to complete reading UBX message, expected %d bytes, got %d bytes", remaining, read);
			continue;
		}

		// Verify checksum
		uint8_t message_ck_a = ubx_message.data[ubx_message.length];
		uint8_t message_ck_b = ubx_message.data[ubx_message.length + 1];

		uint8_t ck_a = 0;
		uint8_t ck_b = 0;
		for (uint8_t *buffer = &ubx_message.class; buffer < &ubx_message.data[ubx_message.length]; buffer++) {
			ck_a += *buffer;
			ck_b += ck_a;
		}

		if (message_ck_a != ck_a || message_ck_b != ck_b) {
			ESP_LOGE(TAG, "Corrupt message received, expected ck_a = %#02X, ck_b = %#02X got ck_a = %#02X, ck_b = %#02X",
					ck_a, ck_b, message_ck_a, message_ck_b);
			continue;
		}

		// Write event
		telemetry_write_event(EVENT_TYPE_POSITION, EVENT_TYPE_POSITION_GPS_RAW, &ubx_message, ubx_message.length + 8);
	}
}