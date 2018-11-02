#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_wifi.h>

#include <driver/uart.h>

#include "telemetry.h"

#define GPS_UART UART_NUM_2
#define GPS_TXD GPIO_NUM_16
#define GPS_RXD GPIO_NUM_17
#define GPS_BUFFER_SIZE 256

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
		uint16_t remaining = (uint16_t) ((data[4] | data[5] << 8) + 2);
		if (uart_read_bytes(GPS_UART, data + 6, remaining, 10 / portTICK_RATE_MS) != remaining) {
			// TODO: Error?
			continue;
		}

		// Write event
		telemetry_write_event(EVENT_TYPE_POSITION, EVENT_TYPE_POSITION_GPS_RAW, data, 6 + remaining);
	}
}