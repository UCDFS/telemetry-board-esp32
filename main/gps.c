#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_wifi.h>

#include <driver/uart.h>
#include <esp_log.h>
#include <ubx_tools.h>
#include <math.h>

#include "ubx_protocol.h"

#include "telemetry.h"
#include "task_priorities.h"

#define GPS_UART UART_NUM_2
#define GPS_TXD GPIO_NUM_17
#define GPS_RXD GPIO_NUM_16
#define GPS_BUFFER_SIZE 8192

#define UBX_SYNC_CHAR_1 0xb5
#define UBX_SYNC_CHAR_2 0x62

// Reset
static const ubx_cfg_rst_t COMMAND_RESET = {
		.navBbrMask = 0x0000,
		.resetMode = 0x01
};

// Prevent receiver from sending directly to radio
static const ubx_cfg_val_val_t CONFIG_DISABLE_UART2 = {
		.key = CFG_UART2_ENABLED,
		.val = false,
		.size = 1
};

static const char* TAG = "gps";

static bool gps_ready = false;

void hexdump(uint8_t *data, size_t size) {
	for (int i = 0; i < size; i++) {
		if (i % 12 == 0 && i > 0) {
			printf("\n");
		}
		printf("%02X ", *(data + i));
	}
	printf("\n");
}

void gps_receive_task();

void gps_init() {
	// Configure GPS UART
	uart_config_t uart_config = {
			.baud_rate 	= 460800,
			.data_bits 	= UART_DATA_8_BITS,
			.parity 	= UART_PARITY_DISABLE,
			.stop_bits 	= UART_STOP_BITS_1,
			.flow_ctrl 	= UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(GPS_UART, &uart_config);
	uart_set_pin(GPS_UART, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(GPS_UART, GPS_BUFFER_SIZE, GPS_BUFFER_SIZE, 0, NULL, 0);

	size_t command_size;
	char *command;

	// Reset module
	command = ubx_message_generate(UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_RST,
			(uint8_t *) &COMMAND_RESET, sizeof(COMMAND_RESET), &command_size);
	uart_write_bytes(GPS_UART, command, command_size);
	free(command);

	// Wait for module to reset
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Clear buffer of any messages from before reset
	uart_flush(GPS_UART);

	// Prevent receiver from sending directly to radio
	command = ubx_message_cfg_val_set_generate(1, &CONFIG_DISABLE_UART2, UBX_CFG_LAYERS_RAM, &command_size);
	uart_write_bytes(GPS_UART, command, command_size);
	free(command);

	// Start GPS receive task
	xTaskCreatePinnedToCore(gps_receive_task, "gps_receive_task", 4096, NULL, PRIORITY_TASK_GPS, NULL, APP_CPU_NUM);

	gps_ready = true;
}

bool gps_send_message(uint8_t *data, size_t length) {
	if (!gps_ready) {
		return false;
	}

	uart_write_bytes(GPS_UART, (const char *) data, length);

	return true;
}

void gps_receive_task()
{
	// Read messages
	uint8_t message[UBX_MESSAGE_MAX_LENGTH];
	size_t message_length;
	ubx_header_t *ubx_header = (ubx_header_t *) message;
	uint8_t *payload = message + sizeof(ubx_header_t);
	ubx_checksum_t *checksum;
	ubx_checksum_t checksum_verify;
	int read;
	while (true) {
		// Read header
		uart_read_bytes(GPS_UART, (uint8_t *) ubx_header, sizeof(ubx_header_t), portMAX_DELAY);

		// Check for µb header
		if (ubx_header->sync_char_1 != UBX_SYNC_CHAR_1 || ubx_header->sync_char_2 != UBX_SYNC_CHAR_2) {
			ESP_LOGE(TAG, "Unknown message format received, flushing");
			ESP_LOGE(TAG, "%02x %02x %02x %02x %02x %02x",
					message[0], message[1], message[2], message[3], message[4], message[5]);

			// Throw away remaining
			uart_flush(GPS_UART);
			continue;
		}

		ESP_LOGD(TAG, "Detected UBX message 0x%02x / 0x%02x with %d byte payload",
				ubx_header->message_class, ubx_header->message_id, ubx_header->payload_length);

		// Verify payload size
		if (ubx_header->payload_length > UBX_MESSAGE_PAYLOAD_MAX_LENGTH) {
			ESP_LOGE(TAG, "UBX message payload reported too large, flushing");
			ESP_LOGE(TAG, "%02x %02x %02x %02x %02x %02x",
					message[0], message[1], message[2], message[3], message[4], message[5]);

			// Throw away remaining
			uart_flush(GPS_UART);
			continue;
		}

		// Read payload
		if ((read = uart_read_bytes(GPS_UART, payload, ubx_header->payload_length, 1000 / portTICK_PERIOD_MS)) < ubx_header->payload_length) {
			ESP_LOGE(TAG, "Failed to read UBX message payload, expected %d bytes, got %d bytes", ubx_header->payload_length, read);
			continue;
		}

		// Update message size
		message_length = sizeof(ubx_header_t) + ubx_header->payload_length + sizeof(ubx_checksum_t);

		// Move checksum to end of payload
		checksum = (ubx_checksum_t *) (payload + ubx_header->payload_length);

		// Read checksum
		if ((read = uart_read_bytes(GPS_UART, (uint8_t *) checksum, sizeof(ubx_checksum_t), 1000 / portTICK_PERIOD_MS)) < sizeof(ubx_checksum_t)) {
			ESP_LOGE(TAG, "Failed to read UBX message checksum, expected %d bytes, got %d bytes", (int) sizeof(ubx_checksum_t), read);
			continue;
		}

		// Calculate checksum
		ubx_message_calculate_checksum(&ubx_header->message_class,
				(uint32_t) checksum - (uint32_t) &ubx_header->message_class, &checksum_verify);

		// Verify checksum
		if (checksum->ck_a != checksum_verify.ck_a || checksum->ck_b != checksum_verify.ck_b) {
			ESP_LOGE(TAG, "Corrupt message received, expected ck_a = 0x%02x, ck_b = 0x%02x got ck_a = 0x%02x, ck_b = 0x%02x",
					checksum_verify.ck_a, checksum_verify.ck_b, checksum->ck_a, checksum->ck_b);
			hexdump(message, message_length);
		}

		// Process messages
		if (ubx_header->message_class == UBX_MESSAGE_CLASS_NAV) {
			if (ubx_header->message_id == UBX_MESSAGE_ID_NAV_PVT) {
				// Memcpy to realign
				ubx_nav_pvt_t nav_pvt;
				memcpy(&nav_pvt, payload, ubx_header->payload_length);

				t_ev_pos_gps_pvt t_ev;
				t_ev.latitude = nav_pvt.lat;
				t_ev.longitude = nav_pvt.lon;
				t_ev.height = nav_pvt.height;
				t_ev.horizontal_accuracy = nav_pvt.hAcc;
				t_ev.vertical_accuracy = nav_pvt.vAcc;
				t_ev.ground_speed = nav_pvt.gSpeed;
				t_ev.speed_accuracy = nav_pvt.sAcc;
				t_ev.heading = nav_pvt.headMot;
				t_ev.heading_accuracy = nav_pvt.headAcc;
				switch (nav_pvt.fixType) {
					case 0:
					case 1:
					case 5:
					default:
						t_ev.fix_mode = GPS_FIX_MODE_NO_FIX;
						break;
					case 2:
						t_ev.fix_mode = GPS_FIX_MODE_2D;
						break;
					case 3:
					case 4:
						t_ev.fix_mode = GPS_FIX_MODE_3D;
						break;
				}
				if (!nav_pvt.flags.diffSoln) {
					t_ev.dgnss_mode = GPS_DGNSS_MODE_NONE;
				} else {
					switch (nav_pvt.flags.carrSoln) {
						case 0:
						default:
							t_ev.dgnss_mode = GPS_DGNSS_MODE_NO_SOL;
							break;
						case 1:
							t_ev.dgnss_mode = GPS_DGNSS_MODE_FLOAT;
							break;
						case 2:
							t_ev.dgnss_mode = GPS_DGNSS_MODE_INT;
							break;
					}
				}

				// Write event
				telemetry_write_event(EVENT_TYPE_POSITION, EVENT_TYPE_POSITION_GPS_COORDINATES, &t_ev, sizeof(t_ev));
			} else if (ubx_header->message_id == UBX_MESSAGE_ID_NAV_SAT) {
				// Memcpy to realign
				ubx_nav_sat_t nav_sat;
				ubx_nav_sat_sat_t nav_sat_sat;
				memcpy(&nav_sat, payload, ubx_header->payload_length);

				t_ev_pos_gps_sats *t_ev = malloc(sizeof(t_ev_pos_gps_sats) + nav_sat.numSvs * sizeof(t_ev_pos_gps_sat));
				t_ev->count = nav_sat.numSvs;

				for (int i = 0; i < nav_sat.numSvs; i++) {
					// Memcpy to realign
					memcpy(&nav_sat_sat, payload + sizeof(ubx_nav_sat_t) + i * sizeof(ubx_nav_sat_sat_t), sizeof(ubx_nav_sat_sat_t));

					t_ev->satellites[i].gnss_id = nav_sat_sat.gnssId;
					t_ev->satellites[i].sv_id = nav_sat_sat.svId;
					t_ev->satellites[i].signal_strength = nav_sat_sat.cno;
					t_ev->satellites[i].elevation = nav_sat_sat.elev;
					t_ev->satellites[i].azimuth = nav_sat_sat.azim;
				}

				// Write event
				telemetry_write_event(EVENT_TYPE_POSITION, EVENT_TYPE_POSITION_GPS_SATELLITES, &t_ev, sizeof(t_ev));
			}
		}
	}
}