#ifndef UCDFS_TELEMETRY_EVENTS_H
#define UCDFS_TELEMETRY_EVENTS_H

#include <stdint.h>
#include <stdlib.h>

#define PROTOCOL_PACKET_BUFFER_SIZE 2048

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
	uint8_t data[PROTOCOL_PACKET_BUFFER_SIZE];
} telemetry_packet_t;

typedef enum
{
	EVENT_TYPE_SYSTEM = 0x00,
	EVENT_TYPE_CAR = 0x01,
	EVENT_TYPE_BATTERY = 0x02,
	EVENT_TYPE_MOTOR = 0x03,
	EVENT_TYPE_SENSOR = 0x04,
	EVENT_TYPE_CONTROL = 0x05,
	EVENT_TYPE_POSITION = 0x06,
} telemetry_event_type_t;

typedef enum
{
	EVENT_TYPE_MOTOR_WHEEL_SPEED = 0x02,
	EVENT_TYPE_SENSOR_ACCELEROMETER = 0x00,
	EVENT_TYPE_POSITION_GPS_COORDINATES = 0x00,
	EVENT_TYPE_POSITION_GPS_RAW = 0x80
} telemetry_event_subtype_t;

typedef struct
{
	telemetry_event_type_t type			:8;
	telemetry_event_subtype_t subtype	:8;
	uint16_t data_length				:16;
	uint32_t time_since_packet			:32;
} telemetry_event_t;

void telemetry_write_event(telemetry_event_type_t event_type, telemetry_event_subtype_t event_subtype, void *data_ptr,
						   size_t data_length);

void telemetry_send_task();

#endif //UCDFS_TELEMETRY_EVENTS_H