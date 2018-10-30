#ifndef EVENTS_H
#define EVENTS_H

typedef enum {
	EVENT_TYPE_SYSTEM = 0x00,
	EVENT_TYPE_CAR = 0x01,
	EVENT_TYPE_BATTERY = 0x02,
	EVENT_TYPE_MOTOR = 0x03,
	EVENT_TYPE_SENSOR = 0x04,
	EVENT_TYPE_CONTROL = 0x05,
	EVENT_TYPE_POSITION = 0x06,
} event_type_t;

typedef enum {
	EVENT_TYPE_MOTOR_WHEEL_SPEED = 0x02,
	EVENT_TYPE_POSITION_GPS_COORDINATES = 0x00,
	EVENT_TYPE_POSITION_GPS_RAW = 0x80
} event_subtype_t;

typedef struct event_t {
	event_type_t type;
	event_subtype_t subtype;
	uint8_t data_length;
	uint8_t time_since_packet;
} event_t;

#endif