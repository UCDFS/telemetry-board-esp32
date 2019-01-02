#ifndef UCDFS_TELEMETRY_EVENTS_H
#define UCDFS_TELEMETRY_EVENTS_H

#include <stdint.h>
#include <stdlib.h>
#include <esp_event_legacy.h>

typedef enum
{
	EVENT_TYPE_SYSTEM = 0,
	EVENT_TYPE_CAR,
	EVENT_TYPE_BATTERY,
	EVENT_TYPE_MOTOR,
	EVENT_TYPE_SENSOR,
	EVENT_TYPE_CONTROL,
	EVENT_TYPE_POSITION,
} telemetry_event_type_t;

typedef enum
{
	EVENT_TYPE_SYSTEM_STARTED = 0x00,
	EVENT_TYPE_SYSTEM_STOPPED,
	EVENT_TYPE_SYSTEM_WIFI_CONNECTED,
	EVENT_TYPE_SYSTEM_WIFI_DISCONNECTED,
	EVENT_TYPE_SYSTEM_WIFI_GOT_IP,
	EVENT_TYPE_SYSTEM_WIFI_LOST_IP,
	EVENT_TYPE_SYSTEM_WIFI_SIGNAL_STRENGTH,
	EVENT_TYPE_SYSTEM_TELEMETRY_PACKET_SENT = 0x80,
	EVENT_TYPE_SYSTEM_TELEMETRY_PACKET_RECEIVED,
	EVENT_TYPE_SYSTEM_TELEMETRY_EVENT_SENT,
	EVENT_TYPE_SYSTEM_TELEMETRY_MESSAGE_RECEIVED,

	EVENT_TYPE_CAR_MOVING = 0x00,
	EVENT_TYPE_CAR_DRIVER_SELECTED = 0x08,
	EVENT_TYPE_CAR_LAP_COMPLETED,
	EVENT_TYPE_CAR_ICE_MODE_STATUS = 0x80,

	EVENT_TYPE_BATTERY_VOLTAGE = 0x00,
	EVENT_TYPE_BATTERY_TEMPERATURE,
	EVENT_TYPE_BATTERY_TEMPERATURE_HIGH,
	EVENT_TYPE_BATTERY_TEMPERATURE_CRITICAL,

	EVENT_TYPE_MOTOR_RPM = 0x00,
	EVENT_TYPE_MOTOR_SPEED,
	EVENT_TYPE_MOTOR_WHEEL_SPEED,

	EVENT_TYPE_SENSOR_ACCELEROMETER = 0x00,
	EVENT_TYPE_SENSOR_GYROSCOPE,
	EVENT_TYPE_SENSOR_TYRE_TEMPERATURE,
	EVENT_TYPE_SENSOR_BRAKE_TEMPERATURE,
	EVENT_TYPE_SENSOR_AMBIENT_TPH = 0x08,

	EVENT_TYPE_CONTROL_THROTTLE_BRAKE_STEERING = 0x00,

	EVENT_TYPE_POSITION_GPS_COORDINATES = 0x00,
	EVENT_TYPE_POSITION_GPS_SATELLITES = 0x01,
	EVENT_TYPE_POSITION_GPS_RAW_UBX = 0x80
} telemetry_event_subtype_t;

typedef enum {
	MESSAGE_TYPE_GPS = 0
} telemetry_message_type_t;

typedef enum {
	MESSAGE_TYPE_GPS_UBX = 0x00,
	MESSAGE_TYPE_GPS_RTCM
} telemetry_message_subtype_t;

typedef struct {
	char ssid[32];
} t_ev_sys_wifi_connected;

typedef struct {
	char ssid[32];
	uint8_t reason;
} t_ev_sys_wifi_disconnected;

typedef struct {
	uint32_t addr;
	bool changed;
} t_ev_sys_wifi_got_ip;

typedef struct {
	uint8_t ssid[32];
	int8_t rssi;
} t_ev_sys_wifi_signal_strength;

typedef struct {
	float ax;
	float ay;
	float az;
} t_ev_sen_accelerometer;

typedef struct {
	float gx;
	float gy;
	float gz;
} t_ev_sen_gyroscope;

typedef struct {
	float brake_fl;
	float brake_fr;
	float brake_rl;
	float brake_rr;
} t_ev_sen_brake_temperature;

typedef struct {
	float temperature;
	float pressure;
	float humidity;
} t_ev_sen_ambient_tph;

enum t_ev_pos_gps_pvt_fix_mode {
	GPS_FIX_MODE_NO_FIX = 0,
	GPS_FIX_MODE_2D = 1,
	GPS_FIX_MODE_3D = 2
};

enum t_ev_pos_gps_pvt_dgnss_mode {
	GPS_DGNSS_MODE_NONE = 0,
	GPS_DGNSS_MODE_NO_SOL = 1,
	GPS_DGNSS_MODE_FLOAT = 2,
	GPS_DGNSS_MODE_INT = 3
};

typedef struct {
	int32_t latitude;
	int32_t longitude;
	int32_t height;
	uint32_t horizontal_accuracy;
	uint32_t vertical_accuracy;
	int32_t ground_speed;
	uint32_t speed_accuracy;
	int32_t heading;
	uint32_t heading_accuracy;
	uint8_t fix_mode;
	uint8_t dgnss_mode;
} t_ev_pos_gps_pvt;

typedef struct {
	uint8_t gnss_id;
	uint8_t sv_id;
	uint8_t signal_strength;
	int8_t elevation;
	int16_t azimuth;
	bool used;
} t_ev_pos_gps_sat;

typedef struct {
	uint8_t count;
	t_ev_pos_gps_sat satellites[];
} t_ev_pos_gps_sats;

void telemetry_init();

bool telemetry_write_event(telemetry_event_type_t event_type, telemetry_event_subtype_t event_subtype, void *data_ptr,
						   size_t data_length);

void telemetry_socket_open();
void telemetry_socket_close();

void telemetry_set_handler(void (*h)(telemetry_event_type_t, telemetry_event_subtype_t, void *, size_t));

#endif //UCDFS_TELEMETRY_EVENTS_H