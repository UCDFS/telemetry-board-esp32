#include <esp_http_server.h>
#include <math.h>
#include <lwip/inet.h>
#include "status_display.h"

#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "telemetry.h"
#include "temperature.h"

#define DEBUG_STATUS_DISPLAY true

enum display_mode_t {
	DISPLAY_MODE_DEFAULT,
	DISPLAY_MODE_ACCELEROMETER,
	DISPLAY_MODE_BRAKE,
	DISPLAY_MODE_GPS,
	DISPLAY_MODE_TELEMETRY,
	DISPLAY_MODE_TEMPERATURE,
	DISPLAY_MODE_WHEEL_SPEED
};

enum display_mode_t display_mode = DISPLAY_MODE_DEFAULT;

static const char* TAG = "status_display";

static ssd1306_handle_t display;

void status_display_event_handler(telemetry_event_type_t event_type, telemetry_event_subtype_t event_subtype, void *data_ptr,
		size_t data_length);

void status_display_init() {
	// Initialize display
	display = ssd1306_init(i2c_bus_get(), SSD1306_I2C_ADDR_1);

	telemetry_set_handler(status_display_event_handler);
}

static char buffer[128];
static bool wifi_connected;
static char wifi_ssid[32];
static uint32_t ip;

static int telemetry_events;
static int telemetry_packets;

static float gps_lat;
static float gps_lon;
static float gps_speed;
static float gps_acc;

static float acc_x;
static float acc_y;
static float acc_z;
static float gyro_x;
static float gyro_y;
static float gyro_z;

static int brake_temp_fl;
static int brake_temp_fr;
static int brake_temp_rl;
static int brake_temp_rr;

static float ambient_temperature;
static float ambient_pressure;
static float ambient_humidity;

void status_display_task() {
	if (display == NULL) {
		vTaskDelete(NULL);
	}

	while (true) {
		int delay = 1000;

		ssd1306_clear_screen(display, 0x00);

		switch (display_mode) {
			case DISPLAY_MODE_DEFAULT: {
				ssd1306_draw_string(display, 0, 0, "Welcome to", 12, true);
				ssd1306_draw_string(display, 0, 12, "UCDFS Telemetry", 16, true);
				ssd1306_fill_rectangle(display, 0, 30, 120, 30, true);

				sprintf(buffer, "WiFi: %s", wifi_connected ? wifi_ssid : "Not Connected");
				ssd1306_draw_string(display, 0, 40, buffer, 12, true);
				if (wifi_connected) {
					sprintf(buffer, "IP: %s", inet_ntoa(ip));
					ssd1306_draw_string(display, 0, 52, buffer, 12, true);
				}
				break;
			}
			case DISPLAY_MODE_ACCELEROMETER: {
				ssd1306_draw_string(display, 0, 0, "Accelerometer & Gyro", 12, true);
				ssd1306_fill_rectangle(display, 0, 12, 127, 12, true);

				sprintf(buffer, "AX: %4.1f", acc_x);
				ssd1306_draw_string(display, 0, 16, buffer, 12, true);
				sprintf(buffer, "AY: %4.1f", acc_y);
				ssd1306_draw_string(display, 0, 28, buffer, 12, true);
				sprintf(buffer, "AZ: %4.1f", acc_z);
				ssd1306_draw_string(display, 0, 40, buffer, 12, true);

				sprintf(buffer, "GX: %4.1f", gyro_x);
				ssd1306_draw_string(display, 64, 16, buffer, 12, true);
				sprintf(buffer, "GY: %4.1f", gyro_y);
				ssd1306_draw_string(display, 64, 28, buffer, 12, true);
				sprintf(buffer, "GZ: %4.1f", gyro_z);
				ssd1306_draw_string(display, 64, 40, buffer, 12, true);
				delay = 20;
				break;
			}
			case DISPLAY_MODE_BRAKE: {
				ssd1306_draw_string(display, 0, 0, "Brake Temperature", 12, true);
				ssd1306_fill_rectangle(display, 0, 12, 127, 12, true);

				sprintf(buffer, "FL:%3dC", brake_temp_fl);
				ssd1306_draw_string(display, 0, 16, buffer, 12, true);
				sprintf(buffer, "FR:%3dC", brake_temp_fr);
				ssd1306_draw_string(display, 64, 16, buffer, 12, true);

				sprintf(buffer, "RL:%3dC", brake_temp_rl);
				ssd1306_draw_string(display, 0, 28, buffer, 12, true);
				sprintf(buffer, "RR:%3dC", brake_temp_rr);
				ssd1306_draw_string(display, 64, 28, buffer, 12, true);
				delay = 100;
				break;
			}
			case DISPLAY_MODE_GPS: {
				ssd1306_draw_string(display, 0, 0, "GPS", 12, true);
				ssd1306_fill_rectangle(display, 0, 12, 127, 12, true);

				sprintf(buffer, "LAT %10.6f%c", fabsf(gps_lat), gps_lat > 0 ? 'N' : 'S');
				ssd1306_draw_string(display, 0, 16, buffer, 12, true);
				sprintf(buffer, "LON %10.6f%c", fabsf(gps_lon), gps_lon > 0 ? 'W' : 'E');
				ssd1306_draw_string(display, 0, 28, buffer, 12, true);

				ssd1306_fill_rectangle(display, 106, 16, 106, 24, true);
				ssd1306_fill_rectangle(display, 102, 20, 110, 20, true);
				ssd1306_fill_rectangle(display, 102, 26, 110, 26, true);
				sprintf(buffer, "%5.2fm", fabsf(gps_acc));
				ssd1306_draw_string(display, 90, 28, buffer, 12, true);

				sprintf(buffer, "SPD %4.1fkm/h", fabsf(gps_speed));
				ssd1306_draw_string(display, 0, 40, buffer, 12, true);
				delay = 100;
				break;
			}
			case DISPLAY_MODE_TELEMETRY: {
				ssd1306_draw_string(display, 0, 0, "Telemetry", 12, true);
				ssd1306_fill_rectangle(display, 0, 12, 127, 12, true);

				sprintf(buffer, "Events: %d", telemetry_events);
				ssd1306_draw_string(display, 0, 16, buffer, 12, true);
				sprintf(buffer, "Packets: %d", telemetry_packets);
				ssd1306_draw_string(display, 0, 28, buffer, 12, true);
				delay = 25;
				break;
			}
			case DISPLAY_MODE_TEMPERATURE: {
				ssd1306_draw_string(display, 0, 0, "Ambient", 12, true);
				ssd1306_fill_rectangle(display, 0, 12, 127, 12, true);

				sprintf(buffer, "TEMP: %3.1fC", ambient_temperature);
				ssd1306_draw_string(display, 0, 16, buffer, 12, true);
				sprintf(buffer, "PRES: %4.1fmb", ambient_pressure);
				ssd1306_draw_string(display, 0, 28, buffer, 12, true);
				sprintf(buffer, "HUM: %3.0f%%", ambient_humidity);
				ssd1306_draw_string(display, 0, 40, buffer, 12, true);
				delay = 500;
				break;
			}
			case DISPLAY_MODE_WHEEL_SPEED: {
				break;
			}
		}

		ssd1306_refresh(display);

		vTaskDelay(delay / portTICK_PERIOD_MS);
	}
}

void status_display_event_handler(telemetry_event_type_t event_type, telemetry_event_subtype_t event_subtype, void *data_ptr,
		size_t data_length) {
	telemetry_events++;

	if (event_type == EVENT_TYPE_SYSTEM) {
		if (event_subtype == EVENT_TYPE_SYSTEM_TELEMETRY_PACKET_SENT) {
			telemetry_packets++;
			telemetry_events--;
		}

		if (event_subtype == EVENT_TYPE_SYSTEM_WIFI_CONNECTED) {
			t_ev_sys_wifi_connected *t_ev = (t_ev_sys_wifi_connected *) data_ptr;
			wifi_connected = true;
			strcpy(wifi_ssid, (const char *) t_ev->ssid);
		}

		if (event_subtype == EVENT_TYPE_SYSTEM_WIFI_DISCONNECTED) {
			wifi_connected = false;
		}

		if (event_subtype == EVENT_TYPE_SYSTEM_WIFI_GOT_IP) {
			t_ev_sys_wifi_got_ip *t_ev = (t_ev_sys_wifi_got_ip *) data_ptr;
			ip = t_ev->addr;
		}

		if (event_subtype == EVENT_TYPE_SYSTEM_WIFI_LOST_IP) {
			ip = 0;
		}
	}

	if (event_type == EVENT_TYPE_POSITION) {
		if (event_subtype == EVENT_TYPE_POSITION_GPS_COORDINATES) {
			t_ev_pos_gps_coordinates *t_ev = (t_ev_pos_gps_coordinates *) data_ptr;
			gps_lat = (float) (t_ev->latitude / 10000000.0);
			gps_lon = (float) (t_ev->longitude / 10000000.0);
			gps_speed = (float) (t_ev->groundSpeed / 277.778);
			gps_acc = (float) (t_ev->horizontalAccuracy / 1000.0);
		}
	}

	if (event_type == EVENT_TYPE_SENSOR) {
		if (event_subtype == EVENT_TYPE_SENSOR_ACCELEROMETER) {
			t_ev_sen_accelerometer *t_ev = (t_ev_sen_accelerometer *) data_ptr;
			acc_x = t_ev->ax;
			acc_y = t_ev->ay;
			acc_z = t_ev->az;
		}

		if (event_subtype == EVENT_TYPE_SENSOR_GYROSCOPE) {
			t_ev_sen_gyroscope *t_ev = (t_ev_sen_gyroscope *) data_ptr;
			gyro_x = t_ev->gx;
			gyro_y = t_ev->gy;
			gyro_z = t_ev->gz;
		}

		if (event_subtype == EVENT_TYPE_SENSOR_AMBIENT_TPH) {
			t_ev_sen_ambient_tph *t_ev = (t_ev_sen_ambient_tph *) data_ptr;
			ambient_temperature = t_ev->temperature;
			ambient_pressure = t_ev->pressure;
			ambient_humidity = t_ev->humidity;
		}

		if (event_subtype == EVENT_TYPE_SENSOR_BRAKE_TEMPERATURE) {
			t_ev_sen_brake_temperature *t_ev = (t_ev_sen_brake_temperature *) data_ptr;
			brake_temp_fl = (int) t_ev->brake_fl;
			brake_temp_fr = (int) t_ev->brake_fr;
			brake_temp_rl = (int) t_ev->brake_rl;
			brake_temp_rr = (int) t_ev->brake_rr;
		}
	}
}

static const char* PAGE_HTML = "<html>\n"
	"\t<head>\n"
	"\t\t<title>UCDFS Telemetry Configuration</title>\n"
	"\t\t<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no\"\\>\n"
	"\t\t<style>\n"
	"\t\t\tbody {\n"
	"\t\t\t\tposition: relative;\n"
	"\t\t\t}\n"
	"\n"
	"\t\t\ta {\n"
	"\t\t\t\tdisplay: block;\n"
	"\t\t\t\twidth: 50%;\n"
	"\t\t\t\tmargin: 5px auto;\n"
	"\t\t\t\tborder: 1px solid black;\n"
	"\t\t\t\tline-height: 30px;\n"
	"\t\t\t\ttext-align: center;\n"
	"\t\t\t\tfont-size: 16px;\n"
	"\t\t\t\tfont-weight: bold;\n"
	"\t\t\t\ttext-decoration: none;\n"
	"\t\t\t\tbackground-color: #ddd;\n"
	"\t\t\t\tcolor: black;\n"
	"\t\t\t}\n"
	"\n"
	"\t\t\t.buttons {\n"
	"\t\t\t\twidth: 100%;\n"
	"\t\t\t\tposition: absolute;\n"
	"\t\t\t\ttop: 50%;\n"
	"\t\t\t\ttransform: translateY(-50%);\n"
	"\t\t\t}\n"
	"\t\t</style>\n"
	"\t</head>\n"
	"\t<body>\n"
	"\t\t<div class=\"buttons\">\n"
	"\t\t\t<a href=\"/\">Default</a>\n"
	"\t\t\t<a href=\"/accelerometer\">Accelerometer</a>\n"
	"\t\t\t<a href=\"/brake\">Brake Temp</a>\n"
	"\t\t\t<a href=\"/gps\">GPS</a>\n"
	"\t\t\t<a href=\"/telemetry\">Telemetry</a>\n"
	"\t\t\t<a href=\"/temperature\">Temperature</a>\n"
	"\t\t\t<a href=\"/wheel_speed\">Wheel Speed</a>\n"
	"\t\t</div>\n"
	"\t</body>\n"
	"</html>";

esp_err_t status_display_http_handler(httpd_req_t *req) {
	if (strcmp(req->uri, "/") == 0) {
		display_mode = DISPLAY_MODE_DEFAULT;
	} else if (strcmp(req->uri, "/accelerometer") == 0) {
		display_mode = DISPLAY_MODE_ACCELEROMETER;
	} else if (strcmp(req->uri, "/brake") == 0) {
		display_mode = DISPLAY_MODE_BRAKE;
	} else if (strcmp(req->uri, "/gps") == 0) {
		display_mode = DISPLAY_MODE_GPS;
	} else if (strcmp(req->uri, "/telemetry") == 0) {
		display_mode = DISPLAY_MODE_TELEMETRY;
	} else if (strcmp(req->uri, "/temperature") == 0) {
		display_mode = DISPLAY_MODE_TEMPERATURE;
	} else if (strcmp(req->uri, "/wheel_speed") == 0) {
		display_mode = DISPLAY_MODE_WHEEL_SPEED;
	}

	if (display != NULL) ssd1306_clear_screen(display, 0x00);

	// Respond with index HTML
	httpd_resp_send(req, PAGE_HTML, strlen(PAGE_HTML));

	return ESP_OK;
}

httpd_handle_t server = NULL;

void status_display_http_server_init() {
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	httpd_uri_t http_uri = {
			.method    = HTTP_GET,
			.handler   = status_display_http_handler,
			.user_ctx  = NULL
	};

	if (httpd_start(&server, &config) == ESP_OK) {
		http_uri.uri = "/";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/accelerometer";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/brake";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/gps";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/i2c";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/telemetry";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/temperature";
		httpd_register_uri_handler(server, &http_uri);
		http_uri.uri = "/wheel_speed";
		httpd_register_uri_handler(server, &http_uri);
	}
}

void status_display_http_server_stop()
{
	httpd_stop(server);
}

ssd1306_handle_t status_display_get() {
	return display;
}


