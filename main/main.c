#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_event_legacy.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>

#include <driver/i2c.h>
#include <esp_log.h>
#include <string.h>
#include "i2c_bus.h"


#include "accelerometer.h"
#include "gps.h"
#include "i2c.h"
#include "telemetry.h"
#include "wheel_speed.h"

#define PRIORITY_TASK_TELEMETRY 10
#define PRIORITY_TASK_GPS 2
#define PRIORITY_TASK_ACCELEROMETER 1
#define PRIORITY_TASK_WHEEL_SPEED 1
#define PRIORITY_TASK_STATUS_LED 0

#define STATUS_LED_GPIO GPIO_NUM_13
#define STATUS_LED_ON_TIME 100
#define STATUS_LED_OFF_TIME 1000

#define DISPLAY_UART UART_NUM_1
#define DISPLAY_TXD GPIO_NUM_2
#define DISPLAY_RXD GPIO_NUM_4
#define DISPLAY_BUFFER_SIZE 32

static TaskHandle_t telemetry_task_handle = NULL;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
		case SYSTEM_EVENT_STA_START: {
			esp_wifi_connect();
			break;
		}
		case SYSTEM_EVENT_STA_CONNECTED: {
		    // Write event
            system_event_sta_connected_t *connected = &event->event_info.connected;
		    t_ev_sys_wifi_connected_t t_ev;
		    memcpy(t_ev.ssid, connected->ssid, sizeof(connected->ssid));
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_CONNECTED, &t_ev, sizeof(t_ev));
			break;
		}
		case SYSTEM_EVENT_STA_DISCONNECTED: {
		    // Write event
            system_event_sta_disconnected_t *disconnected = &event->event_info.disconnected;
            t_ev_sys_wifi_disconnected_t t_ev;
            memcpy(t_ev.ssid, disconnected->ssid, sizeof(disconnected->ssid));
            t_ev.reason = disconnected->reason;
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_CONNECTED, &t_ev, sizeof(t_ev));
			break;
		}
		case SYSTEM_EVENT_STA_GOT_IP: {
		    // Write event
		    system_event_sta_got_ip_t *got_ip = &event->event_info.got_ip;
		    t_ev_sys_wifi_got_ip t_ev;
		    t_ev.addr = got_ip->ip_info.ip.addr;
		    t_ev.changed = got_ip->ip_changed;
            telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_GOT_IP, &t_ev, sizeof(t_ev));

			// Initiate telemetry send task
			xTaskCreatePinnedToCore(telemetry_send_task, "telemetry_send_task", 2048, NULL, PRIORITY_TASK_TELEMETRY,
					&telemetry_task_handle, APP_CPU_NUM);
			break;
		}
		case SYSTEM_EVENT_STA_LOST_IP: {
			// Write event
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_LOST_IP, NULL, 0);

			// Stop telemetry send task
			vTaskDelete(telemetry_task_handle);
		}
		default:
			break;
	}
	return ESP_OK;
}

static void blink_status_led_task()
{
	// Configure status LED
	gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);

	while (true) {
		gpio_set_level(STATUS_LED_GPIO, true);
		vTaskDelay(STATUS_LED_ON_TIME / portTICK_PERIOD_MS);
		gpio_set_level(STATUS_LED_GPIO, false);
		vTaskDelay(STATUS_LED_OFF_TIME / portTICK_PERIOD_MS);
	}
}

void wifi_init()
{
	esp_log_level_set("wifi", ESP_LOG_WARN);

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
			.sta = {
					.ssid = CONFIG_WIFI_SSID,
					.password = CONFIG_WIFI_PASSWORD,
			},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
	// Initiate status led task
	xTaskCreatePinnedToCore(blink_status_led_task, "blink_status_led", 1024, NULL, PRIORITY_TASK_STATUS_LED, NULL,
			APP_CPU_NUM);

	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Initialize telemetry system
	telemetry_init();

	// Initialize WiFi
	wifi_init();

	// Initialize I2C
	i2c_bus_init();

	// Initiate accelerometer read task
	xTaskCreatePinnedToCore(accelerometer_read_task, "accelerometer_read_task", 2048, NULL, PRIORITY_TASK_ACCELEROMETER, NULL,
			APP_CPU_NUM);

	return;

	// Initiate GPS read task
	xTaskCreatePinnedToCore(gps_read_task, "gps_read_task", 256, NULL, PRIORITY_TASK_GPS, NULL, APP_CPU_NUM);

	// Initiate wheel speed calculation
	xTaskCreatePinnedToCore(wheel_speed_calculation_task, "wheel_speed_calculation_task", 64, NULL,
							PRIORITY_TASK_WHEEL_SPEED, NULL, APP_CPU_NUM);
}

