#include <esp_log.h>
#include <tcpip_adapter.h>
#include <esp_event_loop.h>
#include <esp_wifi.h>
#include <string.h>
#include "wifi.h"
#include "telemetry.h"
#include "status_display.h"
#include "task_priorities.h"

#define WIFI_STRENGTH_REPORT_INTERVAL 15000

static void wifi_strength_report_task(void *ctx);

static TaskHandle_t wifi_strength_task_handle = NULL;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
		case SYSTEM_EVENT_STA_START: {
			esp_wifi_connect();
			break;
		}
		case SYSTEM_EVENT_STA_CONNECTED: {
			// Write event
			system_event_sta_connected_t *connected = &event->event_info.connected;
			t_ev_sys_wifi_connected t_ev;
			memcpy(t_ev.ssid, connected->ssid, sizeof(t_ev.ssid));
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_CONNECTED, &t_ev, sizeof(t_ev));

			// Start wifi strength report task
			xTaskCreatePinnedToCore(wifi_strength_report_task, "wifi_strength_task", 1024, NULL, PRIORITY_TASK_WIFI_STRENGTH,
					&wifi_strength_task_handle, APP_CPU_NUM);
			break;
		}
		case SYSTEM_EVENT_STA_DISCONNECTED: {
			// Write event
			system_event_sta_disconnected_t *disconnected = &event->event_info.disconnected;
			t_ev_sys_wifi_disconnected t_ev;
			memcpy(t_ev.ssid, disconnected->ssid, sizeof(t_ev.ssid));
			t_ev.reason = disconnected->reason;
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_CONNECTED, &t_ev, sizeof(t_ev));

			// Stop wifi strength report task
			vTaskDelete(wifi_strength_task_handle);
			break;
		}
		case SYSTEM_EVENT_STA_GOT_IP: {
			// Write event
			system_event_sta_got_ip_t *got_ip = &event->event_info.got_ip;
			t_ev_sys_wifi_got_ip t_ev;
			t_ev.addr = got_ip->ip_info.ip.addr;
			t_ev.changed = got_ip->ip_changed;
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_GOT_IP, &t_ev, sizeof(t_ev));

			// Start HTTP server
			status_display_http_server_init();

			// Open socket to telemetry server
			telemetry_socket_open();
			break;
		}
		case SYSTEM_EVENT_STA_LOST_IP: {
			// Write event
			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_LOST_IP, NULL, 0);

			// Stop HTTP server
			status_display_http_server_stop();

			// Close socket to telemetry server
			telemetry_socket_close();
		}
		default:
			break;
	}
	return ESP_OK;
}

void wifi_init()
{
	esp_log_level_set("wifi", ESP_LOG_WARN);

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

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

static void wifi_strength_report_task(void *ctx) {
	while (true) {
		wifi_ap_record_t wifi_ap_record;
		if (esp_wifi_sta_get_ap_info(&wifi_ap_record) == ESP_OK) {
			t_ev_sys_wifi_signal_strength t_ev;
			memcpy(t_ev.ssid, wifi_ap_record.ssid, sizeof(t_ev.ssid));
			t_ev.rssi = wifi_ap_record.rssi;

			telemetry_write_event(EVENT_TYPE_SYSTEM, EVENT_TYPE_SYSTEM_WIFI_SIGNAL_STRENGTH, &t_ev, sizeof(t_ev));
		};

		vTaskDelay(WIFI_STRENGTH_REPORT_INTERVAL / portTICK_PERIOD_MS);
	}
}