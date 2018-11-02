#include <esp_err.h>
#include <esp_event_legacy.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
		case SYSTEM_EVENT_STA_START:
		case SYSTEM_EVENT_STA_DISCONNECTED:
			esp_wifi_connect();
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
			break;
		default:
			break;
	}
	return ESP_OK;
}

void wifi_init()
{
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
			.sta = {
					.ssid = CONFIG_WIFI_SSID,
					.password = CONFIG_WIFI_PASSWORD
			},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}
