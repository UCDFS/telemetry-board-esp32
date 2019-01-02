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
#include "brake_temperature.h"
#include "gps.h"
#include "i2c.h"
#include "telemetry.h"
#include "temperature.h"
#include "wheel_speed.h"
#include "status_display.h"
#include "task_priorities.h"
#include "nextion_display.h"
#include "wifi.h"

#define STATUS_LED_GPIO GPIO_NUM_5
#define STATUS_LED_ON_TIME 100
#define STATUS_LED_OFF_TIME 1000

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

void app_main(void)
{
	// Initiate status led task
	xTaskCreatePinnedToCore(blink_status_led_task, "blink_status_led", 1024, NULL, PRIORITY_TASK_STATUS_LED, NULL,
			APP_CPU_NUM);

	// Initialize NVS
	#ifdef CONFIG_WIFI_ENABLED
		esp_err_t ret = nvs_flash_init();
		if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK(ret);
	#endif

	// Initialize GPIO
	gpio_install_isr_service(0);

	// Initialize telemetry system
	telemetry_init();

	#ifdef CONFIG_WIFI_ENABLED
		// Initialize WiFi
		wifi_init();
	#endif

	// Initialize I2C
	i2c_bus_init();

	// Initialize status display
	status_display_init();

	// Initialize nextion display
	nextion_display_init();

	// Initialize GPS
	gps_init();

	// Initiate accelerometer read task
	xTaskCreatePinnedToCore(accelerometer_read_task, "accelerometer_read_task", 2048, NULL, PRIORITY_TASK_ACCELEROMETER, NULL, APP_CPU_NUM);

	// Initiate temperature read task
	xTaskCreatePinnedToCore(temperature_read_task, "temperature_read_task", 2048, NULL, PRIORITY_TASK_TEMPERATURE, NULL, APP_CPU_NUM);

	// Initiate brake temperature read task
	xTaskCreatePinnedToCore(brake_temperature_read_task, "brake_temperature_read_task", 2048, NULL, PRIORITY_TASK_BRAKE_TEMPERATURE, NULL, APP_CPU_NUM);

	return;

	// Initiate wheel speed calculation
	xTaskCreatePinnedToCore(wheel_speed_calculation_task, "wheel_speed_calculation_task", 64, NULL,
							PRIORITY_TASK_WHEEL_SPEED, NULL, APP_CPU_NUM);
}

