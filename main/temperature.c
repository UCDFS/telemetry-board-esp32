#include <stdbool.h>
#include <esp_log.h>
#include <math.h>

#include "bme280.h"
#include "i2c.h"
#include "telemetry.h"
#include "temperature.h"

#define DEBUG_TEMPERATURE false

static const char* TAG = "temperature";

void temperature_read_task()
{
	// Initialize sensor
	bme280_handle_t bme280_dev = bme280_init(i2c_bus_get(), BME280_I2C_ADDR_1);

	if (bme280_dev == NULL) {
		vTaskDelete(NULL);
	}

	ESP_ERROR_CHECK(bme280_setup(bme280_dev, BME280_MODE_NORMAL, BME280_STANDBY_500MS, BME280_IIR_FILTER_OFF,
			BME280_SAMPLING_2X, BME280_SAMPLING_2X, BME280_SAMPLING_2X));

	bme280_float_data_t data;
	while (true) {
		// Read values
		bme280_get_float_data(bme280_dev, &data);

		if (DEBUG_TEMPERATURE) {
			ESP_LOGI(TAG, "temp: %4.2f°C, press: %5.1fmb, hum: %4.1f%%", data.temperature, data.pressure, data.humidity);
		}

		// Write event
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_AMBIENT_TPH, &data, sizeof(data));

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}