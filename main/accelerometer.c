#include <stdbool.h>
#include <esp_log.h>

#include "accelerometer.h"
#include "i2c.h"
#include "mma8451.h"
#include "telemetry.h"

#define ACCELEROMETER_REFRESH_RATE 50

void accelerometer_read_task()
{
	// Initialize MMA8451 accelerometer
	mma8451_handle_t mma8451_dev = mma8451_init(i2c_bus_get(), MMA8451_I2C_ADDR_2);
	if (mma8451_set_mode(mma8451_dev, MMA8451_HIGH_RES, MMA8451_DATA_RATE_50HZ, false, false) != ESP_OK) {
		vTaskDelete(NULL);
	};

	mma8451_float_data_t data;
	while (true) {
		// Read values
		mma8451_get_float_data(mma8451_dev, &data);

		// Normalize to linear acceleration
		data.az -= 1;

		// Write event
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_ACCELEROMETER, &data, sizeof(data));

		vTaskDelay(1000 / ACCELEROMETER_REFRESH_RATE / portTICK_PERIOD_MS);
	}
}