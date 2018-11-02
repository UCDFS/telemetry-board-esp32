#include <stdbool.h>

#include "accelerometer.h"
#include "mma8451.h"
#include "telemetry.h"

#define ACCELEROMETER_REFRESH_RATE 50

void accelerometer_read_task()
{
	// Initialize MMA8451 accelerometer
	mma8451_config_t conf = {
			.i2c_port 	= I2C_NUM_0,
			.i2c_addr 	= MMA8451_I2C_ADDR_HIGH,
			.scale 		= MMA8451_SCALE_2G,
			.data_rate 	= MMA8451_DATA_RATE_50HZ
	};
	mma8451_init(&conf);

	mma8451_raw_data_t raw_data;
	while (true) {
		// Read values
		mma8451_read_raw(&raw_data);

		// Write event
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_ACCELEROMETER, &raw_data, sizeof(raw_data));

		vTaskDelay(1000 / ACCELEROMETER_REFRESH_RATE / portTICK_PERIOD_MS);
	}
}