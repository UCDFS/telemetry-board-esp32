#include <stdbool.h>
#include <esp_log.h>

#include "accelerometer.h"
#include "i2c.h"
#include "mpu6050.h"
#include "telemetry.h"

#define ACCELEROMETER_REFRESH_RATE 10

void accelerometer_read_task()
{
	// Initialize MMA8451 accelerometer
	mpu6050_handle_t mpu6050_dev = mpu6050_init(i2c_bus_get(), MPU6050_I2C_ADDR_1);
	if (mpu6050_setup(mpu6050_dev, ACCELEROMETER_REFRESH_RATE, MPU6050_DLPF_94HZ_98HZ) != ESP_OK) {
		vTaskDelete(NULL);
	};
	mpu6050_set_accel_scale(mpu6050_dev, MPU6050_ACCEL_SCALE_2G);
	mpu6050_set_gyro_scale(mpu6050_dev, MPU6050_GYRO_SCALE_2000DPS);

	mpu6050_float_data_t data;
	while (true) {
		// Read values
		mpu6050_get_float_data(mpu6050_dev, &data);

		// Normalize to linear acceleration
		//data.accel.az -= 1;

		ESP_LOGI("MPU6050", "ax: %6.2f, ay: %6.2f, az: %6.2f, gx: %8.2f, gy: %8.2f, gz: %8.2f, temp: %3.1f",
				data.accel.ax, data.accel.ay, data.accel.az, data.gyro.gx, data.gyro.gy, data.gyro.gz, data.temp);

		// Write event
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_ACCELEROMETER, &data.accel, sizeof(data));
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_GYROSCOPE, &data.gyro, sizeof(data));

		vTaskDelay(1000 / ACCELEROMETER_REFRESH_RATE / portTICK_PERIOD_MS);
	}
}