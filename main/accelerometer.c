#include <stdbool.h>
#include <esp_log.h>
#include <math.h>

#include "accelerometer.h"
#include "i2c.h"
#include "mpu6050.h"
#include "telemetry.h"

#define DEBUG_ACCELEROMETER false

#define CALIBRATION_ACCEL_MAX_OFFSET 0.1
#define CALIBRATION_GYRO_MAX_OFFSET 5.0

static const char* TAG = "accelerometer";

void accelerometer_read_task()
{
	// Initialize sensor
	mpu6050_handle_t mpu6050_dev = mpu6050_init(i2c_bus_get(), MOTION_SENSOR_MPU6050_I2C_ADDR);

#if CONFIG_MOTION_SENSOR_SELF_CALIBRATION_ENABLED
	ESP_LOGI(TAG, "Performing self calibration");

	// Use highest data rate and range possible during calibration
	ESP_ERROR_CHECK(mpu6050_setup(mpu6050_dev, 0, MPU6050_DLPF_260HZ_256HZ));
	mpu6050_set_accel_scale(mpu6050_dev, MPU6050_ACCEL_SCALE_2G);
	mpu6050_set_gyro_scale(mpu6050_dev, MPU6050_GYRO_SCALE_250DPS);

	// Wait for values to update
	vTaskDelay(10 / portTICK_PERIOD_MS);

	int64_t calibration_start_time = esp_timer_get_time();

	mpu6050_float_data_t sample;
	mpu6050_float_data_t calibration = {};
	int samples = 0;
	do {
		mpu6050_get_float_data(mpu6050_dev, &sample);

		if (samples == 0) {
			calibration = sample;
		} else {
			calibration.accel.ax += sample.accel.ax;
			calibration.accel.ay += sample.accel.ay;
			calibration.accel.az += sample.accel.az;

			calibration.gyro.gx += sample.gyro.gx;
			calibration.gyro.gy += sample.gyro.gy;
			calibration.gyro.gz += sample.gyro.gz;
		}
		samples++;
	} while (((esp_timer_get_time() - calibration_start_time) / 1000) < CONFIG_MOTION_SENSOR_SELF_CALIBRATION_DURATION);

	// Divide by number of samples taken
	calibration.accel.ax /= samples;
	calibration.accel.ay /= samples;
	calibration.accel.az /= samples;

	calibration.gyro.gx /= samples;
	calibration.gyro.gy /= samples;
	calibration.gyro.gz /= samples;

	// Make sure values are relatively low, in case car is in motion during calibration
	if (fabsf(calibration.accel.ax) > CALIBRATION_ACCEL_MAX_OFFSET ||
	    fabsf(calibration.accel.ay) > CALIBRATION_ACCEL_MAX_OFFSET ||
	    (fabsf(calibration.accel.az) - 1) > CALIBRATION_ACCEL_MAX_OFFSET) {
		ESP_LOGW(TAG, "Self calibration of accelerometer failed, offset greater than %f detected: ax: %.2f, ay: %.2f, az: %.2f",
				CALIBRATION_ACCEL_MAX_OFFSET, calibration.accel.ax, calibration.accel.ay, calibration.accel.az);

		calibration.accel.ax = calibration.accel.ay = calibration.accel.az = 0;
	}

	// Make sure values are relatively low, in case car is in motion during calibration
	if (fabsf(calibration.gyro.gx) > CALIBRATION_GYRO_MAX_OFFSET ||
	    fabsf(calibration.gyro.gy) > CALIBRATION_GYRO_MAX_OFFSET ||
	    (fabsf(calibration.gyro.gz) - 1) > CALIBRATION_GYRO_MAX_OFFSET) {
		ESP_LOGW(TAG, "Self calibration of gyroscope failed, offset greater than %f detected: gx: %.2f, gy: %.2f, gz: %.2f",
				CALIBRATION_GYRO_MAX_OFFSET, calibration.gyro.gx, calibration.gyro.gy, calibration.gyro.gz);

		calibration.gyro.gx = calibration.gyro.gy = calibration.gyro.gz = 0;
	}

	ESP_LOGI(TAG, "Self calibration completed with %d samples, offsets: " \
			"ax: %.2f, ay: %.2f, az: %.2f, gx: %.2f, gy: %.2f, gz: %.2f", samples,
			calibration.accel.ax, calibration.accel.ay, calibration.accel.az,
			calibration.gyro.gx, calibration.gyro.gy, calibration.gyro.gz);

	vTaskDelay(5000 / portTICK_PERIOD_MS);
#endif

	// Return to configured mode
	ESP_ERROR_CHECK(mpu6050_setup(mpu6050_dev, CONFIG_MOTION_SENSOR_MPU6050_SAMPLE_RATE, MPU6050_DLPF_94HZ_98HZ));
	mpu6050_set_accel_scale(mpu6050_dev, MOTION_SENSOR_MPU6050_ACCEL_SCALE);
	mpu6050_set_gyro_scale(mpu6050_dev, MOTION_SENSOR_MPU6050_GYRO_SCALE);

	mpu6050_float_data_t data;
	while (true) {
		// Read values
		mpu6050_get_float_data(mpu6050_dev, &data);

#if CONFIG_MOTION_SENSOR_SELF_CALIBRATION_ENABLED
		// Adjust by calibration values
		data.accel.ax -= calibration.accel.ax;
		data.accel.ay -= calibration.accel.ay;
		data.accel.az -= calibration.accel.az;
		data.gyro.gx -= calibration.gyro.gx;
		data.gyro.gy -= calibration.gyro.gy;
		data.gyro.gz -= calibration.gyro.gz;
#endif

		if (DEBUG_ACCELEROMETER) {
			ESP_LOGI(TAG, "ax: %6.2f, ay: %6.2f, az: %6.2f, gx: %8.2f, gy: %8.2f, gz: %8.2f, temp: %3.1f",
					data.accel.ax, data.accel.ay, data.accel.az, data.gyro.gx, data.gyro.gy, data.gyro.gz, data.temp);
		}

		// Write event
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_ACCELEROMETER, &data.accel, sizeof(data));
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_GYROSCOPE, &data.gyro, sizeof(data));

		vTaskDelay(configTICK_RATE_HZ / CONFIG_MOTION_SENSOR_EVENT_RATE);
	}
}
