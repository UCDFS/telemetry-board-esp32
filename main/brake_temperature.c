#include <stdbool.h>
#include <esp_log.h>
#include <math.h>

#include "mlx90614.h"
#include "i2c.h"
#include "telemetry.h"
#include "temperature.h"

#define DEBUG_BRAKE_TEMPERATURE false

static const char* TAG = "brake_temperature";

void brake_temperature_read_task()
{
	// Initialize sensor
	mlx90614_handle_t mlx90614_dev = mlx90614_init(i2c_bus_get(), MLX90614_I2C_ADDR_DEFAULT);

	if (mlx90614_dev == NULL) {
		vTaskDelete(NULL);
	}

	float data;
	t_ev_sen_brake_temperature t_ev;
	while (true) {
		// Read values
		mlx90614_get_celcius(mlx90614_dev, &data);

		if (DEBUG_BRAKE_TEMPERATURE) {
			ESP_LOGI(TAG, "temp: %4.2f°C", data);
		}

		t_ev.brake_fl = data;
		t_ev.brake_fr = data;
		t_ev.brake_rl = data;
		t_ev.brake_rr = data;

		// Write event
		telemetry_write_event(EVENT_TYPE_SENSOR, EVENT_TYPE_SENSOR_BRAKE_TEMPERATURE, &t_ev, sizeof(t_ev));

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}