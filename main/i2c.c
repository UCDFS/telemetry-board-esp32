#include <i2c_bus.h>

#include "i2c.h"

#define I2C_BUS_NUM I2C_NUM_0
#define I2C_BUS_SDA_GPIO CONFIG_I2C_BUS_SDA_GPIO
#define I2C_BUS_SCL_GPIO CONFIG_I2C_BUS_SCL_GPIO
#define I2C_BUS_FREQUENCY 400000

i2c_bus_handle_t i2c_bus_handle = NULL;

void i2c_bus_init()
{
	if (i2c_bus_handle != NULL) {
		return;
	}

	i2c_config_t conf = {
			.mode 			= I2C_MODE_MASTER,
			.sda_io_num 	= I2C_BUS_SDA_GPIO,
			.sda_pullup_en 	= GPIO_PULLUP_ENABLE,
			.scl_io_num 	= I2C_BUS_SCL_GPIO,
			.scl_pullup_en 	= GPIO_PULLUP_ENABLE,
			.master.clk_speed = I2C_BUS_FREQUENCY
	};
	i2c_bus_handle = i2c_bus_create(I2C_BUS_NUM, &conf);
}

i2c_bus_handle_t i2c_bus_get() {
	return i2c_bus_handle;
}