#include <i2c_bus.h>

#include "i2c.h"

#define I2C_NUM I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_26
#define I2C_SCL_GPIO GPIO_NUM_25
#define I2C_FREQUENCY 400000

i2c_bus_handle_t i2c_bus_handle = NULL;

void i2c_bus_init()
{
	if (i2c_bus_handle != NULL) {
		return;
	}

	i2c_config_t conf = {
			.mode 			= I2C_MODE_MASTER,
			.sda_io_num 	= I2C_SDA_GPIO,
			.sda_pullup_en 	= GPIO_PULLUP_ENABLE,
			.scl_io_num 	= I2C_SCL_GPIO,
			.scl_pullup_en 	= GPIO_PULLUP_ENABLE,
			.master.clk_speed = I2C_FREQUENCY
	};
	i2c_bus_handle = i2c_bus_create(I2C_NUM, &conf);
}

i2c_bus_handle_t i2c_bus_get() {
	return i2c_bus_handle;
}