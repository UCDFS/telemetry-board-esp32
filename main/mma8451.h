#ifndef MMA8451_H
#define MMA8451_H

#include <driver/i2c.h>

typedef enum {
	MMA8451_SCALE_2G = 0,
	MMA8451_SCALE_4G = 1,
	MMA8451_SCALE_8G = 2,
	MMA8451_SCALE_MAX
} mma8451_scale_t;

typedef enum {
	MMA8451_DATA_RATE_800HZ = 0,
	MMA8451_DATA_RATE_400HZ = 1,
	MMA8451_DATA_RATE_200HZ = 2,
	MMA8451_DATA_RATE_100HZ = 3,
	MMA8451_DATA_RATE_50HZ = 4,
	MMA8451_DATA_RATE_12_5HZ = 5,
	MMA8451_DATA_RATE_6_25HZ = 6,
	MMA8451_DATA_RATE_1_56HZ = 7,
	MMA8451_DATA_RATE_MAX
} mma8451_data_rate_t;

typedef struct {
	i2c_port_t i2c_port = I2C_NUM_0,
	uint8_t i2c_addr,
	mma8451_scale_t scale = MMA8451_SCALE_2G,
	mma8451_data_rate_t = MMA8451_DATA_RATE_800HZ
} mma8451_config_t;

typedef struct {
	int16_t	x;
	int16_t y;
	int16_t z;
} mma8451_raw_data_t;

esp_err_t mma8451_init(mma8451_config_t *config);
esp_err_t mma8451_read_raw(mma8451_raw_data_t *acc);

esp_err_t mma8451_read_reg(uint8_t reg, uint8_t *pdata, uint8_t count);
esp_err_t mma8451_write_reg(uint8_t reg, uint8_t *pdata, uint8_t count);


#endif