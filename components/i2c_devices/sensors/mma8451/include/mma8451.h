#ifndef MMA8451_H
#define MMA8451_H

#include <driver/i2c.h>
#include "i2c_bus.h"

#define MMA8451_CHIP_ID 0x1A

#define MMA8451_I2C_ADDR_1 0x1C
#define MMA8451_I2C_ADDR_2 0x1D

typedef enum
{
	MMA8451_SCALE_2G = 0,
	MMA8451_SCALE_4G,
	MMA8451_SCALE_8G,
	MMA8451_SCALE_MAX
} mma8451_scale_t;

typedef enum
{
	MMA8451_DATA_RATE_800HZ = 0,
	MMA8451_DATA_RATE_400HZ,
	MMA8451_DATA_RATE_200HZ,
	MMA8451_DATA_RATE_100HZ,
	MMA8451_DATA_RATE_50HZ,
	MMA8451_DATA_RATE_12_5HZ,
	MMA8451_DATA_RATE_6_25HZ,
	MMA8451_DATA_RATE_1_56HZ,
	MMA8451_DATA_RATE_MAX
} mma8451_data_rate_t;

typedef enum
{
	MMA8451_NORMAL = 0,
	MMA8451_LOW_NOISE,
	MMA8451_HIGH_RES,
	MMA8451_LOW_POWER
} mma8451_oversampling_mode_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
} mma8451_raw_data_t;

typedef struct
{
	float ax;
	float ay;
	float az;
} mma8451_float_data_t;

typedef struct mma8451_dev_t *mma8451_handle_t;

mma8451_handle_t mma8451_init(i2c_bus_handle_t bus, uint8_t dev_addr);

void mma8451_delete(mma8451_handle_t dev, bool del_bus);

esp_err_t mma8451_set_mode(mma8451_handle_t dev, mma8451_oversampling_mode_t oversampling_mode, mma8451_data_rate_t data_rate,
					  bool low_noise, bool fast_read);

esp_err_t mma8451_set_scale(mma8451_handle_t dev, mma8451_scale_t scale);

esp_err_t mma8451_get_float_data(mma8451_handle_t dev, mma8451_float_data_t *data);


#endif