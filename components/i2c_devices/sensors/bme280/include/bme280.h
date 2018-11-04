#ifndef BME280_H
#define BME280_H

#include <driver/i2c.h>
#include "i2c_bus.h"

#define BME280_CHIP_ID 0x60

#define BME280_I2C_ADDR_1 0x76
#define BME280_I2C_ADDR_2 0x77

typedef enum {
	BME280_MODE_SLEEP = 0,
	BME280_MODE_FORCED = 1,
	BME280_MODE_NORMAL = 3
} bme280_mode_t;

typedef enum
{
	BME280_SAMPLING_OFF = 0,
	BME280_SAMPLING_1X,
	BME280_SAMPLING_2X,
	BME280_SAMPLING_4X,
	BME280_SAMPLING_8X,
	BME280_SAMPLING_16X,
	BME280_SAMPLING_MAX
} bme280_sampling_mode_t;

typedef enum
{
	BME280_IIR_FILTER_OFF = 0,
	BME280_IIR_FILTER_COEFFICIENT_2,
	BME280_IIR_FILTER_COEFFICIENT_4,
	BME280_IIR_FILTER_COEFFICIENT_8,
	BME280_IIR_FILTER_COEFFICIENT_16,
	BME280_IIR_FILTER_COEFFICIENT_MAX
} bme280_iir_filter_t;

typedef enum
{
	BME280_STANDBY_0_5MS = 0,
	BME280_STANDBY_62_5MS,
	BME280_STANDBY_125MS,
	BME280_STANDBY_250MS,
	BME280_STANDBY_500MS,
	BME280_STANDBY_1000MS,
	BME280_STANDBY_10MS,
	BME280_STANDBY_20MS,
} bme280_standby_t;

typedef struct {
	int32_t temperature;
	int32_t pressure;
	int32_t humidity;
} bme280_raw_data_t;

typedef struct
{
	float temperature;
	float pressure;
	float humidity;
} bme280_float_data_t;

typedef struct bme280_dev_t *bme280_handle_t;

bme280_handle_t bme280_init(i2c_bus_handle_t bus, uint8_t dev_addr);
void bme280_delete(bme280_handle_t dev, bool del_bus);

esp_err_t bme280_setup(bme280_handle_t dev, bme280_mode_t mode, bme280_standby_t sb_dur, bme280_iir_filter_t iir_filter,
		bme280_sampling_mode_t temp_os, bme280_sampling_mode_t press_os, bme280_sampling_mode_t hum_os);

esp_err_t bme280_get_raw_data(bme280_handle_t dev, bme280_raw_data_t* raw);
esp_err_t bme280_get_float_data(bme280_handle_t dev, bme280_float_data_t *data);


#endif