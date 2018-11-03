#include <stdio.h>

#include "esp_log.h"

#include <driver/i2c.h>
#include "i2c_bus.h"

#include "mpu6050.h"

#define MPU6050_TAG "MPU6050"

#define error_dev(s, f, d, ...) ESP_LOGE(MPU6050_TAG, "%s: bus %d, addr %02x - " s "\n", f, \
		((i2c_bus_t *) d->bus)->i2c_port, d->addr, ## __VA_ARGS__)

enum {
	MPU6050_REG_SMPLRT_DIV = 0x19,

	MPU6050_REG_CONFIG = 0x1a,
	MPU6050_REG_GYRO_CONFIG,
	MPU6050_REG_ACCEL_CONFIG,

	MPU6050_REG_ACCEL_XOUT_H = 0x3b,
	MPU6050_REG_TEMP_OUT_H = 0x41,
	MPU6050_REG_GYTRO_XOUT_H = 0x43,

	MPU6050_REG_PWR_MGMT_1 = 0x6b,

	MPU6050_REG_WHO_AM_I = 0x75
};

struct mpu6050_reg_smplrt_div {
	uint8_t SMPLRT_DIV:8;
};

struct mpu6050_reg_config {
	uint8_t DLPF_CFG		:3;
	uint8_t EXT_SYNC_SET	:3;
	uint8_t unused			:2;
};
struct mpu6050_reg_gyro_config {
	uint8_t unused		:3;
	uint8_t FS_SEL		:2;
	bool ZG_ST			:1;
	bool YG_ST			:1;
	bool XG_ST			:1;
};
struct mpu6050_reg_accel_config {
	uint8_t unused		:3;
	uint8_t AFS_SEL		:2;
	bool ZA_ST			:1;
	bool YA_ST			:1;
	bool XA_ST			:1;
};

struct mpu6050_reg_pwr_mgmt_1 {
	uint8_t CLKSEL		:3;
	bool TEMP_DIS		:1;
	uint8_t unused		:1;
	bool CYCLE			:1;
	bool SLEEP			:1;
	bool DEVICE_RESET	:1;
};

struct mpu6050_dev_t
{
	i2c_bus_handle_t bus;
	uint8_t addr;

	bool active;

	mpu6050_accel_scale_t accel_scale;
	mpu6050_gyro_scale_t gyro_scale;
};

#define mpu6050_update_reg(dev, addr, type, elem, value) \
        { \
            struct type __reg; \
            if (mpu6050_reg_read(dev, (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
            __reg.elem = (value); \
            if (mpu6050_reg_write(dev, (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
		}


static esp_err_t mpu6050_reset(mpu6050_handle_t dev);
static bool mpu6050_is_available(mpu6050_handle_t dev);
static esp_err_t mpu6050_set_active(mpu6050_handle_t dev, bool active);

mpu6050_handle_t mpu6050_init(i2c_bus_handle_t bus, uint8_t dev_addr)
{
	mpu6050_handle_t dev = (mpu6050_handle_t) calloc(1, sizeof(struct mpu6050_dev_t));
	dev->bus = bus;
	dev->addr = dev_addr;

	dev->active = false;

	dev->accel_scale = MPU6050_ACCEL_SCALE_2G;
	dev->gyro_scale = MPU6050_GYRO_SCALE_250DPS;

	if (!mpu6050_is_available(dev)) {
		error_dev("Sensor is not available.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	// reset the sensor
	if (mpu6050_reset(dev) != ESP_OK) {
		error_dev("Could not reset the sensor device.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	return dev;
}

void mpu6050_delete(mpu6050_handle_t dev, bool del_bus)
{
	if (del_bus) {
		i2c_bus_delete(dev->bus);
		dev->bus = NULL;
	}
	free(dev);
}

static esp_err_t mpu6050_reg_read(mpu6050_handle_t dev, uint8_t reg, uint8_t* data, size_t size)
{
	esp_err_t err = i2c_bus_master_read(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on read %d byte from I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t mpu6050_reg_write(mpu6050_handle_t dev, uint8_t reg, uint8_t* data, size_t size)
{
	esp_err_t err = i2c_bus_master_write(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on write %d byte to I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t mpu6050_reset(mpu6050_handle_t dev)
{
	mpu6050_update_reg(dev, MPU6050_REG_PWR_MGMT_1, mpu6050_reg_pwr_mgmt_1, DEVICE_RESET, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	return ESP_OK;
}

static bool mpu6050_is_available(mpu6050_handle_t dev) {
	uint8_t chip_id;
	if (mpu6050_reg_read(dev, MPU6050_REG_WHO_AM_I, &chip_id, 1) != ESP_OK) {
		error_dev("Could not read chip id.", __FUNCTION__, dev);
		return false;
	}

	if (chip_id != MPU6050_CHIP_ID) {
		error_dev("Chip id %02x is wrong, should be %02x.", __FUNCTION__, dev, chip_id, MPU6050_CHIP_ID);
		return false;
	}

	return true;
}

static esp_err_t mpu6050_set_active(mpu6050_handle_t dev, bool active)
{
	mpu6050_update_reg(dev, MPU6050_REG_PWR_MGMT_1, mpu6050_reg_pwr_mgmt_1, SLEEP, !active);

	return ESP_OK;
}

esp_err_t mpu6050_setup(mpu6050_handle_t dev, uint8_t data_rate, mpu6050_dlpf_t dlpf) {
	if (dev->active) mpu6050_set_active(dev, false);

	struct mpu6050_reg_smplrt_div smplrt_div;
	struct mpu6050_reg_config config;

	dev->active  = true;

	smplrt_div.SMPLRT_DIV = dlpf = ((dlpf == MPU6050_DLPF_260HZ_256HZ ? 8000 : 1000) / data_rate) - 1;
	config.DLPF_CFG = dlpf;

	// Write back register values
	if (mpu6050_reg_write(dev, MPU6050_REG_SMPLRT_DIV, (uint8_t*) &smplrt_div, 1) != ESP_OK ||
		mpu6050_reg_write(dev, MPU6050_REG_CONFIG, (uint8_t*) &config, 1) != ESP_OK) {
		return ESP_FAIL;
	}

	if (dev->active) mpu6050_set_active(dev, true);

	return ESP_OK;
}

esp_err_t mpu6050_set_accel_scale(mpu6050_handle_t dev, mpu6050_accel_scale_t scale)
{
	dev->accel_scale = scale;

	mpu6050_update_reg(dev, MPU6050_REG_ACCEL_CONFIG, mpu6050_reg_accel_config, AFS_SEL, scale);

	return ESP_OK;
}

esp_err_t mpu6050_set_gyro_scale(mpu6050_handle_t dev, mpu6050_gyro_scale_t scale)
{
	dev->gyro_scale = scale;

	mpu6050_update_reg(dev, MPU6050_REG_GYRO_CONFIG, mpu6050_reg_gyro_config, FS_SEL, scale);

	return ESP_OK;
}

esp_err_t mpu6050_get_raw_data(mpu6050_handle_t dev, mpu6050_raw_data_t* raw)
{
	uint8_t data[14];
	if (mpu6050_reg_read(dev, MPU6050_REG_ACCEL_XOUT_H, data, 14) != ESP_OK) {
		error_dev("Could not get raw data sample", __FUNCTION__, dev);
		return ESP_FAIL;
	}

	raw->accel.ax = ((uint16_t)data[0] << 8) | data[1];
	raw->accel.ay = ((uint16_t)data[2] << 8) | data[3];
	raw->accel.az = ((uint16_t)data[4] << 8) | data[5];
	raw->temp = ((uint16_t)data[6] << 8) | data[7];
	raw->gyro.gx = ((uint16_t)data[8] << 8) | data[9];
	raw->gyro.gy = ((uint16_t)data[10] << 8) | data[11];
	raw->gyro.gz = ((uint16_t)data[12] << 8) | data[13];

	return ESP_OK;
}

/**
 * Scaling factors for the conversion of raw sensor data to floating point g
 * values. Scaling factors are from mechanical characteristics in datasheet.
 *
 *  scale/sensitivity  resolution
 *       +-2g           16384 counts/g
 *       +-4g           8192 counts/g
 *       +-8g           4096 counts/g
 *       +-16g          2048 counts/g
 */
const static double MPU6050_ACCEL_SCALES[4] = { 1.0/16384, 1.0/8192, 1.0/4096, 1.0/2048 };

/**
 * Scaling factors for the conversion of raw sensor data to floating point g
 * values. Scaling factors are from mechanical characteristics in datasheet.
 *
 *  scale/sensitivity  resolution
 *       +-250dps           131 counts/g
 *       +-500dps           65.6 counts/g
 *       +-1000dps          32.8 counts/g
 *       +-2000dps          16.4 counts/g
 */
const static double MPU6050_GYRO_SCALES[4] = { 1.0/131, 1.0/65.6, 1.0/32.8, 1.0/16.4 };

const static double MPU6050_TEMP_SCALE = 1.0/340;
const static double MPU6050_TEMP_OFFSET = 36.53;

esp_err_t mpu6050_get_float_data(mpu6050_handle_t dev, mpu6050_float_data_t* data)
{
	mpu6050_raw_data_t raw;
	if (mpu6050_get_raw_data(dev, &raw) != ESP_OK) return ESP_FAIL;

	data->accel.ax = (float) (MPU6050_ACCEL_SCALES[dev->accel_scale] * raw.accel.ax);
	data->accel.ay = (float) (MPU6050_ACCEL_SCALES[dev->accel_scale] * raw.accel.ay);
	data->accel.az = (float) (MPU6050_ACCEL_SCALES[dev->accel_scale] * raw.accel.az);

	data->gyro.gx = (float) (MPU6050_GYRO_SCALES[dev->gyro_scale] * raw.gyro.gx);
	data->gyro.gy = (float) (MPU6050_GYRO_SCALES[dev->gyro_scale] * raw.gyro.gy);
	data->gyro.gz = (float) (MPU6050_GYRO_SCALES[dev->gyro_scale] * raw.gyro.gz);

	data->temp = (float) (raw.temp * MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET);

	return true;
}