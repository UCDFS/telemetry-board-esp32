#include <stdio.h>

#include "esp_log.h"

#include "i2c_bus.h"

#include "mma8451.h"

#define MMA8451_TAG "MMA8451"

#define error_dev(s, f, d, ...) ESP_LOGE(MMA8451_TAG, "%s: bus %d, addr %02x - " s "\n", f, \
        ((i2c_bus_t *) d->bus)->i2c_port, d->addr, ## __VA_ARGS__)

enum
{
	MMA8451_REG_OUT_X_MSB = 0x01,

	MMA8451_REG_WHO_AM_I = 0x0d,
	MMA8451_REG_XYZ_DATA_CFG,

	MMA8451_REG_CTRL1 = 0x2a,
	MMA8451_REG_CTRL2
};

struct mma8451_reg_xyz_data_cfg
{
	uint8_t FS      :2; // XYZ_DATA_CFG<1:0> Output buffer data format full scale
	uint8_t unused1 :2; // XYZ_DATA_CFG<3:2> unused
	bool HPF_OUT    :1; // XYZ_DATA_CFG<4>   Enable high-pass output data
	uint8_t unused2 :3; // XYZ_DATA_CFG<7:5> unused
};

struct mma8451_reg_ctrl1
{
	bool ACTIVE       :1; // CTRL1<0>   Full-scale selection
	bool F_READ       :1; // CTRL1<1>   Fast read mode (single byte read)
	bool LNOISE       :1; // CTRL1<2>   Reduced noise, reduced maximum range
	uint8_t DR        :3; // CTRL1<5:3> Data-rate selection.
	uint8_t ASLP_RATE :2; // CTRL1<7:6> Data rate selection
};

struct mma8451_reg_ctrl2
{
	uint8_t MODS   :2; // CTRL2<1:0> Active mode power scheme selection.
	bool SLPE      :1; // CTRL2<2>   Auto-sleep enable
	uint8_t SMODS  :2; // CTRL2<4:3> Sleep mode power scheme selection
	uint8_t unused :1; // CTRL2<5>   unused
	bool RST       :1; // CTRL2<6>   Software reset
	bool ST        :1; // CTRL2<7>   Self-test enable
};

struct mma8451_dev_t
{
	i2c_bus_handle_t bus;
	uint8_t addr;

	bool active;
	bool fast_read;

	mma8451_scale_t scale;
};

#define mma8451_update_reg(dev, addr, type, elem, value) \
        { \
            struct type __reg; \
            if (mma8451_reg_read(dev, (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
            __reg.elem = (value); \
            if (mma8451_reg_write(dev, (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
        }


static esp_err_t mma8451_reset(mma8451_handle_t dev);
static bool mma8451_is_available(mma8451_handle_t dev);

mma8451_handle_t mma8451_init(i2c_bus_handle_t bus, uint8_t dev_addr)
{
	mma8451_handle_t dev = (mma8451_handle_t) calloc(1, sizeof(struct mma8451_dev_t));
	dev->bus = bus;
	dev->addr = dev_addr;

	dev->active = false;
	dev->scale = MMA8451_SCALE_2G;
	dev->fast_read = false;

	if (!mma8451_is_available(dev)) {
		error_dev("Sensor is not available.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	// Reset the sensor
	if (mma8451_reset(dev) != ESP_OK) {
		error_dev("Could not reset the sensor device.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	return dev;
}

void mma8451_delete(mma8451_handle_t dev, bool del_bus)
{
	if (del_bus) {
		i2c_bus_delete(dev->bus);
		dev->bus = NULL;
	}
	free(dev);
}

static esp_err_t mma8451_reg_read(mma8451_handle_t dev, uint8_t reg, uint8_t *data, size_t size)
{
	esp_err_t err = i2c_bus_master_read(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on read %d byte from I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t mma8451_reg_write(mma8451_handle_t dev, uint8_t reg, uint8_t *data, size_t size)
{
	esp_err_t err = i2c_bus_master_write(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on write %d byte to I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t mma8451_reset(mma8451_handle_t dev)
{
	mma8451_update_reg(dev, MMA8451_REG_CTRL2, mma8451_reg_ctrl2, RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	return ESP_OK;
}

static bool mma8451_is_available(mma8451_handle_t dev)
{
	uint8_t chip_id;
	if (mma8451_reg_read(dev, MMA8451_REG_WHO_AM_I, &chip_id, 1) != ESP_OK) {
		error_dev("Could not read chip id.", __FUNCTION__, dev);
		return false;
	}

	if (chip_id != MMA8451_CHIP_ID) {
		error_dev("Chip id %02x is wrong, should be %02x.", __FUNCTION__, dev, chip_id, MMA8451_CHIP_ID);
		return false;
	}

	return true;
}

static esp_err_t mma8451_set_active(mma8451_handle_t dev, bool active)
{
	mma8451_update_reg(dev, MMA8451_REG_CTRL1, mma8451_reg_ctrl1, ACTIVE, active);

	return ESP_OK;
}

esp_err_t mma8451_setup(mma8451_handle_t dev, mma8451_oversampling_mode_t oversampling_mode,
		mma8451_data_rate_t data_rate, bool low_noise, bool fast_read)
{
	if (dev->active) mma8451_set_active(dev, false);

	struct mma8451_reg_ctrl1 ctrl1;
	struct mma8451_reg_ctrl2 ctrl2;

	// Read current register values
	if (mma8451_reg_read(dev, MMA8451_REG_CTRL1, (uint8_t *) &ctrl1, 1) != ESP_OK ||
	    mma8451_reg_read(dev, MMA8451_REG_CTRL2, (uint8_t *) &ctrl2, 1) != ESP_OK) {
		return ESP_FAIL;
	}

	dev->fast_read = fast_read;
	dev->active = true;

	ctrl1.ACTIVE = true; // switch to active mode
	ctrl1.DR = data_rate;
	ctrl1.LNOISE = low_noise;
	ctrl1.F_READ = fast_read;
	ctrl2.MODS = oversampling_mode;

	// Write back register values
	if (mma8451_reg_write(dev, MMA8451_REG_CTRL1, (uint8_t *) &ctrl1, 1) != ESP_OK ||
	    mma8451_reg_write(dev, MMA8451_REG_CTRL2, (uint8_t *) &ctrl2, 1) != ESP_OK) {
		return ESP_FAIL;
	}

	// Wait 100 ms
	vTaskDelay(100 / portTICK_PERIOD_MS);

	return ESP_OK;
}

esp_err_t mma8451_set_scale(mma8451_handle_t dev, mma8451_scale_t scale)
{
	dev->scale = scale;

	if (dev->active) mma8451_set_active(dev, false);

	mma8451_update_reg(dev, MMA8451_REG_XYZ_DATA_CFG, mma8451_reg_xyz_data_cfg, FS, scale);

	if (dev->active) mma8451_set_active(dev, true);

	return ESP_OK;
}

esp_err_t mma8451_get_raw_data(mma8451_handle_t dev, mma8451_raw_data_t *raw)
{
	uint8_t data[6];
	if (mma8451_reg_read(dev, MMA8451_REG_OUT_X_MSB, data, (dev->fast_read) ? 3 : 6) != ESP_OK) {
		error_dev("Could not get raw data sample", __FUNCTION__, dev);
		return ESP_FAIL;
	}

	if (dev->fast_read) {
		raw->ax = (uint16_t) data[0] << 8;
		raw->ay = (uint16_t) data[1] << 8;
		raw->az = (uint16_t) data[2] << 8;
	} else {
		raw->ax = ((uint16_t) data[0] << 8) | data[1];
		raw->ay = ((uint16_t) data[2] << 8) | data[3];
		raw->az = ((uint16_t) data[4] << 8) | data[5];
	}

	return ESP_OK;
}

/**
 * Scaling factors for the conversion of raw sensor data to floating point g
 * values. Scaling factors are from mechanical characteristics in datasheet.
 *
 *  scale/sensitivity  resolution
 *       +-2g           4096 counts/g
 *       +-4g           2048 counts/g
 *       +-8g           1024 counts/g
 */
const static double MMA8451_SCALES[4] = {1.0 / 4096, 1.0 / 2048, 1.0 / 1024};

esp_err_t mma8451_get_float_data(mma8451_handle_t dev, mma8451_float_data_t *data)
{
	mma8451_raw_data_t raw;
	if (mma8451_get_raw_data(dev, &raw) != ESP_OK) return ESP_FAIL;

	data->ax = (float) (MMA8451_SCALES[dev->scale] * (raw.ax >> 2));
	data->ay = (float) (MMA8451_SCALES[dev->scale] * (raw.ay >> 2));
	data->az = (float) (MMA8451_SCALES[dev->scale] * (raw.az >> 2));

	return ESP_OK;
}