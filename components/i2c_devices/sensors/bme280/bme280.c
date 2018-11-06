#include <stdio.h>

#include "esp_log.h"

#include "i2c_bus.h"

#include "bme280.h"

#define BME280_TAG "BME280"

#define error_dev(s, f, d, ...) ESP_LOGE(BME280_TAG, "%s: bus %d, addr %02x - " s, f, \
        ((i2c_bus_t *) d->bus)->i2c_port, d->addr, ## __VA_ARGS__)

#define BME280_RESET_VALUE 0

enum
{
	BME280_REG_DIG_T1 = 0x88,
	BME280_REG_DIG_H1 = 0xa1,
	BME280_REG_DIG_H2 = 0xe1,

	BME280_REG_WHO_AM_I = 0xd0,

	BME280_REG_RESET = 0xe0,

	BME280_REG_CTRL_HUM = 0xf2,
	BME280_REG_STATUS,
	BME280_REG_CTRL_MEAS,
	BME280_REG_CONFIG,

	BME280_REG_PRESS_MSB = 0xf7,

	BME280_REG_TEMP_MSB = 0xfa,

	BME280_REG_HUM_MSB = 0xfd
};

struct bme280_reg_ctrl_hum
{
	uint8_t OSRS_H :3; // Controls oversampling of humidity data.
	uint8_t unused :5;
};

struct bme280_reg_status
{
	bool IM_UPDATE  :1; // Automatically set to '1' when the NVM data are being copied to the image registers.
	uint8_t unused1 :2;
	bool MEASURING  :1; // Automatically set to '1' whenever a conversion is running.
	uint8_t unused2 :4;
};
struct bme280_reg_ctrl_meas
{
	uint8_t MODE   :2; // Controls the sensor mode of the device.
	uint8_t OSRS_P :3; // Controls oversampling of pressure data.
	uint8_t OSRS_T :3; // Controls oversampling of temperature data.
};
struct bme280_reg_config
{
	bool SPI3W_EN  :1; // Enables 3-wire SPI interface when set to '1'.
	uint8_t FILTER :3; // Controls the time constant of the IIR filter.
	uint8_t T_SB   :3; // Controls inactive duration t_sandby in normal mode.
};

struct bme280_calibration_data {
	uint16_t dig_t1 :16;
	int16_t dig_t2  :16;
	int16_t dig_t3  :16;
	uint16_t dig_p1 :16;
	int16_t dig_p2  :16;
	int16_t dig_p3  :16;
	int16_t dig_p4  :16;
	int16_t dig_p5  :16;
	int16_t dig_p6  :16;
	int16_t dig_p7  :16;
	int16_t dig_p8  :16;
	int16_t dig_p9  :16;

	uint8_t dig_h1  :8;
	int16_t dig_h2  :16;
	uint8_t dig_h3  :8;

	uint8_t dig_h4_h :8;
	uint8_t dig_h4_l :4;
	uint8_t dig_h5_l :4;
	uint8_t dig_h5_h :8;

	int8_t dig_h6    :8;

	int16_t dig_h4  :16;
	int16_t dig_h5  :16;
};

struct bme280_dev_t
{
	i2c_bus_handle_t bus;
	uint8_t addr;

	struct bme280_calibration_data data;
	int32_t t_fine;

	bool active;
};

#define bme280_update_reg(dev, addr, type, elem, value) \
        { \
            struct type __reg; \
            if (bme280_reg_read(dev, (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
            __reg.elem = (value); \
            if (bme280_reg_write(dev, (addr), (uint8_t*)&__reg) != ESP_OK) \
                return ESP_FAIL; \
        }


static esp_err_t bme280_reset(bme280_handle_t dev);
static bool bme280_is_available(bme280_handle_t dev);
static bool bme280_is_reading_calibration(bme280_handle_t dev);
static esp_err_t bme280_read_coefficients(bme280_handle_t dev);

bme280_handle_t bme280_init(i2c_bus_handle_t bus, uint8_t dev_addr)
{
	bme280_handle_t dev = (bme280_handle_t) calloc(1, sizeof(struct bme280_dev_t));
	dev->bus = bus;
	dev->addr = dev_addr;

	dev->active = false;

	if (!bme280_is_available(dev)) {
		error_dev("Sensor is not available.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	// Reset the sensor
	if (bme280_reset(dev) != ESP_OK) {
		error_dev("Could not reset the sensor device.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	// Wait for chip to wake up
	vTaskDelay(300 / portTICK_RATE_MS);

	// Wait for calibration data to be read
	while (bme280_is_reading_calibration(dev)) {
		vTaskDelay(100 / portTICK_RATE_MS);
	}

	if (bme280_read_coefficients(dev) != ESP_OK) {
		error_dev("Could not read the sensor calibration data.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	return dev;
}

void bme280_delete(bme280_handle_t dev, bool del_bus)
{
	if (del_bus) {
		i2c_bus_delete(dev->bus);
		dev->bus = NULL;
	}
	free(dev);
}

static esp_err_t bme280_reg_read(bme280_handle_t dev, uint8_t reg, uint8_t *data, size_t size)
{
	esp_err_t err = i2c_bus_master_read(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on read %d byte from I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t bme280_reg_write(bme280_handle_t dev, uint8_t reg, uint8_t *data)
{
	esp_err_t err = i2c_bus_master_write(dev->bus, dev->addr, &reg, data, 1);
	if (err) {
		error_dev("Error %d on write byte to I2C slave register %02x.", __FUNCTION__, dev, err, reg);
	}

	return err;
}

static esp_err_t bme280_reset(bme280_handle_t dev)
{
	uint8_t reset_value = BME280_RESET_VALUE;
	bme280_reg_write(dev, BME280_REG_RESET, &reset_value);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	return ESP_OK;
}

static bool bme280_is_available(bme280_handle_t dev)
{
	uint8_t chip_id;
	if (bme280_reg_read(dev, BME280_REG_WHO_AM_I, &chip_id, 1) != ESP_OK) {
		error_dev("Could not read chip id.", __FUNCTION__, dev);
		return false;
	}

	if (chip_id != BME280_CHIP_ID) {
		error_dev("Chip id %02x is wrong, should be %02x.", __FUNCTION__, dev, chip_id, BME280_CHIP_ID);
		return false;
	}

	return true;
}

static bool bme280_is_reading_calibration(bme280_handle_t dev)
{
	struct bme280_reg_status status;
	if (bme280_reg_read(dev, BME280_REG_STATUS, (uint8_t *) &status, 1) != ESP_OK) {
		error_dev("Could not read status.", __FUNCTION__, dev);
	}

	return status.IM_UPDATE;
}

static esp_err_t bme280_read_coefficients(bme280_handle_t dev)
{
	if (bme280_reg_read(dev, BME280_REG_DIG_T1, (uint8_t *) &dev->data, 24) != ESP_OK ||
			bme280_reg_read(dev, BME280_REG_DIG_H1, (uint8_t *) &dev->data + 24, 1) != ESP_OK ||
			bme280_reg_read(dev, BME280_REG_DIG_H2, (uint8_t *) &dev->data + 26, 8) != ESP_OK) {
		return ESP_FAIL;
	}

	dev->data.dig_h4 = dev->data.dig_h4_h << 4 | dev->data.dig_h4_l;
	dev->data.dig_h5 = dev->data.dig_h5_h << 4 | dev->data.dig_h5_l;

	return ESP_OK;
}


static esp_err_t bme280_set_mode(bme280_handle_t dev, bme280_mode_t mode)
{
	bme280_update_reg(dev, BME280_REG_CTRL_MEAS, bme280_reg_ctrl_meas, MODE, mode);

	return ESP_OK;
}

esp_err_t bme280_setup(bme280_handle_t dev, bme280_mode_t mode, bme280_standby_t sb_dur, bme280_iir_filter_t iir_filter,
		bme280_sampling_mode_t temp_os, bme280_sampling_mode_t press_os, bme280_sampling_mode_t hum_os)
{
	struct bme280_reg_config config;
	struct bme280_reg_ctrl_hum ctrl_hum;
	struct bme280_reg_ctrl_meas ctrl_meas;

	config.T_SB = sb_dur;
	config.FILTER = iir_filter;
	ctrl_meas.MODE = mode;
	ctrl_meas.OSRS_T = temp_os;
	ctrl_meas.OSRS_P = press_os;
	ctrl_hum.OSRS_H = hum_os;

	// Write register values
	if (bme280_reg_write(dev, BME280_REG_CONFIG, (uint8_t *) &config) != ESP_OK ||
			bme280_reg_write(dev, BME280_REG_CTRL_HUM, (uint8_t *) &ctrl_hum) != ESP_OK ||
	        bme280_reg_write(dev, BME280_REG_CTRL_MEAS, (uint8_t *) &ctrl_meas) != ESP_OK) {
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t bme280_get_raw_data(bme280_handle_t dev, bme280_raw_data_t *raw)
{
	uint8_t data[8];
	if (bme280_reg_read(dev, BME280_REG_PRESS_MSB, data, 8) != ESP_OK) {
		error_dev("Could not get raw data sample", __FUNCTION__, dev);
		return ESP_FAIL;
	}

	raw->temperature = ((uint32_t) data[3] << 12) | ((uint32_t) data[4] << 4) | data[5];
	raw->pressure = ((uint32_t) data[0] << 12) | ((uint32_t) data[1] << 4) | data[2];
	raw->humidity = ((uint16_t) data[6] << 8) | data[7];

	return ESP_OK;
}


esp_err_t bme280_get_float_data(bme280_handle_t dev, bme280_float_data_t *data)
{
	bme280_raw_data_t raw;
	if (bme280_get_raw_data(dev, &raw) != ESP_OK) return ESP_FAIL;

	if (raw.temperature != 0x80000) {
		int64_t var1, var2;

		var1 = ((((raw.temperature >> 3) - ((int32_t) dev->data.dig_t1 << 1)))
		        * ((int32_t) dev->data.dig_t2)) >> 11;

		var2 = (((((raw.temperature >> 4) - ((int32_t) dev->data.dig_t1))
		          * ((raw.temperature >> 4) - ((int32_t) dev->data.dig_t1))) >> 12)
		        * ((int32_t) dev->data.dig_t3)) >> 14;

		dev->t_fine = (int32_t) (var1 + var2);

		data->temperature = (float) (((dev->t_fine * 5 + 128) >> 8) / 100.0);
	}

	if (raw.pressure != 0x80000) {
		int64_t var1, var2, p;

		var1 = ((int64_t) dev->t_fine) - 128000;
		var2 = var1 * var1 * (int64_t) dev->data.dig_p6;
		var2 = var2 + ((var1 * (int64_t) dev->data.dig_p5) << 17);
		var2 = var2 + (((int64_t) dev->data.dig_p4) << 35);
		var1 = ((var1 * var1 * (int64_t) dev->data.dig_p3) >> 8)
		       + ((var1 * (int64_t) dev->data.dig_p2) << 12);
		var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dev->data.dig_p1)
				>> 33;

		if (var1 == 0) {
			return ESP_FAIL; // avoid exception caused by division by zero
		}
		p = 1048576 - raw.pressure;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((int64_t) dev->data.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t) dev->data.dig_p8) * p) >> 19;

		p = ((p + var1 + var2) >> 8) + (((int64_t) dev->data.dig_p7) << 4);
		data->pressure = (float) ((p >> 8) / 100.0);
	}

	if (raw.humidity != 0x8000) {
		int32_t v_x1_u32r;

		v_x1_u32r = (dev->t_fine - ((int32_t) 76800));

		v_x1_u32r = (((((raw.humidity << 14) - (((int32_t) dev->data.dig_h4) << 20)
		                - (((int32_t) dev->data.dig_h5) * v_x1_u32r))
		               + ((int32_t) 16384)) >> 15)
		             * (((((((v_x1_u32r * ((int32_t) dev->data.dig_h6)) >> 10)
		                    * (((v_x1_u32r * ((int32_t) dev->data.dig_h3)) >> 11)
		                       + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
		                 * ((int32_t) dev->data.dig_h2) + 8192) >> 14));

		v_x1_u32r = (v_x1_u32r
		             - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
		                 * ((int32_t) dev->data.dig_h1)) >> 4));

		v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
		v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
		data->humidity = (float) ((v_x1_u32r >> 12) / 1024.0);
	}


	return true;
}

