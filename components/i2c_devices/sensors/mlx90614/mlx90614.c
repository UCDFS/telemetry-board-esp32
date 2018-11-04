#include <stdio.h>
#include <rom/crc.h>

#include "esp_log.h"

#include "i2c_bus.h"

#include "mlx90614.h"

#define MLX90614_TAG "MLX90614"

#define error_dev(s, f, d, ...) ESP_LOGE(MLX90614_TAG, "%s: bus %d, addr %02x - " s "\n", f, \
        ((i2c_bus_t *) d->bus)->i2c_port, d->addr, ## __VA_ARGS__)

enum
{
	MLX90614_REG_T_OBJ = 0x07,

	MLX90614_REG_SMBUS_ADDRESS = 0x2e
};

typedef struct {
	uint16_t data;
	uint8_t pec;
} mlx90614_smbus_data_buffer_t;

typedef struct {
	uint8_t address_write;
	uint8_t reg;
	uint8_t address_read;
	uint16_t data;
} mlx90614_smbus_read_bytes_t;

typedef struct {
	uint8_t address_write;
	uint8_t reg;
	uint16_t data;
} mlx90614_smbus_write_bytes_t;

struct mlx90614_dev_t
{
	i2c_bus_handle_t bus;
	uint8_t addr;
};

static bool mlx90614_is_available(mlx90614_handle_t dev);

mlx90614_handle_t mlx90614_init(i2c_bus_handle_t bus, uint8_t dev_addr)
{
	mlx90614_handle_t dev = (mlx90614_handle_t) calloc(1, sizeof(struct mlx90614_dev_t));
	dev->bus = bus;
	dev->addr = dev_addr;

	if (!mlx90614_is_available(dev)) {
		error_dev("Sensor is not available.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	return dev;
}

void mlx90614_delete(mlx90614_handle_t dev, bool del_bus)
{
	if (del_bus) {
		i2c_bus_delete(dev->bus);
		dev->bus = NULL;
	}
	free(dev);
}

static esp_err_t mlx90614_reg_read(mlx90614_handle_t dev, uint8_t reg, uint16_t *data)
{
	mlx90614_smbus_read_bytes_t smbus_bytes = {
			.address_write = dev->addr << 1 | I2C_MASTER_WRITE,
			.reg = reg,
			.address_read = dev->addr << 1 | I2C_MASTER_READ
	};
	mlx90614_smbus_data_buffer_t smbus_buffer;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, smbus_bytes.address_write, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, smbus_bytes.address_read, true);
	i2c_master_read(cmd, (uint8_t *) &smbus_buffer, sizeof(smbus_buffer), I2C_MASTER_ACK);
	i2c_master_stop(cmd);

	esp_err_t err = i2c_bus_cmd_begin(dev->bus, cmd, 1000 / portTICK_RATE_MS);

	// Verify correct PEC
	if (!err) {
		smbus_bytes.data = smbus_buffer.data;
		uint8_t pec = crc8_le(0, (uint8_t *) &smbus_bytes, sizeof(smbus_bytes));

		if (smbus_buffer.pec != pec) {
			err = ESP_FAIL;
		} else {
			*data = smbus_buffer.data;
		}
	}

	i2c_cmd_link_delete(cmd);

	return err;
}

static esp_err_t mlx90614_reg_write(mlx90614_handle_t dev, uint8_t reg, const uint16_t *data)
{
	mlx90614_smbus_write_bytes_t smbus_bytes = {
			.address_write = dev->addr << 1 | I2C_MASTER_WRITE,
			.reg = reg,
			.data = *data
	};
	mlx90614_smbus_data_buffer_t smbus_buffer = {
			.data = *data,
			.pec = crc8_le(0, (uint8_t *) &smbus_bytes, sizeof(smbus_bytes))
	};

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, smbus_bytes.address_write, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write(cmd, (uint8_t *) &smbus_buffer, sizeof(smbus_buffer), true);
	i2c_master_stop(cmd);

	esp_err_t err = i2c_bus_cmd_begin(dev->bus, cmd, 1000 / portTICK_RATE_MS);

	i2c_cmd_link_delete(cmd);

	return err;
}


static bool mlx90614_is_available(mlx90614_handle_t dev)
{
	uint16_t chip_id;

	if (mlx90614_reg_read(dev, MLX90614_REG_SMBUS_ADDRESS, &chip_id) != ESP_OK) {
		error_dev("Could not read chip id.", __FUNCTION__, dev);
		return false;
	}

	if (chip_id != dev->addr) {
		error_dev("Chip id %02x is wrong, should be %02x.", __FUNCTION__, dev, chip_id, dev->addr);
		return false;
	}

	return true;
}

static esp_err_t mlx90614_set_address(mlx90614_handle_t dev, uint16_t addr)
{
	if (mlx90614_reg_write(dev, MLX90614_REG_SMBUS_ADDRESS, &addr) != ESP_OK) {
		error_dev("Could not get set SMBus address", __FUNCTION__, dev);
		return ESP_FAIL;
	}

	dev->addr = (uint8_t) (addr & 0xff);

	return ESP_OK;
}

esp_err_t mlx90614_get_raw(mlx90614_handle_t dev, uint16_t *raw)
{
	if (mlx90614_reg_read(dev, MLX90614_REG_T_OBJ, raw) != ESP_OK) {
		error_dev("Could not get raw data sample", __FUNCTION__, dev);
		return ESP_FAIL;
	}

	return ESP_OK;
}

const static double MLX90614_TEMP_SCALE = 1.0 / 50;
const static double MLX90614_TEMP_OFFSET = 273.15;

esp_err_t mlx90614_get_celcius(mlx90614_handle_t dev, float *data)
{
	uint16_t raw;
	if (mlx90614_get_raw(dev, &raw) != ESP_OK) return ESP_FAIL;

	*data = (float) ((raw * MLX90614_TEMP_SCALE) + MLX90614_TEMP_OFFSET);

	return ESP_OK;
}