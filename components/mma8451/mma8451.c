#include <stdio.h>
#include <esp_log.h>
#include <driver/i2c.h>

#include "mma8451.h"

#define MMA8451_REG_OUT_X_MSB 0x01
#define MMA8451_REG_WHO_AM_I_REG 0x0D
#define MMA8451_WHO_AM_I 0x1A

#define XYZ_DATA_CFG_REG 0x0E

#define	CTRL_REG1 0x2A
#define	CTRL_REG2 0x2B
#define	CTRL_REG3 0x2C
#define	CTRL_REG4 0x2D
#define	CTRL_REG5 0x2E

#define ACTIVE_MASK 0b00000001
#define DATA_RATE_MASK 0b00011000

#define MODS_MASK             0x03
#define MODS1_MASK            0x02
#define MODS0_MASK            0x01

#define TAG "MMA8451"

static i2c_port_t i2c_port;
static uint8_t i2c_addr;

/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, i2c_addr << 1, true);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, true);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_READ, true);
    // read bytes and nack after last byte
    i2c_master_read(cmd, data_rd, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write i2c slave device with registered interface
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, i2c_addr << 1 | I2C_MASTER_WRITE, true);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, true);
    // write the data
    i2c_master_write(cmd, data_wr, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read contents of a MMA8451 register
---------------------------------------------------------------------------*/
esp_err_t mma8451_read_reg(uint8_t reg, uint8_t *pdata, uint8_t count)
{
	return i2c_master_read_slave_reg(i2c_port, i2c_addr, reg, pdata, count);
}

/* Write value to specified MMA8451 register
---------------------------------------------------------------------------*/
esp_err_t mma8451_write_reg(uint8_t reg, uint8_t *pdata, uint8_t count)
{
	return i2c_master_write_slave_reg(i2c_port, i2c_addr, reg, pdata, count);
}


/**
 * @brief MMA8451 initialization
 */
esp_err_t mma8451_init(mma8451_config_t *config)
{
	i2c_port = config->i2c_port;
	i2c_addr = config->i2c_addr;

	uint8_t val;

    mma8451_read_reg(MMA8451_REG_WHO_AM_I_REG, &val, 1);
    if (val != MMA8451_WHO_AM_I) {
        return ESP_ERR_NOT_FOUND;
    }

	// Before re-configuring, must enter 'standby' mode
	mma8451_read_reg(CTRL_REG1, &val, 1);
	val &= ~ACTIVE_MASK;
	mma8451_write_reg(CTRL_REG1, &val, 1);

	// Write data rate
    val &= ~DATA_RATE_MASK;
	val |= config->data_rate << 3;
	mma8451_write_reg(CTRL_REG1, &val, 1);

    // Write scale
    val = config->scale;
	mma8451_write_reg(XYZ_DATA_CFG_REG, &val, 1);

	// Setup Hi-Res mode (14-bit)
	mma8451_read_reg(CTRL_REG2, &val, 1);
	val &= ~(MODS_MASK);
	val |= (MODS1_MASK);
	mma8451_write_reg(CTRL_REG2, &val, 1);

	// reconfig done, make active
	mma8451_read_reg(CTRL_REG1, &val, 1);
	val |= (ACTIVE_MASK);
	mma8451_write_reg(CTRL_REG1, &val, 1);

    return ESP_OK;
}

static uint16_t byte_swap(uint16_t data)
{
	return (data >> 8) | (data << 8);
}

esp_err_t mma8451_read_raw(mma8451_raw_data_t *acc) {
    esp_err_t err = mma8451_read_reg(MMA8451_REG_OUT_X_MSB, (uint8_t *)acc, sizeof(mma8451_raw_data_t));
    if (err != ESP_OK) {
        return err;
    }

    // byte-swap values to make little-endian
    acc->x = byte_swap(acc->x);
    acc->y = byte_swap(acc->y);
    acc->z = byte_swap(acc->z);

    // shift each value to align 14-bits in 16-bit ints
    acc->x /= 4;
    acc->y /= 4;
    acc->z /= 4;

    return err;
}