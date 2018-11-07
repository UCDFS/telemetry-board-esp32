// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c.h"

typedef void *i2c_bus_handle_t;

typedef struct
{
	i2c_config_t i2c_conf;   /*!<I2C bus parameters*/
	i2c_port_t i2c_port;     /*!<I2C port number */
} i2c_bus_t;

/**
 * @brief Create and init I2C bus and return a I2C bus handle
 *
 * @param port I2C port number
 * @param conf Pointer to I2C parameters
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
i2c_bus_handle_t i2c_bus_create(i2c_port_t port, i2c_config_t *conf);

/**
 * @brief Delete and release the I2C bus object
 *
 * @param bus I2C bus handle
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_delete(i2c_bus_handle_t bus);

/**
 * @brief I2C start sending buffered commands
 *
 * @param bus I2C bus handle
 * @param cmd I2C cmd handle
 * @param ticks_to_wait Maximum blocking time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_cmd_begin(i2c_bus_handle_t bus, i2c_cmd_handle_t cmd, TickType_t ticks_to_wait);

/**
 * @brief I2C master read from slave
 *
 * @param bus I2C bus handle
 * @param addr I2C device address
 * @param reg Register to read from (optional)
 * @param data Data to read to
 * @param len Length of data to read
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_master_read(i2c_bus_handle_t bus, uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len);

/**
 * @brief I2C master write to slave
 *
 * @param bus I2C bus handle
 * @param addr I2C device address
 * @param reg Register to write to (optional)
 * @param data Data to write
 * @param len Length of data to write
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t i2c_bus_master_write(i2c_bus_handle_t bus, uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len);

#endif
