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
#ifndef SC16IS752_H
#define SC16IS752_H

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "stdint.h"

#define SC16IS752_I2C_ADDR_VDD_VDD 0x48
#define SC16IS752_I2C_ADDR_VDD_VSS 0x49
#define SC16IS752_I2C_ADDR_VDD_SCL 0x4a
#define SC16IS752_I2C_ADDR_VDD_SDA 0x4b
#define SC16IS752_I2C_ADDR_VSS_VDD 0x4c
#define SC16IS752_I2C_ADDR_VSS_VSS 0x4d
#define SC16IS752_I2C_ADDR_VSS_SCL 0x4e
#define SC16IS752_I2C_ADDR_VSS_SDA 0x4f
#define SC16IS752_I2C_ADDR_SCL_VDD 0x50
#define SC16IS752_I2C_ADDR_SCL_VSS 0x51
#define SC16IS752_I2C_ADDR_SCL_SCL 0x52
#define SC16IS752_I2C_ADDR_SCL_SDA 0x53
#define SC16IS752_I2C_ADDR_SDA_VDD 0x54
#define SC16IS752_I2C_ADDR_SDA_VSS 0x55
#define SC16IS752_I2C_ADDR_SDA_SCL 0x56
#define SC16IS752_I2C_ADDR_SDA_SDA 0x57

/**
 * @brief UART hardware flow control modes
 */
typedef enum {
	SC16IS752_UART_HW_FLOWCTRL_DISABLE = 0x0,   /*!< disable hardware flow control*/
	SC16IS752_UART_HW_FLOWCTRL_RTS     = 0x1,   /*!< enable RX hardware flow control (rts)*/
	SC16IS752_UART_HW_FLOWCTRL_CTS     = 0x2,   /*!< enable TX hardware flow control (cts)*/
	SC16IS752_UART_HW_FLOWCTRL_CTS_RTS = 0x3,   /*!< enable hardware flow control*/
	SC16IS752_UART_HW_FLOWCTRL_MAX     = 0x4,
} sc16is752_uart_hw_flowcontrol_t;

/**
 * @brief UART word length constants
 */
typedef enum {
	SC16IS752_UART_DATA_5_BITS = 0x0,    /*!< word length: 5bits*/
	SC16IS752_UART_DATA_6_BITS = 0x1,    /*!< word length: 6bits*/
	SC16IS752_UART_DATA_7_BITS = 0x2,    /*!< word length: 7bits*/
	SC16IS752_UART_DATA_8_BITS = 0x3,    /*!< word length: 8bits*/
	SC16IS752_UART_DATA_BITS_MAX = 0x4,
} sc16is752_uart_word_length_t;

/**
 * @brief UART stop bits number
 */
typedef enum {
	SC16IS752_UART_STOP_BITS_1   = 0x1,  /*!< stop bit: 1bit*/
	SC16IS752_UART_STOP_BITS_1_5 = 0x2,  /*!< stop bit: 1.5bits*/
	SC16IS752_UART_STOP_BITS_2   = 0x3,  /*!< stop bit: 2bits*/
	SC16IS752_UART_STOP_BITS_MAX = 0x4,
} sc16is752_uart_stop_bits_t;

/**
 * @brief UART peripheral number
 */
typedef enum {
	SC16IS752_UART_CHANNEL_GPIO = 0x0,
	SC16IS752_UART_CHANNEL_A = 0x0,
	SC16IS752_UART_CHANNEL_B = 0x1,
	SC16IS752_UART_NUM_MAX,
} sc16is752_uart_channel_t;

/**
 * @brief UART parity constants
 */
typedef enum {
	SC16IS752_UART_PARITY_DISABLE = 0x0,  /*!< Disable UART parity*/
	SC16IS752_UART_PARITY_EVEN = 0x2,     /*!< Enable UART even parity*/
	SC16IS752_UART_PARITY_ODD  = 0x3      /*!< Enable UART odd parity*/
} sc16is752_uart_parity_t;

/**
 * @brief UART FIFO interrupt trigger level
 */
typedef enum {
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_DISABLED = 0x0,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_4,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_8,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_12,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_16,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_20,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_24,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_28,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_32,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_36,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_40,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_44,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_48,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_52,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_56,
	SC16IS752_UART_FIFO_TRIGGER_LEVEL_60
} sc16is752_uart_fifo_trigger_level;

/**
 * @brief UART configuration parameters for uart_param_config function
 */
typedef struct {
	int baud_rate;                      /*!< UART baud rate*/
	sc16is752_uart_word_length_t data_bits;       /*!< UART byte size*/
	sc16is752_uart_parity_t parity;               /*!< UART parity mode*/
	sc16is752_uart_stop_bits_t stop_bits;         /*!< UART stop bits*/
	sc16is752_uart_hw_flowcontrol_t flow_ctrl;    /*!< UART HW flow control mode (cts/rts)*/
	uint8_t rx_flow_ctrl_thresh;        /*!< UART HW RTS threshold*/
} sc16is752_uart_config_t;

typedef enum {
	SC16IS752_GPIO_NUM_0 = 0,
	SC16IS752_GPIO_NUM_1,
	SC16IS752_GPIO_NUM_2,
	SC16IS752_GPIO_NUM_3,
	SC16IS752_GPIO_NUM_4,
	SC16IS752_GPIO_NUM_5,
	SC16IS752_GPIO_NUM_6,
	SC16IS752_GPIO_NUM_7,
} sc16is752_gpio_num_t;

typedef enum {
	SC16IS752_GPIO_MODE_INPUT = 0,
	SC16IS752_GPIO_MODE_OUTPUT = 1
} sc16is752_gpio_mode_t;

typedef enum {
	SC16IS752_GPIO_INTR_DISABLE = 0,
	SC16IS752_GPIO_INTR_POSEDGE = 1,
	SC16IS752_GPIO_INTR_NEGEDGE = 2,
	SC16IS752_GPIO_INTR_ANYEDGE = 3
} sc16is752_gpio_intr_t;

typedef struct sc16is752_dev_t *sc16is752_handle_t;

/**
 * @brief   Create and initialization device object and return a device handle
 *
 * @param   bus I2C bus object handle
 * @param   dev_addr I2C device address of device
 *
 * @return
 *     - device object handle of ssd1306
 */
sc16is752_handle_t sc16is752_init(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief   Delete and release a device object
 *
 * @param   dev object handle of ssd1306
 * @param   del_bus Whether to delete the I2C bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
void sc16is752_delete(sc16is752_handle_t dev, bool del_bus);

esp_err_t sc16is752_setup(sc16is752_handle_t dev, uint64_t xtal_frequency, gpio_num_t interrupt_gpio_num, uint32_t interrupt_task_priority, int interrupt_task_core);
esp_err_t sc16is752_uart_set_baudrate(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, int baud_rate);
esp_err_t sc16is752_uart_set_word_length(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, sc16is752_uart_word_length_t data_bits);
esp_err_t sc16is752_uart_set_stop_bits(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, sc16is752_uart_stop_bits_t stop_bits);
esp_err_t sc16is752_uart_set_parity(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, sc16is752_uart_parity_t parity_mode);
esp_err_t sc16is752_uart_param_config(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, const sc16is752_uart_config_t *config);
esp_err_t sc16is752_uart_driver_install(sc16is752_handle_t dev, sc16is752_uart_channel_t channel,
		size_t rx_buffer_size, size_t tx_buffer_size, bool fifo_enable,
		sc16is752_uart_fifo_trigger_level fifo_rx_level, sc16is752_uart_fifo_trigger_level fifo_tx_level);
size_t sc16is752_uart_read_bytes(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, const char* src, size_t size, TickType_t ticks_to_wait);
void sc16is752_uart_write_bytes(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, const char* src, size_t size);
esp_err_t sc16is752_gpio_set_intr(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, bool enabled);
esp_err_t sc16is752_gpio_set_direction(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, sc16is752_gpio_mode_t mode);
sc16is752_gpio_mode_t sc16is752_gpio_get_direction(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num);
esp_err_t sc16is752_gpio_set_level(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, bool level);
int sc16is752_gpio_get_level(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num);


#endif