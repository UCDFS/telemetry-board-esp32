// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//	 http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "driver/i2c.h"
#include "sc16is752.h"
#include <esp_log.h>
#include <i2c_bus.h>
#include <string.h>

#define SC16IS752_TAG "SC16IS752"


#define error_dev(s, f, d, ...) ESP_LOGE(SC16IS752_TAG, "%s: bus %d, addr %02x - " s, f, \
        ((i2c_bus_t *) d->bus)->i2c_port, d->addr, ## __VA_ARGS__)

enum
{
	SC16IS752_REG_RHR = 0x00,   // RX FIFO
	SC16IS752_REG_THR = 0x00,   // TX FIFO
	SC16IS752_REG_IER,          // Interrupt enable
	SC16IS752_REG_IIR = 0x02,   // Interrupt identification
	SC16IS752_REG_FCR = 0x02,   // FIFO control
	SC16IS752_REG_LCR,          // Line control
	SC16IS752_REG_MCR,          // Modem control
	SC16IS752_REG_LSR,          // Line status
	SC16IS752_REG_MSR,          // Modem status
	SC16IS752_REG_SPR,          // Scratch pad
	SC16IS752_REG_TXLVL,        // TX FIFO level
	SC16IS752_REG_RXLVL,        // RX FIFO level
	SC16IS752_REG_IODIR,        // I/O direction
	SC16IS752_REG_IOSTATE,      // I/O state
	SC16IS752_REG_IOINTENA,     // I/O interrupt
	SC16IS752_REG_IOCONTROL = 0x0e,    // I/O control
	SC16IS752_REG_EFCR,         // Extra features

	// If ((MCR[2] == 1) && (EFR[4] == 1))
	SC16IS752_REG_TCR = 0x06,   // Transmit control
	SC16IS752_REG_TLR,          // Trigger level

	// If ((LCR[7] == 1) && (LCR != 0xBF))
	SC16IS752_REG_DLL = 0x00,   // Divisor latch low
	SC16IS752_REG_DLH,          // Divisor latch high

	// If (LCR == 0xBF)
	SC16IS752_REG_EFR = 0x02,   // Enhanced features
	SC16IS752_REG_XON1 = 0x04,  // Xon1 word
	SC16IS752_REG_XON2,         // Xon2 word
	SC16IS752_REG_XOFF1,        // Xoff1 word
	SC16IS752_REG_XOFF2,        // Xoff2 word
};

struct sc16is752_reg_ier {
	bool rx_data_available_interrupt :1;
	bool thr_empty_interrupt :1;
	bool receive_line_status_interrupt :1;
	bool modem_status_interrupt :1;
	bool sleep_mode :1;
	bool xoff :1;
	bool rts_interrupt_enable :1;
	bool cts_interrupt_enable :1;
};

enum sc16is752_interrupt_source {
	SC16IS752_INTERRUPT_RX_LINE_ERROR = 0x3,
	SC16IS752_INTERRUPT_RX_TIMEOUT = 0x6,
	SC16IS752_INTERRUPT_RX_FIFO_FULL = 0x2,
	SC16IS752_INTERRUPT_TX_FIFO_FULL = 0x1,
	SC16IS752_INTERRUPT_GPIO = 0x18,
	SC16IS752_INTERRUPT_MODEM = 0x0,
	SC16IS752_INTERRUPT_XOFF = 0x8,
	SC16IS752_INTERRUPT_CTS_RTS = 0x10
};

struct sc16is752_reg_iir {
	bool interrupt_status :1;
	enum sc16is752_interrupt_source interrupt_type :5;
	uint8_t fifo_enabled :2;
};

struct sc16is752_reg_fcr {
	bool fifo_enable :1;
	bool rx_fifo_reset :1;
	bool tx_fifo_reset :1;
	uint8_t unused :1;
	uint8_t tx_trigger_level :2;
	uint8_t rx_trigger_level :2;
};

struct sc16is752_reg_lcr {
	uint8_t word_length :2;
	bool stop_bits :1;
	bool parity_enable :1;
	bool even_parity :1;
	bool set_parity :1;
	bool set_break :1;
	bool divisor_latch_enable :1;
};

struct sc16is752_reg_mcr {
	bool dtr :1;
	bool rts :1;
	bool tcr_tlr_enable :1;
	uint8_t unused :1;
	bool loopback_enable :1;
	bool xon_any :1;
	bool irda_mode_enable :1;
	bool clock_divisor :1;
};

struct sc16is752_reg_lsr {
	bool data_in_receiver :1;
	bool overrun_error :1;
	bool parity_error :1;
	bool framing_error :1;
	bool break_interrupt :1;
	bool thr_empty :1;
	bool thr_tsr_empty :1;
	bool fifo_data_error :1;
};

struct sc16is752_reg_tlr {
	uint16_t tx_level :4;
	uint16_t rx_level :4;
};

struct sc16is752_reg_iocontrol {
	bool latch :1;
	bool io4_io7_io_or_modem :1;
	bool io0_io3_io_or_modem :1;
	bool uart_software_reset :1;
	uint8_t unused :4;
};

struct sc16is752_reg_efcr {
	bool mode_9bit_enable :1;
	bool receiver_disable :1;
	bool transmitter_disable :1;
	uint8_t unused1 :4;
	bool irda_mode :1;
};

struct sc16is752_reg_efr {
	uint8_t software_flow_control :4;
	bool enable_enhanced_functions :1;
	bool special_character_detect :1;
	bool auto_rts :1;
	bool auto_cts :1;
};

struct sc16is752_uart_obj_t {
	bool enabled;
	bool fifo_enabled;

	bool tx_immediate;

	RingbufHandle_t rx_buffer;
	RingbufHandle_t tx_buffer;

	SemaphoreHandle_t rx_semaphore;
	SemaphoreHandle_t tx_semaphore;
};


typedef void (*sc16is752_gpio_isr_t)(void*);
struct sc16is752_gpio_isr_func_t {
	sc16is752_gpio_isr_t fn;   /*!< isr function */
	void* args;      /*!< isr function args */
};

struct sc16is752_dev_t {
	i2c_bus_handle_t bus;
	uint8_t addr;

	bool active;

	uint64_t xtal_frequency;

	uint8_t gpio_state;
	sc16is752_gpio_intr_t gpio_intr_types[8];
	struct sc16is752_gpio_isr_func_t gpio_isr_func[8];

	bool interrupt_mode;
	gpio_num_t interrupt_gpio_num;
	TaskHandle_t interrupt_task;

	struct sc16is752_uart_obj_t uart_obj[2];
};

const uint8_t SC16IS752_LCR_MODE_SPECIAL_REGISTER_SET = 0x80;
const uint8_t SC16IS752_LCR_MODE_ENHANCED_REGISTER_SET = 0xBF;

#define sc16is752_update_reg(dev, channel, addr, type, elem, value) \
        { \
            struct type __reg; \
            if (sc16is752_reg_read(dev, (channel), (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
            __reg.elem = (value); \
            if (sc16is752_reg_write(dev, (channel), (addr), (uint8_t*)&__reg, 1) != ESP_OK) \
                return ESP_FAIL; \
        }

static esp_err_t sc16is752_reset(sc16is752_handle_t dev);
static bool sc16is752_is_available(sc16is752_handle_t dev);

sc16is752_handle_t sc16is752_init(i2c_bus_handle_t bus, uint8_t dev_addr)
{
	sc16is752_handle_t dev = (sc16is752_handle_t) calloc(1, sizeof(struct sc16is752_dev_t));
	dev->bus = bus;
	dev->addr = dev_addr;

	dev->active = false;

	if (!sc16is752_is_available(dev)) {
		error_dev("Sensor is not available.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	// Reset the sensor
	if (sc16is752_reset(dev) != ESP_OK) {
		error_dev("Could not reset the sensor device.", __FUNCTION__, dev);
		free(dev);
		return NULL;
	}

	return dev;
}

void sc16is752_delete(sc16is752_handle_t dev, bool del_bus)
{
	if (del_bus) {
		i2c_bus_delete(dev->bus);
		dev->bus = NULL;
	}

	if (dev->interrupt_mode) {
		gpio_isr_handler_remove(dev->interrupt_gpio_num);
		vTaskDelete(dev->interrupt_task);
	}

	free(dev);
}

static esp_err_t sc16is752_reg_read(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, uint8_t reg, uint8_t *data, size_t size)
{
	reg = reg << 3 | channel << 1;

	esp_err_t err = i2c_bus_master_read(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on read %d byte from I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t sc16is752_reg_write(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, uint8_t reg, uint8_t *data, size_t size)
{
	reg = reg << 3 | channel << 1;

	esp_err_t err = i2c_bus_master_write(dev->bus, dev->addr, &reg, data, size);
	if (err) {
		error_dev("Error %d on write %d byte to I2C slave register %02x.", __FUNCTION__, dev, err, size, reg);
	}

	return err;
}

static esp_err_t sc16is752_reset(sc16is752_handle_t dev)
{
	struct sc16is752_reg_iocontrol iocontrol = { .uart_software_reset = true };

	uint8_t reg = SC16IS752_REG_IOCONTROL << 3 | SC16IS752_UART_CHANNEL_GPIO << 1;

	// I2C write with no ACK due to reset
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev->addr << 1 | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write(cmd, (uint8_t *) &iocontrol, 1, false);
	i2c_master_stop(cmd);

	esp_err_t err = i2c_bus_cmd_begin(dev->bus, cmd, 1000 / portTICK_RATE_MS);
	if (err) {
		error_dev("Error %d reset on write %d byte to I2C slave register %02x.", __FUNCTION__, dev, err, 1, reg);
	}

	i2c_cmd_link_delete(cmd);

	// Allow time for reset to complete
	vTaskDelay(2000 / portTICK_PERIOD_MS);

	return ESP_OK;
}

static bool sc16is752_is_available(sc16is752_handle_t dev) {
	return true;
}

static esp_err_t sc16is752_set_active(sc16is752_handle_t dev, bool active)
{
	sc16is752_update_reg(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IER, sc16is752_reg_ier, sleep_mode, !active);

	return ESP_OK;
}

static void sc16is752_skip_internal(sc16is752_handle_t dev, sc16is752_uart_channel_t channel) {
	struct sc16is752_reg_lsr lsr;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_LSR, (uint8_t *) &lsr, 1);

	uint8_t buffer;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_RHR, &buffer, 1);
}

static void sc16is752_receive_internal(sc16is752_handle_t dev, sc16is752_uart_channel_t channel) {
	uint8_t size;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_RXLVL, &size, 1);
	if (size > 64) {
		return;
	}

	uint8_t buffer[64];
	sc16is752_reg_read(dev, channel, SC16IS752_REG_RHR, (uint8_t *) &buffer, size);

	xSemaphoreTake(dev->uart_obj[channel].rx_semaphore, portMAX_DELAY);
	xRingbufferSend(dev->uart_obj[channel].rx_buffer, buffer, size, 10 / portTICK_PERIOD_MS);
	xSemaphoreGive(dev->uart_obj[channel].rx_semaphore);
}

static void sc16is752_transmit_internal(sc16is752_handle_t dev, sc16is752_uart_channel_t channel) {
	uint8_t size;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_TXLVL, &size, 1);
	if (size > 64) {
		return;
	}

	size_t available;
	xSemaphoreTake(dev->uart_obj[channel].tx_semaphore, portMAX_DELAY);
	uint8_t *buffer = xRingbufferReceiveUpTo(dev->uart_obj[channel].tx_buffer, &available, 0 / portTICK_PERIOD_MS, size);
	xSemaphoreGive(dev->uart_obj[channel].tx_semaphore);

	if (buffer != NULL) {
		sc16is752_reg_write(dev, channel, SC16IS752_REG_THR, buffer, available);

		vRingbufferReturnItem(dev->uart_obj[channel].tx_buffer, buffer);
	} else {
		available = 0;
	}

	dev->uart_obj[channel].tx_immediate = available < size;
}

static void sc16is752_gpio_intr_internal(sc16is752_handle_t dev) {
	uint8_t gpio_state;
	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IOSTATE, &gpio_state, 1);

	uint8_t state_changes = gpio_state ^ dev->gpio_state;
	for (int gpio_num = 0; gpio_num < 8; gpio_num++) {
		sc16is752_gpio_intr_t intr_type = dev->gpio_intr_types[gpio_num];
		if (intr_type == SC16IS752_GPIO_INTR_DISABLE) {
			continue;
		}

		sc16is752_gpio_isr_t fn = dev->gpio_isr_func[gpio_num].fn;
		void *fn_args = dev->gpio_isr_func[gpio_num].args;
		if (fn == NULL) {
			continue;
		}

		bool run_fn = false;
		if (intr_type == SC16IS752_GPIO_INTR_ANYEDGE) {
			run_fn = true;
		} else {
			bool gpio_new_state = ((state_changes >> gpio_num) & 0x1) == 1;
			if (intr_type == SC16IS752_GPIO_INTR_POSEDGE) {
				run_fn = gpio_new_state;
			} else if (intr_type == SC16IS752_GPIO_INTR_NEGEDGE) {
				run_fn = !gpio_new_state;
			}
		}

		if (run_fn) {
			fn(fn_args);
		}
	}
}

static esp_err_t sc16is752_process_irq_iir(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, struct sc16is752_reg_iir iir) {
	if (iir.interrupt_status) {
		return ESP_OK;
	}

	switch (iir.interrupt_type) {
		case SC16IS752_INTERRUPT_RX_LINE_ERROR:
			sc16is752_skip_internal(dev, channel);
			break;
		case SC16IS752_INTERRUPT_RX_TIMEOUT:
		case SC16IS752_INTERRUPT_RX_FIFO_FULL:
			sc16is752_receive_internal(dev, channel);
			break;
		case SC16IS752_INTERRUPT_TX_FIFO_FULL:
			sc16is752_transmit_internal(dev, channel);
			break;
		case SC16IS752_INTERRUPT_GPIO:
			sc16is752_gpio_intr_internal(dev);
			break;
		case SC16IS752_INTERRUPT_MODEM:
		case SC16IS752_INTERRUPT_XOFF:
		case SC16IS752_INTERRUPT_CTS_RTS:
			break;
	}

	return ESP_OK;
}

static void sc16is752_process_irq(sc16is752_handle_t dev) {
	struct sc16is752_reg_iir iir_a = {};
	struct sc16is752_reg_iir iir_b = {};

	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_A, SC16IS752_REG_IIR, (uint8_t *) &iir_a, 1);
	sc16is752_process_irq_iir(dev, SC16IS752_UART_CHANNEL_A, iir_a);

	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_B, SC16IS752_REG_IIR, (uint8_t *) &iir_b, 1);
	sc16is752_process_irq_iir(dev, SC16IS752_UART_CHANNEL_B, iir_b);
}

static void sc16is752_irq_task(void* arg) {
	sc16is752_handle_t dev = arg;

	while (true) {
		ulTaskNotifyTake(true, portMAX_DELAY);

		while (gpio_get_level(dev->interrupt_gpio_num) == 0) {
			sc16is752_process_irq(dev);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
	}
}

static void IRAM_ATTR sc16is752_irq_handler(void* arg) {
	sc16is752_handle_t dev = arg;
	vTaskNotifyGiveFromISR(dev->interrupt_task, NULL);
}

esp_err_t sc16is752_setup(sc16is752_handle_t dev, uint64_t xtal_frequency, gpio_num_t interrupt_gpio_num,
		uint32_t interrupt_task_priority, int interrupt_task_core)
{
	dev->xtal_frequency = xtal_frequency;
	dev->interrupt_mode = true;
	dev->interrupt_gpio_num = interrupt_gpio_num;

	gpio_set_pull_mode(interrupt_gpio_num, GPIO_FLOATING);
	gpio_set_direction(interrupt_gpio_num, GPIO_MODE_INPUT);
	gpio_set_intr_type(interrupt_gpio_num, GPIO_INTR_NEGEDGE);

	xTaskCreatePinnedToCore(sc16is752_irq_task, "sc16is752_interrupt_task", 2048, (void *) dev, interrupt_task_priority, &dev->interrupt_task, interrupt_task_core);
	gpio_isr_handler_add(interrupt_gpio_num, sc16is752_irq_handler, (void *) dev);

	return ESP_OK;
}

esp_err_t sc16is752_uart_set_baudrate(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, int baud_rate) {
	// Calculate divisor
	uint64_t divisor = dev->xtal_frequency / 16 / baud_rate;
	bool prescaler = false;
	if (divisor > 0xffff) {
		divisor /= 4;
		prescaler = true;
	}

	// Save LCR to restore later
	struct sc16is752_reg_lcr lcr;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &lcr, 1);

	// Enable enhanced features
	sc16is752_reg_write(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &SC16IS752_LCR_MODE_ENHANCED_REGISTER_SET, 1);
	sc16is752_update_reg(dev, channel, SC16IS752_REG_EFR, sc16is752_reg_efr, enable_enhanced_functions, true);

	// Restore LCR
	sc16is752_update_reg(dev, channel, SC16IS752_REG_LCR, sc16is752_reg_lcr, word_length, 0);

	// Set clock divisor (/4)
	sc16is752_update_reg(dev, channel, SC16IS752_REG_MCR, sc16is752_reg_mcr, clock_divisor, prescaler);

	// Enable writing divisor
	sc16is752_reg_write(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &SC16IS752_LCR_MODE_SPECIAL_REGISTER_SET, 1);

	// Write divisor
	uint8_t divisor_l = (uint8_t) (divisor & 0xff);
	uint8_t divisor_h = (uint8_t) ((divisor >> 8) & 0xff);
	sc16is752_reg_write(dev, channel, SC16IS752_REG_DLL, &divisor_l, 1);
	sc16is752_reg_write(dev, channel, SC16IS752_REG_DLH, &divisor_h, 1);

	// Restore LCR
	sc16is752_reg_write(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &lcr, 1);

	return ESP_OK;
}

esp_err_t sc16is752_uart_set_word_length(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, sc16is752_uart_word_length_t data_bits) {
	sc16is752_update_reg(dev, channel, SC16IS752_REG_LCR, sc16is752_reg_lcr, word_length, data_bits);

	return ESP_OK;
}

esp_err_t sc16is752_uart_set_stop_bits(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, sc16is752_uart_stop_bits_t stop_bits) {
	sc16is752_update_reg(dev, channel, SC16IS752_REG_LCR, sc16is752_reg_lcr, stop_bits, stop_bits);

	return ESP_OK;
}

esp_err_t sc16is752_uart_set_parity(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, sc16is752_uart_parity_t parity_mode) {
	struct sc16is752_reg_lcr lcr;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &lcr, 1);

	if (parity_mode != SC16IS752_UART_PARITY_DISABLE) {
		lcr.parity_enable = true;
		lcr.even_parity = parity_mode == SC16IS752_UART_PARITY_EVEN;
	} else {
		lcr.parity_enable = false;
	}

	sc16is752_reg_write(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &lcr, 1);

	return ESP_OK;
}

esp_err_t sc16is752_uart_param_config(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, const sc16is752_uart_config_t *config) {
	esp_err_t r;

	r = sc16is752_uart_set_baudrate(dev, channel, config->baud_rate);
	if (r != ESP_OK) return r;
	r = sc16is752_uart_set_word_length(dev, channel, config->data_bits);
	if (r != ESP_OK) return r;
	r = sc16is752_uart_set_stop_bits(dev, channel, config->stop_bits);
	if (r != ESP_OK) return r;
	r = sc16is752_uart_set_parity(dev, channel, config->parity);
	if (r != ESP_OK) return r;

	return ESP_OK;
}

esp_err_t sc16is752_uart_driver_install(sc16is752_handle_t dev, sc16is752_uart_channel_t channel,
		size_t rx_buffer_size, size_t tx_buffer_size, bool fifo_enable,
		sc16is752_uart_fifo_trigger_level fifo_rx_level, sc16is752_uart_fifo_trigger_level fifo_tx_level) {

	// Configuration
	struct sc16is752_uart_obj_t *uart_obj = &dev->uart_obj[channel];
	if (uart_obj->enabled) {
		error_dev("UART %c is already installed.", __FUNCTION__, dev, channel == SC16IS752_UART_CHANNEL_A ? 'A' : 'B');
		return ESP_FAIL;
	}

	uart_obj->enabled = true;
	uart_obj->fifo_enabled = fifo_enable;

	uart_obj->rx_semaphore = xSemaphoreCreateBinary();
	uart_obj->tx_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(uart_obj->rx_semaphore);
	xSemaphoreGive(uart_obj->tx_semaphore);

	uart_obj->rx_buffer = xRingbufferCreate(rx_buffer_size, RINGBUF_TYPE_BYTEBUF);
	uart_obj->tx_buffer = xRingbufferCreate(tx_buffer_size, RINGBUF_TYPE_BYTEBUF);

	uart_obj->tx_immediate = true;

	// Read FIFO configuration
	struct sc16is752_reg_fcr fcr;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_FCR, (uint8_t *) &fcr, 1);

	fcr.fifo_enable = fifo_enable;
	if (fifo_enable) {
		// Use TLR for trigger level
		fcr.rx_trigger_level = 0;
		fcr.tx_trigger_level = 0;

		// Reset FIFOs
		fcr.rx_fifo_reset = true;
		fcr.tx_fifo_reset = true;

		// Write FIFO configuration
		sc16is752_reg_write(dev, channel, SC16IS752_REG_FCR, (uint8_t *) &fcr, 1);

		struct sc16is752_reg_tlr tlr;
		tlr.rx_level = fifo_rx_level;
		tlr.tx_level = fifo_tx_level;

		// Save LCR to restore later
		struct sc16is752_reg_lcr lcr;
		sc16is752_reg_read(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &lcr, 1);

		// Enable enhanced features
		sc16is752_reg_write(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &SC16IS752_LCR_MODE_ENHANCED_REGISTER_SET, 1);
		sc16is752_update_reg(dev, channel, SC16IS752_REG_EFR, sc16is752_reg_efr, enable_enhanced_functions, true);

		// Restore LCR
		sc16is752_reg_write(dev, channel, SC16IS752_REG_LCR, (uint8_t *) &lcr, 1);

		// Enable TLR
		sc16is752_update_reg(dev, channel, SC16IS752_REG_MCR, sc16is752_reg_mcr, tcr_tlr_enable, true);

		// Write FIFO trigger levels
		sc16is752_reg_write(dev, channel, SC16IS752_REG_TLR, (uint8_t *) &tlr, 1);
	} else {
		// Write FIFO configuration
		sc16is752_reg_write(dev, channel, SC16IS752_REG_FCR, (uint8_t *) &fcr, 1);
	}

	// Set up interrupts
	struct sc16is752_reg_ier ier;
	sc16is752_reg_read(dev, channel, SC16IS752_REG_IER, (uint8_t *) &ier, 1);
	ier.receive_line_status_interrupt = true;
	ier.thr_empty_interrupt = true;
	ier.rx_data_available_interrupt = true;
	sc16is752_reg_write(dev, channel, SC16IS752_REG_IER, (uint8_t *) &ier, 1);

	return ESP_OK;
}

size_t sc16is752_uart_read_bytes(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, const char* src, size_t size, TickType_t ticks_to_wait) {
	size_t available = 0;
	xSemaphoreTake(dev->uart_obj[channel].rx_semaphore, ticks_to_wait);
	uint8_t *buffer = xRingbufferReceiveUpTo(dev->uart_obj[channel].rx_buffer, &available, ticks_to_wait, size);
	if (buffer != NULL) {
		memcpy((void *) src, buffer, available);

		vRingbufferReturnItem(dev->uart_obj[channel].rx_buffer, (void *) buffer);
	}
	xSemaphoreGive(dev->uart_obj[channel].rx_semaphore);

	return available;
}

void sc16is752_uart_write_bytes(sc16is752_handle_t dev, sc16is752_uart_channel_t channel, const char* src, size_t size) {
	xSemaphoreTake(dev->uart_obj[channel].tx_semaphore, portMAX_DELAY);
	xRingbufferSend(dev->uart_obj[channel].tx_buffer, src, size, 10 / portTICK_PERIOD_MS);
	xSemaphoreGive(dev->uart_obj[channel].tx_semaphore);

	if (dev->uart_obj[channel].tx_immediate) {
		sc16is752_transmit_internal(dev, channel);
	}
}

esp_err_t sc16is752_gpio_set_intr_type(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, sc16is752_gpio_intr_t intr_type) {
	uint8_t iointena;
	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IOINTENA, &iointena, 1);

	if (intr_type != SC16IS752_GPIO_INTR_DISABLE) {
		iointena |= BIT(gpio_num);
	} else {
		iointena &= ~BIT(gpio_num);
	}

	sc16is752_reg_write(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IOINTENA, &iointena, 1);

	dev->gpio_intr_types[gpio_num] = intr_type;

	return ESP_OK;
}

esp_err_t sc16is752_gpio_set_direction(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, sc16is752_gpio_mode_t mode) {
	uint8_t gpio_direction;
	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IODIR, &gpio_direction, 1);

	if (mode) {
		gpio_direction |= BIT(gpio_num);
	} else {
		gpio_direction &= ~BIT(gpio_num);
	}

	sc16is752_reg_write(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IODIR, &gpio_direction, 1);

	return ESP_OK;
}

sc16is752_gpio_mode_t sc16is752_gpio_get_direction(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num) {
	uint8_t gpio_direction;
	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IODIR, &gpio_direction, 1);

	return (gpio_direction >> gpio_num) & 0x1;
}

esp_err_t sc16is752_gpio_set_level(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, bool level) {
	uint8_t gpio_state;
	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IOSTATE, &gpio_state, 1);

	if (level) {
		gpio_state |= BIT(gpio_num);
	} else {
		gpio_state &= ~BIT(gpio_num);
	}

	sc16is752_reg_write(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IOSTATE, &gpio_state, 1);

	return ESP_OK;
}

int sc16is752_gpio_get_level(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num) {
	uint8_t gpio_state;
	sc16is752_reg_read(dev, SC16IS752_UART_CHANNEL_GPIO, SC16IS752_REG_IOSTATE, &gpio_state, 1);

	return (gpio_state >> gpio_num) & 0x1;
}

void sc16is752_gpio_isr_handler_add(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num, void (*fn)(void *), void *fn_args) {
	dev->gpio_isr_func[gpio_num].fn = fn;
	dev->gpio_isr_func[gpio_num].args = fn_args;
}

void sc16us752_gpio_isr_handler_remove(sc16is752_handle_t dev, sc16is752_gpio_num_t gpio_num) {
	dev->gpio_isr_func[gpio_num].fn = NULL;
	dev->gpio_isr_func[gpio_num].args = NULL;
}


