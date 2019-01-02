#include <sc16is752.h>
#include <string.h>
#include <esp_log.h>
#include "nextion_display.h"

#include "i2c.h"
#include "task_priorities.h"

static const char* TAG = "nextion_display";

#define SC16IS752_XTAL_FREQUENCY 14745600

#define NEXTION_BAUD_RATE 115200
#define NEXTION_SC16IS752_UART_CHANNEL SC16IS752_UART_CHANNEL_A

#define DISPLAY_BUFFER_SIZE 512

static sc16is752_handle_t sc16is752_handle;

void nextion_display_receive_task();

void nextion_display_init() {
	sc16is752_handle = sc16is752_init(i2c_bus_get(), SC16IS752_I2C_ADDR_VDD_VDD);

	// Quit if device not present
	if (sc16is752_handle == NULL) {
		return;
	}

	sc16is752_setup(sc16is752_handle, SC16IS752_XTAL_FREQUENCY, GPIO_NUM_15, uxTaskPriorityGet(NULL), APP_CPU_NUM);

	sc16is752_uart_config_t uart_config = {
			.baud_rate = NEXTION_BAUD_RATE,
			.data_bits = SC16IS752_UART_DATA_8_BITS,
			.parity = SC16IS752_UART_PARITY_DISABLE,
			.stop_bits = SC16IS752_UART_STOP_BITS_1
	};

	sc16is752_uart_param_config(sc16is752_handle, NEXTION_SC16IS752_UART_CHANNEL, &uart_config);
	sc16is752_uart_driver_install(sc16is752_handle, NEXTION_SC16IS752_UART_CHANNEL, DISPLAY_BUFFER_SIZE, DISPLAY_BUFFER_SIZE, true,
			SC16IS752_UART_FIFO_TRIGGER_LEVEL_32, SC16IS752_UART_FIFO_TRIGGER_LEVEL_48);

	// Initiate receive task
	xTaskCreatePinnedToCore(nextion_display_receive_task, "nextion_display_receive_task", 4096, NULL, PRIORITY_TASK_NEXTION_DISPLAY, NULL, APP_CPU_NUM);
}

void nextion_display_receive_task() {
	vTaskDelay(portMAX_DELAY);
}


