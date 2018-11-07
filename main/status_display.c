#include "status_display.h"

#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "telemetry.h"
#include "temperature.h"

#define DEBUG_STATUS_DISPLAY true

static const char* TAG = "status_display";

static ssd1306_handle_t display;

void status_display_init() {
	// Initialize display
	display = ssd1306_init(i2c_bus_get(), SSD1306_I2C_ADDR_1);
}

ssd1306_handle_t status_display_get() {
	return display;
}


