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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <esp_log.h>
#include <time.h>
#include <sys/time.h>
#include <i2c_bus.h>
#include <string.h>
#include <math.h>

#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

#define SSD1306_CMD OLED_CONTROL_BYTE_CMD_STREAM
#define SSD1306_DATA OLED_CONTROL_BYTE_DATA_STREAM

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Scrolling Command Table (pg.29)
#define OLED_CMD_SCROLL_RIGHT           0x26
#define OLED_CMD_SCROLL_LEFT            0x27
#define OLED_CMD_SCROLL_ACTIVATE        0x2F
#define OLED_CMD_SCROLL_DEACTIVATE      0x2E

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_NORMAL    0xC0
#define OLED_CMD_SET_COM_SCAN_REMAP     0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

static const uint8_t INIT_COMMANDS[] = {
		OLED_CMD_SET_MUX_RATIO,
		0x3F,
		OLED_CMD_SET_DISPLAY_OFFSET,
		0x00,
		OLED_CMD_SET_DISPLAY_START_LINE,
		OLED_CMD_SET_SEGMENT_REMAP,
		OLED_CMD_SET_COM_SCAN_REMAP,
		OLED_CMD_SET_PRECHARGE,
		0xF1,
		OLED_CMD_SET_VCOMH_DESELCT,
		0x30,
		OLED_CMD_SET_MEMORY_ADDR_MODE,
		0x00,
		OLED_CMD_SET_CONTRAST,
		0x7F,
		OLED_CMD_DISPLAY_RAM,
		OLED_CMD_DISPLAY_NORMAL,
		OLED_CMD_SET_DISPLAY_CLK_DIV,
		0x80,
		OLED_CMD_SET_CHARGE_PUMP,
		0x14,
		OLED_CMD_DISPLAY_ON,
		OLED_CMD_SET_PAGE_RANGE,
		0x00,
		0x07,
		OLED_CMD_SET_COLUMN_RANGE,
		0x00,
		0x7F,
		OLED_CMD_SCROLL_DEACTIVATE
};

#define SSD1306_TAG "SSD1306"

#define error_dev(s, f, d, ...) ESP_LOGE(SSD1306_TAG, "%s: bus %d, addr %02x - " s, f, \
        ((i2c_bus_t *) d->bus)->i2c_port, d->addr, ## __VA_ARGS__)

struct ssd1306_dev_t {
	i2c_bus_handle_t bus;
	uint8_t addr;

	uint8_t display_buffer[8][128];

	bool updated;

	uint8_t updated_page_min;
	uint8_t updated_page_max;

	uint8_t updated_column_min;
	uint8_t updated_column_max;

	uint8_t page_start;
	uint8_t page_end;

	uint8_t column_start;
	uint8_t column_end;
};

static uint32_t _pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--) {
		result *= m;
	}
	return result;
}

esp_err_t ssd1306_write_command(ssd1306_handle_t dev, uint8_t data)
{
	uint8_t mode = OLED_CONTROL_BYTE_CMD_SINGLE;
	return i2c_bus_master_write(dev->bus, dev->addr, &mode, &data, 1);
}

esp_err_t ssd1306_write(ssd1306_handle_t dev, uint8_t mode, uint8_t *data, size_t size)
{
	esp_err_t err = i2c_bus_master_write(dev->bus, dev->addr, &mode, data, size);
	if (err) {
		if (mode == OLED_CONTROL_BYTE_DATA_STREAM) {
			error_dev("Error %d on write %d byte data to I2C slave.", __FUNCTION__, dev, err, size);
		} else if (mode == OLED_CONTROL_BYTE_CMD_SINGLE) {
			error_dev("Error %d on write command 0x%02hhx to I2C slave.", __FUNCTION__, dev, err, *data);
		} else if (mode == OLED_CONTROL_BYTE_CMD_STREAM) {
			error_dev("Error %d on write %d commands (0x%02hhx...) to I2C slave.", __FUNCTION__, dev, err, size, *data);
		}
	}

	return err;
}

void ssd1306_fill_rectangle(ssd1306_handle_t dev, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool fill)
{
	for (uint8_t x = x1; x <= x2; x++) {
		for (uint8_t y = y1; y <= y2; y++) {
			ssd1306_fill_point(dev, x, y, fill);
		}
	}
}

void ssd1306_draw_num(ssd1306_handle_t dev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
	bool show = 0;

	for (uint8_t i = 0; i < length; i++) {
		uint8_t digit = (number / _pow(10, length - i - 1)) % 10;
		if (!show && i < (length - 1)) {
			if (digit == 0) {
				ssd1306_draw_char(dev, x + (font_size / 2) * i, y, ' ', font_size, 1);
				continue;
			} else {
				show = true;
			}
		}
		ssd1306_draw_char(dev, x + (font_size / 2) * i, y, digit + '0', font_size, 1);
	}
}

void ssd1306_draw_char(ssd1306_handle_t dev, uint8_t x, uint8_t y, char character, uint8_t font_size, bool fill)
{
	uint8_t i, j;
	uint8_t chTemp, chYpos0 = y;

	uint8_t char_offset = (uint8_t) (character - ' ');
	for (i = 0; i < font_size; i++) {
		if (font_size == 12) {
			if (fill) {
				chTemp = c_chFont1206[char_offset][i];
			} else {
				chTemp = ~c_chFont1206[char_offset][i];
			}
		} else {
			if (fill) {
				chTemp = c_chFont1608[char_offset][i];
			} else {
				chTemp = ~c_chFont1608[char_offset][i];
			}
		}

		for (j = 0; j < 8; j++) {
			ssd1306_fill_point(dev, x, y, chTemp & 0x80 ? true : false);
			chTemp <<= 1;
			y++;

			if ((y - chYpos0) == font_size) {
				y = chYpos0;
				x++;
				break;
			}
		}
	}
}

esp_err_t ssd1306_draw_string(ssd1306_handle_t dev, uint8_t x,
		uint8_t y, const char *text, uint8_t font_size,
		bool fill)
{
	esp_err_t ret = ESP_OK;
	while (*text != '\0') {
		if (x > (SSD1306_WIDTH - font_size / 2)) {
			x = 0;
			y += font_size;
			if (y > (SSD1306_HEIGHT - font_size)) {
				y = x = 0;
				ssd1306_clear_screen(dev, 0x00);
			}
		}
		ssd1306_draw_char(dev, x, y, *text, font_size, fill);
		x += font_size / 2;
		text++;
	}
	return ret;
}

void ssd1306_fill_point(ssd1306_handle_t dev, uint8_t x, uint8_t y, bool fill)
{
	if (x > 127 || y > 63) {
		return;
	}

	uint8_t page, bit, mask = 0;

	page = y / 8;
	bit = y % 8;
	mask = 1 << bit;

	if (fill) {
		dev->display_buffer[page][x] |= mask;
	} else {
		dev->display_buffer[page][x] &= ~mask;
	}

	if (!dev->updated || dev->updated_column_min > x) {
		dev->updated_column_min = x;
	}
	if (!dev->updated || dev->updated_column_max < x) {
		dev->updated_column_max = x;
	}

	if (!dev->updated || dev->updated_page_min > page) {
		dev->updated_page_min = page;
	}
	if (!dev->updated || dev->updated_page_max < page) {
		dev->updated_page_max = page;
	}

	dev->updated = true;
}

void ssd1306_draw_1616char(ssd1306_handle_t dev, uint8_t x, uint8_t y, uint8_t character)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = y;

	for (i = 0; i < 32; i++) {
		chTemp = c_chFont1612[character - 0x30][i];
		for (j = 0; j < 8; j++) {
			ssd1306_fill_point(dev, x, y, chTemp & 0x80 ? true : false);
			chTemp <<= 1;
			y++;
			if ((y - chYpos0) == 16) {
				y = chYpos0;
				x++;
				break;
			}
		}
	}
}

void ssd1306_draw_3216char(ssd1306_handle_t dev, uint8_t x, uint8_t y, uint8_t character)
{
	uint8_t i, j;
	uint8_t chTemp = 0, chYpos0 = y;

	for (i = 0; i < 64; i++) {
		chTemp = c_chFont3216[character - 0x30][i];
		for (j = 0; j < 8; j++) {
			ssd1306_fill_point(dev, x, y, chTemp & 0x80 ? true : false);
			chTemp <<= 1;
			y++;
			if ((y - chYpos0) == 32) {
				y = chYpos0;
				x++;
				break;
			}
		}
	}
}

void ssd1306_draw_bitmap(ssd1306_handle_t dev, uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h)
{
	uint16_t byte_width = (w + 7) / 8;

	for (uint8_t j = 0; j < h; j++) {
		for (uint8_t i = 0; i < w; i++) {
			if (*(bitmap + j * byte_width + i / 8) & (128 >> (i & 7))) {
				ssd1306_fill_point(dev, x + i, y + j, 1);
			}
		}
	}
}

ssd1306_handle_t ssd1306_init(i2c_bus_handle_t bus, uint8_t addr)
{
	ssd1306_handle_t dev = (ssd1306_handle_t) calloc(1, sizeof(struct ssd1306_dev_t));
	dev->bus = bus;
	dev->addr = addr;

	if (ssd1306_write(dev, SSD1306_CMD, (uint8_t *) INIT_COMMANDS, sizeof(INIT_COMMANDS)) != ESP_OK) {
		error_dev("Could not run init commands", __FUNCTION__, dev);

		free(dev);
		return NULL;
	}

	dev->column_start = 0;
	dev->column_end = 127;

	ssd1306_clear_screen(dev, 0x00);
	if (ssd1306_refresh(dev) != ESP_OK) {
		error_dev("Could not clear screen", __FUNCTION__, dev);

		free(dev);
		return NULL;
	}

	return dev;
}

esp_err_t ssd1306_delete(ssd1306_handle_t dev, bool del_bus)
{
	esp_err_t ret = ESP_OK;
	if (del_bus) {
		ret = i2c_bus_delete(dev->bus);
		if (ret == ESP_FAIL) {
			return ret;
		}
		dev->bus = NULL;
	}
	free(dev);
	return ret;
}

esp_err_t ssd1306_refresh(ssd1306_handle_t dev)
{
	esp_err_t ret = ESP_OK;

	if (!dev->updated) {
		return ESP_OK;
	}

	ssd1306_set_columns(dev, dev->updated_column_min, dev->updated_column_max);
	ssd1306_set_pages(dev, dev->updated_page_min, dev->updated_page_max);

	dev->updated = false;

	for (uint8_t i = dev->page_start; i <= dev->page_end; i++) {
		ret = ssd1306_write(dev, SSD1306_DATA, dev->display_buffer[i] + dev->column_start,
				(size_t) (dev->column_end - dev->column_start + 1));
		if (ret == ESP_FAIL) {
			return ret;
		}
	}
	return ret;
}

esp_err_t ssd1306_set_pages(ssd1306_handle_t dev, uint8_t start, uint8_t end) {
	dev->page_start = start;
	dev->page_end = end;

	uint8_t command[] = {
			OLED_CMD_SET_PAGE_RANGE,
			start,
			end
	};

	return ssd1306_write(dev, SSD1306_CMD, command, sizeof(command));
}

esp_err_t ssd1306_set_columns(ssd1306_handle_t dev, uint8_t start, uint8_t end) {
	dev->column_start = start;
	dev->column_end = end;

	uint8_t command[] = {
			OLED_CMD_SET_COLUMN_RANGE,
			start,
			end
	};

	return ssd1306_write(dev, SSD1306_CMD, command, sizeof(command));
}

esp_err_t ssd1306_scroll_disable(ssd1306_handle_t dev) {
	return ssd1306_write_command(dev, OLED_CMD_SCROLL_DEACTIVATE);
}

esp_err_t ssd1306_scroll(ssd1306_handle_t dev, ssd1306_scroll_direction_t direction, ssd1306_scroll_interval_t interval,
		uint8_t start_page, uint8_t end_page)
{
	uint8_t commands[] = {
			(uint8_t) (direction == SSD1306_SCROLL_LEFT ? OLED_CMD_SCROLL_LEFT : OLED_CMD_SCROLL_RIGHT),
			0x00,
			start_page,
			interval,
			end_page,
			0x00,
			0xFF,
			OLED_CMD_SCROLL_ACTIVATE
	};

	return ssd1306_write(dev, SSD1306_CMD, commands, sizeof(commands));
}

void ssd1306_clear_screen(ssd1306_handle_t dev, uint8_t chFill)
{
	memset(dev->display_buffer, chFill, 8 * 128);
	dev->updated = true;
	dev->updated_column_min = 0;
	dev->updated_column_max = 127;
	dev->updated_page_min = 0;
	dev->updated_page_max = 7;
}

