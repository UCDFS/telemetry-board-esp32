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
#ifndef SSD_1306_H
#define SSD_1306_H

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "stdint.h"

#define SSD1306_I2C_ADDR_1 0x3C
#define SSD1306_I2C_ADDR_2 0x3D

#define SSD1306_SET_LOWER_ADDRESS   0x02
#define SSD1306_SET_HIGHER_ADDRESS  0x10

#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64
#define SSD1306_SET_PAGE_ADDR       0xB0

typedef enum {
	SSD1306_SCROLL_RIGHT = 0,
	SSD1306_SCROLL_LEFT
} ssd1306_scroll_direction_t;

typedef enum {
	SSD1306_SCROLL_INTERVAL_5FRAMES = 0,
	SSD1306_SCROLL_INTERVAL_64FRAMES,
	SSD1306_SCROLL_INTERVAL_128FRAMES,
	SSD1306_SCROLL_INTERVAL_256FRAMES,
	SSD1306_SCROLL_INTERVAL_3FRAMES,
	SSD1306_SCROLL_INTERVAL_4FRAMES,
	SSD1306_SCROLL_INTERVAL_25FRAMES,
	SSD1306_SCROLL_INTERVAL_2FRAMES
} ssd1306_scroll_interval_t;

typedef struct ssd1306_dev_t *ssd1306_handle_t;

/**
 * @brief   Create and initialization device object and return a device handle
 *
 * @param   bus I2C bus object handle
 * @param   dev_addr I2C device address of device
 *
 * @return
 *     - device object handle of ssd1306
 */
ssd1306_handle_t ssd1306_init(i2c_bus_handle_t bus, uint8_t dev_addr);

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
esp_err_t ssd1306_delete(ssd1306_handle_t dev, bool del_bus);

/**
 * @brief   Write command or data to ssd1306
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ssd1306_write(ssd1306_handle_t dev, uint8_t mode, uint8_t *data, size_t size);

/**
 * @brief   draw point on (x, y)
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   fill Whether pixel is on or off
 *
 * @return
 *     - NULL
 */
void ssd1306_fill_point(ssd1306_handle_t dev, uint8_t x, uint8_t y, bool fill);

/**
 * @brief   Draw rectangle on (x1,y1)-(x2,y2)
 *
 * @param   dev object handle of ssd1306
 * @param   x1
 * @param   y1
 * @param   x2
 * @param   y2
 * @param   fill fill point
 */
void ssd1306_fill_rectangle(ssd1306_handle_t dev,
        uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
        bool fill);

/**
 * @brief   display char on (x, y),and set size, mode
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   font_size char size
 * @param   character draw char
 * @param   chMode display mode
 *
 * @return
 *     - NULL
 */
void ssd1306_draw_char(ssd1306_handle_t dev, uint8_t x,
        uint8_t y, char character, uint8_t font_size, bool fill);

/**
 * @brief   display number on (x, y),and set length, size, mode
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   number draw num
 * @param   length length
 * @param   font_size display size
 *
 * @return
 *     - NULL
 */
void ssd1306_draw_num(ssd1306_handle_t dev, uint8_t x,
        uint8_t y, uint32_t number, uint8_t length, uint8_t font_size);

/**
 * @brief   display 1616char on (x, y)
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   character draw char
 *
 * @return
 *     - NULL
 */
void ssd1306_draw_1616char(ssd1306_handle_t dev, uint8_t x,
        uint8_t y, uint8_t character);

/**
 * @brief   display 3216char on (x, y)
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   character draw char
 *
 * @return
 *     - NULL
 */
void ssd1306_draw_3216char(ssd1306_handle_t dev, uint8_t x,
        uint8_t y, uint8_t character);

/**
 * @brief   draw bitmap on (x, y),and set width, height
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   bitmap point to BMP data
 * @param   w picture width
 * @param   h picture heght
 *
 * @return
 *     - NULL
 */
void ssd1306_draw_bitmap(ssd1306_handle_t dev, uint8_t x,
        uint8_t y, const uint8_t *bitmap, uint8_t w,
        uint8_t h);

/**
 * @brief   refresh dot matrix panel
 *
 * @param   dev object handle of ssd1306

 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 **/
esp_err_t ssd1306_refresh(ssd1306_handle_t dev);

/**
 * @brief   Clear screen
 *
 * @param   dev object handle of ssd1306
 * @param   chFill whether fill and fill char
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 **/
void ssd1306_clear_screen(ssd1306_handle_t dev, uint8_t fill);

/**
 * @brief   Displays a string on the screen
 *
 * @param   dev object handle of ssd1306
 * @param   x Specifies the X position
 * @param   y Specifies the Y position
 * @param   text Pointer to a string to display on the screen
 * @param   font_size char size
 * @param   fill display mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 **/
esp_err_t ssd1306_draw_string(ssd1306_handle_t dev, uint8_t x,
		uint8_t y, const char *text, uint8_t font_size,
		bool fill);

esp_err_t ssd1306_scroll_disable(ssd1306_handle_t dev);
esp_err_t ssd1306_scroll(ssd1306_handle_t dev, ssd1306_scroll_direction_t direction, ssd1306_scroll_interval_t interval,
		uint8_t start_page, uint8_t end_page);
esp_err_t ssd1306_set_pages(ssd1306_handle_t dev, uint8_t start, uint8_t end);
esp_err_t ssd1306_set_columns(ssd1306_handle_t dev, uint8_t start, uint8_t end);

#endif