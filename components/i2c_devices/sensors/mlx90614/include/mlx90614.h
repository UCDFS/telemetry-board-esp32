#ifndef MLX90614_H
#define MLX90614_H

#include <driver/i2c.h>
#include "i2c_bus.h"

#define MLX90614_I2C_ADDR_DEFAULT 0x5a

typedef struct mlx90614_dev_t *mlx90614_handle_t;

mlx90614_handle_t mlx90614_init(i2c_bus_handle_t bus, uint8_t dev_addr);
void mlx90614_delete(mlx90614_handle_t dev, bool del_bus);

static esp_err_t mlx90614_set_address(mlx90614_handle_t dev, uint16_t addr);

esp_err_t mlx90614_get_raw(mlx90614_handle_t dev, uint16_t *raw);
esp_err_t mlx90614_get_celcius(mlx90614_handle_t dev, float *data);


#endif