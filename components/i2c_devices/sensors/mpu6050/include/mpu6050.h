#ifndef MMA8451_H
#define MMA8451_H

#include <driver/i2c.h>
#include "i2c_bus.h"

#define MPU6050_CHIP_ID 0x68

#define MPU6050_I2C_ADDR_1 0x68
#define MPU6050_I2C_ADDR_2 0x69

typedef enum
{
	MPU6050_ACCEL_SCALE_2G = 0,
	MPU6050_ACCEL_SCALE_4G,
	MPU6050_ACCEL_SCALE_8G,
	MPU6050_ACCEL_SCALE_16G,
	MPU6050_ACCEL_SCALE_MAX
} mpu6050_accel_scale_t;

typedef enum
{
	MPU6050_GYRO_SCALE_250DPS = 0,
	MPU6050_GYRO_SCALE_500DPS,
	MPU6050_GYRO_SCALE_1000DPS,
	MPU6050_GYRO_SCALE_2000DPS,
	MPU6050_GYRO_SCALE_MAX
} mpu6050_gyro_scale_t;

typedef enum
{
	MPU6050_DLPF_260HZ_256HZ = 0,
	MPU6050_DLPF_184HZ_188HZ,
	MPU6050_DLPF_94HZ_98HZ,
	MPU6050_DLPF_44HZ_42HZ,
	MPU6050_DLPF_21HZ_20HZ,
	MPU6050_DLPF_10HZ_10HZ,
	MPU6050_DLPF_5HZ_5HZ,
	MPU6050_DLPF_MAX
} mpu6050_dlpf_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
} mpu6050_raw_accel_data_t;
typedef struct
{
	int16_t gx;
	int16_t gy;
	int16_t gz;
} mpu6050_raw_gyro_data_t;
typedef struct {
	mpu6050_raw_accel_data_t accel;
	int16_t temp;
	mpu6050_raw_gyro_data_t gyro;
} mpu6050_raw_data_t;

typedef struct {
	float ax;
	float ay;
	float az;
} mpu6050_float_accel_data_t;
typedef struct {
	float gx;
	float gy;
	float gz;
} mpu6050_float_gyro_data_t;
typedef struct
{
	mpu6050_float_accel_data_t accel;
	mpu6050_float_gyro_data_t gyro;
	float temp;
} mpu6050_float_data_t;

typedef struct mpu6050_dev_t *mpu6050_handle_t;

mpu6050_handle_t mpu6050_init(i2c_bus_handle_t bus, uint8_t dev_addr);
void mpu6050_delete(mpu6050_handle_t dev, bool del_bus);

esp_err_t mpu6050_setup(mpu6050_handle_t dev, uint8_t data_rate, mpu6050_dlpf_t dlpf);

esp_err_t mpu6050_set_accel_scale(mpu6050_handle_t dev, mpu6050_accel_scale_t scale);
esp_err_t mpu6050_set_gyro_scale(mpu6050_handle_t dev, mpu6050_gyro_scale_t scale);

esp_err_t mpu6050_get_raw_data(mpu6050_handle_t dev, mpu6050_raw_data_t* raw);
esp_err_t mpu6050_get_float_data(mpu6050_handle_t dev, mpu6050_float_data_t *data);


#endif