/*
 * imu.c
 *
 *  Created on: Nov 25, 2026
 *      Author: melon
 */

#include "imu.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

uint8_t imu_read_reg(uint8_t reg){
	  uint8_t tx = reg | 0x80;
	  uint8_t value = 0;
	  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
	  HAL_SPI_Receive(&hspi1, &value, 1, 100);
	  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
	  return value;
}

void imu_write_reg(uint8_t reg, uint8_t value){
	  uint8_t tx [2] = {reg, value};
	  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, tx, 2, 100);
	  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void imu_burst_read(uint8_t start_reg, uint8_t *buf, uint8_t len){
	  uint8_t tx = start_reg | 0x80;
	  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
	  HAL_SPI_Receive(&hspi1, buf, len, 100);
	  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

int imu_init(void){
	uint8_t id = imu_read_reg(0x0F);
	if (id!=0x6C){
		return -1;
	}
    imu_write_reg(0x10, 0x60);
	imu_write_reg(0x11, 0x94);
	imu_write_reg(0x12, 0x9C);

	return 0;
}

void imu_read(imu_data_t *out)
{
    uint8_t buf[12];

    imu_burst_read(0x22, buf, 12);

    // Combine gyroscope bytes
    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);

    // Combine accelerometer bytes
    int16_t ax = (int16_t)((buf[7]  << 8) | buf[6]);
    int16_t ay = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

    // Convert gyroscope to degrees per second
    out->gx = gx * 0.00875f;
    out->gy = gy * 0.00875f;
    out->gz = gz * 0.00875f;

    // Convert accelerometer to g
    out->ax = ax * 0.000061f;
    out->ay = ay * 0.000061f;
    out->az = az * 0.000061f;
}
