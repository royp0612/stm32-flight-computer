/*
 * spi_bus.c
 *
 *  Created on: Nov 26, 2026
 *      Author: melon
 */
#include "spi_bus.h"
#include "main.h"
#include <stddef.h>

extern SPI_HandleTypeDef hspi1;

#define SPI_BUS_TIMEOUT_MS 100U

static void spi_set_cs(spi_device_t device, GPIO_PinState state){
	switch(device){
		case SPI_DEV_IMU:
			HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, state);
			break;
		case SPI_DEV_BARO:
			HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, state);
			break;
		case SPI_DEV_FLASH:
			HAL_GPIO_WritePin(FLASH_CS_GPIO_Port,FLASH_CS_Pin, state);
			break;
		default:
			break;
	}
}
static uint8_t spi_valid_device(spi_device_t device)
{
    return (device == SPI_DEV_IMU ||
            device == SPI_DEV_BARO ||
            device == SPI_DEV_FLASH);
}
static spi_bus_status_t spi_check_status(
    HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:
            return SPI_BUS_OK;

        case HAL_TIMEOUT:
            return SPI_BUS_TIMEOUT;

        case HAL_BUSY:
            return SPI_BUS_BUSY;

        default:
            return SPI_BUS_ERR;
    }
}




spi_bus_status_t spi_bus_write(
    spi_device_t device,
    const uint8_t *data,
    uint16_t len)
{
	if(!spi_valid_device(device) || data == NULL || len ==0){
		return SPI_BUS_INVALID_ARG;
	}
	spi_set_cs(device, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len,
	                          SPI_BUS_TIMEOUT_MS);
	spi_set_cs(device, GPIO_PIN_SET);
	return spi_check_status(status);
}

spi_bus_status_t spi_bus_write_read(
    spi_device_t device,
    const uint8_t *tx,
    uint16_t tx_len,
    uint8_t *rx,
    uint16_t rx_len)
{
    // 1. Validate all arguments
	if (!spi_valid_device(device) ||
	    tx == NULL || tx_len == 0 ||
	    rx == NULL || rx_len == 0)
	{
	    return SPI_BUS_INVALID_ARG;
	}
    // 2. Pull CS LOW
	spi_set_cs(device, GPIO_PIN_RESET);
    // 3. Transmit command
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, tx_len,
	                          SPI_BUS_TIMEOUT_MS);
    // 4. If transmission succeeded, receive data
	if (status == HAL_OK){
		status = HAL_SPI_Receive(&hspi1, rx, rx_len,SPI_BUS_TIMEOUT_MS);
	}
    // 5. Pull CS HIGH
	spi_set_cs(device, GPIO_PIN_SET);
    // 6. Convert and return status
	return spi_check_status(status);
}
