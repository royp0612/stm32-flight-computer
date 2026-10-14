#ifndef SPI_BUS_H_
#define SPI_BUS_H_

#include <stdint.h>

typedef enum{
	SPI_DEV_IMU,
	SPI_DEV_BARO,
	SPI_DEV_FLASH
} spi_device_t;

typedef enum{
	SPI_BUS_OK = 0,
	SPI_BUS_ERR = -1,
	SPI_BUS_TIMEOUT = -2,
	SPI_BUS_BUSY = -3,
	SPI_BUS_INVALID_ARG = -4
} spi_bus_status_t;

spi_bus_status_t spi_bus_write(
    spi_device_t device,
    const uint8_t *data,
    uint16_t len
);

spi_bus_status_t spi_bus_write_read(
    spi_device_t device,
    const uint8_t *tx,
    uint16_t tx_len,
    uint8_t *rx,
    uint16_t rx_len
);
#endif
