/* BMP388 register transport */

#include "baro.h"
#include "spi_bus.h"

#include <stddef.h>
#include <string.h>

#define BARO_SPI_READ           0x80U
#define BARO_SPI_WRITE_MASK     0x7FU
#define BARO_MAX_BURST          32U

static baro_status_t baro_check_status(
    spi_bus_status_t status)
{
    switch (status)
    {
        case SPI_BUS_OK:
            return BARO_OK;

        case SPI_BUS_TIMEOUT:
            return BARO_ERR_TIMEOUT;

        case SPI_BUS_BUSY:
            return BARO_ERR_BUSY;

        case SPI_BUS_INVALID_ARG:
            return BARO_ERR_INVALID_ARG;

        default:
            return BARO_ERR_SPI;
    }
}

baro_status_t baro_write_reg(uint8_t reg,
                            uint8_t value)
{
    uint8_t tx[2] = {
        reg & BARO_SPI_WRITE_MASK,
        value
    };

    spi_bus_status_t status = spi_bus_write(
        SPI_DEV_BARO, tx, 2);

    return baro_check_status(status);
}

baro_status_t baro_read_reg(uint8_t reg,
                           uint8_t *value)
{
    if (value == NULL)
    {
        return BARO_ERR_INVALID_ARG;
    }

    uint8_t tx = reg | BARO_SPI_READ;
    uint8_t rx[2];

    spi_bus_status_t status = spi_bus_write_read(
        SPI_DEV_BARO, &tx, 1, rx, 2);

    if (status != SPI_BUS_OK)
    {
        return baro_check_status(status);
    }

    /* Discard dummy byte */
    *value = rx[1];

    return BARO_OK;
}

baro_status_t baro_burst_read(uint8_t start_reg,
                             uint8_t *buf,
                             uint8_t len)
{
    if (buf == NULL || len == 0 ||
        len > BARO_MAX_BURST)
    {
        return BARO_ERR_INVALID_ARG;
    }

    uint8_t tx = start_reg | BARO_SPI_READ;
    uint8_t rx[BARO_MAX_BURST + 1U];

    spi_bus_status_t status = spi_bus_write_read(
        SPI_DEV_BARO,
        &tx,
        1,
        rx,
        (uint16_t)len + 1U);

    if (status != SPI_BUS_OK)
    {
        return baro_check_status(status);
    }

    /* Copy received data, excluding dummy byte */
    memcpy(buf, &rx[1], len);

    return BARO_OK;
}
