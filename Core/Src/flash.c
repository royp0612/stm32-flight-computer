#include "flash.h"
#include "spi_bus.h"
#include "platform.h"

#include <stddef.h>
#include <string.h>

/* ---------- SPI commands ---------- */
#define FLASH_CMD_JEDEC_ID        0x9FU
#define FLASH_CMD_READ_STATUS     0x05U
#define FLASH_CMD_WRITE_ENABLE    0x06U
#define FLASH_CMD_WRITE_DISABLE   0x04U
#define FLASH_CMD_READ_DATA       0x03U
#define FLASH_CMD_PAGE_PROGRAM    0x02U
#define FLASH_CMD_SECTOR_ERASE    0x20U
#define FLASH_CMD_RESET_ENABLE    0x66U
#define FLASH_CMD_RESET           0x99U

/* ---------- Device identification ---------- */
#define FLASH_MANUFACTURER_ID     0xEFU   /* Winbond */
#define FLASH_MEMORY_TYPE         0x40U
#define FLASH_MEMORY_CAPACITY     0x18U   /* 2^24 bytes */

/* ---------- Status register 1 bits ---------- */
#define FLASH_STATUS_BUSY         0x01U
#define FLASH_STATUS_WEL          0x02U

/* ---------- Timing (verify against datasheet AC characteristics) ---------- */
#define FLASH_RESET_DELAY_MS      1U      /* reset recovery (datasheet: tens of us) */
#define FLASH_READY_TIMEOUT_MS    500U    /* wait for idle before any operation */
#define FLASH_PROGRAM_TIMEOUT_MS  10U     /* page program */
#define FLASH_ERASE_TIMEOUT_MS    500U    /* 4 KB sector erase */

/* ---------- Helpers ---------- */

static void flash_encode_address(uint32_t address, uint8_t *buf)
{
    buf[0] = (uint8_t)(address >> 16);
    buf[1] = (uint8_t)(address >> 8);
    buf[2] = (uint8_t)address;
}

static flash_status_t flash_check_status(spi_bus_status_t status)
{
    switch (status)
    {
        case SPI_BUS_OK:          return FLASH_OK;
        case SPI_BUS_TIMEOUT:     return FLASH_ERR_TIMEOUT;
        case SPI_BUS_BUSY:        return FLASH_ERR_BUSY;
        case SPI_BUS_INVALID_ARG: return FLASH_ERR_INVALID_ARG;
        default:                  return FLASH_ERR_SPI;
    }
}

static flash_status_t flash_send_command(uint8_t cmd)
{
    return flash_check_status(spi_bus_write(SPI_DEV_FLASH, &cmd, 1U));
}

/* ---------- Public API ---------- */

flash_status_t flash_read_id(uint8_t *id)
{
    if (id == NULL)
    {
        return FLASH_ERR_INVALID_ARG;
    }

    uint8_t tx = FLASH_CMD_JEDEC_ID;
    return flash_check_status(spi_bus_write_read(SPI_DEV_FLASH, &tx, 1U, id, 3U));
}

flash_status_t flash_read_status(uint8_t *status)
{
    if (status == NULL)
    {
        return FLASH_ERR_INVALID_ARG;
    }

    uint8_t tx = FLASH_CMD_READ_STATUS;
    return flash_check_status(spi_bus_write_read(SPI_DEV_FLASH, &tx, 1U, status, 1U));
}

flash_status_t flash_write_enable(void)
{
    flash_status_t status = flash_send_command(FLASH_CMD_WRITE_ENABLE);
    if (status != FLASH_OK)
    {
        return status;
    }

    uint8_t reg_status = 0U;
    status = flash_read_status(&reg_status);
    if (status != FLASH_OK)
    {
        return status;
    }

    if ((reg_status & FLASH_STATUS_WEL) == 0U)
    {
        return FLASH_ERR_WRITE_ENABLE;
    }

    return FLASH_OK;
}

flash_status_t flash_write_disable(void)
{
    return flash_send_command(FLASH_CMD_WRITE_DISABLE);
}

flash_status_t flash_wait_ready(uint32_t timeout_ms)
{
    uint32_t elapsed = 0U;
    uint8_t reg_status = 0U;

    while (1)
    {
        flash_status_t status = flash_read_status(&reg_status);
        if (status != FLASH_OK)
        {
            return status;
        }

        if ((reg_status & FLASH_STATUS_BUSY) == 0U)
        {
            return FLASH_OK;
        }

        if (elapsed >= timeout_ms)
        {
            return FLASH_ERR_TIMEOUT;
        }

        platform_delay_ms(1U);
        elapsed++;
    }
}

flash_status_t flash_init(void)
{
    uint8_t id[3];
    flash_status_t status;

    /* 1. Software reset: known state even after a warm restart */
    if ((status = flash_send_command(FLASH_CMD_RESET_ENABLE)) != FLASH_OK) return status;
    if ((status = flash_send_command(FLASH_CMD_RESET))        != FLASH_OK) return status;
    platform_delay_ms(FLASH_RESET_DELAY_MS);

    /* 2. Is it the right chip? (before waiting: an absent chip reads as "busy") */
    if ((status = flash_read_id(id)) != FLASH_OK) return status;

    if (id[0] != FLASH_MANUFACTURER_ID ||
        id[1] != FLASH_MEMORY_TYPE ||
        id[2] != FLASH_MEMORY_CAPACITY)
    {
        return FLASH_ERR_DEVICE_ID;
    }

    /* 3. Wait until idle */
    return flash_wait_ready(FLASH_READY_TIMEOUT_MS);
}

flash_status_t flash_read(uint32_t address, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U)
    {
        return FLASH_ERR_INVALID_ARG;
    }

    if (address >= FLASH_CAPACITY || len > FLASH_CAPACITY - address)
    {
        return FLASH_ERR_ADDRESS;
    }

    flash_status_t status = flash_wait_ready(FLASH_READY_TIMEOUT_MS);
    if (status != FLASH_OK)
    {
        return status;
    }

    uint8_t tx[4];
    tx[0] = FLASH_CMD_READ_DATA;
    flash_encode_address(address, &tx[1]);

    return flash_check_status(spi_bus_write_read(SPI_DEV_FLASH, tx, 4U, buf, len));
}

flash_status_t flash_page_program(uint32_t address, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > FLASH_PAGE_SIZE)
    {
        return FLASH_ERR_INVALID_ARG;
    }

    if (address >= FLASH_CAPACITY || len > FLASH_CAPACITY - address)
    {
        return FLASH_ERR_ADDRESS;
    }

    /* A page program must not cross a 256-byte page boundary */
    if ((address % FLASH_PAGE_SIZE) + len > FLASH_PAGE_SIZE)
    {
        return FLASH_ERR_PAGE_BOUNDARY;
    }

    flash_status_t status = flash_wait_ready(FLASH_READY_TIMEOUT_MS);
    if (status != FLASH_OK)
    {
        return status;
    }

    status = flash_write_enable();
    if (status != FLASH_OK)
    {
        return status;
    }

    uint8_t tx[4U + FLASH_PAGE_SIZE];
    tx[0] = FLASH_CMD_PAGE_PROGRAM;
    flash_encode_address(address, &tx[1]);
    memcpy(&tx[4], data, len);

    spi_bus_status_t bus_status = spi_bus_write(SPI_DEV_FLASH, tx, (uint16_t)(len + 4U));
    if (bus_status != SPI_BUS_OK)
    {
        return flash_check_status(bus_status);
    }

    return flash_wait_ready(FLASH_PROGRAM_TIMEOUT_MS);
}

flash_status_t flash_sector_erase(uint32_t address)
{
    if (address >= FLASH_CAPACITY || (address % FLASH_SECTOR_SIZE) != 0U)
    {
        return FLASH_ERR_ADDRESS;
    }

    flash_status_t status = flash_wait_ready(FLASH_READY_TIMEOUT_MS);
    if (status != FLASH_OK)
    {
        return status;
    }

    status = flash_write_enable();
    if (status != FLASH_OK)
    {
        return status;
    }

    uint8_t tx[4];
    tx[0] = FLASH_CMD_SECTOR_ERASE;
    flash_encode_address(address, &tx[1]);

    spi_bus_status_t bus_status = spi_bus_write(SPI_DEV_FLASH, tx, 4U);
    if (bus_status != SPI_BUS_OK)
    {
        return flash_check_status(bus_status);
    }

    return flash_wait_ready(FLASH_ERASE_TIMEOUT_MS);
}
