#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

#define FLASH_CAPACITY      16777216U   /* 16 MB = 128 Mbit */
#define FLASH_PAGE_SIZE     256U
#define FLASH_SECTOR_SIZE   4096U

typedef enum {
    FLASH_OK = 0,
    FLASH_ERR_INVALID_ARG,
    FLASH_ERR_SPI,
    FLASH_ERR_TIMEOUT,
    FLASH_ERR_BUSY,
    FLASH_ERR_DEVICE_ID,
    FLASH_ERR_WRITE_ENABLE,
    FLASH_ERR_ADDRESS,
    FLASH_ERR_PAGE_BOUNDARY,
} flash_status_t;

flash_status_t flash_init(void);
flash_status_t flash_read_id(uint8_t *id);            /* id must hold 3 bytes */
flash_status_t flash_read_status(uint8_t *status);
flash_status_t flash_write_enable(void);
flash_status_t flash_write_disable(void);
flash_status_t flash_wait_ready(uint32_t timeout_ms);
flash_status_t flash_read(uint32_t address, uint8_t *buf, uint16_t len);
flash_status_t flash_page_program(uint32_t address, const uint8_t *data, uint16_t len);
flash_status_t flash_sector_erase(uint32_t address);

#endif
