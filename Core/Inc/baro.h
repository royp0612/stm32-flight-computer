#ifndef BARO_H_
#define BARO_H_

#include <stdint.h>

typedef enum
{
    BARO_OK = 0,
    BARO_ERR_SPI = -1,
    BARO_ERR_TIMEOUT = -2,
    BARO_ERR_BUSY = -3,
    BARO_ERR_DEVICE_ID = -4,
    BARO_ERR_INVALID_ARG = -5,
    BARO_ERR_CONFIG = -6,
    BARO_ERR_NOT_READY = -7,
    BARO_ERR_FATAL_DEVICE = -8,
    BARO_ERR_COMMAND = -9,
	BARO_ERR_INVALID_DATA = -10,
	BARO_ERR_NOT_INITIALIZED = -11

} baro_status_t;


baro_status_t baro_read_reg(uint8_t reg,
                           uint8_t *value);

baro_status_t baro_write_reg(uint8_t reg,
                            uint8_t value);

baro_status_t baro_burst_read(uint8_t start_reg,
                             uint8_t *buf,
                             uint8_t len);

#endif
