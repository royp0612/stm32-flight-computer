#ifndef MOCK_SPI_BUS_H_
#define MOCK_SPI_BUS_H_

#include "spi_bus.h"

#define MOCK_MAX_TRANSACTIONS 64U
#define MOCK_MAX_DATA 260U

typedef struct
{
    spi_device_t device;
    uint8_t tx[MOCK_MAX_DATA];
    uint16_t tx_len;
    uint16_t rx_len;

} mock_spi_transaction_t;


/* Existing mock functions */

void mock_spi_reset(void);

void mock_spi_set_status(spi_bus_status_t status);

void mock_spi_set_rx_data(const uint8_t *data,
                          uint16_t len);


/* New queued response function */

int mock_spi_queue_response(spi_bus_status_t status,
                            const uint8_t *data,
                            uint16_t len);


/* Most recent transaction */

extern spi_device_t mock_last_device;
extern uint8_t mock_last_tx[MOCK_MAX_DATA];
extern uint16_t mock_last_tx_len;
extern uint16_t mock_last_rx_len;
extern uint32_t mock_call_count;


/* Complete transaction history */

extern mock_spi_transaction_t
    mock_history[MOCK_MAX_TRANSACTIONS];

extern uint32_t mock_history_count;
extern uint32_t mock_response_index;
extern uint32_t mock_response_count;

#endif
