#include "mock_spi_bus.h"
#include <string.h>
#include <stddef.h>


/* Default response */

static spi_bus_status_t mock_status = SPI_BUS_OK;

static uint8_t mock_rx_data[MOCK_MAX_DATA];
static uint16_t mock_rx_data_len;


/* Most recent transaction */

spi_device_t mock_last_device;
uint8_t mock_last_tx[MOCK_MAX_DATA];
uint16_t mock_last_tx_len;
uint16_t mock_last_rx_len;
uint32_t mock_call_count;


/* Transaction history */

mock_spi_transaction_t
    mock_history[MOCK_MAX_TRANSACTIONS];

uint32_t mock_history_count;


/* Queued SPI responses */

typedef struct
{
    spi_bus_status_t status;
    uint8_t data[MOCK_MAX_DATA];
    uint16_t len;

} mock_spi_response_t;

static mock_spi_response_t
    mock_responses[MOCK_MAX_TRANSACTIONS];

uint32_t mock_response_count;
uint32_t mock_response_index;


/* Reset mock state */

void mock_spi_reset(void)
{
    mock_status = SPI_BUS_OK;

    mock_last_device = SPI_DEV_IMU;
    mock_last_tx_len = 0;
    mock_last_rx_len = 0;
    mock_rx_data_len = 0;
    mock_call_count = 0;

    mock_history_count = 0;
    mock_response_count = 0;
    mock_response_index = 0;

    memset(mock_last_tx, 0, sizeof(mock_last_tx));
    memset(mock_rx_data, 0, sizeof(mock_rx_data));
    memset(mock_history, 0, sizeof(mock_history));
    memset(mock_responses, 0, sizeof(mock_responses));
}


/* Set default SPI status */

void mock_spi_set_status(spi_bus_status_t status)
{
    mock_status = status;
}


/* Set default receive data */

void mock_spi_set_rx_data(const uint8_t *data,
                          uint16_t len)
{
    if (data == NULL || len > MOCK_MAX_DATA)
    {
        return;
    }

    memcpy(mock_rx_data, data, len);
    mock_rx_data_len = len;
}


/* Queue a response for a future transaction */

int mock_spi_queue_response(spi_bus_status_t status,
                            const uint8_t *data,
                            uint16_t len)
{
    if (mock_response_count >= MOCK_MAX_TRANSACTIONS ||
        len > MOCK_MAX_DATA ||
        (len > 0 && data == NULL))
    {
        return 0;
    }

    mock_spi_response_t *response =
        &mock_responses[mock_response_count];

    response->status = status;
    response->len = len;

    if (len > 0)
    {
        memcpy(response->data, data, len);
    }

    mock_response_count++;

    return 1;
}


/* Record SPI transaction */

static void mock_record_transaction(
    spi_device_t device,
    const uint8_t *tx,
    uint16_t tx_len,
    uint16_t rx_len)
{
    mock_last_device = device;
    mock_last_tx_len = tx_len;
    mock_last_rx_len = rx_len;
    mock_call_count++;

    memcpy(mock_last_tx, tx, tx_len);

    if (mock_history_count < MOCK_MAX_TRANSACTIONS)
    {
        mock_spi_transaction_t *entry =
            &mock_history[mock_history_count];

        entry->device = device;
        entry->tx_len = tx_len;
        entry->rx_len = rx_len;

        memcpy(entry->tx, tx, tx_len);

        mock_history_count++;
    }
}


/* Process next simulated response */

static spi_bus_status_t mock_get_response(
    uint8_t *rx,
    uint16_t rx_len)
{
    spi_bus_status_t status = mock_status;

    const uint8_t *data = mock_rx_data;
    uint16_t len = mock_rx_data_len;

    /* Use queued responses when available */

    if (mock_response_count > 0)
    {
        if (mock_response_index >= mock_response_count)
        {
            return SPI_BUS_ERR;
        }

        mock_spi_response_t *response =
            &mock_responses[mock_response_index];

        status = response->status;
        data = response->data;
        len = response->len;

        mock_response_index++;
    }

    if (status != SPI_BUS_OK)
    {
        return status;
    }

    if (rx_len > len)
    {
        return SPI_BUS_ERR;
    }

    if (rx_len > 0)
    {
        memcpy(rx, data, rx_len);
    }

    return SPI_BUS_OK;
}


/* Mock SPI write */

spi_bus_status_t spi_bus_write(
    spi_device_t device,
    const uint8_t *data,
    uint16_t len)
{
    if (data == NULL || len == 0 ||
        len > MOCK_MAX_DATA)
    {
        return SPI_BUS_INVALID_ARG;
    }

    mock_record_transaction(device, data, len, 0);

    return mock_get_response(NULL, 0);
}


/* Mock SPI write and read */

spi_bus_status_t spi_bus_write_read(
    spi_device_t device,
    const uint8_t *tx,
    uint16_t tx_len,
    uint8_t *rx,
    uint16_t rx_len)
{
    if (tx == NULL || rx == NULL ||
        tx_len == 0 || rx_len == 0 ||
        tx_len > MOCK_MAX_DATA ||
        rx_len > MOCK_MAX_DATA)
    {
        return SPI_BUS_INVALID_ARG;
    }

    mock_record_transaction(
        device, tx, tx_len, rx_len);

    return mock_get_response(rx, rx_len);
}
