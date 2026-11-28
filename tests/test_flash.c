#include "unity.h"
#include "flash.h"
#include "mock_spi_bus.h"
#include "mock_platform.h"

#include <stddef.h>
#include <string.h>

static const uint8_t status_ready[] = {0x00};
static const uint8_t status_wel[] = {0x02};
static const uint8_t status_busy[] = {0x01};
static const uint8_t read_status_command[] = {0x05};
static const uint8_t write_enable_command[] = {0x06};

void setUp(void)
{
    mock_spi_reset();
    mock_platform_reset();
}

void tearDown(void)
{
    TEST_ASSERT_EQUAL_UINT32(mock_response_count, mock_call_count);
    TEST_ASSERT_EQUAL_UINT32(mock_response_count, mock_response_index);
}

static void queue_status(const uint8_t *status)
{
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, status, 1));
}

static void queue_successful_write(void)
{
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, NULL, 0));
}

static void assert_transaction(uint32_t index, const uint8_t *expected_tx,
                               uint16_t tx_len, uint16_t rx_len)
{
    TEST_ASSERT_LESS_THAN_UINT32(mock_history_count, index);
    const mock_spi_transaction_t *call = &mock_history[index];

    TEST_ASSERT_EQUAL_INT(SPI_DEV_FLASH, call->device);
    TEST_ASSERT_EQUAL_UINT16(tx_len, call->tx_len);
    TEST_ASSERT_EQUAL_UINT16(rx_len, call->rx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_tx, call->tx, tx_len);
}

static void queue_program_or_erase(void)
{
    queue_status(status_ready);
    queue_successful_write();
    queue_status(status_wel);
    queue_successful_write();
    queue_status(status_ready);
}

static void assert_program_or_erase_sequence(const uint8_t *command, uint16_t len)
{
    assert_transaction(0, read_status_command, 1, 1);
    assert_transaction(1, write_enable_command, 1, 0);
    assert_transaction(2, read_status_command, 1, 1);
    assert_transaction(3, command, len, 0);
    assert_transaction(4, read_status_command, 1, 1);
}

void test_read_sends_command_and_msb_first_address(void)
{
    const uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    const uint8_t expected_tx[] = {0x03, 0x01, 0x23, 0x45};
    uint8_t buf[sizeof(data)];

    queue_status(status_ready);
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, data, sizeof(data)));

    TEST_ASSERT_EQUAL_INT(FLASH_OK, flash_read(0x012345, buf, sizeof(buf)));
    assert_transaction(0, read_status_command, 1, 1);
    assert_transaction(1, expected_tx, sizeof(expected_tx), sizeof(buf));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, buf, sizeof(buf));
}

void test_page_program_rejects_page_crossing(void)
{
    const uint8_t data[16] = {0};

    TEST_ASSERT_EQUAL_INT(FLASH_ERR_PAGE_BOUNDARY,
                          flash_page_program(0x0000F8, data, sizeof(data)));
    TEST_ASSERT_EQUAL_UINT32(0, mock_call_count);
}

void test_page_program_stops_when_write_enable_fails(void)
{
    const uint8_t data[] = {1, 2, 3, 4};

    queue_status(status_ready);
    queue_successful_write();
    queue_status(status_ready);

    TEST_ASSERT_EQUAL_INT(FLASH_ERR_WRITE_ENABLE,
                          flash_page_program(0x000100, data, sizeof(data)));
    TEST_ASSERT_EQUAL_UINT32(3, mock_call_count);
    assert_transaction(0, read_status_command, 1, 1);
    assert_transaction(1, write_enable_command, 1, 0);
    assert_transaction(2, read_status_command, 1, 1);
}

void test_read_id_rejects_null(void)
{
    TEST_ASSERT_EQUAL_INT(FLASH_ERR_INVALID_ARG, flash_read_id(NULL));
    TEST_ASSERT_EQUAL_UINT32(0, mock_call_count);
}

void test_init_sends_reset_before_reading_id(void)
{
    const uint8_t reset_enable[] = {0x66};
    const uint8_t reset[] = {0x99};
    const uint8_t read_id[] = {0x9F};
    const uint8_t id[] = {0xEF, 0x40, 0x18};

    queue_successful_write();
    queue_successful_write();
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, id, sizeof(id)));
    queue_status(status_ready);

    TEST_ASSERT_EQUAL_INT(FLASH_OK, flash_init());
    assert_transaction(0, reset_enable, 1, 0);
    assert_transaction(1, reset, 1, 0);
    assert_transaction(2, read_id, 1, sizeof(id));
    assert_transaction(3, read_status_command, 1, 1);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, mock_delay_total_ms);
}

void test_init_rejects_wrong_id(void)
{
    const uint8_t id[] = {0xFF, 0xFF, 0xFF};

    queue_successful_write();
    queue_successful_write();
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, id, sizeof(id)));

    TEST_ASSERT_EQUAL_INT(FLASH_ERR_DEVICE_ID, flash_init());
    TEST_ASSERT_EQUAL_UINT32(3, mock_call_count);
}

void test_write_enable_checks_wel(void)
{
    queue_successful_write();
    queue_status(status_wel);

    TEST_ASSERT_EQUAL_INT(FLASH_OK, flash_write_enable());
    assert_transaction(0, write_enable_command, 1, 0);
    assert_transaction(1, read_status_command, 1, 1);
}

void test_write_enable_propagates_bus_timeout(void)
{
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_TIMEOUT, NULL, 0));

    TEST_ASSERT_EQUAL_INT(FLASH_ERR_TIMEOUT, flash_write_enable());
    TEST_ASSERT_EQUAL_UINT32(1, mock_call_count);
    assert_transaction(0, write_enable_command, 1, 0);
}

void test_wait_ready_polls_until_ready(void)
{
    queue_status(status_busy);
    queue_status(status_busy);
    queue_status(status_ready);

    TEST_ASSERT_EQUAL_INT(FLASH_OK, flash_wait_ready(2));
    TEST_ASSERT_EQUAL_UINT32(3, mock_call_count);
    TEST_ASSERT_EQUAL_UINT32(2, mock_delay_total_ms);
}

void test_wait_ready_times_out_while_busy(void)
{
    const uint32_t timeout_ms = 3;

    /* The driver checks once immediately, then once per elapsed millisecond. */
    for (uint32_t i = 0; i <= timeout_ms; ++i)
    {
        queue_status(status_busy);
    }

    TEST_ASSERT_EQUAL_INT(FLASH_ERR_TIMEOUT, flash_wait_ready(timeout_ms));
    TEST_ASSERT_EQUAL_UINT32(timeout_ms + 1, mock_call_count);
    TEST_ASSERT_EQUAL_UINT32(timeout_ms, mock_delay_total_ms);
}

void test_read_rejects_range_past_end_of_flash(void)
{
    uint8_t buf[2];

    TEST_ASSERT_EQUAL_INT(FLASH_ERR_ADDRESS, flash_read(0xFFFFFF, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT32(0, mock_call_count);
}

void test_page_program_sends_full_page(void)
{
    uint8_t data[FLASH_PAGE_SIZE];
    uint8_t expected_tx[4 + FLASH_PAGE_SIZE] = {0x02, 0x01, 0x23, 0x00};

    for (uint16_t i = 0; i < sizeof(data); ++i)
    {
        data[i] = (uint8_t)i;
    }
    memcpy(expected_tx + 4, data, sizeof(data));
    queue_program_or_erase();

    TEST_ASSERT_EQUAL_INT(FLASH_OK, flash_page_program(0x012300, data, sizeof(data)));
    assert_program_or_erase_sequence(expected_tx, sizeof(expected_tx));
}

void test_sector_erase_rejects_unaligned_address(void)
{
    TEST_ASSERT_EQUAL_INT(FLASH_ERR_ADDRESS, flash_sector_erase(0x012345));
    TEST_ASSERT_EQUAL_UINT32(0, mock_call_count);
}

void test_sector_erase_sends_command_and_address(void)
{
    const uint8_t expected_tx[] = {0x20, 0x12, 0x30, 0x00};

    queue_program_or_erase();

    TEST_ASSERT_EQUAL_INT(FLASH_OK, flash_sector_erase(0x123000));
    assert_program_or_erase_sequence(expected_tx, sizeof(expected_tx));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_read_sends_command_and_msb_first_address);
    RUN_TEST(test_page_program_rejects_page_crossing);
    RUN_TEST(test_page_program_stops_when_write_enable_fails);
    RUN_TEST(test_read_id_rejects_null);
    RUN_TEST(test_init_sends_reset_before_reading_id);
    RUN_TEST(test_init_rejects_wrong_id);
    RUN_TEST(test_write_enable_checks_wel);
    RUN_TEST(test_write_enable_propagates_bus_timeout);
    RUN_TEST(test_wait_ready_polls_until_ready);
    RUN_TEST(test_wait_ready_times_out_while_busy);
    RUN_TEST(test_read_rejects_range_past_end_of_flash);
    RUN_TEST(test_page_program_sends_full_page);
    RUN_TEST(test_sector_erase_rejects_unaligned_address);
    RUN_TEST(test_sector_erase_sends_command_and_address);
    return UNITY_END();
}
