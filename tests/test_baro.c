#include "unity.h"
#include "baro.h"
#include "mock_spi_bus.h"
#include "mock_platform.h"
#include <stddef.h>


/* ==================================================
 * Test calibration and measurement data
 * ================================================== */

static const uint8_t test_calibration[22] =
{
    0xFF,         /* Dummy byte */

    0x20, 0x4E,   /* T1 = 20000 */
    0x00, 0x80,   /* T2 = 32768 */
    0x00,         /* T3 = 0 */

    0x00, 0x50,   /* P1 = 20480 */
    0x00, 0x40,   /* P2 = 16384 */
    0x00,         /* P3 = 0 */
    0x00,         /* P4 = 0 */
    0x10, 0x27,   /* P5 = 10000 */
    0x00, 0x00,   /* P6 = 0 */
    0x00,         /* P7 = 0 */
    0x00,         /* P8 = 0 */
    0x00, 0x00,   /* P9 = 0 */
    0x00,         /* P10 = 0 */
    0x00          /* P11 = 0 */
};


static const uint8_t test_measurement[7] =
{
    0xFF,                /* Dummy byte */
    0x00, 0x20, 0x4E,    /* Pressure = 5120000 */
    0x00, 0xA0, 0x5A     /* Temperature = 5939200 */
};


/* ==================================================
 * Unity setup and teardown
 * ================================================== */

void setUp(void)
{
    mock_spi_reset();
    mock_platform_reset();
}


void tearDown(void)
{
}


/* ==================================================
 * Test helper functions
 * ================================================== */

static void queue_read_byte(uint8_t value)
{
    uint8_t response[2] = {0xFF, value};

    TEST_ASSERT_TRUE(
        mock_spi_queue_response(
            SPI_BUS_OK, response, 2));
}


static void queue_successful_write(void)
{
    TEST_ASSERT_TRUE(
        mock_spi_queue_response(
            SPI_BUS_OK, NULL, 0));
}


static void queue_init_with_calibration(
    uint8_t final_error,
    const uint8_t *calibration)
{
    /* Chip ID */
    queue_read_byte(0x50);

    /* Command ready */
    queue_read_byte(0x10);

    /* Soft reset */
    queue_successful_write();

    /* Command ready after reset */
    queue_read_byte(0x10);

    /* No internal errors */
    queue_read_byte(0x00);

    /* Calibration bytes */
    TEST_ASSERT_TRUE(
        mock_spi_queue_response(
            SPI_BUS_OK, calibration, 22));

    /* OSR */
    queue_successful_write();

    /* ODR */
    queue_successful_write();

    /* CONFIG */
    queue_successful_write();

    /* PWR_CTRL */
    queue_successful_write();

    /* Final error register */
    queue_read_byte(final_error);
}


static void queue_successful_init(uint8_t final_error)
{
    uint8_t calibration[22] = {0};

    calibration[0] = 0xFF;

    queue_init_with_calibration(
        final_error, calibration);
}


static void queue_valid_measurement(void)
{
    /* Pressure and temperature ready */
    queue_read_byte(0x60);

    TEST_ASSERT_TRUE(
        mock_spi_queue_response(
            SPI_BUS_OK,
            test_measurement,
            sizeof(test_measurement)));
}


/* ==================================================
 * SPI communication tests
 * ================================================== */


/* Test 1: Register read command */

void test_baro_read_register_sets_read_bit(void)
{
    uint8_t response[2] = {0x00, 0x50};
    uint8_t value = 0;

    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, response, 2));

    baro_status_t status =
        baro_read_reg(0x00, &value);

    TEST_ASSERT_EQUAL_INT(BARO_OK, status);

    TEST_ASSERT_EQUAL_INT(
        SPI_DEV_BARO, mock_last_device);

    TEST_ASSERT_EQUAL_HEX8(
        0x80, mock_last_tx[0]);

    TEST_ASSERT_EQUAL_UINT16(
        1, mock_last_tx_len);

    TEST_ASSERT_EQUAL_UINT16(
        2, mock_last_rx_len);
}


/* Test 2: Dummy byte handling */

void test_baro_read_discards_dummy_byte(void)
{
    uint8_t response[2] = {0xFF, 0x50};
    uint8_t value = 0;

    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, response, 2));

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_read_reg(0x00, &value));

    TEST_ASSERT_EQUAL_HEX8(0x50, value);
}


/* Test 3: Register write */

void test_baro_write_register_clears_read_bit(void)
{
    queue_successful_write();

    baro_status_t status =
        baro_write_reg(0x9C, 0x03);

    TEST_ASSERT_EQUAL_INT(BARO_OK, status);

    TEST_ASSERT_EQUAL_INT(
        SPI_DEV_BARO, mock_last_device);

    TEST_ASSERT_EQUAL_UINT16(
        2, mock_last_tx_len);

    TEST_ASSERT_EQUAL_HEX8(
        0x1C, mock_last_tx[0]);

    TEST_ASSERT_EQUAL_HEX8(
        0x03, mock_last_tx[1]);
}


/* Test 4: Burst read */

void test_baro_burst_read_discards_dummy_byte(void)
{
    uint8_t response[7] =
    {
        0xFF,
        0x11, 0x22, 0x33,
        0x44, 0x55, 0x66
    };

    uint8_t expected[6] =
    {
        0x11, 0x22, 0x33,
        0x44, 0x55, 0x66
    };

    uint8_t buf[6] = {0};

    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, response, 7));

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_burst_read(0x04, buf, 6));

    TEST_ASSERT_EQUAL_HEX8_ARRAY(
        expected, buf, 6);

    TEST_ASSERT_EQUAL_HEX8(
        0x84, mock_last_tx[0]);

    TEST_ASSERT_EQUAL_UINT16(
        7, mock_last_rx_len);
}


/* Test 5: Invalid arguments */

void test_baro_invalid_arguments_skip_spi(void)
{
    uint8_t buf[33];

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_ARG,
        baro_read_reg(0x00, NULL));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_ARG,
        baro_burst_read(0x04, NULL, 6));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_ARG,
        baro_burst_read(0x04, buf, 0));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_ARG,
        baro_burst_read(0x04, buf, 33));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_ARG,
        baro_read(NULL));

    TEST_ASSERT_EQUAL_UINT32(
        0, mock_call_count);
}


/* Test 6: Incorrect chip ID */

void test_baro_init_rejects_wrong_chip_id(void)
{
    uint8_t response[2] = {0x00, 0xFF};

    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_OK, response, 2));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_DEVICE_ID, baro_init());

    TEST_ASSERT_EQUAL_UINT32(
        1, mock_call_count);

    TEST_ASSERT_EQUAL_UINT32(
        0, mock_delay_call_count);
}


/* Test 7: SPI error propagation */

void test_baro_propagates_spi_errors(void)
{
    uint8_t value = 0;

    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_TIMEOUT, NULL, 0));
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_BUSY, NULL, 0));
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_ERR, NULL, 0));
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_INVALID_ARG, NULL, 0));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_TIMEOUT,
        baro_read_reg(0x00, &value));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_BUSY,
        baro_read_reg(0x00, &value));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_SPI,
        baro_read_reg(0x00, &value));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_ARG,
        baro_read_reg(0x00, &value));
}


/* Test 8: Preserve buffers after communication failure */

void test_baro_read_failure_preserves_data(void)
{
    uint8_t value = 0xAB;

    uint8_t buf[3] =
        {0x11, 0x22, 0x33};

    uint8_t expected[3] =
        {0x11, 0x22, 0x33};

    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_TIMEOUT, NULL, 0));
    TEST_ASSERT_TRUE(mock_spi_queue_response(SPI_BUS_TIMEOUT, NULL, 0));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_TIMEOUT,
        baro_read_reg(0x00, &value));

    TEST_ASSERT_EQUAL_HEX8(0xAB, value);

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_TIMEOUT,
        baro_burst_read(0x04, buf, 3));

    TEST_ASSERT_EQUAL_HEX8_ARRAY(
        expected, buf, 3);
}


/* ==================================================
 * Initialization tests
 * ================================================== */


/* Test 9: Successful initialization */

void test_baro_successful_initialization(void)
{
    queue_successful_init(0x00);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    TEST_ASSERT_EQUAL_UINT32(
        11, mock_call_count);

    TEST_ASSERT_EQUAL_UINT32(
        11, mock_history_count);

    TEST_ASSERT_EQUAL_UINT32(
        11, mock_response_index);

    /* Verify chip ID read */
    TEST_ASSERT_EQUAL_HEX8(
        0x80, mock_history[0].tx[0]);

    /* Verify reset command */
    TEST_ASSERT_EQUAL_HEX8(
        0x7E, mock_history[2].tx[0]);

    TEST_ASSERT_EQUAL_HEX8(
        0xB6, mock_history[2].tx[1]);

    /* Verify calibration read */
    TEST_ASSERT_EQUAL_HEX8(
        0xB1, mock_history[5].tx[0]);

    TEST_ASSERT_EQUAL_UINT16(
        22, mock_history[5].rx_len);

    /* Verify oversampling */
    TEST_ASSERT_EQUAL_HEX8(
        0x1C, mock_history[6].tx[0]);

    TEST_ASSERT_EQUAL_HEX8(
        0x03, mock_history[6].tx[1]);

    /* Verify output data rate */
    TEST_ASSERT_EQUAL_HEX8(
        0x1D, mock_history[7].tx[0]);

    TEST_ASSERT_EQUAL_HEX8(
        0x02, mock_history[7].tx[1]);

    /* Verify IIR configuration */
    TEST_ASSERT_EQUAL_HEX8(
        0x1F, mock_history[8].tx[0]);

    TEST_ASSERT_EQUAL_HEX8(
        0x00, mock_history[8].tx[1]);

    /* Verify normal mode */
    TEST_ASSERT_EQUAL_HEX8(
        0x1B, mock_history[9].tx[0]);

    TEST_ASSERT_EQUAL_HEX8(
        0x33, mock_history[9].tx[1]);

    /* Verify delays */
    TEST_ASSERT_EQUAL_UINT32(
        22, mock_delay_total_ms);

    TEST_ASSERT_EQUAL_UINT32(
        2, mock_delay_call_count);
}


/* Test 10: Command not ready */

void test_baro_command_not_ready(void)
{
    queue_read_byte(0x50);
    queue_read_byte(0x00);

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_NOT_READY, baro_init());

    TEST_ASSERT_EQUAL_UINT32(
        2, mock_call_count);
}


/* Test 11: Fatal device error */

void test_baro_detects_fatal_error(void)
{
    queue_read_byte(0x50);
    queue_read_byte(0x10);

    queue_successful_write();

    queue_read_byte(0x10);
    queue_read_byte(0x01);

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_FATAL_DEVICE, baro_init());

    TEST_ASSERT_EQUAL_UINT32(
        5, mock_call_count);
}


/* Test 12: Configuration error */

void test_baro_detects_configuration_error(void)
{
    queue_successful_init(0x04);

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_CONFIG, baro_init());

    TEST_ASSERT_EQUAL_UINT32(
        11, mock_call_count);
}


/* ==================================================
 * Numerical compensation tests
 * ================================================== */


/* Test 13: Temperature compensation */

void test_baro_temperature_compensation(void)
{
    queue_init_with_calibration(
        0x00, test_calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    queue_valid_measurement();

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_read(&data));

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f, 25.0f, data.temperature);
}


/* Test 14: Pressure compensation */

void test_baro_pressure_compensation(void)
{
    queue_init_with_calibration(
        0x00, test_calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    queue_valid_measurement();

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_read(&data));

    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, 100000.0f, data.pressure);
}


/* Test 15: Altitude calculation */

void test_baro_altitude_calculation(void)
{
    queue_init_with_calibration(
        0x00, test_calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    queue_valid_measurement();

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_read(&data));

    TEST_ASSERT_FLOAT_WITHIN(
        0.1f, 110.90f, data.altitude);
}


/* ==================================================
 * Measurement error handling tests
 * ================================================== */


/* Test 16: Reject read after failed initialization */

void test_baro_read_before_initialization(void)
{
    queue_read_byte(0xFF);

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_DEVICE_ID, baro_init());

    uint32_t previous_calls = mock_call_count;

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_NOT_INITIALIZED,
        baro_read(&data));

    TEST_ASSERT_EQUAL_UINT32(
        previous_calls, mock_call_count);
}


/* Test 17: Data not ready */

void test_baro_data_not_ready(void)
{
    queue_init_with_calibration(
        0x00, test_calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    /* Only pressure is ready */
    queue_read_byte(0x20);

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_NOT_READY,
        baro_read(&data));

    TEST_ASSERT_EQUAL_UINT32(
        12, mock_call_count);
}


/* Test 18: Invalid compensated pressure */

void test_baro_rejects_invalid_pressure(void)
{
    uint8_t calibration[22] = {0};

    queue_init_with_calibration(
        0x00, calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    queue_valid_measurement();

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_INVALID_DATA,
        baro_read(&data));
}


/* Test 19: Preserve previous valid measurement */

void test_baro_preserves_previous_sample(void)
{
    queue_init_with_calibration(
        0x00, test_calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    queue_valid_measurement();

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_read(&data));

    baro_data_t previous = data;

    /* Next measurement ready */
    queue_read_byte(0x60);

    /* Measurement burst read fails */
    TEST_ASSERT_TRUE(
        mock_spi_queue_response(
            SPI_BUS_TIMEOUT, NULL, 0));

    TEST_ASSERT_EQUAL_INT(
        BARO_ERR_TIMEOUT,
        baro_read(&data));

    TEST_ASSERT_EQUAL_FLOAT(
        previous.temperature,
        data.temperature);

    TEST_ASSERT_EQUAL_FLOAT(
        previous.pressure,
        data.pressure);

    TEST_ASSERT_EQUAL_FLOAT(
        previous.altitude,
        data.altitude);
}


/* Test 20: Verify measurement SPI transaction */

void test_baro_measurement_transaction(void)
{
    queue_init_with_calibration(
        0x00, test_calibration);

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_init());

    queue_valid_measurement();

    baro_data_t data;

    TEST_ASSERT_EQUAL_INT(
        BARO_OK, baro_read(&data));

    TEST_ASSERT_EQUAL_INT(
        SPI_DEV_BARO, mock_last_device);

    TEST_ASSERT_EQUAL_HEX8(
        0x84, mock_last_tx[0]);

    TEST_ASSERT_EQUAL_UINT16(
        1, mock_last_tx_len);

    TEST_ASSERT_EQUAL_UINT16(
        7, mock_last_rx_len);

    TEST_ASSERT_EQUAL_UINT32(
        13, mock_call_count);
}


/* ==================================================
 * Unity test runner
 * ================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* SPI communication */
    RUN_TEST(test_baro_read_register_sets_read_bit);
    RUN_TEST(test_baro_read_discards_dummy_byte);
    RUN_TEST(test_baro_write_register_clears_read_bit);
    RUN_TEST(test_baro_burst_read_discards_dummy_byte);
    RUN_TEST(test_baro_invalid_arguments_skip_spi);
    RUN_TEST(test_baro_init_rejects_wrong_chip_id);
    RUN_TEST(test_baro_propagates_spi_errors);
    RUN_TEST(test_baro_read_failure_preserves_data);

    /* Initialization */
    RUN_TEST(test_baro_successful_initialization);
    RUN_TEST(test_baro_command_not_ready);
    RUN_TEST(test_baro_detects_fatal_error);
    RUN_TEST(test_baro_detects_configuration_error);

    /* Compensation */
    RUN_TEST(test_baro_temperature_compensation);
    RUN_TEST(test_baro_pressure_compensation);
    RUN_TEST(test_baro_altitude_calculation);

    /* Measurement error handling */
    RUN_TEST(test_baro_read_before_initialization);
    RUN_TEST(test_baro_data_not_ready);
    RUN_TEST(test_baro_rejects_invalid_pressure);
    RUN_TEST(test_baro_preserves_previous_sample);
    RUN_TEST(test_baro_measurement_transaction);

    return UNITY_END();
}
