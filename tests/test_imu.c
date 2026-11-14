#include "unity.h"
#include "imu.h"
#include "mock_spi_bus.h"

void setUp(void)
{
    mock_spi_reset();
}

void tearDown(void)
{
}

void test_imu_read_register(void)
{
    uint8_t expected = 0x6C;
    uint8_t value = 0;

    mock_spi_set_rx_data(&expected, 1);

    imu_status_t status = imu_read_reg(0x0F, &value);

    TEST_ASSERT_EQUAL_INT(IMU_OK, status);
    TEST_ASSERT_EQUAL_HEX8(0x6C, value);

    TEST_ASSERT_EQUAL_INT(SPI_DEV_IMU, mock_last_device);
    TEST_ASSERT_EQUAL_HEX8(0x8F, mock_last_tx[0]);

    TEST_ASSERT_EQUAL_UINT16(1, mock_last_tx_len);
    TEST_ASSERT_EQUAL_UINT16(1, mock_last_rx_len);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_imu_read_register);

    return UNITY_END();
}
