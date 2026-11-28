/*
 * imu.c
 *
 * Created on: Nov 25, 2026
 */

#include "imu.h"
#include "spi_bus.h"
#include <stddef.h>

/* Register addresses */
#define IMU_REG_WHO_AM_I         0x0FU
#define IMU_REG_CTRL1_XL         0x10U
#define IMU_REG_CTRL2_G          0x11U
#define IMU_REG_CTRL3_C          0x12U
#define IMU_REG_OUTX_L_G         0x22U

/* SPI configuration */
#define IMU_SPI_READ             0x80U
#define IMU_SPI_WRITE_MASK       0x7FU

/* Device identification */
#define IMU_WHO_AM_I_VALUE       0x6CU

/* Accelerometer configuration */
#define IMU_CTRL1_XL_416HZ_2G    0x60U
#define IMU_CTRL1_XL_104HZ_2G    0x40U

/* Gyroscope configuration */
#define IMU_CTRL2_G_ODR_416HZ    0x60U
#define IMU_CTRL2_G_FS_500DPS    0x04U

/* Control register configuration */
#define IMU_CTRL3_C_BDU          0x40U
#define IMU_CTRL3_C_IF_INC       0x04U

/* Sensor sensitivities */
#define IMU_GYRO_SENS            0.0175f
#define IMU_ACCEL_SENS           0.000061f


/* Convert SPI bus status into IMU status */
static imu_status_t imu_check_status(spi_bus_status_t status)
{
    switch (status)
    {
        case SPI_BUS_OK:
            return IMU_OK;

        case SPI_BUS_TIMEOUT:
            return IMU_ERR_TIMEOUT;

        case SPI_BUS_BUSY:
            return IMU_ERR_BUSY;

        case SPI_BUS_INVALID_ARG:
            return IMU_ERR_INVALID_ARG;

        default:
            return IMU_ERR_SPI;
    }
}


/* Read a single register */
imu_status_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL)
    {
        return IMU_ERR_INVALID_ARG;
    }

    uint8_t tx = reg | IMU_SPI_READ;

    spi_bus_status_t status = spi_bus_write_read(
        SPI_DEV_IMU, &tx, 1, value, 1);

    return imu_check_status(status);
}


/* Write a single register */
imu_status_t imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & IMU_SPI_WRITE_MASK, value};

    spi_bus_status_t status = spi_bus_write(
        SPI_DEV_IMU, tx, 2);

    return imu_check_status(status);
}


/* Read multiple consecutive registers */
imu_status_t imu_burst_read(uint8_t start_reg,
                            uint8_t *buf,
                            uint8_t len)
{
    if (buf == NULL || len == 0)
    {
        return IMU_ERR_INVALID_ARG;
    }

    uint8_t tx = start_reg | IMU_SPI_READ;

    spi_bus_status_t status = spi_bus_write_read(
        SPI_DEV_IMU, &tx, 1, buf, len);

    return imu_check_status(status);
}


/* Initialize IMU */
imu_status_t imu_init(void)
{
    uint8_t id;
    imu_status_t status;

    status = imu_read_reg(IMU_REG_WHO_AM_I, &id);

    if (status != IMU_OK)
    {
        return status;
    }

    if (id != IMU_WHO_AM_I_VALUE)
    {
        return IMU_ERR_DEVICE_ID;
    }

    status = imu_write_reg(IMU_REG_CTRL3_C,
                           IMU_CTRL3_C_BDU |
                           IMU_CTRL3_C_IF_INC);

    if (status != IMU_OK)
    {
        return status;
    }

    status = imu_write_reg(IMU_REG_CTRL1_XL,
                           IMU_CTRL1_XL_416HZ_2G);

    if (status != IMU_OK)
    {
        return status;
    }

    status = imu_write_reg(IMU_REG_CTRL2_G,
                           IMU_CTRL2_G_ODR_416HZ |
                           IMU_CTRL2_G_FS_500DPS);

    if (status != IMU_OK)
    {
        return status;
    }

    return IMU_OK;
}

imu_status_t imu_read(imu_data_t *out)
{
    if (out == NULL)
    {
        return IMU_ERR_INVALID_ARG;
    }

    uint8_t buf[12];

    imu_status_t status =
        imu_burst_read(IMU_REG_OUTX_L_G, buf, 12);

    if (status != IMU_OK)
    {
        return status;
    }
    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t ax = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t ay = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

    /* Convert gyroscope to degrees per second */
    out->gx = gx * IMU_GYRO_SENS;
    out->gy = gy * IMU_GYRO_SENS;
    out->gz = gz * IMU_GYRO_SENS;

    /* Convert accelerometer to g */
    out->ax = ax * IMU_ACCEL_SENS;
    out->ay = ay * IMU_ACCEL_SENS;
    out->az = az * IMU_ACCEL_SENS;

    return IMU_OK;
}
