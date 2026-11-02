/*
 * baro.c
 *
 * BMP388 pressure and temperature sensor driver
 */

#include "baro.h"
#include "spi_bus.h"
#include "platform.h"

#include <stddef.h>
#include <string.h>
#include <math.h>


/* ---------- Register addresses ---------- */

#define BARO_REG_CHIP_ID        0x00U
#define BARO_REG_ERR            0x02U
#define BARO_REG_STATUS         0x03U
#define BARO_REG_DATA           0x04U
#define BARO_REG_INT_CTRL       0x19U
#define BARO_REG_PWR_CTRL       0x1BU
#define BARO_REG_OSR            0x1CU
#define BARO_REG_ODR            0x1DU
#define BARO_REG_CONFIG         0x1FU
#define BARO_REG_CALIB          0x31U
#define BARO_REG_CMD            0x7EU


/* ---------- Identity and commands ---------- */

#define BARO_DEVICE_ID          0x50U
#define BARO_CMD_SOFT_RESET     0xB6U
#define BARO_RESET_DELAY_MS     2U


/* ---------- SPI configuration ---------- */

#define BARO_SPI_READ           0x80U
#define BARO_SPI_WRITE_MASK     0x7FU
#define BARO_MAX_BURST          32U


/* ---------- Error register bits ---------- */

#define BARO_ERRREG_FATAL       0x01U
#define BARO_ERRREG_CMD         0x02U
#define BARO_ERRREG_CONF        0x04U


/* ---------- Status register bits ---------- */

#define BARO_STATUS_CMD_RDY     0x10U
#define BARO_STATUS_DRDY_PRESS  0x20U
#define BARO_STATUS_DRDY_TEMP   0x40U


/* ---------- Power control ---------- */

#define BARO_PWR_SLEEP_MODE     0x00U
#define BARO_PWR_PRESS_EN       0x01U
#define BARO_PWR_TEMP_EN        0x02U
#define BARO_PWR_NORMAL_MODE    0x30U


/* ---------- Oversampling ---------- */

#define BARO_OSR_PRESS_X8       0x03U
#define BARO_OSR_TEMP_X1        0x00U


/* ---------- Output data rate ---------- */

#define BARO_ODR_50HZ           0x02U


/* ---------- IIR filter ---------- */

#define BARO_IIR_OFF            0x00U
#define BARO_IIR_COEF_3         0x04U


/* ---------- Calibration and sensor data ---------- */

#define BARO_CALIB_LEN          21U
#define BARO_DATA_LEN           6U
#define BARO_SEA_LEVEL_PA       101325.0f


/* ---------- Calibration structure ---------- */

typedef struct
{
    double t1;
    double t2;
    double t3;

    double p1;
    double p2;
    double p3;
    double p4;
    double p5;
    double p6;
    double p7;
    double p8;
    double p9;
    double p10;
    double p11;

} baro_calib_t;

static baro_calib_t baro_calib;
static uint8_t baro_initialized = 0U;

/* ==================================================
 * Private helper functions
 * ================================================== */


/* Convert SPI bus status into barometer status */
static uint32_t baro_decode_u24(const uint8_t *buf)
{
    return ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[1] << 8)  |
           ((uint32_t)buf[0]);
}
static double baro_compensate_pressure(
    uint32_t raw_press,
    double temperature)
{
    double p = (double)raw_press;
    double t = temperature;

    double t2 = t * t;
    double t3 = t2 * t;

    double p2 = p * p;
    double p3 = p2 * p;

    double offset =
        baro_calib.p5 +
        baro_calib.p6 * t +
        baro_calib.p7 * t2 +
        baro_calib.p8 * t3;

    double sensitivity =
        p * (baro_calib.p1 +
             baro_calib.p2 * t +
             baro_calib.p3 * t2 +
             baro_calib.p4 * t3);

    double nonlinear =
        p2 * (baro_calib.p9 +
              baro_calib.p10 * t) +
        p3 * baro_calib.p11;

    return offset + sensitivity + nonlinear;
}
static double baro_compensate_temperature(uint32_t raw_temp)
{
    double delta = (double)raw_temp - baro_calib.t1;

    return delta * baro_calib.t2 +
           delta * delta * baro_calib.t3;
}
static double baro_calculate_altitude(double pressure)
{
    return 44330.0 *
        (1.0 - pow(pressure / BARO_SEA_LEVEL_PA,
                   0.1903));
}
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


/* Decode a little-endian 16-bit value */

static uint16_t baro_decode_u16(const uint8_t *buf)
{
    return (uint16_t)(
        ((uint16_t)buf[1] << 8) | buf[0]);
}


/* Decode and scale factory calibration coefficients */

static void baro_parse_calibration(const uint8_t *raw)
{
    /* Temperature coefficients */

    baro_calib.t1 = ldexp(
        baro_decode_u16(&raw[0]), 8);

    baro_calib.t2 = ldexp(
        baro_decode_u16(&raw[2]), -30);

    baro_calib.t3 = ldexp(
        (int8_t)raw[4], -48);


    /* Pressure coefficients */

    baro_calib.p1 = ldexp(
        (int16_t)baro_decode_u16(&raw[5]) - 16384.0,
        -20);

    baro_calib.p2 = ldexp(
        (int16_t)baro_decode_u16(&raw[7]) - 16384.0,
        -29);

    baro_calib.p3 = ldexp(
        (int8_t)raw[9], -32);

    baro_calib.p4 = ldexp(
        (int8_t)raw[10], -37);

    baro_calib.p5 = ldexp(
        baro_decode_u16(&raw[11]), 3);

    baro_calib.p6 = ldexp(
        baro_decode_u16(&raw[13]), -6);

    baro_calib.p7 = ldexp(
        (int8_t)raw[15], -8);

    baro_calib.p8 = ldexp(
        (int8_t)raw[16], -15);

    baro_calib.p9 = ldexp(
        (int16_t)baro_decode_u16(&raw[17]), -48);

    baro_calib.p10 = ldexp(
        (int8_t)raw[19], -48);

    baro_calib.p11 = ldexp(
        (int8_t)raw[20], -65);
}


/* Check internal sensor errors */

static baro_status_t baro_check_device_errors(void)
{
    uint8_t err;

    baro_status_t status =
        baro_read_reg(BARO_REG_ERR, &err);

    if (status != BARO_OK)
    {
        return status;
    }

    if (err & BARO_ERRREG_FATAL)
    {
        return BARO_ERR_FATAL_DEVICE;
    }

    if (err & BARO_ERRREG_CMD)
    {
        return BARO_ERR_COMMAND;
    }

    if (err & BARO_ERRREG_CONF)
    {
        return BARO_ERR_CONFIG;
    }

    return BARO_OK;
}


/* ==================================================
 * Register communication functions
 * ================================================== */


/* Write a single register */

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


/* Read a single register */

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


/* Read multiple consecutive registers */

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


/* ==================================================
 * Barometer initialization
 * ================================================== */

baro_status_t baro_init(void)
{
    uint8_t id;
    uint8_t reg_status;
    uint8_t baro_calib_raw[BARO_CALIB_LEN];
    baro_initialized = 0U;

    baro_status_t status;


    /* 1. Verify chip ID */

    status = baro_read_reg(BARO_REG_CHIP_ID, &id);

    if (status != BARO_OK)
    {
        return status;
    }

    if (id != BARO_DEVICE_ID)
    {
        return BARO_ERR_DEVICE_ID;
    }


    /* 2. Check command readiness */

    status = baro_read_reg(
        BARO_REG_STATUS, &reg_status);

    if (status != BARO_OK)
    {
        return status;
    }

    if (!(reg_status & BARO_STATUS_CMD_RDY))
    {
        return BARO_ERR_NOT_READY;
    }


    /* 3. Perform soft reset */

    status = baro_write_reg(
        BARO_REG_CMD,
        BARO_CMD_SOFT_RESET);

    if (status != BARO_OK)
    {
        return status;
    }


    /* 4. Wait for reset completion */

    platform_delay_ms(BARO_RESET_DELAY_MS);

    status = baro_read_reg(
        BARO_REG_STATUS, &reg_status);

    if (status != BARO_OK)
    {
        return status;
    }

    if (!(reg_status & BARO_STATUS_CMD_RDY))
    {
        return BARO_ERR_NOT_READY;
    }


    /* 5. Check internal device errors */

    status = baro_check_device_errors();

    if (status != BARO_OK)
    {
        return status;
    }


    /* 6. Read factory calibration coefficients */

    status = baro_burst_read(
        BARO_REG_CALIB,
        baro_calib_raw,
        BARO_CALIB_LEN);

    if (status != BARO_OK)
    {
        return status;
    }

    baro_parse_calibration(baro_calib_raw);


    /* 7. Configure oversampling */

    status = baro_write_reg(
        BARO_REG_OSR,
        BARO_OSR_PRESS_X8 |
        BARO_OSR_TEMP_X1);

    if (status != BARO_OK)
    {
        return status;
    }


    /* 8. Configure output data rate */

    status = baro_write_reg(
        BARO_REG_ODR,
        BARO_ODR_50HZ);

    if (status != BARO_OK)
    {
        return status;
    }


    /* 9. Disable IIR filter initially */

    status = baro_write_reg(
        BARO_REG_CONFIG,
        BARO_IIR_OFF);

    if (status != BARO_OK)
    {
        return status;
    }


    /* 10. Enable sensors and enter normal mode */

    status = baro_write_reg(
        BARO_REG_PWR_CTRL,
        BARO_PWR_PRESS_EN |
        BARO_PWR_TEMP_EN |
        BARO_PWR_NORMAL_MODE);

    if (status != BARO_OK)
    {
        return status;
    }


    /* 11. Allow one measurement period */

    platform_delay_ms(20U);


    /* 12. Verify final configuration */

    status = baro_check_device_errors();

    if (status == BARO_OK)
    {
        baro_initialized = 1U;
    }

    return status;
}
baro_status_t baro_read(baro_data_t *out)
{
    if (out == NULL)
    {
        return BARO_ERR_INVALID_ARG;
    }

    if (!baro_initialized)
    {
        return BARO_ERR_NOT_INITIALIZED;
    }

    uint8_t reg_status;

    baro_status_t status =
        baro_read_reg(BARO_REG_STATUS, &reg_status);

    if (status != BARO_OK)
    {
        return status;
    }

    /* Verify both measurements are ready */
    uint8_t ready_mask =
        BARO_STATUS_DRDY_PRESS |
        BARO_STATUS_DRDY_TEMP;

    if ((reg_status & ready_mask) != ready_mask)
    {
        return BARO_ERR_NOT_READY;
    }

    /* Read six consecutive measurement bytes */
    uint8_t buf[BARO_DATA_LEN];

    status = baro_burst_read(
        BARO_REG_DATA, buf, BARO_DATA_LEN);

    if (status != BARO_OK)
    {
        return status;
    }

    /* Decode raw measurements */
    uint32_t raw_press = baro_decode_u24(&buf[0]);
    uint32_t raw_temp  = baro_decode_u24(&buf[3]);

    /* Temperature must be compensated first */
    double temperature =
        baro_compensate_temperature(raw_temp);

    double pressure =
        baro_compensate_pressure(
            raw_press, temperature);

    /* Reject invalid compensated measurements */
    if (!isfinite(temperature) ||
        !isfinite(pressure) ||
        pressure <= 0.0)
    {
        return BARO_ERR_INVALID_DATA;
    }

    double altitude =
        baro_calculate_altitude(pressure);

    /* Update output only after successful processing */
    out->temperature = (float)temperature;
    out->pressure = (float)pressure;
    out->altitude = (float)altitude;

    return BARO_OK;
}
