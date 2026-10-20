#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>

typedef enum
{
    IMU_OK              =  0,
    IMU_ERR_SPI         = -1,
    IMU_ERR_TIMEOUT     = -2,
    IMU_ERR_BUSY        = -3,
    IMU_ERR_DEVICE_ID   = -4,
    IMU_ERR_INVALID_ARG = -5

} imu_status_t;

typedef struct
{
    float gx, gy, gz;
    float ax, ay, az;

} imu_data_t;

imu_status_t imu_read_reg(uint8_t reg, uint8_t *value);

imu_status_t imu_write_reg(uint8_t reg, uint8_t value);

imu_status_t imu_burst_read(uint8_t start_reg,
                            uint8_t *buf,
                            uint8_t len);

imu_status_t imu_init(void);

imu_status_t imu_read(imu_data_t *out);

#endif /* INC_IMU_H_ */
