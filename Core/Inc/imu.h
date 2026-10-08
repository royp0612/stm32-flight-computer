/*
 * imu.h
 *
 *  Created on: Nov 25, 2026
 *      Author: melon
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
uint8_t	imu_read_reg(uint8_t reg);
void imu_write_reg(uint8_t reg, uint8_t value);
void imu_burst_read(uint8_t start_reg, uint8_t *buf, uint8_t len);
int imu_init(void);
typedef struct {
    float gx, gy, gz;   // gyro in degrees per second
    float ax, ay, az;   // accelerometer in g
} imu_data_t;

void imu_read(imu_data_t *out);
#endif /* INC_IMU_H_ */
