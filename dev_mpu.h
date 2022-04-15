/*
 * dev_mpu.h
 *
 *  Created on: 18 Sep 2021
 *      Author: chime
 */

#ifndef DEV_MPU_H_
#define DEV_MPU_H_

#include "php_i2c.h"

#define MPU_SAMPLE_DATA_SIZE       14
#define DEV_MPU_ADDR               (0x68)        // Default address of MPU60X0 Pg.45 (WHOAMI Register)
#define DEV_MPU_ACCEL_XOUT_H_ADDR  (0x3B)        // First memory address for sample data burst read. Pg.6
/*
 * DLPF: Digital Low Pass Filter
 * -----------------------------
 * if DLPF is disabled (DLPF_CFG= 0 or 7) : Gyroscope Output rate = 8kHz
 * if DLPF is enabled                     : Gyroscope Output rate = 1kHz
 */

#define MPU_REG_CONFIG_ADDR        (0x1A)        // MPU Configuration Register addr Pg.13
#define MPU_REG_CONFIG_DATA        (0x00)        // MPU Configuration Data Pg.13
                                                 //   -> EXT_SYNC_SET bits[5:3] = 0 (Input disabled)
                                                 //   -> DLPF_CFG     bits[2:0] = 0 (Fs[ACC:1KHz; GYR:8KHz])
/*
 * The MPU sensor output register, FIFO output and DMP sampling are all based on
 * the sample rate.
 * With DLPF disabled, the Fs (Sample Frequency) for both ACC and GYR will be 1KHz
 * Sample Rate = Gyroscope Output rate / (1 + SMPLRT_DIV)
 * Sample Rate = 8KHz /(1+7) = 1KHz
*/
#define MPU_REG_SMPLRT_DIV_ADDR    (0x19)        // MPU Sample rate divider Register addr Pg.11
#define MPU_REG_SMPLRT_DIV_DATA    (0x07)        // MPU Sample rate divider Register data Pg.11


#define MPU_REG_PWR_MGMT_1_ADDR    (0x6B)        // MPU Power Management_1 reg addr Pg.40
#define MPU_REG_PWR_MGMT_1_DATA    (0x01)        // Select Clock source with one of gyroscopes bits[2:0] Pg.40


#define MPU_REG_GYRO_CONFIG_ADDR   (0x1B)        // MPU Gyroscope Configuration reg addr Pg.14
#define MPU_REG_GYRO_CONFIG_DATA   (0x18)        // MPU Gyroscope Configuration data Pg.14
                                                 //     -> bits[7:5] = 0 (no self test)
                                                 //     -> bits[4:3] = 3 (+/- 2000deg/s gyroscope scale range)


#define MPU_REG_ACCEL_CONFIG_ADDR  (0x1C)        // MPU Accelerometer Configuration reg addr Pg.15
#define MPU_REG_ACCEL_CONFIG_DATA  (0x00)        // MPU Accelerometer Configuration data Pg.15
                                                 //     -> bits[7:5] = 0 (no self test)
                                                 //     -> bits[4:3] = 0 (+/- 2g accelerometer scale range)


#define MPU_REG_INT_ENABLE_ADDR    (0x38)        // MPU Interrupt Enable Register Addr Pg.27
#define MPU_REG_INT_ENABLE_DATA    (0x01)        // Enable data ready interrupt. bit[0] = 1

void DEV_MPU_Init(I2C0_Type *);


#endif /* DEV_MPU_H_ */
