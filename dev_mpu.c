/*
 * dev_mpu.c
 *
 *  Created on: 18 Sep 2021
 *      Author: chime
 */
// Motion Processing Unit MPU6050

#include "dev_mpu.h"
#include "php_uart.h"


void DEV_MPU_Init(I2C0_Type * I2C){

    char Error;
    char mem_data;    // this is used to confirm data


    // Enable I2C Peripheral before initializing
    PHP_I2C_Init(I2C);


    // Configure the MPU Power Management_1 register
    Error = PHP_I2C_ByteWrite(I2C, DEV_MPU_ADDR, MPU_REG_PWR_MGMT_1_ADDR, MPU_REG_PWR_MGMT_1_DATA);

    PHP_I2C_ByteRead(I2C, DEV_MPU_ADDR, MPU_REG_PWR_MGMT_1_ADDR, &mem_data);
    PHP_DelaySec(2);
    if (MPU_REG_PWR_MGMT_1_DATA == mem_data) PHP_UART_TxString(UART0, "\n\rPWR -> PASS");


    // Configure the sampling rate divider register
    Error = PHP_I2C_ByteWrite(I2C, DEV_MPU_ADDR, MPU_REG_SMPLRT_DIV_ADDR, MPU_REG_SMPLRT_DIV_DATA);
    PHP_I2C_ByteRead(I2C, DEV_MPU_ADDR, MPU_REG_SMPLRT_DIV_ADDR, &mem_data);
    PHP_DelaySec(2);
    if (MPU_REG_SMPLRT_DIV_DATA == mem_data) PHP_UART_TxString(UART0, "\n\rSMP -> PASS");

    // Configure the MPU Configuration register
    Error = PHP_I2C_ByteWrite(I2C, DEV_MPU_ADDR, MPU_REG_CONFIG_ADDR, MPU_REG_CONFIG_DATA);
    PHP_I2C_ByteRead(I2C, DEV_MPU_ADDR, MPU_REG_CONFIG_ADDR, &mem_data);
    PHP_DelaySec(2);
    if (MPU_REG_CONFIG_DATA == mem_data) PHP_UART_TxString(UART0, "\n\rCONFIG -> PASS");



    // Configure the Gyroscope configuration register
    Error = PHP_I2C_ByteWrite(I2C, DEV_MPU_ADDR, MPU_REG_GYRO_CONFIG_ADDR, MPU_REG_GYRO_CONFIG_DATA);
    PHP_I2C_ByteRead(I2C, DEV_MPU_ADDR, MPU_REG_GYRO_CONFIG_ADDR, &mem_data);
    PHP_DelaySec(2);
    if (MPU_REG_GYRO_CONFIG_DATA == mem_data) PHP_UART_TxString(UART0, "\n\rGYRO CONFIG -> PASS");

    // Configure the Accelerometer configuration register
    Error = PHP_I2C_ByteWrite(I2C, DEV_MPU_ADDR, MPU_REG_ACCEL_CONFIG_ADDR, MPU_REG_ACCEL_CONFIG_DATA);
    PHP_I2C_ByteRead(I2C, DEV_MPU_ADDR, MPU_REG_ACCEL_CONFIG_ADDR, &mem_data);
    PHP_DelaySec(2);
    if (MPU_REG_ACCEL_CONFIG_DATA == mem_data) PHP_UART_TxString(UART0, "\n\rACCEL CONFIG -> PASS");


    // Configure the Interrupt Enable register
    Error = PHP_I2C_ByteWrite(I2C, DEV_MPU_ADDR, MPU_REG_INT_ENABLE_ADDR, MPU_REG_INT_ENABLE_DATA);
    PHP_I2C_ByteRead(I2C, DEV_MPU_ADDR, MPU_REG_INT_ENABLE_ADDR, &mem_data);
    PHP_DelaySec(2);
    if (MPU_REG_INT_ENABLE_DATA == mem_data) PHP_UART_TxString(UART0, "\n\rINT_EN -> PASS");


}



