/*
 * UART.h
 *
 *  Created on: 21 Aug 2018
 *      Author: Sylvan Morris
 *      Updated: Stuart Lunt 2018
 */

#include "stm32f4xx.h"
#include "I2C.h"

#ifndef IMU_H_
#define IMU_H_

//#define USE_BNO055


#define GYRO_SCALE 	1.0825510176338222
#define GYRO_OFFSET -0.7129753210365033

#define X_OFFSET 	0.02709099327962326
#define Y_OFFSET 	0.05
#define X_SCALE  	1.14
#define Y_SCALE   	1.0019266482630083

#ifdef USE_BNO055
// defines for BNO055
#define		TEMP				0x34
#define		IMU					0x28
#define		ACC_CONFIG			0x08
#define		ACC_CONFIG_SETTINGS	0b00011101 // 4G, 1000Hz,
#define		GYRO_CONFIG_0		0x0A
#define		GYRO_INIT_SETTINGS	0b000000 // 523Hz, 2000dps
#define		UNIT_SEL			0x3B
#define		UNIT_SEL_SETTINGS	0b00110 //deg C
#define		OPR_MODE			0x3D
#define		ACCGYRO				0b0101 // ACCGYRO MODE
#define		AMG					0b0111	// ACCMAGGYRO MODE
#define		ACC_START_REG		0x8
#define		ACC_END_REG			0xD
#define		GYRO_START_REG		0x14
#define		GYRO_END_REG		0x19
#define		CONFIG_MODE			0b0000
#endif

#ifndef USE_BNO055
// defines for MPU 9250
#define		MPU_ADDRESS			0xD0//0x68
#define 	MPU_ALT				0x69
#define		PWR_MGMT_1			0x6B
#define 	ACC_START_ADDRESS	0x3B

#define 	GYRO_CONFIG_AD 			0x1B
#define 	ACCEL_CONFIG_1_AD 		0x1C
#define 	ACCEL_CONFIG_2_AD 		0x1D
#define 	CONFIG_AD 				0x1A
#define 	GYRO_INIT_SETTINGS		0b00011000 //+2000 dps, 16.4 LSB/(deg/s)
#define 	ACC_CONFIG_1_SETTINGS	0b00001000 //4G 8192 LSB/g
#define 	ACC_CONFIG_2_SETTINGS 	0b00000001 //Turn on internal low-pass filter.
#define		CONFIG_INIT				0b00000001
#define		FIFO_EN					35
#define		FIFO_INIT				0b11111000
#define		USER_CTRL				106
#define		USER_CTRL_INIT			0b01010000

#endif

void MPU_Init(I2C_TypeDef* I2Cx);
void MPU_GetAccGyro(I2C_TypeDef* I2Cx, int16_t acc[3], int16_t gyro[3], uint8_t temp[2]);
void IMU_Calibration(I2C_TypeDef* I2Cx);
#endif /* IMU_H_ */
