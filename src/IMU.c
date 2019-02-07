/*
 * UART.c
 *
 *  Created on: 21 Aug 2018
 *      Author: Sylvan Morris
 *      Updated: Stuart Lunt 2019
 */

#include <IMU.h>

void MPU_Init(I2C_TypeDef* I2Cx){
	uint8_t temp[] = {0};
	I2C_SelectRegister(I2C3,MPU_ADDRESS,0x75);
	I2C_ReadIMU(I2C3,MPU_ADDRESS,temp);


	I2C_WriteIMU(I2C3,MPU_ADDRESS,PWR_MGMT_1,0x00);
	I2C_WriteIMU(I2C3,MPU_ADDRESS,PWR_MGMT_1,0x01);

	I2C_WriteIMU(I2C3,MPU_ADDRESS,GYRO_CONFIG_AD,GYRO_INIT_SETTINGS);
	I2C_WriteIMU(I2C3,MPU_ADDRESS, ACCEL_CONFIG_1_AD,ACC_CONFIG_1_SETTINGS);
	I2C_WriteIMU(I2C3,MPU_ADDRESS, ACCEL_CONFIG_2_AD,ACC_CONFIG_2_SETTINGS);
	I2C_WriteIMU(I2C3,MPU_ADDRESS, CONFIG_AD,CONFIG_INIT);
}

void MPU_GetAccGyro(I2C_TypeDef* I2Cx, int16_t acc[3], int16_t gyro[3], uint8_t temp[2]){
	I2C_SelectRegister(I2Cx,MPU_ADDRESS,ACC_START_ADDRESS);
	 uint8_t data[14];

	 I2C_Read_Multiple(I2Cx, MPU_ADDRESS, data, 14);
	 for( int index = 0; index < 3; index ++){
		  acc[index] = (int16_t)((uint16_t)data[index * 2+1] + ((uint16_t)(data[index * 2]) << 8));
		  gyro[index] = (int16_t)((uint16_t)data[index * 2 + 9] + ((int16_t)(data[index * 2 + 8]) << 8));

	  }

	 temp[0] = data[6];
	 temp[1] = data[7];
}

void IMU_Calibration(I2C_TypeDef* I2Cx){
	// TODO
}
