/*
 * I2C.h
 *
 *  Created on: Jun 6, 2014
 *      Author: Callen Fisher
 *      Updated by Roberto Aldera 2017
 *      Updated by Stuart Lunt 2018
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx.h"
#include <stddef.h>
//#include "AccMag.h"
//I2C1 setup
//#define I2C_CLOCK_1			RCC_AHB1Periph_GPIOB
//#define	I2C_SCL_1			GPIO_Pin_6
//#define	I2C_SDA_1			GPIO_Pin_7
//#define I2C_PORT_1			GPIOB
//#define I2C_SPEED_1			400000 // fast mode requires APB1 clock speed in multiples of 10Mhz for 16/9 duty cycle, but https://forum.sparkfun.com/viewtopic.php?t=28870 's post by hsutherl
//#define SLAVE				0x28
//#define MASTER				0x00

//I2C3 setup
#define I2C_CLOCK_3			RCC_AHB1Periph_GPIOB
#define	I2C_SCL_2			GPIO_Pin_8
#define	I2C_SDA_2			GPIO_Pin_9
#define I2C_PORT_2			GPIOB
#define I2C_SPEED_2			400000

#define	TRANSMIT			0
#define	RECEIVE				1

void init_I2C3(void);
void I2C_SelectRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t start_register);
void I2C_ReadIMU(I2C_TypeDef* I2Cx, uint8_t address, uint8_t buffer[]) ;
void I2C_Read_Multiple(I2C_TypeDef* I2Cx, uint8_t address, uint8_t buffer[], uint8_t buff_size);
void I2C_WriteIMU(I2C_TypeDef* I2Cx, uint8_t address,uint8_t reg, uint8_t data);

#endif /* I2C_H_ */
