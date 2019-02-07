/*
 * I2C.c
 *
 *  Created on: 20 Nov 2018
 *      Author:  Sylvan Morris
 *      Updated: Stuart Lunt 2018
 */

#include "I2C.h"

void init_I2C3(void) {
	/*
	 * PA8 SCL
	 * PC9 SDA
	 */
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;


	/* Enable I2C and GPIO clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, ENABLE);
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C3, DISABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);



	/* Configure I2C pins: SCL*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure I2C pins: SDA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);

	/* I2C configuration */
	I2C_DeInit(I2C3);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; //https://community.st.com/s/feed/0D50X00009XkaUGSAZ
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED_2;

	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C3, &I2C_InitStructure);

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C3, ENABLE);
}

/*
 * reads single byte
 */

void I2C_SelectRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t start_register){
	//only need to run once and then can run multiple I2C_ReadIMU
	//Transmit
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> start condition correctly released on the bus
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, address, TRANSMIT);

	// wait for EV6 --> Slave has acknowledged
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2Cx, start_register);

	// wait for EV8 --> transmission to start
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

}

void I2C_ReadIMU(I2C_TypeDef* I2Cx, uint8_t address, uint8_t buffer[]) {

	//only reads single byte
	//Receive

	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> start condition correctly released on the bus
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, address, RECEIVE);

	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	buffer[0] = I2C_ReceiveData(I2Cx);
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_Read_Multiple(I2C_TypeDef* I2Cx, uint8_t address, uint8_t buffer[], uint8_t buff_size){

	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> start condition correctly released on the bus
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, address, RECEIVE);

	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	for (int i = 0; i < buff_size; i++){
		if (i < buff_size - 1){
			I2Cx->CR1 |= I2C_CR1_ACK;
			while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
			buffer[i] = I2C_ReceiveData(I2Cx);

		} else {
			while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
			buffer[i] = I2C_ReceiveData(I2Cx);
			I2Cx->CR1 &= ~I2C_CR1_ACK;
			I2C_GenerateSTOP(I2Cx, ENABLE);

		}
	}

}

void I2C_WriteIMU(I2C_TypeDef* I2Cx, uint8_t address,uint8_t reg, uint8_t data) {
	//Transmit
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> start condition correctly released on the bus
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, address, TRANSMIT);

	// wait for EV6 --> Slave has acknowledged
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2Cx, reg);

	// wait for EV8 --> transmission to start
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTING));

	I2C_SendData(I2Cx, data);

	// wait for EV8 --> transmission to start
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTING));

	I2C_GenerateSTOP(I2Cx, ENABLE);
}
