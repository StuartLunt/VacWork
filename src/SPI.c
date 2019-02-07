/*
 * SPI.c
 *
 *  Created on: 18 Jan 2019
 *      Author: Stuart Lunt
 */

#include "SPI.h"

void initSPI(){
	//Enable clock for GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/*
	 * PB12 - SS
	 * PB13 - SCK
	 * PB14 - MISO
	 * PB15 - MOSI
	 */

	GPIO_InitTypeDef gpio;

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	//Set Pin 13 to SCL
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);

	//MISO
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Pin = GPIO_Pin_14;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);

	//Set up MOSI
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Pin = GPIO_Pin_15;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);

	//Configure GPIO AF config;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	SPI_InitTypeDef spi;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_High;
	spi.SPI_CPHA = SPI_CPHA_2Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &spi);
	SPI_Cmd(SPI2, ENABLE);

	writeSPI(USER_CTRL, USER_CTRL_INIT);
	writeSPI(PWR_MGMT_1, 0x00);
	writeSPI(PWR_MGMT_1, 0x01);
	writeSPI(GYRO_CONFIG_AD, GYRO_INIT_SETTINGS);
	writeSPI(ACCEL_CONFIG_1_AD, ACC_CONFIG_1_SETTINGS);
	writeSPI(ACCEL_CONFIG_2_AD, ACC_CONFIG_2_SETTINGS);
	writeSPI(CONFIG_AD, CONFIG_INIT);
	writeSPI(FIFO_EN, FIFO_INIT);
}

uint8_t readSingleSPI(uint8_t address){
	address = address|rw;

	// select the chip
	select();

	// wait for a transmission to come through
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	// send the address byte
	sendData(SPI2, address);
	// wait to receive a byte
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	// receive the actual byte
	SPI_I2S_ReceiveData(SPI2);
	// make sure the line is free
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	// send an empty byte (dummy data)
	sendData(SPI2, 0x00);
	// wait to receive a byte
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY));
	// deselect the chip
	for(int i = 0; i<5;i++);
	deselect();

	// return the SPI data
	return SPI2->DR;
}

void readMultipleSPI(uint8_t address, uint8_t *readBuf, uint8_t bytes){
	address = address|rw;
	select();
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	sendData(SPI2, address);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_FLAG_RXNE)==RESET);
	for(uint8_t i = 0; i<bytes;i++){
		address++;
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
		sendData(SPI2, 0x00);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_FLAG_RXNE)==RESET);
		readBuf[i] = SPI2->DR;
	}
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY));
	deselect();

}

void writeSPI(uint8_t address, uint8_t data){
	address = address;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	select();
	sendData(SPI2, address);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	sendData(SPI2, data);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY));
	for(int i = 0; i <5; i++);
	deselect();
}

void writeMultipleSPI(uint8_t address, uint8_t *writeBuf, uint8_t bytes){
	address = address&rw;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	select();
	sendData(SPI2, address);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	for(int i = 0; i<bytes;i++){
		sendData(SPI2, writeBuf[i]);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	}
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY));
	deselect();
}

void swap (uint8_t *array, int num){
	for(int i = 0; i<num; i=i+2){
		uint8_t temp = array[i];
		array[i] = array[i+1];
		array[i+1] = temp;
	}
}
