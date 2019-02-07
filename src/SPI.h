/*
 * SPI.h
 *
 *  Created on: 18 Jan 2019
 *      Author: Stuart Lunt
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include "IMU.h"

#define deselect()	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
#define select()	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
#define rw			0b10000000
#define sendData		SPI_I2S_SendData

void initSPI();
uint8_t readSingleSPI(uint8_t address);
void readMultipleSPI(uint8_t address, uint8_t *readBuf, uint8_t bytes);
void writeSPI(uint8_t address, uint8_t data);
void writeMultipleSPI(uint8_t address, uint8_t *writeBuf, uint8_t bytes);
void swap (uint8_t *array, int num);

#endif /* SPI_H_ */
