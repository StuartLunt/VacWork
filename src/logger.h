/*
 * logger.h
 *
 *  Created on: 23 Nov 2018
 *      Author: Stuart Lunt
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "math.h"
#include "string.h"

#define DMA_TX_BUFFER_SIZE          40		//TODO make sure these constants are correct 30
#define COMMS_TX_BUFFER_SIZE 		40
#define OF_BAD_PACKET 				0b10000
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];
int32_t USART_BaudRate;


typedef struct {
  uint8_t data[COMMS_TX_BUFFER_SIZE];
  uint16_t bytes_to_tx;
} CommsTask_TransmitPacketStruct;

union Data{
	uint32_t i[6];
	int16_t j[12];
	uint16_t a[12];
	uint8_t k[24];
};

void loggingInit();
void USART_PackageData(uint8_t data[], uint16_t data_length);
void LOG_CombineData( uint32_t loop_count, int16_t acc_norm, int16_t acc_tang, int16_t gyro_vel, uint32_t count, uint8_t temp[2], float states[3]);
CommsTask_TransmitPacketStruct serialTerminal_packetize(uint8_t* payload_to_pack, uint16_t length_of_payload);
#endif /* LOGGER_H_ */
