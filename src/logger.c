/*
 * logger.c
 *
 *  Created on: 23 Nov 2018
 *      Author: Stuart Lunt
 *      Using code from Sylvan Morris and James Traversham
 */

#include "logger.h"

//DMA buffer data



//Initialise UART and DMA
void loggingInit(){
	/*
	 * https://stm32f4-discovery.net/2014/04/library-04-connect-stm32f429-discovery-to-computer-with-usart/
	 * Initializing UART for reading the IMU values,
	 * PB6 - Tx
	 * PB7 - Rx
	 *
	 */

		USART_InitTypeDef USART_InitStruct;
		GPIO_InitTypeDef GPIO_InitStruct;
		DMA_InitTypeDef DMA_InitStruct;
		NVIC_InitTypeDef NVIC_InitStruct;

		USART_BaudRate = 230400;

		// enable clocks
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);

		// init pins
		GPIO_StructInit(&GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

		GPIO_Init(GPIOB, &GPIO_InitStruct);

		// selective alternative functions
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

		// init  UART


		USART_InitStruct.USART_BaudRate = USART_BaudRate;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;

		USART_Init(USART1, &USART_InitStruct);



//		/* Configure DMA for USART TX, DMA2, Stream7, Channel4 */
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)DMA_TX_Buffer;
		DMA_InitStruct.DMA_BufferSize = DMA_TX_BUFFER_SIZE;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
		DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStruct.DMA_Channel = DMA_Channel_4;

		DMA_Init(DMA2_Stream7, &DMA_InitStruct);

//		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

//		/* Enable global interrupts for DMA stream */
		NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_InitStruct);

//		/* Enable transfer complete interrupt */


		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		/* Enable USART */
		USART_Cmd(USART1, ENABLE);
}

void DMA2_Stream7_IRQHandler(void) {
	DMA_Cmd(DMA2_Stream7, DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
}

CommsTask_TransmitPacketStruct serialTerminal_packetize(uint8_t* payload_to_pack, uint16_t length_of_payload)
{
  CommsTask_TransmitPacketStruct pkt_to_tx;
  uint16_t raw_data_pointer = 0;
  uint16_t packet_data_pointer = 0;
  uint8_t char_to_pack;
  memset(pkt_to_tx.data, 0, COMMS_TX_BUFFER_SIZE);

  //put start char
  pkt_to_tx.data[packet_data_pointer++] = 0x7E; //0x7E = '~'

  //put opcode 0x23 = '#'
  pkt_to_tx.data[packet_data_pointer++] = 0x23;

  //put data
  for (raw_data_pointer = 0; raw_data_pointer < length_of_payload; raw_data_pointer++, packet_data_pointer++)
  { //for both bytes of the pkt_code,
    char_to_pack = payload_to_pack[raw_data_pointer];
    if (char_to_pack == 0x7E)
    {
      pkt_to_tx.data[packet_data_pointer++] = 0x7D;
      pkt_to_tx.data[packet_data_pointer] = 0x5E;
    }
    else if (char_to_pack == 0x7D)
    {
      pkt_to_tx.data[packet_data_pointer++] = 0x7D;
      pkt_to_tx.data[packet_data_pointer] = 0x5D;
    }
    else
    {
      pkt_to_tx.data[packet_data_pointer] = char_to_pack;
    }
  }

  //put end char
  pkt_to_tx.data[packet_data_pointer++] = 0x7E;

  //set pkt length
  pkt_to_tx.bytes_to_tx = packet_data_pointer;

  return pkt_to_tx;
}

void USART_PackageData(uint8_t data[], uint16_t data_length){
	while(DMA_GetCmdStatus(DMA2_Stream7));

	for (int i = 0; i < DMA_TX_BUFFER_SIZE; i++){
				DMA_TX_Buffer[i] = 0;
			}

	for (int i = 0; i < DMA_TX_BUFFER_SIZE; i++){
			DMA_TX_Buffer[i] = data[i];
	}

	DMA_Cmd(DMA2_Stream7,DISABLE);

	while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);

	DMA_SetCurrDataCounter(DMA2_Stream7,data_length);

	DMA_Cmd(DMA2_Stream7, ENABLE);

	while(DMA_GetCmdStatus(DMA2_Stream7) != ENABLE);
}



