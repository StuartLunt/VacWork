/*
 * Control.c
 *
 *  Created on: 20 Nov 2018
 *      Author: Stuart Lunt
 */

#include "control.h"

void TIM7_INIT(void){
	/*Initialise TIM7 on APB1 @ 48MHz*/
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	//Enable clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	//Reset values to default
	TIM_TimeBaseStructInit(&TIM_BaseStruct);

	//set up timer for 1kHz
	TIM_BaseStruct.TIM_Prescaler = 39; //Timer frequency = 84/(prescaler+1)	3 = 1khz
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 20999; // 21MHz/1kHz - 1
	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;

	//Initialize TIM7
	TIM_TimeBaseInit(TIM7, &TIM_BaseStruct);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	//Start TIM7
	TIM_Cmd(TIM7, ENABLE);

	run = 0;
}

void TIM7_NVIC_Init(void){
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);
}

void TIM7_IRQHandler(void){
	run = 1;
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
}

//Check status of the loop control variable
int start(){
	return run;
}

//Resets the loop control variable
void reset(){
	run =0;
}
