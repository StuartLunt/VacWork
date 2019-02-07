/*
 * motor.c
 *
 *  Created on: 21 Nov 2018
 *      Author: Stuart Lunt
 */

#include "motor.h"

//Set up motor PWM
void PWM_Init(void){
	//Enable clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD, ENABLE);

	//Set up GPIO E5 for PWM
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	//Set up GPIO for direction control
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	//Set source to the timer
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	//Initialise timer for 10kHz PWM
	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 8399;

	//Initialise TIM9
	TIM_TimeBaseInit(TIM9, &TIM_BaseStruct);


	//Initialise PWM
	TIM_OCInitTypeDef TIM_OCStruct;
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCStruct.TIM_Pulse = 4199;
	TIM_OC1Init(TIM9, &TIM_OCStruct);

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_Cmd(TIM9, ENABLE);
}
/*
 * Changes the duty cycle of the PWM
 * 0% = 0
 * 100% = 8399
 */
void setDutyCycle(int percent){
	int comp = percent*89;
	TIM9->CCR1 = comp;
}
