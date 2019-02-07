/*
 * ADC.c
 *
 *  Created on: 12 Dec 2018
 *      Author: Stuart Lunt
 */

#include "ADC.h"

void initADC(){
	//Enable clock for GPIOA
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//Set up PA1 to ADC1 channel 1
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	//common init defaults are mode = independent, prescaler = 2, DMA disabled, two sampling delay = 5
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitTypeDef adc;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_ExternalTrigConv = DISABLE;
	adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc.ADC_NbrOfConversion = 1;
	adc.ADC_Resolution = ADC_Resolution_12b;
	adc.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &adc);
	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles); //should be fast enough. reduce if an issue
}

uint16_t readADC(){
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
