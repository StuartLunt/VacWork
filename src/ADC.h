/*
 * ADC.h
 *
 *  Created on: 12 Dec 2018
 *      Author: Stuart Lunt
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"

void initADC();
uint16_t readADC();

#endif /* ADC_H_ */
