/*
 * control.h
 *
 *  Created on: 20 Nov 2018
 *      Author: Stuart Lunt
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "stm32f4xx.h"

volatile int run;

void TIM7_INIT(void);
void TIM7_NVIC_Init(void);
int start();
void reset();

#endif /* CONTROL_H_ */
