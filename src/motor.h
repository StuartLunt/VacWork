/*
 * motor.h
 *
 *  Created on: 21 Nov 2018
 *      Author: Stuart Lunt
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

void PWM_Init(void);
void setDutyCycle(int percent);


#endif /* MOTOR_H_ */
