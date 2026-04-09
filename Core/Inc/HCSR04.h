/*
 * HCSR04.h
 *
 *  Created on: Apr 7, 2026
 *      Author: rajap
 */

#ifndef HCSR04_H_
#define HCSR04_H_

#include "gpio.h"
#include "tim.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t TIM_Channal;
	HAL_TIM_ActiveChannel TIM_Active_Channal;
} Timer_TypeDef;

typedef struct {
	Timer_TypeDef TRIG_PWM_Timer;
	Timer_TypeDef ECHO_IC_Timer;
} HCSR04_TypeDef;

void HCSR04_Init(HCSR04_TypeDef*);
void HCSR04_Echo_Callback(TIM_HandleTypeDef*);
void HCSR04_Start();
void HCSR04_Stop();
float HCSR04_Read();

#endif /* HCSR04_H_ */
