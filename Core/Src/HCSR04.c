/*
 * HCSR04.c
 *
 *  Created on: Apr 7, 2026
 *      Author: rajap
 */

#include "HCSR04.h"

HCSR04_TypeDef *_HCSR;
GPIO_PinState _State;
float _Distance;

void HCSR04_Init(HCSR04_TypeDef *hcsr) {
	_HCSR = hcsr;
	_State = GPIO_PIN_RESET;
	_Distance = 0.0f;

	__HAL_TIM_SET_AUTORELOAD(_HCSR->TRIG_PWM_Timer.htim, 59999);	// 16 Hz
	__HAL_TIM_SET_COMPARE(_HCSR->TRIG_PWM_Timer.htim,
			_HCSR->TRIG_PWM_Timer.TIM_Channal, 10);

	HAL_TIM_IC_Start_IT(_HCSR->ECHO_IC_Timer.htim,
			_HCSR->ECHO_IC_Timer.TIM_Channal);
}

void HCSR04_Echo_Callback(TIM_HandleTypeDef *htim) {
	if (htim->Instance != _HCSR->ECHO_IC_Timer.htim->Instance
			|| htim->Channel != _HCSR->ECHO_IC_Timer.TIM_Active_Channal)
		return;

	if (_State == GPIO_PIN_RESET) {
		__HAL_TIM_SET_COUNTER(_HCSR->ECHO_IC_Timer.htim, 0);
		_State = GPIO_PIN_SET;

		// Switch to FALLING edge
		__HAL_TIM_SET_CAPTUREPOLARITY(_HCSR->ECHO_IC_Timer.htim,
				_HCSR->ECHO_IC_Timer.TIM_Channal,
				TIM_INPUTCHANNELPOLARITY_FALLING);
	} else {
		uint32_t width_us = __HAL_TIM_GET_COUNTER(_HCSR->ECHO_IC_Timer.htim);

		// Convert to distance (cm)
		_Distance = (float) width_us * 0.034f / 2.0f;

		_State = GPIO_PIN_RESET;

		// Reset to RISING edge
		__HAL_TIM_SET_CAPTUREPOLARITY(_HCSR->ECHO_IC_Timer.htim,
				_HCSR->ECHO_IC_Timer.TIM_Channal,
				TIM_INPUTCHANNELPOLARITY_RISING);
	}
}

void HCSR04_Start() {
	HAL_TIM_PWM_Start(_HCSR->TRIG_PWM_Timer.htim,
			_HCSR->TRIG_PWM_Timer.TIM_Channal);
}

void HCSR04_Stop() {
	HAL_TIM_PWM_Stop(_HCSR->TRIG_PWM_Timer.htim,
			_HCSR->TRIG_PWM_Timer.TIM_Channal);
}

float HCSR04_Read() {
	float dist = _Distance;
//	_Distance = 0;
	return dist;
}
