/*
 * sen0257.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sen0257.h"

void	sen0257_init(t_sen0257 *sen0257, ADC_HandleTypeDef *hadc)
{
	sen0257->hadc = hadc;
	sen0257->raw = 0;
	sen0257->pressure = 0.0f;
	sen0257->voltage = 0.0f;
}

void	sen0257_read(t_sen0257 *sen0257)
{
	HAL_ADC_Start(sen0257->hadc);
	HAL_ADC_PollForConversion(sen0257->hadc, HAL_MAX_DELAY);
	sen0257->raw = HAL_ADC_GetValue(sen0257->hadc);
	HAL_ADC_Stop(sen0257->hadc);
	sen0257->voltage = sen0257->raw * SEN0257_INPUT_VOLTAGE / SEN0257_FLUID_DENSITY;
	sen0257->pressure = (sen0257->voltage - SEN0257_OFFSET) * SEN0257_SCALER;
}
