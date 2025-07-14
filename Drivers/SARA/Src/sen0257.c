/*
 * sen0257.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sen0257.h"

uint16_t	sen0257_init(t_sen0257 *sen0257, ADC_HandleTypeDef *hadc)
{
	if (!sen0257 || !hadc)
		return (SEN0257_STAT_ERR_SELF_NULL_PTR);
	sen0257->hadc = hadc;
	sen0257->raw = 0;
	sen0257->pressure = 0.0f;
	sen0257->voltage = 0.0f;
	return (SEN0257_STAT_OK);
}

uint16_t	sen0257_update(t_sen0257 *sen0257)
{
	HAL_StatusTypeDef	stat;

	if (!sen0257)
		return (SEN0257_STAT_ERR_SELF_NULL_PTR);
	stat = HAL_ADC_Start(sen0257->hadc);
	if (stat != HAL_OK)
		return (SEN0257_MAKE_STAT(SEN0257_STAT_ERR_HAL_ADC_START, stat));
	stat = HAL_ADC_PollForConversion(sen0257->hadc, HAL_MAX_DELAY);
	if (stat != HAL_OK)
		return (SEN0257_MAKE_STAT(SEN0257_STAT_ERR_HAL_ADC_CONVERSION, stat));
	sen0257->raw = HAL_ADC_GetValue(sen0257->hadc);
	stat = HAL_ADC_Stop(sen0257->hadc);
	if (stat != HAL_OK)
		return (SEN0257_MAKE_STAT(SEN0257_STAT_ERR_HAL_ADC_STOP, stat));
	sen0257->voltage = sen0257->raw * SEN0257_INPUT_VOLTAGE / SEN0257_FLUID_DENSITY;
	sen0257->pressure = (sen0257->voltage - SEN0257_OFFSET) * SEN0257_SCALER;
	return (SEN0257_STAT_OK);
}
