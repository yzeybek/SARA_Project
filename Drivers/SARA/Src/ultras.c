/*
 * ultras.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "ultras.h"

uint16_t	ultras_init(t_ultras *ultras, TIM_HandleTypeDef *htim, int channel, int min_pulse, int max_pulse)
{
	HAL_StatusTypeDef	stat;

	if (!ultras || !htim)
		return (ULTRAS_STAT_ERR_SELF_NULL_PTR);
    ultras->htim      = htim;
    ultras->channel   = channel;
    ultras->min_pulse = min_pulse;
    ultras->max_pulse = max_pulse;
    stat = HAL_TIM_PWM_Start(htim, channel);
    if (stat != HAL_OK)
    	return (ULTRAS_MAKE_STAT(ULTRAS_STAT_ERR_HAL_PWM_START, stat));
    __HAL_TIM_SET_COMPARE(htim, channel, min_pulse);
    HAL_Delay(700);
    return (ULTRAS_STAT_OK);
}

uint16_t ultras_update(t_ultras *ultras, int pulse)
{
	if (!ultras)
		return (ULTRAS_STAT_ERR_SELF_NULL_PTR);
    if (pulse < ultras->min_pulse)
    	pulse = ultras->min_pulse;
    else if (pulse > ultras->max_pulse)
    	pulse = ultras->max_pulse;
    __HAL_TIM_SET_COMPARE(ultras->htim, ultras->channel, pulse);
    return (ULTRAS_STAT_OK);
}
