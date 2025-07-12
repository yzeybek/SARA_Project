/*
 * ultras.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "ultras.h"

void ultras_init(t_ultras *ultras, TIM_HandleTypeDef *htim, int channel, int min_pulse, int max_pulse)
{
    ultras->htim      = htim;
    ultras->channel   = channel;
    ultras->min_pulse = min_pulse;
    ultras->max_pulse = max_pulse;
    HAL_TIM_PWM_Start(htim, channel);
}

void ultras_update(t_ultras *ultras, int pulse)
{
    if (pulse < ultras->min_pulse)
    	pulse = ultras->min_pulse;
    else if (pulse > ultras->max_pulse)
    	pulse = ultras->max_pulse;
    __HAL_TIM_SET_COMPARE(ultras->htim, ultras->channel, pulse);
}
