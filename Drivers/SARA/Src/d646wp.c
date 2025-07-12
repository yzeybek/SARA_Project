/*
 * d646wp.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "d646wp.h"

void	d646wp_init(t_d646wp *d646wp, TIM_HandleTypeDef* htim, int channel, int min_us, int max_us)
{
    d646wp->htim    = htim;
    d646wp->channel = channel;
    d646wp->min_us  = min_us;
    d646wp->max_us  = max_us;
    HAL_TIM_PWM_Start(htim, channel);
}

void	d646wp_update(t_d646wp *d646wp, uint8_t angle)
{
	 uint32_t	pulselen;

	 if (angle > 180)
    	angle = 180;
    pulselen = d646wp->min_us + ((uint32_t)angle * (d646wp->max_us - d646wp->min_us)) / 180U;
    __HAL_TIM_SET_COMPARE(d646wp->htim, d646wp->channel, pulselen);
}
