/*
 * d646wp.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "d646wp.h"

uint16_t	d646wp_init(t_d646wp *d646wp, TIM_HandleTypeDef* htim, int channel, int min_us, int max_us)
{
	HAL_StatusTypeDef	stat;

	if (!d646wp || !htim)
		return (D646WP_STAT_ERR_SELF_NULL_PTR);
    d646wp->htim    = htim;
    d646wp->channel = channel;
    d646wp->min_us  = min_us;
    d646wp->max_us  = max_us;
    stat = HAL_TIM_PWM_Start(htim, channel);
    if (stat != HAL_OK)
    	return (D646WP_MAKE_STAT(D646WP_STAT_ERR_HAL_PWM_START, stat));
    return (D646WP_STAT_OK);
}

uint16_t	d646wp_update(t_d646wp *d646wp, uint8_t angle)
{
	 uint32_t	pulselen;

	 if (!d646wp)
		 return (D646WP_STAT_ERR_SELF_NULL_PTR);
	 if (angle > 180)
    	angle = 180;
    pulselen = d646wp->min_us + ((uint32_t)angle * (d646wp->max_us - d646wp->min_us)) / 180U;
    __HAL_TIM_SET_COMPARE(d646wp->htim, d646wp->channel, pulselen);
    return (D646WP_STAT_OK);
}
