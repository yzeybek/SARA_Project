/*
 * d646wp.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "d646wp.h"

void	d646wp_init(t_d646wp *d646wp, TIM_HandleTypeDef* htim, int channel, int min_us, int max_us, int period)
{
    TIM_OC_InitTypeDef	sConfig = {0};
    int					timer_clk;
    int					prescaler;

    d646wp->htim    = htim;
    d646wp->channel = channel;
    d646wp->min_us  = min_us;
    d646wp->max_us  = max_us;
    d646wp->period  = period;
    timer_clk = HAL_RCC_GetPCLK1Freq() * (( (RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? 1 : 2);
    prescaler = timer_clk / 1000000 - 1;
    htim->Init.Prescaler         = prescaler;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = period - 1;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
        return ;
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.Pulse = 0;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfig, channel) != HAL_OK)
        return ;
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
