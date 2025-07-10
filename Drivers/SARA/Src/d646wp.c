/*
 * d646wp.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "d646wp.h"

void	d646wp_init(t_d646wp *dp, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t min_us, uint32_t max_us, uint32_t period)
{
    TIM_OC_InitTypeDef	sConfig = {0};
    uint32_t			timer_clk;
    uint32_t			prescaler;

    dp->htim    = htim;
    dp->channel = channel;
    dp->min_us  = min_us;
    dp->max_us  = max_us;
    dp->period  = period;

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

void	d646wp_update(t_d646wp *dp, uint32_t angle)
{
	 uint32_t	pulselen;

	 if (angle > 180)
    	angle = 180;
    pulselen = dp->min_us + ((uint32_t)angle * (dp->max_us - dp->min_us)) / 180U;
    __HAL_TIM_SET_COMPARE(dp->htim, dp->channel, pulselen);
}
