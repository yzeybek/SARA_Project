/*
 * ultras.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "ultras.h"

void ultras_init(t_ultras *ultras, TIM_HandleTypeDef *htim, int channel, int min_pulse, int max_pulse, int period)
{
    TIM_OC_InitTypeDef	sConfig = {0};
    int					timer_clk;
	int					prescaler;

    ultras->htim      = htim;
    ultras->channel   = channel;
    ultras->min_pulse = min_pulse;
    ultras->max_pulse = max_pulse;
    ultras->period    = period;
    timer_clk = HAL_RCC_GetPCLK1Freq() * (((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? 1U : 2U);
    prescaler = (timer_clk / 1000000U) - 1U;
    htim->Init.Prescaler         = prescaler;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = period - 1U;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
        return ;
    sConfig.OCMode     = TIM_OCMODE_PWM1;
    sConfig.Pulse      = min_pulse;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfig, channel) != HAL_OK)
        return ;
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
