/*
 * ultras.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_ULTRAS_H_
#define SARA_INC_ULTRAS_H_

# include "stm32f4xx_hal.h"

typedef struct s_ultras
{
	TIM_HandleTypeDef	*htim;
	uint32_t			min_pulse;
	uint32_t			max_pulse;
	uint32_t			channel;
	uint32_t			period;

} t_ultras;

void	ultras_init(t_ultras *ultras, TIM_HandleTypeDef *htim, uint32_t min_pulse, uint32_t max_pulse, uint32_t channel, uint32_t period);
void	ultras_update(t_ultras *ultras, uint32_t pulse);

#endif /* SARA_INC_ULTRAS_H_ */
