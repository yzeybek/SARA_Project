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
	int					min_pulse;
	int					max_pulse;
	int					channel;

} t_ultras;

void	ultras_init(t_ultras *ultras, TIM_HandleTypeDef *htim, int min_pulse, int max_pulse, int channel);
void	ultras_update(t_ultras *ultras, int pulse);

#endif /* SARA_INC_ULTRAS_H_ */
