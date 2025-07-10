/*
 * d646wp.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_D646WP_H_
#define SARA_INC_D646WP_H_

# include "stm32f4xx_hal.h"

typedef struct s_d646wp
{
    TIM_HandleTypeDef	*htim;
    uint32_t			channel;
    uint32_t          	min_us;
    uint32_t           	max_us;
    uint32_t           	period;

}	t_d646wp;

void	d646wp_init(t_d646wp *dp, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t min_us, uint32_t max_us, uint32_t period);
void	d646wp_update(t_d646wp *dp, uint32_t angle);

#endif /* SARA_INC_D646WP_H_ */
