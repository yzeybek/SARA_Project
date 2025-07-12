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
    int					channel;
    int         		min_us;
    int           		max_us;

}	t_d646wp;

void	d646wp_init(t_d646wp *d646wp, TIM_HandleTypeDef* htim, int channel, int min_us, int max_us);
void	d646wp_update(t_d646wp *d646wp, uint8_t angle);

#endif /* SARA_INC_D646WP_H_ */
