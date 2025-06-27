/*
 * d646wp.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_D646WP_H_
#define SARA_INC_D646WP_H_

# include "stm32f4xx_hal.h"

typedef struct s_dp646wp
{
    TIM_HandleTypeDef	*htim;
    uint32_t			ch;
    uint16_t          	min_us;
    uint16_t           	max_us;
    uint16_t           	period;

}	t_dp646wp;

HAL_StatusTypeDef	dp646wp_init(t_dp646wp *dp, TIM_HandleTypeDef* htim, uint32_t ch, uint16_t min_us, uint16_t max_us, uint16_t period);
void				dp646wp_set_angle(t_dp646wp *dp, uint8_t angle);

#endif /* SARA_INC_D646WP_H_ */
