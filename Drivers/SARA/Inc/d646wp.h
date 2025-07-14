/*
 * d646wp.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_D646WP_H_
#define SARA_INC_D646WP_H_

# include "stm32f4xx_hal.h"

# define D646WP_MAKE_STAT(base_stat, sub_stat) \
    (uint16_t)( \
       ( ((uint16_t)((sub_stat) & 0xFF)) << 8 ) \
     |   ((uint16_t)((base_stat) & 0xFF)))

typedef enum e_d646wp_stat
{
	D646WP_STAT_OK,
	D646WP_STAT_ERR_SELF_NULL_PTR,
	D646WP_STAT_ERR_HAL_PWM_START,

} t_d646wp_stat;

typedef struct s_d646wp
{
    TIM_HandleTypeDef	*htim;
    int					channel;
    int         		min_us;
    int           		max_us;

}	t_d646wp;

uint16_t	d646wp_init(t_d646wp *d646wp, TIM_HandleTypeDef* htim, int channel, int min_us, int max_us);
uint16_t	d646wp_update(t_d646wp *d646wp, uint8_t angle);

#endif /* SARA_INC_D646WP_H_ */
