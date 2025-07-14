/*
 * ultras.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_ULTRAS_H_
#define SARA_INC_ULTRAS_H_

# include "stm32f4xx_hal.h"

# define ULTRAS_MAKE_STAT(base_stat, sub_stat) \
    (uint16_t)( \
       ( ((uint16_t)((sub_stat) & 0xFF)) << 8 ) \
     |   ((uint16_t)((base_stat) & 0xFF)))

typedef enum e_ultras_stat
{
	ULTRAS_STAT_OK,
	ULTRAS_STAT_ERR_SELF_NULL_PTR,
	ULTRAS_STAT_ERR_HAL_PWM_START,

} t_ultras_stat;

typedef struct s_ultras
{
	TIM_HandleTypeDef	*htim;
	int					min_pulse;
	int					max_pulse;
	int					channel;

} t_ultras;

uint16_t	ultras_init(t_ultras *ultras, TIM_HandleTypeDef *htim, int min_pulse, int max_pulse, int channel);
uint16_t	ultras_update(t_ultras *ultras, int pulse);

#endif /* SARA_INC_ULTRAS_H_ */
