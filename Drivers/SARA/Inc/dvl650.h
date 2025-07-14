/*
 * dvl650.h
 *
 *  Created on: Jul 7, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_DVL650_H_
#define SARA_INC_DVL650_H_

# include "stm32f4xx_hal.h"

# define DVL650_MAKE_STAT(base_stat, sub_stat) \
    (uint16_t)( \
       ( ((uint16_t)((sub_stat) & 0xFF)) << 8 ) \
     |   ((uint16_t)((base_stat) & 0xFF))

typedef enum e_dvl650_stat
{
	DVL650_STAT_OK,
	DVL650_STAT_ERR_SELF_NULL_PTR,

}	t_dvl650_stat;

typedef struct s_dvl650
{

	float	vel_x;
	float	vel_y;

} t_dvl650;

uint16_t	dvl650_init(t_dvl650 *dvl650);
uint16_t	dvl650_update(t_dvl650 *dvl650);

#endif /* SARA_INC_DVL650_H_ */
