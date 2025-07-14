/*
 * sen0257.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_SEN0257_H_
#define SARA_INC_SEN0257_H_

# include "stm32f4xx_hal.h"

# define SEN0257_OFFSET 0.483f
# define SEN0257_INPUT_VOLTAGE 5.0f
# define SEN0257_FLUID_DENSITY 1024.0f
# define SEN0257_SCALER 250.0f

# define SEN0257_MAKE_STAT(base_stat, sub_stat) \
    (uint16_t)( \
       ( ((uint16_t)((sub_stat) & 0xFF)) << 8 ) \
     |   ((uint16_t)((base_stat) & 0xFF)))

typedef enum e_sen0257_stat
{
	SEN0257_STAT_OK,
	SEN0257_STAT_ERR_SELF_NULL_PTR,
	SEN0257_STAT_ERR_HAL_ADC_START,
	SEN0257_STAT_ERR_HAL_ADC_CONVERSION,
	SEN0257_STAT_ERR_HAL_ADC_STOP,

} t_sen0257_stat;

typedef struct s_sen0257
{
	ADC_HandleTypeDef	*hadc;
	unsigned int		raw;
	float				voltage;
	float				pressure;

}	t_sen0257;


uint16_t	sen0257_init(t_sen0257 *sen0257, ADC_HandleTypeDef *hadc);
uint16_t	sen0257_update(t_sen0257 *sen0257);

#endif /* SARA_INC_SEN0257_H_ */
