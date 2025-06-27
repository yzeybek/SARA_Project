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

typedef struct s_sen0257
{
	ADC_HandleTypeDef	*hadc;
	uint32_t			raw;
	float				voltage;
	float				pressure;

}	t_sen0257;


void	sen0257_init(t_sen0257 *sen0257, ADC_HandleTypeDef *hadc);
void	sen0257_read(t_sen0257 *sen0257);

#endif /* SARA_INC_SEN0257_H_ */
