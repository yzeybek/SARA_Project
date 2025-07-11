/*
 * dvl650.h
 *
 *  Created on: Jul 7, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_DVL650_H_
#define SARA_INC_DVL650_H_

typedef struct s_dvl650
{

	float	vel_x;
	float	vel_y;

} t_dvl650;

void	dvl650_init(t_dvl650 *dvl650);
void	dvl650_update(t_dvl650 *dvl650);

#endif /* SARA_INC_DVL650_H_ */
