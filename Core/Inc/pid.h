/*
 * pid.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <float.h>

typedef struct s_pid
{
    float	Kp;
    float	Ki;
    float	Kd;
    float	integrator;
    float	prev_error;
    float	out_min;
    float	out_max;
    float	int_min;
    float	int_max;

}	t_pid;

void	pid_init(t_pid *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float int_min, float int_max);
float	pid_update(t_pid *pid, float error, float dt);

#endif /* INC_PID_H_ */
