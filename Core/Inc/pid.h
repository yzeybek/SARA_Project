/*
 * pid.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_PID_H_
#define INC_PID_H_

# include "stdint.h"
# include "float.h"

typedef enum e_pid_stat
{
	PID_STAT_OK,
	PID_STAT_ERR_SELF_NULL_PTR,

} t_pid_stat;

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

uint16_t	pid_init(t_pid *pid, float pid_values[7]);
float		pid_update(t_pid *pid, float error, float dt);

#endif /* INC_PID_H_ */
