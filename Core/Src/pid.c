/*
 * pid.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "pid.h"

uint16_t pid_init(t_pid *pid, float pid_values[7])
{
	if (!pid || !pid_values)
		return (PID_STAT_ERR_SELF_NULL_PTR);
    pid->Kp = pid_values[0];
    pid->Ki = pid_values[1];
    pid->Kd = pid_values[2];
    pid->out_min = pid_values[3];
    pid->out_max = pid_values[4];
    pid->int_min = pid_values[5];
    pid->int_max = pid_values[6];
    pid->integrator = 0.0f;
    pid->prev_error = FLT_MAX;
    return (PID_STAT_OK);
}

float	pid_update(t_pid *pid, float error, float dt)
{
    float	p;
    float	i;
    float	d;
    float	output;

    p = pid->Kp * error;
    pid->integrator += error * dt;
    if (pid->int_min == pid->int_min)
    {
        if (pid->integrator < pid->int_min)
        	pid->integrator = pid->int_min;
        if (pid->integrator > pid->int_max)
        	pid->integrator = pid->int_max;
    }
    i = pid->Ki * pid->integrator;
    d = 0.0f;
    if (pid->prev_error != FLT_MAX)
        d = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    output = p + i + d;
    if (output < pid->out_min)
    	output = pid->out_min;
    if (output > pid->out_max)
    	output = pid->out_max;
    return (output);
}
