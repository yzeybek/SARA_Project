/*
 * pid.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "pid.h"

void pid_init(t_pid *pid, float pid_values[7])
{
    pid->Kp = values[0];
    pid->Ki = values[1];
    pid->Kd = values[2];
    pid->out_min = values[3];
    pid->out_max = values[4];
    pid->int_min = values[5];
    pid->int_max = values[6];
    pid->integrator = 0.0f;
    pid->prev_error = FLT_MAX;
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
