/*
 * pid.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "pid.h"

void pid_init(t_pid *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float int_min, float int_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->int_min = int_min;
    pid->int_max = int_max;
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
