/*
 * helpers.c
 *
 *  Created on: Jul 8, 2025
 *      Author: yzeybek
 */

# include "helpers.h"

void quat_mult(const float q1[4], const float q2[4], float out[4])
{
    const float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    const float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

    out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // w
    out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // x
    out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // y
    out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // z
}

void	integrate_quat(const float in_q[4], const float omega[3], float dt, float out_q[4])
{
    const float	omega_norm = sqrtf(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
	float		tmp[4];
	float		dq[4];
    float		angle;
	float		ax;
	float		ay;
	float		az;
	float		sh;
	float		ch;

    if (omega_norm < 1e-9f)
    {
    	arm_quaternion_normalize_f32(in_q, out_q, 1);
        return ;
    }
    angle = omega_norm * dt;
    ax = omega[0] / omega_norm;
    ay = omega[1] / omega_norm;
    az = omega[2] / omega_norm;
    sh = sinf(angle * 0.5f);
    ch = cosf(angle * 0.5f);
    dq[0] = ch;
    dq[1] = ax * sh;
    dq[2] = ay * sh;
    dq[3] = az * sh;
    quat_mult(in_q, dq, tmp);
    arm_quaternion_normalize_f32(tmp, out_q, 1);
}

void	arm_matrix_identity_f32(arm_matrix_instance_f32* p_mat, int size)
{
    memset(p_mat->pData, 0, size * size * sizeof(float));
    for (int i = 0; i < size; i++)
    {
        p_mat->pData[i * size + i] = 1.0f;
    }
}

void	quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
	float	sinr_cosp;
	float	cosr_cosp;
	float	cosy_cosp;
	float	siny_cosp;
	float	sinp;
    float	qw;
    float	qx;
    float	qy;
    float	qz;

    qw = q[0];
    qx = q[1];
    qy = q[2];
    qz = q[3];
    sinr_cosp = 2.0f * (qw * qx + qy * qz);
    cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f)
    	sinp = 1.0f;
    else if (sinp < -1.0f)
    	sinp = -1.0f;
    *pitch = asinf(sinp);
    siny_cosp = 2.0f * (qw * qz + qx * qy);
    cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

float	wrap_pi(float value)
{
	if (value > M_PI)
		value -= 2.0f * M_PI;
	else if (value < -M_PI)
		value += 2.0f * M_PI;
	return (value);
}

float	wrap_degree(float value)
{
	float w = fmodf(value + 180.0f, 360.0f);
	if (w < 0.0f)
		w += 360.0f;
	return (w - 180.0f);
}

float	degree_wrap(float value)
{
	if (value < 0.0f)
		return (value + 360.0f);
	else
		return (value);
}

