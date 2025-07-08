/*
 * helpers.c
 *
 *  Created on: Jul 8, 2025
 *      Author: yzeybek
 */

void	quat_to_rotm(const float in_q[4], float R[3][3])
{
    float q[4];
    arm_quaternion_normalize_f32(in_q, q);
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float b2w[3][3] =
    {
        {1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)},
        {2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)},
        {2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)}
    };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = b2w[j][i];
}

void	rotm_to_quat(const float R[3][3], float out_q[4])
{
    float M[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M[i][j] = R[j][i];
    float t = M[0][0] + M[1][1] + M[2][2];
    if (t > 0.0f)
    {
        float S = sqrtf(t + 1.0f) * 2.0f;
        out_q[0] = 0.25f * S;
        out_q[1] = (M[2][1] - M[1][2]) / S;
        out_q[2] = (M[0][2] - M[2][0]) / S;
        out_q[3] = (M[1][0] - M[0][1]) / S;
    }
    else if ((M[0][0] > M[1][1]) && (M[0][0] > M[2][2]))
    {
        float S = sqrtf(1.0f + M[0][0] - M[1][1] - M[2][2]) * 2.0f;
        out_q[0] = (M[2][1] - M[1][2]) / S;
        out_q[1] = 0.25f * S;
        out_q[2] = (M[0][1] + M[1][0]) / S;
        out_q[3] = (M[0][2] + M[2][0]) / S;
    }
    else if (M[1][1] > M[2][2])
    {
        float S = sqrtf(1.0f + M[1][1] - M[0][0] - M[2][2]) * 2.0f;
        out_q[0] = (M[0][2] - M[2][0]) / S;
        out_q[1] = (M[0][1] + M[1][0]) / S;
        out_q[2] = 0.25f * S;
        out_q[3] = (M[1][2] + M[2][1]) / S;
    }
    else
    {
        float S = sqrtf(1.0f + M[2][2] - M[0][0] - M[1][1]) * 2.0f;
        out_q[0] = (M[1][0] - M[0][1]) / S;
        out_q[1] = (M[0][2] + M[2][0]) / S;
        out_q[2] = (M[1][2] + M[2][1]) / S;
        out_q[3] = 0.25f * S;
    }
    float tmp[4];
    arm_quaternion_normalize_f32(out_q, tmp);
    memcpy(out_q, tmp, 4 * sizeof(float));
}

void	integrate_quat(const float in_q[4], const float omega[3], float dt, float out_q[4])
{
    float omega_norm = sqrtf(omega[0]*omega[0] +
                              omega[1]*omega[1] +
                              omega[2]*omega[2]);
    if (omega_norm < 1e-9f)
    {
    	arm_quaternion_normalize_f32(in_q, out_q);
        return;
    }
    float angle = omega_norm * dt;
    float ax = omega[0] / omega_norm;
    float ay = omega[1] / omega_norm;
    float az = omega[2] / omega_norm;
    float sh = sinf(angle * 0.5f);
    float ch = cosf(angle * 0.5f);
    float dq[4] = { ch, ax*sh, ay*sh, az*sh };
    float tmp[4];
    arm_quaternion_mult_f32((float32_t*)in_q,
                            (float32_t*)dq,
                            (float32_t*)tmp);
    arm_quaternion_normalize_f32(tmp, (float32_t*)out_q);
}
