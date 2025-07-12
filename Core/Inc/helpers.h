/*
 * helpers.h
 *
 *  Created on: Jul 8, 2025
 *      Author: yzeybek
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

# define ARM_MATH_CM4
# include "arm_math.h"
# include "math.h"
# include "string.h"

void	quat_mult(const float q1[4], const float q2[4], float out[4]);
void	integrate_quat(const float in_q[4], const float omega[3], float dt, float out_q[4]);
void	arm_matrix_identity_f32(arm_matrix_instance_f32* p_mat, int size);
void	quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);
float	wrap_pi(float value);
float	wrap_degree(float value);
float	degree_wrap(float value);

#endif /* INC_HELPERS_H_ */
