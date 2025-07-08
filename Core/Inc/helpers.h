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

void	quat_to_rotm(const float in_q[4], float R[3][3]);
void	rotm_to_quat(const float R[3][3], float out_q[4]) ;
void	integrate_quat(const float in_q[4], const float omega[3], float dt, float out_q[4]);

#endif /* INC_HELPERS_H_ */
