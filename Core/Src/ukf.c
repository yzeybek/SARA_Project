/*
 * ukf.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sara.h"

void	ukf_init(t_ukf *ukf, float *ukf_matrix_x[UKF_SIZE_STATE], float *ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE], float *ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE], float *ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS], t_ukf_f ukf_f, t_ukf_h ukf_h)
{
	ukf->ukf_matrix_x = *ukf_matrix_x;
	ukf->ukf_matrix_p = *ukf_matrix_p;
	ukf->ukf_matrix_q = *ukf_matrix_q;
	ukf->ukf_matrix_r = *ukf_matrix_r;
	ukf->ukf_f = ukf_f;
	ukf->ukf_h = ukf_h;

	float common = 1.0f / (2.0f * (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA));
	for (int i = 0; i < count; i++)
	{
		ukf->ukf_weight_m[i] = common;
		ukf->ukf_weight_c[i] = common;
	}
	ukf->ukf_weight_m[0] = UKF_SIGMA_LAMBDA / (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA);
	ukf->ukf_weight_c[0] = UKF_SIGMA_LAMBDA / (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA) + (1 - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
}
