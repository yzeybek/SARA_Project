/*
 * ukf.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_UKF_H_
#define INC_UKF_H_

#ifndef UKF_SIZE_STATE
# define UKF_SIZE_STATE 22
#endif

#ifndef UKF_SIZE_MEAS
# define UKF_SIZE_MEAS 3
#endif

#ifndef UKF_SIZE_INPUT
# define UKF_SIZE_INPUT 9
#endif

# define UKF_SIZE_SIGMA (2 * UKF_STATE_SIZE + 1)
# define UKF_ALPHA 0.1f
# define UKF_KAPPA 0.0f
# define UKF_BETA 2.0f
# define UKF_SIGMA_LAMBDA (UKF_ALPHA * UKF_ALPHA * (UKF_STATE_SIZE + UKF_KAPPA) - UKF_STATE_SIZE);

typedef void (*t_ukf_f)(float[UKF_SIZE_STATE], float[UKF_SIZE_INPUT], float*[UKF_SIZE_STATE]);
typedef void (*t_ukf_h)(float[UKF_SIZE_STATE], float*[UKF_SIZE_MEAS]);

typedef struct s_ukf
{
	float	ukf_matrix_x[UKF_SIZE_STATE];
	float	ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE];
	float	ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE];
	float	ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS];
	float	ukf_weight_m[UKF_SIZE_SIGMA];
	float	ukf_weight_c[UKF_SIZE_SIGMA];
	t_ukf_f	ukf_f;
	t_ukf_h	ukf_h;

} t_ukf;

void	ukf_init(t_ukf *ukf, float *ukf_matrix_x[UKF_SIZE_STATE], float *ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE], float *ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE], float *ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS], t_ukf_f ukf_f, t_ukf_h ukf_h);

#endif /* INC_UKF_H_ */
