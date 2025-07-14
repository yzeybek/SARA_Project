/*
 * ukf.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_UKF_H_
#define INC_UKF_H_

# include "helpers.h"

#ifndef UKF_SIZE_STATE
# define UKF_SIZE_STATE 22
#endif

#ifndef UKF_SIZE_MEAS
# define UKF_SIZE_MEAS 3
#endif

#ifndef UKF_SIZE_INPUT
# define UKF_SIZE_INPUT 9
#endif

# define UKF_SIZE_SIGMA (2 * UKF_SIZE_STATE + 1)
# define UKF_ALPHA 0.1f
# define UKF_KAPPA 0.0f
# define UKF_BETA 2.0f
# define UKF_SIGMA_LAMBDA (UKF_ALPHA * UKF_ALPHA * (UKF_SIZE_STATE + UKF_KAPPA) - UKF_SIZE_STATE)

# define UKF_MAKE_STAT(base_stat, sub_stat) \
    (uint16_t)( \
       ( ((uint16_t)((sub_stat) & 0xFF)) << 8 ) \
     |   ((uint16_t)((base_stat) & 0xFF)) )

typedef void (*t_ukf_f)(float[UKF_SIZE_STATE], float[UKF_SIZE_INPUT], float[UKF_SIZE_STATE], float dt);
typedef void (*t_ukf_h)(float[UKF_SIZE_STATE], float[UKF_SIZE_MEAS]);

typedef enum e_ukf_stat
{
	UKF_STAT_OK,
	UKF_STAT_ERR_SELF_NULL_PTR,
	UKF_STAT_ERR_ARM_SCALE_1,
	UKF_STAT_ERR_ARM_SCALE_2,
	UKF_STAT_ERR_ARM_SCALE_3,
	UKF_STAT_ERR_ARM_SCALE_4,
	UKF_STAT_ERR_ARM_SCALE_5,
	UKF_STAT_ERR_ARM_SCALE_6,
	UKF_STAT_ERR_ARM_SCALE_7,
	UKF_STAT_ERR_ARM_SCALE_8,
	UKF_STAT_ERR_ARM_SCALE_9,
	UKF_STAT_ERR_ARM_SCALE_10,
	UKF_STAT_ERR_ARM_SCALE_11,
	UKF_STAT_ERR_ARM_SCALE_12,
	UKF_STAT_ERR_ARM_ADD_1,
	UKF_STAT_ERR_ARM_ADD_2,
	UKF_STAT_ERR_ARM_ADD_3,
	UKF_STAT_ERR_ARM_ADD_4,
	UKF_STAT_ERR_ARM_ADD_5,
	UKF_STAT_ERR_ARM_ADD_6,
	UKF_STAT_ERR_ARM_ADD_7,
	UKF_STAT_ERR_ARM_ADD_8,
	UKF_STAT_ERR_ARM_ADD_9,
	UKF_STAT_ERR_ARM_ADD_10,
	UKF_STAT_ERR_ARM_ADD_11,
	UKF_STAT_ERR_ARM_MULT_1,
	UKF_STAT_ERR_ARM_MULT_2,
	UKF_STAT_ERR_ARM_MULT_3,
	UKF_STAT_ERR_ARM_MULT_4,
	UKF_STAT_ERR_ARM_MULT_5,
	UKF_STAT_ERR_ARM_MULT_6,
	UKF_STAT_ERR_ARM_MULT_7,
	UKF_STAT_ERR_ARM_MULT_8,
	UKF_STAT_ERR_ARM_MULT_9,
	UKF_STAT_ERR_ARM_MULT_10,
	UKF_STAT_ERR_ARM_TRANS_1,
	UKF_STAT_ERR_ARM_TRANS_2,
	UKF_STAT_ERR_ARM_TRANS_3,
	UKF_STAT_ERR_ARM_TRANS_4,
	UKF_STAT_ERR_ARM_CHOLESKY,
	UKF_STAT_ERR_ARM_INVERSE,
	UKF_STAT_ERR_ARM_SUB,

} t_ukf_stat;

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

uint16_t	ukf_init(t_ukf *ukf, float ukf_matrix_x[UKF_SIZE_STATE], float ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE], float ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE], float ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS], t_ukf_f ukf_f, t_ukf_h ukf_h);
uint16_t	ukf_update(t_ukf *ukf, float ukf_matrix_u[UKF_SIZE_INPUT], float dt);

#endif /* INC_UKF_H_ */
