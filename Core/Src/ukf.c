/*
 * ukf.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sara.h"

uint16_t	ukf_init(t_ukf *ukf, float ukf_matrix_x[UKF_SIZE_STATE], float ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE], float ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE], float ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS], t_ukf_f ukf_f, t_ukf_h ukf_h)
{
	float	common;

	if (!ukf || !ukf_matrix_x || !ukf_matrix_p || !ukf_matrix_q || !ukf_matrix_r || !ukf_f || !ukf_h)
		return (UKF_STAT_ERR_SELF_NULL_PTR);
	memcpy(ukf->ukf_matrix_x, ukf_matrix_x, sizeof(float) * UKF_SIZE_STATE);
	memcpy(ukf->ukf_matrix_p, ukf_matrix_p, sizeof(float) * UKF_SIZE_STATE * UKF_SIZE_STATE);
	memcpy(ukf->ukf_matrix_q, ukf_matrix_q, sizeof(float) * UKF_SIZE_STATE * UKF_SIZE_STATE);
	memcpy(ukf->ukf_matrix_r, ukf_matrix_r, sizeof(float) * UKF_SIZE_MEAS * UKF_SIZE_MEAS);
	ukf->ukf_f = ukf_f;
	ukf->ukf_h = ukf_h;
	common = 1.0f / (2.0f * (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA));
	for (int i = 0; i < UKF_SIZE_SIGMA; i++)
	{
		ukf->ukf_weight_m[i] = common;
		ukf->ukf_weight_c[i] = common;
	}
	ukf->ukf_weight_m[0] = UKF_SIGMA_LAMBDA / (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA);
	ukf->ukf_weight_c[0] = UKF_SIGMA_LAMBDA / (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA) + (1 - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
	return (UKF_STAT_OK);
}

uint16_t	ukf_update(t_ukf *ukf, float ukf_matrix_u[UKF_SIZE_INPUT], float dt)
{
	static float			sigmas_k_plus_1_predicted_data[UKF_SIZE_STATE * UKF_SIZE_SIGMA];
	static float			z_sigmas_predicted_data[UKF_SIZE_MEAS * UKF_SIZE_SIGMA];
	static float			temp_nxn_mat_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
	static float			p_predicted_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
	static float			temp_nxm_mat_data[UKF_SIZE_STATE * UKF_SIZE_MEAS];
	static float			sigmas_k_data[UKF_SIZE_STATE * UKF_SIZE_SIGMA];
	static float			p_symm_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
	static float			p_sqrt_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
	static float			k_gain_data[UKF_SIZE_STATE * UKF_SIZE_MEAS];
	static float			p_z_inv_data[UKF_SIZE_MEAS * UKF_SIZE_MEAS];
	static float			eye_n_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
	static float			eye_m_data[UKF_SIZE_MEAS * UKF_SIZE_MEAS];
	static float			p_xz_data[UKF_SIZE_STATE * UKF_SIZE_MEAS];
	static float			p_z_data[UKF_SIZE_MEAS * UKF_SIZE_MEAS];
	static float			temp_state_vec_data[UKF_SIZE_STATE];
	static float			z_m_predicted_data[UKF_SIZE_MEAS];
	static float			temp_meas_vec_data[UKF_SIZE_MEAS];
	static float			x_predicted_data[UKF_SIZE_STATE];
	static float			innovation_data[UKF_SIZE_MEAS];
	float					*p;
	float					*q;
	float					*r;
	float					*x;
	arm_matrix_instance_f32	P_mat;
	arm_matrix_instance_f32	Q_mat;
	arm_matrix_instance_f32	R_mat;
	arm_matrix_instance_f32	P_symm;
	arm_matrix_instance_f32	P_sqrt;
	arm_matrix_instance_f32	Sigmas_k;
	arm_matrix_instance_f32	Sigmas_k_plus_1;
	arm_matrix_instance_f32	X_predicted;
	arm_matrix_instance_f32	P_predicted;
	arm_matrix_instance_f32	Z_sigmas;
	arm_matrix_instance_f32	Z_m_predicted;
	arm_matrix_instance_f32	Pz;
	arm_matrix_instance_f32	Pz_inv;
	arm_matrix_instance_f32	Pxz;
	arm_matrix_instance_f32	K_gain;
	arm_matrix_instance_f32	W_m;
	arm_matrix_instance_f32	temp_nxn;
	arm_matrix_instance_f32	temp_nxm;
	arm_matrix_instance_f32	eye_n;
	arm_matrix_instance_f32	eye_m;
    arm_status				stat;

    if (!ukf || !ukf_matrix_u)
    	return (UKF_STAT_ERR_SELF_NULL_PTR);
    p = &ukf->ukf_matrix_p[0][0];
    q = &ukf->ukf_matrix_q[0][0];
    r = &ukf->ukf_matrix_r[0][0];
    x = ukf->ukf_matrix_x;
    arm_mat_init_f32(&P_mat, UKF_SIZE_STATE, UKF_SIZE_STATE, p);
    arm_mat_init_f32(&Q_mat, UKF_SIZE_STATE, UKF_SIZE_STATE, q);
    arm_mat_init_f32(&R_mat, UKF_SIZE_MEAS,  UKF_SIZE_MEAS, r);
    arm_mat_init_f32(&P_symm, UKF_SIZE_STATE, UKF_SIZE_STATE, p_symm_data);
    arm_mat_init_f32(&P_sqrt, UKF_SIZE_STATE, UKF_SIZE_STATE, p_sqrt_data);
    arm_mat_init_f32(&Sigmas_k, UKF_SIZE_STATE, UKF_SIZE_SIGMA, sigmas_k_data);
    arm_mat_init_f32(&Sigmas_k_plus_1, UKF_SIZE_STATE, UKF_SIZE_SIGMA, sigmas_k_plus_1_predicted_data);
    arm_mat_init_f32(&X_predicted, UKF_SIZE_STATE, 1, x_predicted_data);
    arm_mat_init_f32(&P_predicted, UKF_SIZE_STATE, UKF_SIZE_STATE, p_predicted_data);
    arm_mat_init_f32(&Z_sigmas, UKF_SIZE_MEAS,  UKF_SIZE_SIGMA, z_sigmas_predicted_data);
    arm_mat_init_f32(&Z_m_predicted, UKF_SIZE_MEAS, 1, z_m_predicted_data);
    arm_mat_init_f32(&Pz, UKF_SIZE_MEAS, UKF_SIZE_MEAS, p_z_data);
    arm_mat_init_f32(&Pz_inv, UKF_SIZE_MEAS, UKF_SIZE_MEAS, p_z_inv_data);
    arm_mat_init_f32(&Pxz, UKF_SIZE_STATE, UKF_SIZE_MEAS, p_xz_data);
    arm_mat_init_f32(&K_gain, UKF_SIZE_STATE, UKF_SIZE_MEAS, k_gain_data);
    arm_mat_init_f32(&W_m, UKF_SIZE_SIGMA, 1, ukf->ukf_weight_m);
    arm_mat_init_f32(&temp_nxn, UKF_SIZE_STATE, UKF_SIZE_STATE, temp_nxn_mat_data);
    arm_mat_init_f32(&temp_nxm, UKF_SIZE_STATE, UKF_SIZE_MEAS, temp_nxm_mat_data);
    arm_mat_init_f32(&eye_n, UKF_SIZE_STATE, UKF_SIZE_STATE, eye_n_data);
    arm_mat_init_f32(&eye_m, UKF_SIZE_MEAS, UKF_SIZE_MEAS, eye_m_data);
    stat = arm_mat_trans_f32(&P_mat, &temp_nxn);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_TRANS_1, stat));
    stat = arm_mat_add_f32(&P_mat, &temp_nxn, &P_symm);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_1, stat));
    stat = arm_mat_scale_f32(&P_symm, 0.5f, &P_symm);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_1, stat));
    stat = arm_mat_scale_f32(&P_symm, UKF_SIGMA_LAMBDA + UKF_SIZE_STATE, &P_sqrt);
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_2, stat));
    stat = arm_mat_cholesky_f32(&P_sqrt, &P_sqrt);
    if (stat != ARM_MATH_SUCCESS)
    {
        matrix_identity(&eye_n, UKF_SIZE_STATE);
        stat = arm_mat_scale_f32(&eye_n, 1e-9f, &eye_n);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_3, stat));
        arm_mat_scale_f32(&P_symm, UKF_SIGMA_LAMBDA + UKF_SIZE_STATE, &P_sqrt);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_4, stat));
        arm_mat_add_f32(&P_sqrt, &eye_n, &P_sqrt);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_2, stat));
        stat = arm_mat_cholesky_f32(&P_sqrt, &P_sqrt);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_CHOLESKY, stat));
    }
    memcpy(sigmas_k_data, x, UKF_SIZE_STATE * sizeof(float));
    for (int i = 0; i < UKF_SIZE_STATE; i++)
    {
        arm_add_f32(x, &p_sqrt_data[i * UKF_SIZE_STATE], &sigmas_k_data[(i + 1) * UKF_SIZE_STATE], UKF_SIZE_STATE);
        arm_sub_f32(x, &p_sqrt_data[i * UKF_SIZE_STATE], &sigmas_k_data[(UKF_SIZE_STATE + i + 1) * UKF_SIZE_STATE], UKF_SIZE_STATE);
    }
    for (int i = 0; i < UKF_SIZE_SIGMA; i++)
    {
        float* current_sigma_in = &sigmas_k_data[i * UKF_SIZE_STATE];
        float* current_sigma_out = &sigmas_k_plus_1_predicted_data[i * UKF_SIZE_STATE];
        ukf->ukf_f(current_sigma_in, ukf_matrix_u, current_sigma_out, dt);
        arm_quaternion_normalize_f32(&current_sigma_out[9], &current_sigma_out[9], 1);
    }
    stat = arm_mat_mult_f32(&Sigmas_k_plus_1, &W_m, &X_predicted);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_1, stat));
    arm_quaternion_normalize_f32(&x_predicted_data[9], &x_predicted_data[9], 1);
    memset(p_predicted_data, 0, UKF_SIZE_STATE * UKF_SIZE_STATE * sizeof(float));
    for (int i = 0; i < UKF_SIZE_SIGMA; i++)
    {
        arm_sub_f32(&sigmas_k_plus_1_predicted_data[i * UKF_SIZE_STATE], x_predicted_data, temp_state_vec_data, UKF_SIZE_STATE);
        arm_matrix_instance_f32 diff_col;
        arm_mat_init_f32(&diff_col, UKF_SIZE_STATE, 1, temp_state_vec_data);
        arm_matrix_instance_f32 diff_row;
        arm_mat_init_f32(&diff_row, 1, UKF_SIZE_STATE, temp_state_vec_data);
        stat = arm_mat_mult_f32(&diff_col, &diff_row, &temp_nxn);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_2, stat));
        stat = arm_mat_scale_f32(&temp_nxn, ukf->ukf_weight_c[i], &temp_nxn);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_5, stat));
        stat = arm_mat_add_f32(&P_predicted, &temp_nxn, &P_predicted);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_3, stat));
    }
    stat = arm_mat_scale_f32(&Q_mat, dt, &temp_nxn);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_6, stat));
    stat = arm_mat_add_f32(&P_predicted, &temp_nxn, &P_predicted);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_4, stat));
    stat = arm_mat_trans_f32(&P_predicted, &temp_nxn);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_TRANS_2, stat));
    stat = arm_mat_add_f32(&P_predicted, &temp_nxn, &P_predicted);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_5, stat));
    stat = arm_mat_scale_f32(&P_predicted, 0.5f, &P_predicted);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_7, stat));
    for (int i = 0; i < UKF_SIZE_SIGMA; i++)
    {
        float *current_sigma_in = &sigmas_k_plus_1_predicted_data[i * UKF_SIZE_STATE];
        float *current_z_out = &z_sigmas_predicted_data[i * UKF_SIZE_MEAS];
        ukf->ukf_h(current_sigma_in, current_z_out);
    }
    stat = arm_mat_mult_f32(&Z_sigmas, &W_m, &Z_m_predicted);
    if (stat != ARM_MATH_SUCCESS)
       return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_3, stat));
    memset(p_z_data, 0, UKF_SIZE_MEAS * UKF_SIZE_MEAS * sizeof(float));
    memset(p_xz_data, 0, UKF_SIZE_STATE * UKF_SIZE_MEAS * sizeof(float));
    for (int i = 0; i < UKF_SIZE_SIGMA; i++)
    {
        arm_sub_f32(&sigmas_k_plus_1_predicted_data[i * UKF_SIZE_STATE], x_predicted_data, temp_state_vec_data, UKF_SIZE_STATE);
        arm_sub_f32(&z_sigmas_predicted_data[i * UKF_SIZE_MEAS], z_m_predicted_data, temp_meas_vec_data, UKF_SIZE_MEAS);
        arm_matrix_instance_f32 diff_z_col;
        arm_mat_init_f32(&diff_z_col, UKF_SIZE_MEAS, 1, temp_meas_vec_data);
        arm_matrix_instance_f32 diff_z_row;
        arm_mat_init_f32(&diff_z_row, 1, UKF_SIZE_MEAS, temp_meas_vec_data);
        arm_matrix_instance_f32 Pz_term;
        arm_mat_init_f32(&Pz_term, UKF_SIZE_MEAS, UKF_SIZE_MEAS, temp_nxn_mat_data);
        arm_mat_mult_f32(&diff_z_col, &diff_z_row, &Pz_term);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_4, stat));
        stat = arm_mat_scale_f32(&Pz_term, ukf->ukf_weight_c[i], &Pz_term);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_8, stat));
        stat = arm_mat_add_f32(&Pz, &Pz_term, &Pz);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_6, stat));
        arm_matrix_instance_f32 diff_x_col;
        arm_mat_init_f32(&diff_x_col, UKF_SIZE_STATE, 1, temp_state_vec_data);
        arm_matrix_instance_f32 Pxz_term;
        arm_mat_init_f32(&Pxz_term, UKF_SIZE_STATE, UKF_SIZE_MEAS, temp_nxm_mat_data);
        stat = arm_mat_mult_f32(&diff_x_col, &diff_z_row, &Pxz_term);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_5, stat));
        stat = arm_mat_scale_f32(&Pxz_term, ukf->ukf_weight_c[i], &Pxz_term);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_9, stat));
        stat = arm_mat_add_f32(&Pxz, &Pxz_term, &Pxz);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_7, stat));
    }
    stat = arm_mat_add_f32(&Pz, &R_mat, &Pz);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_8, stat));
    stat = arm_mat_trans_f32(&Pz, &temp_nxm);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_6, stat));
    stat = arm_mat_add_f32(&Pz, &temp_nxm, &Pz);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_9, stat));
    stat = arm_mat_scale_f32(&Pz, 0.5f, &Pz);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_10, stat));
    stat = arm_mat_inverse_f32(&Pz, &Pz_inv);
    if (stat != ARM_MATH_SUCCESS)
    {
        matrix_identity(&eye_m, UKF_SIZE_MEAS);
        stat = arm_mat_scale_f32(&eye_m, 1e-9f, &eye_m);
        if (stat != ARM_MATH_SUCCESS)
        	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_11, stat));
        stat = arm_mat_add_f32(&Pz, &eye_m, &Pz);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_10, stat));
        stat = arm_mat_inverse_f32(&Pz, &Pz_inv);
        if (stat != ARM_MATH_SUCCESS)
            return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_INVERSE, stat));
    }
    stat = arm_mat_mult_f32(&Pxz, &Pz_inv, &K_gain);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_7, stat));
    float* z_meas = &ukf_matrix_u[6];
    arm_sub_f32(z_meas, z_m_predicted_data, innovation_data, UKF_SIZE_MEAS);
    arm_matrix_instance_f32 innov_vec;
    arm_mat_init_f32(&innov_vec, UKF_SIZE_MEAS, 1, innovation_data);
    arm_matrix_instance_f32 K_innov;
    arm_mat_init_f32(&K_innov, UKF_SIZE_STATE, 1, temp_state_vec_data);
    stat = arm_mat_mult_f32(&K_gain, &innov_vec, &K_innov);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_8, stat));
    arm_add_f32(x_predicted_data, temp_state_vec_data, x, UKF_SIZE_STATE);
    arm_quaternion_normalize_f32(&x[3], &x[3], 1);
    arm_matrix_instance_f32 K_gain_T;
    stat = arm_mat_trans_f32(&K_gain, &temp_nxm);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_TRANS_3, stat));
    arm_mat_init_f32(&K_gain_T, UKF_SIZE_MEAS, UKF_SIZE_STATE, temp_nxm_mat_data);
    stat = arm_mat_mult_f32(&K_gain, &Pz, &Pxz);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_9, stat));
    stat = arm_mat_mult_f32(&Pxz, &K_gain_T, &temp_nxn);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_MULT_10, stat));
    stat = arm_mat_sub_f32(&P_predicted, &temp_nxn, &P_mat);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SUB, stat));
    stat = arm_mat_trans_f32(&P_mat, &temp_nxn);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_TRANS_4, stat));
    stat = arm_mat_add_f32(&P_mat, &temp_nxn, &P_mat);
    if (stat != ARM_MATH_SUCCESS)
        return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_ADD_11, stat));
    stat = arm_mat_scale_f32(&P_mat, 0.5f, &P_mat);
    if (stat != ARM_MATH_SUCCESS)
    	return (UKF_MAKE_STAT(UKF_STAT_ERR_ARM_SCALE_12, stat));
    return (UKF_STAT_OK);
}
