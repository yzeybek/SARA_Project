/*
 * ukf.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sara.h"

void	ukf_init(t_ukf *ukf, float ukf_matrix_x[UKF_SIZE_STATE], float ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE], float ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE], float ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS], t_ukf_f ukf_f, t_ukf_h ukf_h)
{
	memcpy(ukf->ukf_matrix_x, ukf_matrix_x, sizeof(float) * UKF_SIZE_STATE);
	memcpy(ukf->ukf_matrix_p, ukf_matrix_p, sizeof(float) * UKF_SIZE_STATE * UKF_SIZE_STATE);
	memcpy(ukf->ukf_matrix_q, ukf_matrix_q, sizeof(float) * UKF_SIZE_STATE * UKF_SIZE_STATE);
	memcpy(ukf->ukf_matrix_r, ukf_matrix_r, sizeof(float) * UKF_SIZE_MEAS * UKF_SIZE_MEAS);
	ukf->ukf_f = ukf_f;
	ukf->ukf_h = ukf_h;

	float common = 1.0f / (2.0f * (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA));
	for (int i = 0; i < UKF_SIZE_SIGMA; i++)
	{
		ukf->ukf_weight_m[i] = common;
		ukf->ukf_weight_c[i] = common;
	}
	ukf->ukf_weight_m[0] = UKF_SIGMA_LAMBDA / (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA);
	ukf->ukf_weight_c[0] = UKF_SIGMA_LAMBDA / (UKF_SIZE_STATE + UKF_SIGMA_LAMBDA) + (1 - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
}

void ukf_update(t_ukf *ukf, float ukf_matrix_u[UKF_SIZE_INPUT], float dt)
{
    const int				n_state = UKF_SIZE_STATE;
    const int				n_meas = UKF_SIZE_MEAS;
    const int				n_sigma = UKF_SIZE_SIGMA;
    arm_status				status;
    float					scale_factor;
    float					*x = ukf->ukf_matrix_x;
    float					*p = &ukf->ukf_matrix_p[0][0];
    float					*q = &ukf->ukf_matrix_q[0][0];
    float					*r = &ukf->ukf_matrix_r[0][0];
    static float			p_symm_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
    static float			p_sqrt_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
    static float			sigmas_k_data[UKF_SIZE_STATE * UKF_SIZE_SIGMA];
    static float			sigmas_k_plus_1_predicted_data[UKF_SIZE_STATE * UKF_SIZE_SIGMA];
    static float			x_predicted_data[UKF_SIZE_STATE];
    static float			p_predicted_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
    static float			z_sigmas_predicted_data[UKF_SIZE_MEAS * UKF_SIZE_SIGMA];
    static float			z_m_predicted_data[UKF_SIZE_MEAS];
    static float			p_z_data[UKF_SIZE_MEAS * UKF_SIZE_MEAS];
    static float			p_xz_data[UKF_SIZE_STATE * UKF_SIZE_MEAS];
    static float			p_z_inv_data[UKF_SIZE_MEAS * UKF_SIZE_MEAS];
    static float			k_gain_data[UKF_SIZE_STATE * UKF_SIZE_MEAS];
    static float			innovation_data[UKF_SIZE_MEAS];
    static float			temp_state_vec_data[UKF_SIZE_STATE];
    static float			temp_meas_vec_data[UKF_SIZE_MEAS];
    static float			temp_nxn_mat_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
    static float			temp_nxm_mat_data[UKF_SIZE_STATE * UKF_SIZE_MEAS];
    static float			eye_n_data[UKF_SIZE_STATE * UKF_SIZE_STATE];
    static float			eye_m_data[UKF_SIZE_MEAS * UKF_SIZE_MEAS];
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

    arm_mat_init_f32(&P_mat, n_state, n_state, p);
    arm_mat_init_f32(&Q_mat, n_state, n_state, q);
    arm_mat_init_f32(&R_mat, n_meas,  n_meas, r);
    arm_mat_init_f32(&P_symm, n_state, n_state, p_symm_data);
    arm_mat_init_f32(&P_sqrt, n_state, n_state, p_sqrt_data);
    arm_mat_init_f32(&Sigmas_k, n_state, n_sigma, sigmas_k_data);
    arm_mat_init_f32(&Sigmas_k_plus_1, n_state, n_sigma, sigmas_k_plus_1_predicted_data);
    arm_mat_init_f32(&X_predicted, n_state, 1, x_predicted_data);
    arm_mat_init_f32(&P_predicted, n_state, n_state, p_predicted_data);
    arm_mat_init_f32(&Z_sigmas, n_meas,  n_sigma, z_sigmas_predicted_data);
    arm_mat_init_f32(&Z_m_predicted, n_meas, 1, z_m_predicted_data);
    arm_mat_init_f32(&Pz, n_meas, n_meas, p_z_data);
    arm_mat_init_f32(&Pz_inv, n_meas, n_meas, p_z_inv_data);
    arm_mat_init_f32(&Pxz, n_state, n_meas, p_xz_data);
    arm_mat_init_f32(&K_gain, n_state, n_meas, k_gain_data);
    arm_mat_init_f32(&W_m, n_sigma, 1, ukf->ukf_weight_m);
    arm_mat_init_f32(&temp_nxn, n_state, n_state, temp_nxn_mat_data);
    arm_mat_init_f32(&temp_nxm, n_state, n_meas, temp_nxm_mat_data);
    arm_mat_init_f32(&eye_n, n_state, n_state, eye_n_data);
    arm_mat_init_f32(&eye_m, n_meas, n_meas, eye_m_data);
    arm_mat_trans_f32(&P_mat, &temp_nxn);
    arm_mat_add_f32(&P_mat, &temp_nxn, &P_symm);
    arm_mat_scale_f32(&P_symm, 0.5f, &P_symm);
    scale_factor = UKF_SIGMA_LAMBDA + n_state;
    arm_mat_scale_f32(&P_symm, scale_factor, &P_sqrt);
    status = arm_mat_cholesky_f32(&P_sqrt, &P_sqrt);
    if (status != ARM_MATH_SUCCESS)
    {
        arm_matrix_identity_f32(&eye_n, n_state);
        arm_mat_scale_f32(&eye_n, 1e-9f, &eye_n);
        arm_mat_scale_f32(&P_symm, scale_factor, &P_sqrt);
        arm_mat_add_f32(&P_sqrt, &eye_n, &P_sqrt);
        status = arm_mat_cholesky_f32(&P_sqrt, &P_sqrt);
        if (status != ARM_MATH_SUCCESS)
            return;
    }
    memcpy(sigmas_k_data, x, n_state * sizeof(float));
    for (int i = 0; i < n_state; i++)
    {
        arm_add_f32(x, &p_sqrt_data[i * n_state], &sigmas_k_data[(i + 1) * n_state], n_state);
        arm_sub_f32(x, &p_sqrt_data[i * n_state], &sigmas_k_data[(n_state + i + 1) * n_state], n_state);
    }
    for (int i = 0; i < n_sigma; i++)
    {
        float* current_sigma_in = &sigmas_k_data[i * n_state];
        float* current_sigma_out = &sigmas_k_plus_1_predicted_data[i * n_state];
        ukf->ukf_f(current_sigma_in, ukf_matrix_u, current_sigma_out, dt);
        arm_quaternion_normalize_f32(&current_sigma_out[9], &current_sigma_out[9], 1);
    }
    arm_mat_mult_f32(&Sigmas_k_plus_1, &W_m, &X_predicted);
    arm_quaternion_normalize_f32(&x_predicted_data[9], &x_predicted_data[9], 1);
    memset(p_predicted_data, 0, n_state * n_state * sizeof(float));
    for (int i = 0; i < n_sigma; i++)
    {
        arm_sub_f32(&sigmas_k_plus_1_predicted_data[i * n_state], x_predicted_data, temp_state_vec_data, n_state);
        arm_matrix_instance_f32 diff_col; arm_mat_init_f32(&diff_col, n_state, 1, temp_state_vec_data);
        arm_matrix_instance_f32 diff_row; arm_mat_init_f32(&diff_row, 1, n_state, temp_state_vec_data);
        arm_mat_mult_f32(&diff_col, &diff_row, &temp_nxn);
        arm_mat_scale_f32(&temp_nxn, ukf->ukf_weight_c[i], &temp_nxn);
        arm_mat_add_f32(&P_predicted, &temp_nxn, &P_predicted);
    }
    arm_mat_scale_f32(&Q_mat, dt, &temp_nxn);
    arm_mat_add_f32(&P_predicted, &temp_nxn, &P_predicted);
    arm_mat_trans_f32(&P_predicted, &temp_nxn);
    arm_mat_add_f32(&P_predicted, &temp_nxn, &P_predicted);
    arm_mat_scale_f32(&P_predicted, 0.5f, &P_predicted);
    for (int i = 0; i < n_sigma; i++)
    {
        float *current_sigma_in = &sigmas_k_plus_1_predicted_data[i * n_state];
        float *current_z_out = &z_sigmas_predicted_data[i * n_meas];
        ukf->ukf_h(current_sigma_in, current_z_out);
    }
    arm_mat_mult_f32(&Z_sigmas, &W_m, &Z_m_predicted);
    memset(p_z_data, 0, n_meas * n_meas * sizeof(float));
    memset(p_xz_data, 0, n_state * n_meas * sizeof(float));
    for (int i = 0; i < n_sigma; i++)
    {
        arm_sub_f32(&sigmas_k_plus_1_predicted_data[i * n_state], x_predicted_data, temp_state_vec_data, n_state);
        arm_sub_f32(&z_sigmas_predicted_data[i * n_meas], z_m_predicted_data, temp_meas_vec_data, n_meas);
        arm_matrix_instance_f32 diff_z_col; arm_mat_init_f32(&diff_z_col, n_meas, 1, temp_meas_vec_data);
        arm_matrix_instance_f32 diff_z_row; arm_mat_init_f32(&diff_z_row, 1, n_meas, temp_meas_vec_data);
        arm_matrix_instance_f32 Pz_term; arm_mat_init_f32(&Pz_term, n_meas, n_meas, temp_nxn_mat_data);
        arm_mat_mult_f32(&diff_z_col, &diff_z_row, &Pz_term);
        arm_mat_scale_f32(&Pz_term, ukf->ukf_weight_c[i], &Pz_term);
        arm_mat_add_f32(&Pz, &Pz_term, &Pz);
        arm_matrix_instance_f32 diff_x_col; arm_mat_init_f32(&diff_x_col, n_state, 1, temp_state_vec_data);
        arm_matrix_instance_f32 Pxz_term; arm_mat_init_f32(&Pxz_term, n_state, n_meas, temp_nxm_mat_data);
        arm_mat_mult_f32(&diff_x_col, &diff_z_row, &Pxz_term);
        arm_mat_scale_f32(&Pxz_term, ukf->ukf_weight_c[i], &Pxz_term);
        arm_mat_add_f32(&Pxz, &Pxz_term, &Pxz);
    }
    arm_mat_add_f32(&Pz, &R_mat, &Pz);
    arm_mat_trans_f32(&Pz, &temp_nxm);
    arm_mat_add_f32(&Pz, &temp_nxm, &Pz);
    arm_mat_scale_f32(&Pz, 0.5f, &Pz);
    status = arm_mat_inverse_f32(&Pz, &Pz_inv);
    if (status != ARM_MATH_SUCCESS)
    {
        arm_matrix_identity_f32(&eye_m, n_meas);
        arm_mat_scale_f32(&eye_m, 1e-9f, &eye_m);
        arm_mat_add_f32(&Pz, &eye_m, &Pz);
        status = arm_mat_inverse_f32(&Pz, &Pz_inv);
        if (status != ARM_MATH_SUCCESS)
            return ;
    }
    arm_mat_mult_f32(&Pxz, &Pz_inv, &K_gain);
    float* z_meas = &ukf_matrix_u[6];
    arm_sub_f32(z_meas, z_m_predicted_data, innovation_data, n_meas);
    arm_matrix_instance_f32 innov_vec; arm_mat_init_f32(&innov_vec, n_meas, 1, innovation_data);
    arm_matrix_instance_f32 K_innov;   arm_mat_init_f32(&K_innov, n_state, 1, temp_state_vec_data);
    arm_mat_mult_f32(&K_gain, &innov_vec, &K_innov);
    arm_add_f32(x_predicted_data, temp_state_vec_data, x, n_state);
    arm_quaternion_normalize_f32(&x[3], &x[3], 1);
    arm_matrix_instance_f32 K_gain_T;
    arm_mat_trans_f32(&K_gain, &temp_nxm);
    arm_mat_init_f32(&K_gain_T, n_meas, n_state, temp_nxm_mat_data);
    arm_mat_mult_f32(&K_gain, &Pz, &Pxz);
    arm_mat_mult_f32(&Pxz, &K_gain_T, &temp_nxn);
    arm_mat_sub_f32(&P_predicted, &temp_nxn, &P_mat);
    arm_mat_trans_f32(&P_mat, &temp_nxn);
    arm_mat_add_f32(&P_mat, &temp_nxn, &P_mat);
    arm_mat_scale_f32(&P_mat, 0.5f, &P_mat);
}
