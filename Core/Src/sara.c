/*
 * sara.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sara.h"

void	sara_init(t_sara *sara, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim_fin, TIM_HandleTypeDef *htim_ultras)
{
	sara_init_dof(&sara->sara_dof);
	sara_init_state(sara->sara_states);
	sara->sara_state = 0;
	sara_init_ukf(&sara->sara_ukf);
	sara_init_sensor(&sara->sara_sensor, hi2c, hadc);
	sara_init_control(&sara->sara_control, htim_fin, htim_ultras);
}

void	sara_init_dof(t_dof *sara_dof)
{
	float pid_values[7];

	pid_values[0] = PID_HEAVE_POS_KP;
	pid_values[1] = PID_HEAVE_POS_KI;
	pid_values[2] = PID_HEAVE_POS_KD;
	pid_values[3] = PID_HEAVE_POS_OUT_MIN;
	pid_values[4] = PID_HEAVE_POS_OUT_MAX;
	pid_values[5] = PID_HEAVE_POS_INT_MIN;
	pid_values[6] = PID_HEAVE_POS_INT_MAX;
	pid_init(&sara_dof->pid_heave_pos, pid_values);

	pid_values[0] = PID_SWAY_POS_KP;
	pid_values[1] = PID_SWAY_POS_KI;
	pid_values[2] = PID_SWAY_POS_KD;
	pid_values[3] = PID_SWAY_POS_OUT_MIN;
	pid_values[4] = PID_SWAY_POS_OUT_MAX;
	pid_values[5] = PID_SWAY_POS_INT_MIN;
	pid_values[6] = PID_SWAY_POS_INT_MAX;
	pid_init(&sara_dof->pid_sway_pos, pid_values);

	pid_values[0] = PID_SURGE_POS_KP;
	pid_values[1] = PID_SURGE_POS_KI;
	pid_values[2] = PID_SURGE_POS_KD;
	pid_values[3] = PID_SURGE_POS_OUT_MIN;
	pid_values[4] = PID_SURGE_POS_OUT_MAX;
	pid_values[5] = PID_SURGE_POS_INT_MIN;
	pid_values[6] = PID_SURGE_POS_INT_MAX;
	pid_init(&sara_dof->pid_surge_pos, pid_values);

	pid_values[0] = PID_HEAVE_VEL_KP;
	pid_values[1] = PID_HEAVE_VEL_KI;
	pid_values[2] = PID_HEAVE_VEL_KD;
	pid_values[3] = PID_HEAVE_VEL_OUT_MIN;
	pid_values[4] = PID_HEAVE_VEL_OUT_MAX;
	pid_values[5] = PID_HEAVE_VEL_INT_MIN;
	pid_values[6] = PID_HEAVE_VEL_INT_MAX;
	pid_init(&sara_dof->pid_heave_vel, pid_values);

	pid_values[0] = PID_SWAY_VEL_KP;
	pid_values[1] = PID_SWAY_VEL_KI;
	pid_values[2] = PID_SWAY_VEL_KD;
	pid_values[3] = PID_SWAY_VEL_OUT_MIN;
	pid_values[4] = PID_SWAY_VEL_OUT_MAX;
	pid_values[5] = PID_SWAY_VEL_INT_MIN;
	pid_values[6] = PID_SWAY_VEL_INT_MAX;
	pid_init(&sara_dof->pid_sway_vel, pid_values);

	pid_values[0] = PID_SURGE_VEL_KP;
	pid_values[1] = PID_SURGE_VEL_KI;
	pid_values[2] = PID_SURGE_VEL_KD;
	pid_values[3] = PID_SURGE_VEL_OUT_MIN;
	pid_values[4] = PID_SURGE_VEL_OUT_MAX;
	pid_values[5] = PID_SURGE_VEL_INT_MIN;
	pid_values[6] = PID_SURGE_VEL_INT_MAX;
	pid_init(&sara_dof->pid_surge_vel, pid_values);

	pid_values[0] = PID_ROLL_KP;
	pid_values[1] = PID_ROLL_KI;
	pid_values[2] = PID_ROLL_KD;
	pid_values[3] = PID_ROLL_OUT_MIN;
	pid_values[4] = PID_ROLL_OUT_MAX;
	pid_values[5] = PID_ROLL_INT_MIN;
	pid_values[6] = PID_ROLL_INT_MAX;
	pid_init(&sara_dof->pid_roll, pid_values);

	pid_values[0] = PID_PITCH_KP;
	pid_values[1] = PID_PITCH_KI;
	pid_values[2] = PID_PITCH_KD;
	pid_values[3] = PID_PITCH_OUT_MIN;
	pid_values[4] = PID_PITCH_OUT_MAX;
	pid_values[5] = PID_PITCH_INT_MIN;
	pid_values[6] = PID_PITCH_INT_MAX;
	pid_init(&sara_dof->pid_pitch, pid_values);

	pid_values[0] = PID_YAW_KP;
	pid_values[1] = PID_YAW_KI;
	pid_values[2] = PID_YAW_KD;
	pid_values[3] = PID_YAW_OUT_MIN;
	pid_values[4] = PID_YAW_OUT_MAX;
	pid_values[5] = PID_YAW_INT_MIN;
	pid_values[6] = PID_YAW_INT_MAX;
	pid_init(&sara_dof->pid_yaw, pid_values);
}

void	sara_init_state(t_state sara_states[STATE_COUNT])
{
	t_range	state_ranges[9];

	if (STATE_COUNT >= 1)
	{
		state_ranges[0].start = STATE_1_POS_X_START;
		state_ranges[0].end = STATE_1_POS_X_END;
		state_ranges[1].start = STATE_1_POS_Y_START;
		state_ranges[1].end = STATE_1_POS_Y_END;
		state_ranges[2].start = STATE_1_POS_Z_START;
		state_ranges[2].end = STATE_1_POS_Z_END;
		state_ranges[3].start = STATE_1_VEL_X_START;
		state_ranges[3].end = STATE_1_VEL_X_END;
		state_ranges[4].start = STATE_1_VEL_Y_START;
		state_ranges[4].end = STATE_1_VEL_Y_END;
		state_ranges[5].start = STATE_1_VEL_Z_START;
		state_ranges[5].end = STATE_1_VEL_Z_END;
		state_ranges[6].start = STATE_1_EUL_X_START;
		state_ranges[6].end = STATE_1_EUL_X_END;
		state_ranges[7].start = STATE_1_EUL_Y_START;
		state_ranges[7].end = STATE_1_EUL_Y_END;
		state_ranges[8].start = STATE_1_EUL_Z_START;
		state_ranges[8].end = STATE_1_EUL_Z_END;
		state_init(&sara_states[0], state_ranges);
	}

	if (STATE_COUNT >= 2)
	{
		state_ranges[0].start = STATE_2_POS_X_START;
		state_ranges[0].end = STATE_2_POS_X_END;
		state_ranges[1].start = STATE_2_POS_Y_START;
		state_ranges[1].end = STATE_2_POS_Y_END;
		state_ranges[2].start = STATE_2_POS_Z_START;
		state_ranges[2].end = STATE_2_POS_Z_END;
		state_ranges[3].start = STATE_2_VEL_X_START;
		state_ranges[3].end = STATE_2_VEL_X_END;
		state_ranges[4].start = STATE_2_VEL_Y_START;
		state_ranges[4].end = STATE_2_VEL_Y_END;
		state_ranges[5].start = STATE_2_VEL_Z_START;
		state_ranges[5].end = STATE_2_VEL_Z_END;
		state_ranges[6].start = STATE_2_EUL_X_START;
		state_ranges[6].end = STATE_2_EUL_X_END;
		state_ranges[7].start = STATE_2_EUL_Y_START;
		state_ranges[7].end = STATE_2_EUL_Y_END;
		state_ranges[8].start = STATE_2_EUL_Z_START;
		state_ranges[8].end = STATE_2_EUL_Z_END;
		state_init(&sara_states[1], state_ranges);
	}

	if (STATE_COUNT >= 3)
	{
		state_ranges[0].start = STATE_3_POS_X_START;
		state_ranges[0].end = STATE_3_POS_X_END;
		state_ranges[1].start = STATE_3_POS_Y_START;
		state_ranges[1].end = STATE_3_POS_Y_END;
		state_ranges[2].start = STATE_3_POS_Z_START;
		state_ranges[2].end = STATE_3_POS_Z_END;
		state_ranges[3].start = STATE_3_VEL_X_START;
		state_ranges[3].end = STATE_3_VEL_X_END;
		state_ranges[4].start = STATE_3_VEL_Y_START;
		state_ranges[4].end = STATE_3_VEL_Y_END;
		state_ranges[5].start = STATE_3_VEL_Z_START;
		state_ranges[5].end = STATE_3_VEL_Z_END;
		state_ranges[6].start = STATE_3_EUL_X_START;
		state_ranges[6].end = STATE_3_EUL_X_END;
		state_ranges[7].start = STATE_3_EUL_Y_START;
		state_ranges[7].end = STATE_3_EUL_Y_END;
		state_ranges[8].start = STATE_3_EUL_Z_START;
		state_ranges[8].end = STATE_3_EUL_Z_END;
		state_init(&sara_states[2], state_ranges);
	}

	if (STATE_COUNT >= 4)
	{
		state_ranges[0].start = STATE_4_POS_X_START;
		state_ranges[0].end = STATE_4_POS_X_END;
		state_ranges[1].start = STATE_4_POS_Y_START;
		state_ranges[1].end = STATE_4_POS_Y_END;
		state_ranges[2].start = STATE_4_POS_Z_START;
		state_ranges[2].end = STATE_4_POS_Z_END;
		state_ranges[3].start = STATE_4_VEL_X_START;
		state_ranges[3].end = STATE_4_VEL_X_END;
		state_ranges[4].start = STATE_4_VEL_Y_START;
		state_ranges[4].end = STATE_4_VEL_Y_END;
		state_ranges[5].start = STATE_4_VEL_Z_START;
		state_ranges[5].end = STATE_4_VEL_Z_END;
		state_ranges[6].start = STATE_4_EUL_X_START;
		state_ranges[6].end = STATE_4_EUL_X_END;
		state_ranges[7].start = STATE_4_EUL_Y_START;
		state_ranges[7].end = STATE_4_EUL_Y_END;
		state_ranges[8].start = STATE_4_EUL_Z_START;
		state_ranges[8].end = STATE_4_EUL_Z_END;
		state_init(&sara_states[3], state_ranges);
	}

	if (STATE_COUNT >= 5)
	{
		state_ranges[0].start = STATE_5_POS_X_START;
		state_ranges[0].end = STATE_5_POS_X_END;
		state_ranges[1].start = STATE_5_POS_Y_START;
		state_ranges[1].end = STATE_5_POS_Y_END;
		state_ranges[2].start = STATE_5_POS_Z_START;
		state_ranges[2].end = STATE_5_POS_Z_END;
		state_ranges[3].start = STATE_5_VEL_X_START;
		state_ranges[3].end = STATE_5_VEL_X_END;
		state_ranges[4].start = STATE_5_VEL_Y_START;
		state_ranges[4].end = STATE_5_VEL_Y_END;
		state_ranges[5].start = STATE_5_VEL_Z_START;
		state_ranges[5].end = STATE_5_VEL_Z_END;
		state_ranges[6].start = STATE_5_EUL_X_START;
		state_ranges[6].end = STATE_5_EUL_X_END;
		state_ranges[7].start = STATE_5_EUL_Y_START;
		state_ranges[7].end = STATE_5_EUL_Y_END;
		state_ranges[8].start = STATE_5_EUL_Z_START;
		state_ranges[8].end = STATE_5_EUL_Z_END;
		state_init(&sara_states[4], state_ranges);
	}

	if (STATE_COUNT >= 6)
	{
		state_ranges[0].start = STATE_6_POS_X_START;
		state_ranges[0].end = STATE_6_POS_X_END;
		state_ranges[1].start = STATE_6_POS_Y_START;
		state_ranges[1].end = STATE_6_POS_Y_END;
		state_ranges[2].start = STATE_6_POS_Z_START;
		state_ranges[2].end = STATE_6_POS_Z_END;
		state_ranges[3].start = STATE_6_VEL_X_START;
		state_ranges[3].end = STATE_6_VEL_X_END;
		state_ranges[4].start = STATE_6_VEL_Y_START;
		state_ranges[4].end = STATE_6_VEL_Y_END;
		state_ranges[5].start = STATE_6_VEL_Z_START;
		state_ranges[5].end = STATE_6_VEL_Z_END;
		state_ranges[6].start = STATE_6_EUL_X_START;
		state_ranges[6].end = STATE_6_EUL_X_END;
		state_ranges[7].start = STATE_6_EUL_Y_START;
		state_ranges[7].end = STATE_6_EUL_Y_END;
		state_ranges[8].start = STATE_6_EUL_Z_START;
		state_ranges[8].end = STATE_6_EUL_Z_END;
		state_init(&sara_states[5], state_ranges);
	}

	if (STATE_COUNT >= 7)
	{
		state_ranges[0].start = STATE_7_POS_X_START;
		state_ranges[0].end = STATE_7_POS_X_END;
		state_ranges[1].start = STATE_7_POS_Y_START;
		state_ranges[1].end = STATE_7_POS_Y_END;
		state_ranges[2].start = STATE_7_POS_Z_START;
		state_ranges[2].end = STATE_7_POS_Z_END;
		state_ranges[3].start = STATE_7_VEL_X_START;
		state_ranges[3].end = STATE_7_VEL_X_END;
		state_ranges[4].start = STATE_7_VEL_Y_START;
		state_ranges[4].end = STATE_7_VEL_Y_END;
		state_ranges[5].start = STATE_7_VEL_Z_START;
		state_ranges[5].end = STATE_7_VEL_Z_END;
		state_ranges[6].start = STATE_7_EUL_X_START;
		state_ranges[6].end = STATE_7_EUL_X_END;
		state_ranges[7].start = STATE_7_EUL_Y_START;
		state_ranges[7].end = STATE_7_EUL_Y_END;
		state_ranges[8].start = STATE_7_EUL_Z_START;
		state_ranges[8].end = STATE_7_EUL_Z_END;
		state_init(&sara_states[6], state_ranges);
	}
}

void	sara_ukf_f(float ukf_matrix_x[UKF_SIZE_STATE], float ukf_matrix_u[UKF_SIZE_INPUT], float ukf_matrix_x_new[UKF_SIZE_STATE], float dt)
{
	const float	w_g[3] = { 0.0f, 0.0f, 9.81f };
	const float	*p_k  = &ukf_matrix_x[0];
	const float	*v_k  = &ukf_matrix_x[3];
	const float	*a_k  = &ukf_matrix_x[6];
	const float	*q_k  = &ukf_matrix_x[9];
	const float	*b_a  = &ukf_matrix_x[13];
	const float	*b_g  = &ukf_matrix_x[16];
	float		b_p  = ukf_matrix_x[19];
	float		b_vx = ukf_matrix_x[20];
	float		b_vy = ukf_matrix_x[21];
	float		accel_body[3];
	float		omega_corr[3];
	float		R_w2b[3][3];
	float		R_b2w[3][3];
	float		a_k1[3];
	float		half_dt2;
	float		sum;

	accel_body[0] = ukf_matrix_u[0] - b_a[0];
	accel_body[1] = ukf_matrix_u[1] - b_a[1];
	accel_body[2] = ukf_matrix_u[2] - b_a[2];
	omega_corr[0] = ukf_matrix_u[3] - b_g[0];
	omega_corr[1] = ukf_matrix_u[4] - b_g[1];
	omega_corr[2] = ukf_matrix_u[5] - b_g[2];
	memset(ukf_matrix_x_new, 0, UKF_SIZE_STATE * sizeof(float));
	integrate_quat(q_k, omega_corr, dt, &ukf_matrix_x_new[9]);
	arm_quaternion2rotation_f32(&ukf_matrix_x_new[9], (float *)R_w2b, 1);
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			R_b2w[i][j] = R_w2b[j][i];
	for (int i = 0; i < 3; i++)
	{
		sum = 0.0f;
		for (int j = 0; j < 3; j++)
			sum += R_b2w[i][j] * accel_body[j];
		a_k1[i] = sum - w_g[i];
		ukf_matrix_x_new[6 + i] = a_k1[i];
	}
	for (int i = 0; i < 3; i++)
		ukf_matrix_x_new[3 + i] = v_k[i] + a_k[i] * dt;
	half_dt2 = 0.5f * dt * dt;
	for (int i = 0; i < 3; i++)
		ukf_matrix_x_new[i] = p_k[i] + v_k[i] * dt + a_k[i] * half_dt2;
	memcpy(&ukf_matrix_x_new[13],  b_a, 3 * sizeof(float));
	memcpy(&ukf_matrix_x_new[16], b_g, 3 * sizeof(float));
	ukf_matrix_x_new[19] = b_p;
	ukf_matrix_x_new[20] = b_vx;
	ukf_matrix_x_new[21] = b_vy;
}

void	sara_ukf_h(float ukf_matrix_x[UKF_SIZE_STATE], float ukf_matrix_z[UKF_SIZE_MEAS])
{
	const float *p = &ukf_matrix_x[0];
	const float *v = &ukf_matrix_x[3];
	const float *q = &ukf_matrix_x[9];
	float R_w2b[3][3];
	float v_b[3];

	ukf_matrix_z[0] = p[2];
	arm_quaternion2rotation_f32(q, (float *)R_w2b, 1);
	for (int i = 0; i < 3; i++)
		v_b[i] = R_w2b[i][0] * v[0] + R_w2b[i][1] * v[1] + R_w2b[i][2] * v[2];
	ukf_matrix_z[1] = v_b[0];
	ukf_matrix_z[2] = v_b[1];
}

void	sara_init_ukf(t_ukf *sara_ukf)
{
	float	ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE] = {0};
	float	ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE] = {0};
	float	ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS] = {0};
	float	ukf_matrix_x[UKF_SIZE_STATE] = {0};

	ukf_matrix_x[0] = UKF_MATRIX_X_POS_X;
	ukf_matrix_x[1] = UKF_MATRIX_X_POS_Y;
	ukf_matrix_x[2] = UKF_MATRIX_X_POS_Z;
	ukf_matrix_x[3] = UKF_MATRIX_X_VEL_X;
	ukf_matrix_x[4] = UKF_MATRIX_X_VEL_Y;
	ukf_matrix_x[5] = UKF_MATRIX_X_VEL_Z;
	ukf_matrix_x[6] = UKF_MATRIX_X_ACC_X;
	ukf_matrix_x[7] = UKF_MATRIX_X_ACC_Y;
	ukf_matrix_x[8] = UKF_MATRIX_X_ACC_Z;
	ukf_matrix_x[9] = UKF_MATRIX_X_QUAT_W;
	ukf_matrix_x[10] = UKF_MATRIX_X_QUAT_X;
	ukf_matrix_x[11] = UKF_MATRIX_X_QUAT_Y;
	ukf_matrix_x[12] = UKF_MATRIX_X_QUAT_Z;
	ukf_matrix_x[13] = UKF_MATRIX_X_BIAS_ACC_X;
	ukf_matrix_x[14] = UKF_MATRIX_X_BIAS_ACC_Y;
	ukf_matrix_x[15] = UKF_MATRIX_X_BIAS_ACC_Z;
	ukf_matrix_x[16] = UKF_MATRIX_X_BIAS_GYRO_X;
	ukf_matrix_x[17] = UKF_MATRIX_X_BIAS_GYRO_Y;
	ukf_matrix_x[18] = UKF_MATRIX_X_BIAS_GYRO_Z;
	ukf_matrix_x[19] = UKF_MATRIX_X_BIAS_PRESS;
	ukf_matrix_x[20] = UKF_MATRIX_X_BIAS_VEL_X;
	ukf_matrix_x[21] = UKF_MATRIX_X_BIAS_VEL_Y;

	for (int k = 0; k < 3; k++)
		ukf_matrix_p[k][k] = UKF_MATRIX_P_POS * UKF_MATRIX_P_POS;
	for (int k = 0; k < 3; k++)
		ukf_matrix_p[3 + k][3 + k] = UKF_MATRIX_P_VEL * UKF_MATRIX_P_VEL;
	for (int k = 0; k < 3; k++)
		ukf_matrix_p[6 + k][6 + k] = UKF_MATRIX_P_ACC * UKF_MATRIX_P_ACC;
	for (int k = 0; k < 4; k++)
		ukf_matrix_p[9 + k][9 + k] = UKF_MATRIX_P_QUAT * UKF_MATRIX_P_QUAT;
	for (int k = 0; k < 3; k++)
		ukf_matrix_p[13 + k][13 + k] = UKF_MATRIX_P_BIAS_ACC * UKF_MATRIX_P_BIAS_ACC;
	for (int k = 0; k < 3; k++)
		ukf_matrix_p[16 + k][16 + k] = UKF_MATRIX_P_BIAS_GYRO * UKF_MATRIX_P_BIAS_GYRO;
	ukf_matrix_p[19][19] = UKF_MATRIX_P_BIAS_PRESS * UKF_MATRIX_P_BIAS_PRESS;
	ukf_matrix_p[20][20] = UKF_MATRIX_P_BIAS_VEL_X * UKF_MATRIX_P_BIAS_VEL_X;
	ukf_matrix_p[21][21] = UKF_MATRIX_P_BIAS_VEL_Y * UKF_MATRIX_P_BIAS_VEL_Y;

	ukf_matrix_r[0][0] = UKF_MATRIX_R_PRESS * UKF_MATRIX_R_PRESS;
	ukf_matrix_r[1][1] = UKF_MATRIX_R_VEL_X * UKF_MATRIX_R_VEL_X;
	ukf_matrix_r[2][2] = UKF_MATRIX_R_VEL_Y * UKF_MATRIX_R_VEL_Y;

	for (int k = 0; k < 3; k++)
		ukf_matrix_q[6 + k][6 + k] = UKF_MATRIX_Q_SD_ACC * UKF_MATRIX_Q_SD_ACC * UKF_MATRIX_Q_FACT_DYNA;
	for (int k = 0; k < 4; k++)
		ukf_matrix_q[9 + k][9 + k] = UKF_MATRIX_Q_SD_GYRO * UKF_MATRIX_Q_SD_GYRO * UKF_MATRIX_Q_FACT_DYNA;
	for (int k = 0; k < 3; k++)
		ukf_matrix_q[13 + k][13 + k] = UKF_MATRIX_Q_SD_BIAS_ACC * UKF_MATRIX_Q_SD_BIAS_ACC * UKF_MATRIX_Q_FACT_BIAS;
	for (int k = 0; k < 3; k++)
		ukf_matrix_q[16 + k][16 + k] = UKF_MATRIX_Q_SD_BIAS_GYRO * UKF_MATRIX_Q_SD_BIAS_GYRO * UKF_MATRIX_Q_FACT_BIAS;
	ukf_matrix_q[19][19] = UKF_MATRIX_Q_SD_BIAS_PRESS * UKF_MATRIX_Q_SD_BIAS_PRESS;
	ukf_matrix_q[20][20] = UKF_MATRIX_Q_SD_BIAS_VEL_X * UKF_MATRIX_Q_SD_BIAS_VEL_X;
	ukf_matrix_q[21][21] = UKF_MATRIX_Q_SD_BIAS_VEL_Y * UKF_MATRIX_Q_SD_BIAS_VEL_Y;

	ukf_init(sara_ukf, ukf_matrix_x, ukf_matrix_p, ukf_matrix_q, ukf_matrix_r, sara_ukf_f, sara_ukf_h);
}

void	sara_init_sensor(t_sensor *sensor, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc)
{
	sensor->bno055 = (t_bno055){ .i2c = hi2c, .addr = BNO055_ADDR, .opr_mode = BNO055_OPR_MODE_IMU };
	bno055_init(&sensor->bno055);
	bno055_set_unit(&sensor->bno055, BNO055_GYRO_UNIT_DPS, BNO055_ACCEL_UNITSEL_M_S2, BNO055_EUL_UNIT_DEG);
	sen0257_init(&sensor->sen0257, hadc);
}

void	sara_init_control(t_control *control, TIM_HandleTypeDef *htim_fin, TIM_HandleTypeDef *htim_ultras)
{
	d646wp_init(&control->fin_1, htim_fin, TIM_CHANNEL_1, CONTROL_FIN_1_US_MIN, CONTROL_FIN_1_US_MAX);
	d646wp_init(&control->fin_2, htim_fin, TIM_CHANNEL_2, CONTROL_FIN_2_US_MIN, CONTROL_FIN_2_US_MAX);
	d646wp_init(&control->fin_3, htim_fin, TIM_CHANNEL_3, CONTROL_FIN_3_US_MIN, CONTROL_FIN_3_US_MAX);
	d646wp_init(&control->fin_4, htim_fin, TIM_CHANNEL_4, CONTROL_FIN_4_US_MIN, CONTROL_FIN_4_US_MAX);
	ultras_init(&control->ultras, htim_ultras, TIM_CHANNEL_1, CONTROL_ULTRAS_PULSE_MIN, CONTROL_ULTRAS_PULSE_MAX);
	ultras_update(&control->ultras, CONTROL_ULTRAS_START);
}

void	sara_update(t_sara *sara)
{
	const float flap_theta[4] = { M_PI/4.0f, 3.0f*M_PI/4.0f, 5.0f*M_PI/4.0f, 7.0f*M_PI/4.0f };
	float			state_curr[9];
	float			state_next[9];
	float			write_buf[6];
	float			commands[6];
	float			read_buf[9];
	uint32_t static	last_ms;
	uint32_t		now_ms;
	float			dt;

	now_ms = HAL_GetTick();
	dt = (now_ms - last_ms) * 1e-3f;
	last_ms = now_ms;
	if (dt <= 0.0f)
		dt = 1e-3f;
	if (dt > 0.1f)
		dt = 1e-3f;
	sara_update_read(&sara->sara_sensor, read_buf);
	ukf_update(&sara->sara_ukf, read_buf, dt);
	state_curr[0] = sara->sara_ukf.ukf_matrix_x[0];
	state_curr[1] = sara->sara_ukf.ukf_matrix_x[1];
	state_curr[2] = sara->sara_ukf.ukf_matrix_x[2];
	state_curr[3] = sara->sara_ukf.ukf_matrix_x[3];
	state_curr[4] = sara->sara_ukf.ukf_matrix_x[4];
	state_curr[5] = sara->sara_ukf.ukf_matrix_x[5];
	quat_to_euler(&sara->sara_ukf.ukf_matrix_x[6], &state_curr[6], &state_curr[7], &state_curr[8]);
	sara->sara_state = state_machine(sara->sara_states, sara->sara_state, state_curr);
	if (sara->sara_state < STATE_COUNT)
	{
		state_next[0] = (sara->sara_states[sara->sara_state].pos_x.start + sara->sara_states[sara->sara_state].pos_x.end) / 2;
		state_next[1] = (sara->sara_states[sara->sara_state].pos_y.start + sara->sara_states[sara->sara_state].pos_y.end) / 2;
		state_next[2] = (sara->sara_states[sara->sara_state].pos_z.start + sara->sara_states[sara->sara_state].pos_z.end) / 2;
		state_next[3] = (sara->sara_states[sara->sara_state].vel_x.start + sara->sara_states[sara->sara_state].vel_x.end) / 2;
		state_next[4] = (sara->sara_states[sara->sara_state].vel_y.start + sara->sara_states[sara->sara_state].vel_y.end) / 2;
		state_next[5] = (sara->sara_states[sara->sara_state].vel_z.start + sara->sara_states[sara->sara_state].vel_z.end) / 2;
		state_next[6] = (sara->sara_states[sara->sara_state].eul_x.start + sara->sara_states[sara->sara_state].eul_x.end) / 2;
		state_next[7] = (sara->sara_states[sara->sara_state].eul_y.start + sara->sara_states[sara->sara_state].eul_y.end) / 2;
		state_next[8] = (sara->sara_states[sara->sara_state].eul_z.start + sara->sara_states[sara->sara_state].eul_z.end) / 2;
		commands[0] = pid_update(&sara->sara_dof.pid_surge_vel, pid_update(&sara->sara_dof.pid_surge_pos, state_next[0] - state_curr[0], dt) - state_curr[3], dt);
		commands[1] = pid_update(&sara->sara_dof.pid_sway_vel, pid_update(&sara->sara_dof.pid_sway_pos, state_next[1] - state_curr[1], dt) - state_curr[4], dt);
		commands[2] = pid_update(&sara->sara_dof.pid_heave_vel, pid_update(&sara->sara_dof.pid_heave_pos, state_next[2] - state_curr[2], dt) - state_curr[5], dt);
		commands[3] = wrap_pi(pid_update(&sara->sara_dof.pid_roll, state_next[6] - state_curr[6], dt));
		commands[4] = wrap_pi(pid_update(&sara->sara_dof.pid_pitch, state_next[7] - state_curr[7], dt));
		commands[5] = wrap_pi(pid_update(&sara->sara_dof.pid_yaw, state_next[8] - state_curr[8], dt));
		write_buf[5] = CONTROL_GAIN_SURGE * commands[0];
		for (int i = 0; i < CONTROL_FIN_COUNT; i++)
		{
			float c = cosf(flap_theta[i]);
			float s = sinf(flap_theta[i]);
			float F_sway = CONTROL_GAIN_SWAY * commands[1] * c;
			float F_heave = CONTROL_GAIN_HEAVE * commands[2] * s;
			float M_roll = CONTROL_GAIN_ROLL * commands[3] / CONTROL_FIN_RADIUS;
			float M_pitch = CONTROL_GAIN_PITCH * commands[4] / CONTROL_FIN_RADIUS;
			float M_yaw = CONTROL_GAIN_YAW * commands[5] / CONTROL_FIN_RADIUS;
			float F_roll = M_roll * s;
			float F_pitch = M_pitch * c;
			float F_yaw = M_yaw * c;
			write_buf[i] = F_heave + F_sway + F_roll + F_pitch + F_yaw;
		}
	}
	else
	{
		write_buf[0] = CONTROL_FIN_1_FINISH;
		write_buf[1] = CONTROL_FIN_2_FINISH;
		write_buf[2] = CONTROL_FIN_3_FINISH;
		write_buf[3] = CONTROL_FIN_4_FINISH;
		write_buf[4] = CONTROL_ULTRAS_START;
	}
	sara_update_write(&sara->sara_control, write_buf);
	HAL_Delay(1);
}

void	sara_update_read(t_sensor *sensor, float read_buf[9])
{
	bno055_linear_acc(&sensor->bno055, &sensor->bno055.linear_acc);
	bno055_gyro(&sensor->bno055, &sensor->bno055.gyro);
	sen0257_update(&sensor->sen0257);
	dvl650_update(&sensor->dvl650);
	read_buf[0] = sensor->bno055.linear_acc.x;
	read_buf[1] = sensor->bno055.linear_acc.y;
	read_buf[2] = sensor->bno055.linear_acc.z;
	read_buf[3] = sensor->bno055.gyro.x;
	read_buf[4] = sensor->bno055.gyro.y;
	read_buf[5] = sensor->bno055.gyro.z;
	read_buf[6] = sensor->sen0257.pressure;
	read_buf[7] = sensor->dvl650.vel_x;
	read_buf[8] = sensor->dvl650.vel_y;
}

void	sara_update_write(t_control *control, float write_buf[5])
{
	d646wp_update(&control->fin_1, write_buf[0]);
	d646wp_update(&control->fin_2, write_buf[1]);
	d646wp_update(&control->fin_3, write_buf[2]);
	d646wp_update(&control->fin_4, write_buf[3]);
	ultras_update(&control->ultras, write_buf[4]);
}
