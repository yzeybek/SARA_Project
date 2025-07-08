/*
 * sara.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "sara.h"

void	sara_init(t_sara *sara)
{
	sara_init_dof(&sara->dof);
	sara_init_state(&sara->states);
	sara_init_ukf(&sara->ukf);
}

void	sara_init_pid(t_dof *sara_dof)
{
	float pid_values[7];

	pid_values[0] = PID_HEAVE_KP;
	pid_values[1] = PID_HEAVE_KI;
	pid_values[2] = PID_HEAVE_KD;
	pid_values[3] = PID_HEAVE_OUT_MIN;
	pid_values[4] = PID_HEAVE_OUT_MAX;
	pid_values[5] = PID_HEAVE_INT_MIN;
	pid_values[6] = PID_HEAVE_INT_MAX;
	pid_init(&dof->pid_heave, pid_values);

	pid_values[0] = PID_SWAY_KP;
	pid_values[1] = PID_SWAY_KI;
	pid_values[2] = PID_SWAY_KD;
	pid_values[3] = PID_SWAY_OUT_MIN;
	pid_values[4] = PID_SWAY_OUT_MAX;
	pid_values[5] = PID_SWAY_INT_MIN;
	pid_values[6] = PID_SWAY_INT_MAX;
	pid_init(&dof->pid_sway, pid_values);

	pid_values[0] = PID_SURGE_KP;
	pid_values[1] = PID_SURGE_KI;
	pid_values[2] = PID_SURGE_KD;
	pid_values[3] = PID_SURGE_OUT_MIN;
	pid_values[4] = PID_SURGE_OUT_MAX;
	pid_values[5] = PID_SURGE_INT_MIN;
	pid_values[6] = PID_SURGE_INT_MAX;
	pid_init(&dof->pid_surge, pid_values);

	pid_values[0] = PID_ROLL_KP;
	pid_values[1] = PID_ROLL_KI;
	pid_values[2] = PID_ROLL_KD;
	pid_values[3] = PID_ROLL_OUT_MIN;
	pid_values[4] = PID_ROLL_OUT_MAX;
	pid_values[5] = PID_ROLL_INT_MIN;
	pid_values[6] = PID_ROLL_INT_MAX;
	pid_init(&dof->pid_roll, pid_values);

	pid_values[0] = PID_PITCH_KP;
	pid_values[1] = PID_PITCH_KI;
	pid_values[2] = PID_PITCH_KD;
	pid_values[3] = PID_PITCH_OUT_MIN;
	pid_values[4] = PID_PITCH_OUT_MAX;
	pid_values[5] = PID_PITCH_INT_MIN;
	pid_values[6] = PID_PITCH_INT_MAX;
	pid_init(&dof->pid_pitch, pid_values);

	pid_values[0] = PID_YAW_KP;
	pid_values[1] = PID_YAW_KI;
	pid_values[2] = PID_YAW_KD;
	pid_values[3] = PID_YAW_OUT_MIN;
	pid_values[4] = PID_YAW_OUT_MAX;
	pid_values[5] = PID_YAW_INT_MIN;
	pid_values[6] = PID_YAW_INT_MAX;
	pid_init(&dof->pid_yaw, pid_values);
}

void	sara_init_state(t_state **sara_states)
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
		state_init(states[0], state_ranges);
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
		state_init(states[1], state_ranges);
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
		state_init(states[2], state_ranges);
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
		state_init(states[3], state_ranges);
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
		state_init(states[4], state_ranges);
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
		state_init(states[5], state_ranges);
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
		state_init(states[6], state_ranges);
	}

	states[STATE_COUNT - 1] = &(t_state){0};
}

void	sara_ukf_f(float ukf_matrix_x[UKF_SIZE_STATE], float ukf_matrix_u[UKF_SIZE_INPUT], float *ukf_matrix_x_new[UKF_SIZE_STATE])
{

}

void	sara_ukf_h(float ukf_matrix_x[UKF_SIZE_STATE], float *ukf_matrix_z[UKF_SIZE_MEAS])
{

}

void	sara_init_ukf(t_ukf *sara_ukf)
{
	float	ukf_matrix_x[UKF_SIZE_STATE];
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

	float	ukf_matrix_p[UKF_SIZE_STATE][UKF_SIZE_STATE] = (float[UKF_SIZE_STATE][UKF_SIZE_STATE]){0};
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

	float	ukf_matrix_r[UKF_SIZE_MEAS][UKF_SIZE_MEAS] = (float[UKF_SIZE_MEAS][UKF_SIZE_MEAS]){0};
	ukf_matrix_r[0][0] = UKF_MATRIX_R_PRESS * UKF_MATRIX_R_PRESS;
	ukf_matrix_r[1][1] = UKF_MATRIX_R_VEL_X * UKF_MATRIX_R_VEL_X;
	ukf_matrix_r[2][2] = UKF_MATRIX_R_VEL_X * UKF_MATRIX_R_VEL_X;

	float	ukf_matrix_q[UKF_SIZE_STATE][UKF_SIZE_STATE];
	//...


	ukf_init(sara_ukf, &ukf_matrix_x, &ukf_matrix_p, &ukf_matrix_q, &ukf_matrix_r, sara_ukf_f, sara_ukf_h);
}
