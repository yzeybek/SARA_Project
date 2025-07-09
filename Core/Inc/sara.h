/*
 * sara.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_SARA_H_
#define INC_SARA_H_

# include "helpers.h"
# include "pid.h"
# include "ukf.h"
# include "state.h"
# include "bno055.h"
# include "sen0257.h"
# include "dvl650.h"

// PID Tune Begin

# define PID_HEAVE_KP 1.0f
# define PID_HEAVE_KI 1.0f
# define PID_HEAVE_KD 1.0f
# define PID_HEAVE_OUT_MIN 1.0f
# define PID_HEAVE_OUT_MAX 1.0f
# define PID_HEAVE_INT_MIN 1.0f
# define PID_HEAVE_INT_MAX 1.0f

# define PID_SWAY_KP 1.0f
# define PID_SWAY_KI 1.0f
# define PID_SWAY_KD 1.0f
# define PID_SWAY_OUT_MIN 1.0f
# define PID_SWAY_OUT_MAX 1.0f
# define PID_SWAY_INT_MIN 1.0f
# define PID_SWAY_INT_MAX 1.0f

# define PID_SURGE_KP 1.0f
# define PID_SURGE_KI 1.0f
# define PID_SURGE_KD 1.0f
# define PID_SURGE_OUT_MIN 1.0f
# define PID_SURGE_OUT_MAX 1.0f
# define PID_SURGE_INT_MIN 1.0f
# define PID_SURGE_INT_MAX 1.0f

# define PID_ROLL_KP 1.0f
# define PID_ROLL_KI 1.0f
# define PID_ROLL_KD 1.0f
# define PID_ROLL_OUT_MIN 1.0f
# define PID_ROLL_OUT_MAX 1.0f
# define PID_ROLL_INT_MIN 1.0f
# define PID_ROLL_INT_MAX 1.0f

# define PID_PITCH_KP 1.0f
# define PID_PITCH_KI 1.0f
# define PID_PITCH_KD 1.0f
# define PID_PITCH_OUT_MIN 1.0f
# define PID_PITCH_OUT_MAX 1.0f
# define PID_PITCH_INT_MIN 1.0f
# define PID_PITCH_INT_MAX 1.0f

# define PID_YAW_KP 1.0f
# define PID_YAW_KI 1.0f
# define PID_YAW_KD 1.0f
# define PID_YAW_OUT_MIN 1.0f
# define PID_YAW_OUT_MAX 1.0f
# define PID_YAW_INT_MIN 1.0f
# define PID_YAW_INT_MAX 1.0f

// PID Tune End

// State Tune Begin

# define STATE_COUNT 8

# define STATE_1_POS_X_START 1.0f
# define STATE_1_POS_X_END 1.0f
# define STATE_1_POS_Y_START 1.0f
# define STATE_1_POS_Y_END 1.0f
# define STATE_1_POS_Z_START 1.0f
# define STATE_1_POS_Z_END 1.0f
# define STATE_1_VEL_X_START 1.0f
# define STATE_1_VEL_X_END 1.0f
# define STATE_1_VEL_Y_START 1.0f
# define STATE_1_VEL_Y_END 1.0f
# define STATE_1_VEL_Z_START 1.0f
# define STATE_1_VEL_Z_END 1.0f
# define STATE_1_EUL_X_START 1.0f
# define STATE_1_EUL_X_END 1.0f
# define STATE_1_EUL_Y_START 1.0f
# define STATE_1_EUL_Y_END 1.0f
# define STATE_1_EUL_Z_START 1.0f
# define STATE_1_EUL_Z_END 1.0f

# define STATE_2_POS_X_START 1.0f
# define STATE_2_POS_X_END 1.0f
# define STATE_2_POS_Y_START 1.0f
# define STATE_2_POS_Y_END 1.0f
# define STATE_2_POS_Z_START 1.0f
# define STATE_2_POS_Z_END 1.0f
# define STATE_2_VEL_X_START 1.0f
# define STATE_2_VEL_X_END 1.0f
# define STATE_2_VEL_Y_START 1.0f
# define STATE_2_VEL_Y_END 1.0f
# define STATE_2_VEL_Z_START 1.0f
# define STATE_2_VEL_Z_END 1.0f
# define STATE_2_EUL_X_START 1.0f
# define STATE_2_EUL_X_END 1.0f
# define STATE_2_EUL_Y_START 1.0f
# define STATE_2_EUL_Y_END 1.0f
# define STATE_2_EUL_Z_START 1.0f
# define STATE_2_EUL_Z_END 1.0f

# define STATE_3_POS_X_START 1.0f
# define STATE_3_POS_X_END 1.0f
# define STATE_3_POS_Y_START 1.0f
# define STATE_3_POS_Y_END 1.0f
# define STATE_3_POS_Z_START 1.0f
# define STATE_3_POS_Z_END 1.0f
# define STATE_3_VEL_X_START 1.0f
# define STATE_3_VEL_X_END 1.0f
# define STATE_3_VEL_Y_START 1.0f
# define STATE_3_VEL_Y_END 1.0f
# define STATE_3_VEL_Z_START 1.0f
# define STATE_3_VEL_Z_END 1.0f
# define STATE_3_EUL_X_START 1.0f
# define STATE_3_EUL_X_END 1.0f
# define STATE_3_EUL_Y_START 1.0f
# define STATE_3_EUL_Y_END 1.0f
# define STATE_3_EUL_Z_START 1.0f
# define STATE_3_EUL_Z_END 1.0f

# define STATE_4_POS_X_START 1.0f
# define STATE_4_POS_X_END 1.0f
# define STATE_4_POS_Y_START 1.0f
# define STATE_4_POS_Y_END 1.0f
# define STATE_4_POS_Z_START 1.0f
# define STATE_4_POS_Z_END 1.0f
# define STATE_4_VEL_X_START 1.0f
# define STATE_4_VEL_X_END 1.0f
# define STATE_4_VEL_Y_START 1.0f
# define STATE_4_VEL_Y_END 1.0f
# define STATE_4_VEL_Z_START 1.0f
# define STATE_4_VEL_Z_END 1.0f
# define STATE_4_EUL_X_START 1.0f
# define STATE_4_EUL_X_END 1.0f
# define STATE_4_EUL_Y_START 1.0f
# define STATE_4_EUL_Y_END 1.0f
# define STATE_4_EUL_Z_START 1.0f
# define STATE_4_EUL_Z_END 1.0f

# define STATE_5_POS_X_START 1.0f
# define STATE_5_POS_X_END 1.0f
# define STATE_5_POS_Y_START 1.0f
# define STATE_5_POS_Y_END 1.0f
# define STATE_5_POS_Z_START 1.0f
# define STATE_5_POS_Z_END 1.0f
# define STATE_5_VEL_X_START 1.0f
# define STATE_5_VEL_X_END 1.0f
# define STATE_5_VEL_Y_START 1.0f
# define STATE_5_VEL_Y_END 1.0f
# define STATE_5_VEL_Z_START 1.0f
# define STATE_5_VEL_Z_END 1.0f
# define STATE_5_EUL_X_START 1.0f
# define STATE_5_EUL_X_END 1.0f
# define STATE_5_EUL_Y_START 1.0f
# define STATE_5_EUL_Y_END 1.0f
# define STATE_5_EUL_Z_START 1.0f
# define STATE_5_EUL_Z_END 1.0f

# define STATE_6_POS_X_START 1.0f
# define STATE_6_POS_X_END 1.0f
# define STATE_6_POS_Y_START 1.0f
# define STATE_6_POS_Y_END 1.0f
# define STATE_6_POS_Z_START 1.0f
# define STATE_6_POS_Z_END 1.0f
# define STATE_6_VEL_X_START 1.0f
# define STATE_6_VEL_X_END 1.0f
# define STATE_6_VEL_Y_START 1.0f
# define STATE_6_VEL_Y_END 1.0f
# define STATE_6_VEL_Z_START 1.0f
# define STATE_6_VEL_Z_END 1.0f
# define STATE_6_EUL_X_START 1.0f
# define STATE_6_EUL_X_END 1.0f
# define STATE_6_EUL_Y_START 1.0f
# define STATE_6_EUL_Y_END 1.0f
# define STATE_6_EUL_Z_START 1.0f
# define STATE_6_EUL_Z_END 1.0f

# define STATE_7_POS_X_START 1.0f
# define STATE_7_POS_X_END 1.0f
# define STATE_7_POS_Y_START 1.0f
# define STATE_7_POS_Y_END 1.0f
# define STATE_7_POS_Z_START 1.0f
# define STATE_7_POS_Z_END 1.0f
# define STATE_7_VEL_X_START 1.0f
# define STATE_7_VEL_X_END 1.0f
# define STATE_7_VEL_Y_START 1.0f
# define STATE_7_VEL_Y_END 1.0f
# define STATE_7_VEL_Z_START 1.0f
# define STATE_7_VEL_Z_END 1.0f
# define STATE_7_EUL_X_START 1.0f
# define STATE_7_EUL_X_END 1.0f
# define STATE_7_EUL_Y_START 1.0f
# define STATE_7_EUL_Y_END 1.0f
# define STATE_7_EUL_Z_START 1.0f
# define STATE_7_EUL_Z_END 1.0f

// State Tune End

// UKF Tune Begin

# define UKF_MATRIX_X_POS_X 0.0f
# define UKF_MATRIX_X_POS_Y 0.0f
# define UKF_MATRIX_X_POS_Z 0.0f
# define UKF_MATRIX_X_VEL_X 0.0f
# define UKF_MATRIX_X_VEL_Y 0.0f
# define UKF_MATRIX_X_VEL_Z 0.0f
# define UKF_MATRIX_X_ACC_X 0.0f
# define UKF_MATRIX_X_ACC_Y 0.0f
# define UKF_MATRIX_X_ACC_Z 0.0f
# define UKF_MATRIX_X_QUAT_W 1.0f
# define UKF_MATRIX_X_QUAT_X 0.0f
# define UKF_MATRIX_X_QUAT_Y 0.0f
# define UKF_MATRIX_X_QUAT_Z 0.0f
# define UKF_MATRIX_X_BIAS_ACC_X 0.01f
# define UKF_MATRIX_X_BIAS_ACC_Y -0.01f
# define UKF_MATRIX_X_BIAS_ACC_Z 0.02f
# define UKF_MATRIX_X_BIAS_GYRO_X 0.0005f
# define UKF_MATRIX_X_BIAS_GYRO_Y 0.0005f
# define UKF_MATRIX_X_BIAS_GYRO_Z -0.0005f
# define UKF_MATRIX_X_BIAS_PRESS 0.0f
# define UKF_MATRIX_X_BIAS_VEL_X 0.0f
# define UKF_MATRIX_X_BIAS_VEL_Y 0.0f

# define UKF_MATRIX_P_POS 1.0f
# define UKF_MATRIX_P_VEL 0.5f
# define UKF_MATRIX_P_ACC 0.1f
# define UKF_MATRIX_P_QUAT (5.0f * (M_PI / 180.0f))
# define UKF_MATRIX_P_BIAS_ACC 0.49f
# define UKF_MATRIX_P_BIAS_GYRO 0.01745f
# define UKF_MATRIX_P_BIAS_PRESS 0.15f
# define UKF_MATRIX_P_BIAS_VEL_X 0.03f
# define UKF_MATRIX_P_BIAS_VEL_Y 0.03f

# define UKF_MATRIX_R_PRESS 0.8155f
# define UKF_MATRIX_R_VEL_X 0.1f
# define UKF_MATRIX_R_VEL_Y 0.1f

# define UKF_MATRIX_Q_FACT_DYNA 10.0f
# define UKF_MATRIX_Q_FACT_BIAS 5.0f
# define UKF_MATRIX_Q_SD_ACC 0.0014715f
# define UKF_MATRIX_Q_SD_GYRO 0.00024435f
# define UKF_MATRIX_Q_SD_BIAS_ACC 0.0005f
# define UKF_MATRIX_Q_SD_BIAS_GYRO 0.0001f
# define UKF_MATRIX_Q_SD_BIAS_PRESS 0.0001f
# define UKF_MATRIX_Q_SD_BIAS_VEL_X 0.002f
# define UKF_MATRIX_Q_SD_BIAS_VEL_Y 0.002f

// UKF Tune End

typedef struct s_dof
{
	t_pid	pid_heave;
	t_pid	pid_sway;
	t_pid	pid_surge;
	t_pid	pid_roll;
	t_pid	pid_pitch;
	t_pid	pid_yaw;

} t_dof;

typedef struct s_sensor
{
	t_bno055	bno055;
	t_sen0257	sen0257;
	t_dvl650	dvl650;

} t_sensor;

typedef struct s_sara
{
	t_dof		sara_dof;
	t_state 	sara_states[STATE_COUNT];
	t_ukf		sara_ukf;
	t_sensor	sara_sensor;

} t_sara;

void	sara_init(t_sara *sara, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc);
void	sara_init_dof(t_dof *sara_dof);
void	sara_init_state(t_state sara_states[STATE_COUNT]);
void	sara_init_ukf(t_ukf *sara_ukf);
void	sara_init_sensor(t_sensor *sensor, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc);

#endif /* INC_SARA_H_ */
