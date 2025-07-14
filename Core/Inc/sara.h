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
# include "d646wp.h"
# include "ultras.h"

// PID Tune Begin

# define PID_SURGE_POS_KP 0.5f
# define PID_SURGE_POS_KI 0.1f
# define PID_SURGE_POS_KD 0.01f
# define PID_SURGE_POS_OUT_MIN -0.8f
# define PID_SURGE_POS_OUT_MAX 0.8f
# define PID_SURGE_POS_INT_MIN -0.5f
# define PID_SURGE_POS_INT_MAX 0.5f

# define PID_SWAY_POS_KP 0.5f
# define PID_SWAY_POS_KI 0.1f
# define PID_SWAY_POS_KD 0.01f
# define PID_SWAY_POS_OUT_MIN -0.8f
# define PID_SWAY_POS_OUT_MAX 0.8f
# define PID_SWAY_POS_INT_MIN -0.5f
# define PID_SWAY_POS_INT_MAX 0.5f

# define PID_HEAVE_POS_KP 0.5f
# define PID_HEAVE_POS_KI 0.1f
# define PID_HEAVE_POS_KD 0.01f
# define PID_HEAVE_POS_OUT_MIN -0.8f
# define PID_HEAVE_POS_OUT_MAX 0.8f
# define PID_HEAVE_POS_INT_MIN -0.5f
# define PID_HEAVE_POS_INT_MAX 0.5f

# define PID_SURGE_VEL_KP 1.5f
# define PID_SURGE_VEL_KI 0.1f
# define PID_SURGE_VEL_KD 0.05f
# define PID_SURGE_VEL_OUT_MIN -100.0f
# define PID_SURGE_VEL_OUT_MAX 100.0f
# define PID_SURGE_VEL_INT_MIN -10.0f
# define PID_SURGE_VEL_INT_MAX 10.0f

# define PID_SWAY_VEL_KP 1.5f
# define PID_SWAY_VEL_KI 0.1f
# define PID_SWAY_VEL_KD 0.05f
# define PID_SWAY_VEL_OUT_MIN -100.0f
# define PID_SWAY_VEL_OUT_MAX 100.0f
# define PID_SWAY_VEL_INT_MIN -10.0f
# define PID_SWAY_VEL_INT_MAX 10.0f

# define PID_HEAVE_VEL_KP 1.5f
# define PID_HEAVE_VEL_KI 0.1f
# define PID_HEAVE_VEL_KD 0.05f
# define PID_HEAVE_VEL_OUT_MIN -30.0f
# define PID_HEAVE_VEL_OUT_MAX 30.0f
# define PID_HEAVE_VEL_INT_MIN 0.0f
# define PID_HEAVE_VEL_INT_MAX 0.0f

# define PID_ROLL_KP 2.0f
# define PID_ROLL_KI 0.0f
# define PID_ROLL_KD 0.1f
# define PID_ROLL_OUT_MIN -30.0f
# define PID_ROLL_OUT_MAX 30.0f
# define PID_ROLL_INT_MIN 0.0f
# define PID_ROLL_INT_MAX 0.0f

# define PID_PITCH_KP 2.0f
# define PID_PITCH_KI 0.0f
# define PID_PITCH_KD 0.10f
# define PID_PITCH_OUT_MIN -30.0f
# define PID_PITCH_OUT_MAX 30.0f
# define PID_PITCH_INT_MIN 0.0f
# define PID_PITCH_INT_MAX 0.0f

# define PID_YAW_KP 1.5f
# define PID_YAW_KI 0.0f
# define PID_YAW_KD 0.05f
# define PID_YAW_OUT_MIN -30.0f
# define PID_YAW_OUT_MAX 30.0f
# define PID_YAW_INT_MIN 0.0f
# define PID_YAW_INT_MAX 0.0f

// PID Tune End

// State Tune Begin

# define STATE_COUNT 7

# define STATE_1_POS_X_START -0.5f
# define STATE_1_POS_X_END 0.5f
# define STATE_1_POS_Y_START -0.3f
# define STATE_1_POS_Y_END 0.3f
# define STATE_1_POS_Z_START -0.3f
# define STATE_1_POS_Z_END 0.3f
# define STATE_1_VEL_X_START -0.1f
# define STATE_1_VEL_X_END 0.1f
# define STATE_1_VEL_Y_START -0.1f
# define STATE_1_VEL_Y_END 0.1f
# define STATE_1_VEL_Z_START -0.1f
# define STATE_1_VEL_Z_END 0.1f
# define STATE_1_EUL_X_START 350.0f
# define STATE_1_EUL_X_END 10.0f
# define STATE_1_EUL_Y_START 350.0f
# define STATE_1_EUL_Y_END 10.0f
# define STATE_1_EUL_Z_START 350.0f
# define STATE_1_EUL_Z_END 10.0f

# define STATE_2_POS_X_START 9.5f
# define STATE_2_POS_X_END 10.5f
# define STATE_2_POS_Y_START -0.3f
# define STATE_2_POS_Y_END 0.3f
# define STATE_2_POS_Z_START -0.3f
# define STATE_2_POS_Z_END 0.3f
# define STATE_2_VEL_X_START 0.3f
# define STATE_2_VEL_X_END 0.4f
# define STATE_2_VEL_Y_START -0.1f
# define STATE_2_VEL_Y_END 0.1f
# define STATE_2_VEL_Z_START -0.1f
# define STATE_2_VEL_Z_END 0.1f
# define STATE_2_EUL_X_START 350.0f
# define STATE_2_EUL_X_END 10.0f
# define STATE_2_EUL_Y_START 350.0f
# define STATE_2_EUL_Y_END 10.0f
# define STATE_2_EUL_Z_START 350.0f
# define STATE_2_EUL_Z_END 10.0f

# define STATE_3_POS_X_START 12.5f
# define STATE_3_POS_X_END 13.5f
# define STATE_3_POS_Y_START -0.3f
# define STATE_3_POS_Y_END 0.3f
# define STATE_3_POS_Z_START -1.5f
# define STATE_3_POS_Z_END -2.0f
# define STATE_3_VEL_X_START 0.3f
# define STATE_3_VEL_X_END 0.4f
# define STATE_3_VEL_Y_START -0.1f
# define STATE_3_VEL_Y_END 0.1f
# define STATE_3_VEL_Z_START 0.25f
# define STATE_3_VEL_Z_END 0.35f
# define STATE_3_EUL_X_START 350.0f
# define STATE_3_EUL_X_END 10.0f
# define STATE_3_EUL_Y_START 350.0f
# define STATE_3_EUL_Y_END 10.0f
# define STATE_3_EUL_Z_START 350.0f
# define STATE_3_EUL_Z_END 10.0f

# define STATE_4_POS_X_START 49.5f
# define STATE_4_POS_X_END 50.5f
# define STATE_4_POS_Y_START -0.3f
# define STATE_4_POS_Y_END 0.3f
# define STATE_4_POS_Z_START -1.5f
# define STATE_4_POS_Z_END -2.0f
# define STATE_4_VEL_X_START 0.7f
# define STATE_4_VEL_X_END 0.9f
# define STATE_4_VEL_Y_START -0.1f
# define STATE_4_VEL_Y_END 0.1f
# define STATE_4_VEL_Z_START 0.25f
# define STATE_4_VEL_Z_END 0.35f
# define STATE_4_EUL_X_START 350.0f
# define STATE_4_EUL_X_END 10.0f
# define STATE_4_EUL_Y_START 350.0f
# define STATE_4_EUL_Y_END 10.0f
# define STATE_4_EUL_Z_START 350.0f
# define STATE_4_EUL_Z_END 10.0f

# define STATE_5_POS_X_START 49.5f
# define STATE_5_POS_X_END 50.5f
# define STATE_5_POS_Y_START -9.0f
# define STATE_5_POS_Y_END -11.0f
# define STATE_5_POS_Z_START -1.5f
# define STATE_5_POS_Z_END -2.0f
# define STATE_5_VEL_X_START 0.7f
# define STATE_5_VEL_X_END 0.9f
# define STATE_5_VEL_Y_START -0.1f
# define STATE_5_VEL_Y_END 0.1f
# define STATE_5_VEL_Z_START -0.1f
# define STATE_5_VEL_Z_END 0.1f
# define STATE_5_EUL_X_START 350.0f
# define STATE_5_EUL_X_END 10.0f
# define STATE_5_EUL_Y_START 350.0f
# define STATE_5_EUL_Y_END 10.0f
# define STATE_5_EUL_Z_START 170.0f
# define STATE_5_EUL_Z_END 190.0f

# define STATE_6_POS_X_START 9.5f
# define STATE_6_POS_X_END 10.5f
# define STATE_6_POS_Y_START -9.5f
# define STATE_6_POS_Y_END -10.5f
# define STATE_6_POS_Z_START -1.5f
# define STATE_6_POS_Z_END -2.0f
# define STATE_6_VEL_X_START 0.3f
# define STATE_6_VEL_X_END 0.4f
# define STATE_6_VEL_Y_START -0.1f
# define STATE_6_VEL_Y_END 0.1f
# define STATE_6_VEL_Z_START 0.25f
# define STATE_6_VEL_Z_END 0.35f
# define STATE_6_EUL_X_START 350.0f
# define STATE_6_EUL_X_END 10.0f
# define STATE_6_EUL_Y_START 350.0f
# define STATE_6_EUL_Y_END 10.0f
# define STATE_6_EUL_Z_START 170.0f
# define STATE_6_EUL_Z_END 190.0f

# define STATE_7_POS_X_START 9.5f
# define STATE_7_POS_X_END 10.5f
# define STATE_7_POS_Y_START -0.3f
# define STATE_7_POS_Y_END 0.3f
# define STATE_7_POS_Z_START -0.3f
# define STATE_7_POS_Z_END 0.3f
# define STATE_7_VEL_X_START -0.1f
# define STATE_7_VEL_X_END 0.1f
# define STATE_7_VEL_Y_START -0.1f
# define STATE_7_VEL_Y_END 0.1f
# define STATE_7_VEL_Z_START -0.1f
# define STATE_7_VEL_Z_END 0.1f
# define STATE_7_EUL_X_START 350.0f
# define STATE_7_EUL_X_END 10.0f
# define STATE_7_EUL_Y_START 350.0f
# define STATE_7_EUL_Y_END 10.0f
# define STATE_7_EUL_Z_START 350.0f
# define STATE_7_EUL_Z_END 10.0f

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
# define UKF_MATRIX_X_BIAS_ACC_X 0.08f
# define UKF_MATRIX_X_BIAS_ACC_Y 0.08f
# define UKF_MATRIX_X_BIAS_ACC_Z 0.08f
# define UKF_MATRIX_X_BIAS_GYRO_X 0.03f
# define UKF_MATRIX_X_BIAS_GYRO_Y 0.03f
# define UKF_MATRIX_X_BIAS_GYRO_Z 0.03f
# define UKF_MATRIX_X_BIAS_PRESS 0.05f
# define UKF_MATRIX_X_BIAS_VEL_X 0.002f
# define UKF_MATRIX_X_BIAS_VEL_Y 0.002f

# define UKF_MATRIX_P_POS 0.005f
# define UKF_MATRIX_P_VEL 0.02f
# define UKF_MATRIX_P_ACC 0.1f
# define UKF_MATRIX_P_QUAT 0.01f
# define UKF_MATRIX_P_BIAS_ACC 0.1
# define UKF_MATRIX_P_BIAS_GYRO 0.05f
# define UKF_MATRIX_P_BIAS_PRESS 1.0f
# define UKF_MATRIX_P_BIAS_VEL_X 0.005f
# define UKF_MATRIX_P_BIAS_VEL_Y 0.005f

# define UKF_MATRIX_R_PRESS 0.8f
# define UKF_MATRIX_R_VEL_X 0.1f
# define UKF_MATRIX_R_VEL_Y 0.1f

# define UKF_MATRIX_Q_FACT_DYNA 10.0f
# define UKF_MATRIX_Q_FACT_BIAS 5.0f
# define UKF_MATRIX_Q_SD_ACC 0.001f
# define UKF_MATRIX_Q_SD_GYRO 0.00052f
# define UKF_MATRIX_Q_SD_BIAS_ACC 0.0005f
# define UKF_MATRIX_Q_SD_BIAS_GYRO 0.0001f
# define UKF_MATRIX_Q_SD_BIAS_PRESS 0.0001f
# define UKF_MATRIX_Q_SD_BIAS_VEL_X 0.002f
# define UKF_MATRIX_Q_SD_BIAS_VEL_Y 0.002f

// UKF Tune End

// Control Tune Start

# define CONTROL_FIN_RADIUS 0.085f
# define CONTROL_FIN_COUNT 4

# define CONTROL_FIN_1_START 0
# define CONTROL_FIN_1_FINISH 0
# define CONTROL_FIN_1_US_MIN 900
# define CONTROL_FIN_1_US_MAX 2100

# define CONTROL_FIN_2_START 0
# define CONTROL_FIN_2_FINISH 0
# define CONTROL_FIN_2_US_MIN 900
# define CONTROL_FIN_2_US_MAX 2100

# define CONTROL_FIN_3_START 0
# define CONTROL_FIN_3_FINISH 0
# define CONTROL_FIN_3_US_MIN 900
# define CONTROL_FIN_3_US_MAX 2100

# define CONTROL_FIN_4_START 0
# define CONTROL_FIN_4_FINISH 0
# define CONTROL_FIN_4_US_MIN 900
# define CONTROL_FIN_4_US_MAX 2100

# define CONTROL_ULTRAS_START 1000
# define CONTROL_ULTRAS_FINISH 1000
# define CONTROL_ULTRAS_PULSE_MIN 1000
# define CONTROL_ULTRAS_PULSE_MAX 2000

#define CONTROL_GAIN_SURGE 1.0f
#define CONTROL_GAIN_HEAVE 1.0f
#define CONTROL_GAIN_SWAY 1.0f
#define CONTROL_GAIN_ROLL 1.0f
#define CONTROL_GAIN_PITCH 1.0f
#define CONTROL_GAIN_YAW 1.0f

// Control Tune End

#define SARA_MAKE_STAT(base_stat, sub_stat) \
  (uint32_t)( \
      (((uint32_t)(sub_stat) & 0xFFFF) << 8) \
    |  ((uint32_t)(base_stat) & 0xFF)        \
   )

typedef enum e_sara_stat
{
	SARA_STAT_OK,
	SARA_STAT_ERR_SELF_NULL_PTR,
	SARA_STAT_ERR_PID_INIT_1,
	SARA_STAT_ERR_PID_INIT_2,
	SARA_STAT_ERR_PID_INIT_3,
	SARA_STAT_ERR_PID_INIT_4,
	SARA_STAT_ERR_PID_INIT_5,
	SARA_STAT_ERR_PID_INIT_6,
	SARA_STAT_ERR_PID_INIT_7,
	SARA_STAT_ERR_PID_INIT_8,
	SARA_STAT_ERR_PID_INIT_9,
	SARA_STAT_ERR_STATE_INIT_1,
	SARA_STAT_ERR_STATE_INIT_2,
	SARA_STAT_ERR_STATE_INIT_3,
	SARA_STAT_ERR_STATE_INIT_4,
	SARA_STAT_ERR_STATE_INIT_5,
	SARA_STAT_ERR_STATE_INIT_6,
	SARA_STAT_ERR_STATE_INIT_7,
	SARA_STAT_ERR_UKF_NULL_INIT,
	SARA_STAT_ERR_DVL650_INIT,
	SARA_STAT_ERR_BNO055_INIT,
	SARA_STAT_ERR_BNO055_UNIT,
	SARA_STAT_ERR_SEN0257_INIT,
	SARA_STAT_ERR_D646WP_INIT_1,
	SARA_STAT_ERR_D646WP_INIT_2,
	SARA_STAT_ERR_D646WP_INIT_3,
	SARA_STAT_ERR_D646WP_INIT_4,
	SARA_STAT_ERR_ULTRAS_INIT,
	SARA_STAT_ERR_BNO055_READ_ACCEL,
	SARA_STAT_ERR_BNO055_READ_GYRO,
	SARA_STAT_ERR_SEN0257_READ,
	SARA_STAT_ERR_DVL650_READ,
	SARA_STAT_ERR_UKF_UPDATE,
	SARA_STAT_ERR_D646WP_WRITE_1,
	SARA_STAT_ERR_D646WP_WRITE_2,
	SARA_STAT_ERR_D646WP_WRITE_3,
	SARA_STAT_ERR_D646WP_WRITE_4,
	SARA_STAT_ERR_ULTRAS_WRITE,

} t_sara_stat;

typedef struct s_dof
{
	t_pid	pid_heave_pos;
	t_pid	pid_sway_pos;
	t_pid	pid_surge_pos;
	t_pid	pid_heave_vel;
	t_pid	pid_sway_vel;
	t_pid	pid_surge_vel;
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

typedef struct s_control
{
	t_d646wp	fin_1;
	t_d646wp	fin_2;
	t_d646wp	fin_3;
	t_d646wp	fin_4;
	t_ultras	ultras;

} t_control;

typedef struct s_sara
{
	t_dof		sara_dof;
	t_state 	sara_states[STATE_COUNT];
	uint8_t		sara_state;
	t_ukf		sara_ukf;
	t_sensor	sara_sensor;
	t_control	sara_control;

} t_sara;

uint32_t	sara_init(t_sara *sara, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim_fin, TIM_HandleTypeDef *htim_ultras);
uint32_t	sara_init_dof(t_dof *sara_dof);
uint32_t	sara_init_state(t_state sara_states[STATE_COUNT]);
uint32_t	sara_init_ukf(t_ukf *sara_ukf);
uint32_t	sara_init_sensor(t_sensor *sensor, I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *hadc);
uint32_t	sara_init_control(t_control *control, TIM_HandleTypeDef *htim_fin, TIM_HandleTypeDef *htim_ultras);

uint32_t	sara_update(t_sara *sara);
uint32_t	sara_update_read(t_sensor *sensor, float read_buf[9]);
uint32_t	sara_update_write(t_control *control, float write_buf[5]);

#endif /* INC_SARA_H_ */
