/*
 * sara.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_SARA_H_
#define INC_SARA_H_

/* PID Coefficients Begin */

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

/* PID Coefficients End */

/* State Ranges Begin */

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
# define STATE_2_VEL_Z_START 1.0f
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

/* State Ranges End */


#endif /* INC_SARA_H_ */
