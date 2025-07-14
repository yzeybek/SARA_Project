/*
 * state.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include "stdbool.h"
#include "stddef.h"
#include "stdint.h"
#include "helpers.h"

typedef enum e_state_stat
{
	STATE_STAT_OK,
	STATE_STAT_ERR_SELF_NULL_PTR,

} t_state_stat;

typedef struct s_range
{
	float	start;
	float	end;

}	t_range;

typedef struct s_state
{
	t_range	pos_x;
	t_range	pos_y;
	t_range	pos_z;
	t_range	vel_x;
	t_range	vel_y;
	t_range	vel_z;
	t_range	eul_x;
	t_range	eul_y;
	t_range	eul_z;

}	t_state;

uint16_t	state_init(t_state *state, t_range state_ranges[9]);
bool		state_validate(t_state desired, float currents[9]);
uint8_t		state_machine(t_state *states, uint8_t index, float currents[9]);

#endif /* INC_STATE_H_ */
