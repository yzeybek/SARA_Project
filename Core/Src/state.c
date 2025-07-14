/*
 * state.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "state.h"

uint16_t	state_init(t_state *state, t_range state_ranges[9])
{
	if (!state || !state_ranges)
		return (STATE_STAT_ERR_SELF_NULL_PTR);
	state->pos_x = state_ranges[0];
	state->pos_y = state_ranges[1];
	state->pos_z = state_ranges[2];
	state->vel_x = state_ranges[3];
	state->vel_y = state_ranges[4];
	state->vel_z = state_ranges[5];
	state->eul_x = state_ranges[6];
	state->eul_y = state_ranges[7];
	state->eul_z = state_ranges[8];
	return (STATE_STAT_OK);
}

bool	state_validate(t_state desired, float currents[9])
{
	if (currents[0] >= desired.pos_x.start && currents[0] <= desired.pos_x.end
		&& currents[1] >= desired.pos_y.start && currents[1] <= desired.pos_y.end
		&& currents[2] >= desired.pos_z.start && currents[2] <= desired.pos_z.end
		&& currents[3] >= desired.vel_x.start && currents[3] <= desired.vel_x.end
		&& currents[4] >= desired.vel_y.start && currents[4] <= desired.vel_y.end
		&& currents[5] >= desired.vel_z.start && currents[5] <= desired.vel_z.end
		&& currents[6] >= desired.eul_x.start && currents[6] <= desired.eul_x.end
		&& currents[7] >= desired.eul_y.start && currents[7] <= desired.eul_y.end
		&& currents[8] >= desired.eul_z.start && currents[8] <= desired.eul_z.end)
			return (true);
	return (false);
}

uint8_t	state_machine(t_state *states, uint8_t index, float currents[9])
{
	if (states && states[index].pos_x.start && state_validate(states[index], currents))
		return (index + 1);
	return (index);
}

