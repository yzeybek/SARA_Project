/*
 * dvl650.c
 *
 *  Created on: Jul 7, 2025
 *      Author: yzeybek
 */

#include "dvl650.h"

uint16_t	dvl650_init(t_dvl650 *dvl650)
{
	if (!dvl650)
		return (DVL650_STAT_ERR_SELF_NULL_PTR);
	(void)dvl650;
	return (DVL650_STAT_OK);
}

uint16_t	dvl650_update(t_dvl650 *dvl650)
{
	if (!dvl650)
		return (DVL650_STAT_ERR_SELF_NULL_PTR);
	(void)dvl650;
	return (DVL650_STAT_OK);
}
