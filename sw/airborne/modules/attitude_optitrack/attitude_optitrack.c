/*
 * Copyright (C) Sihao Sun
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/attitude_optitrack/attitude_optitrack.c"
 * @author Sihao Sun
 * Get attitude information from Optitrack
 */

#include "modules/attitude_optitrack/attitude_optitrack.h"
#include "subsystems/datalink/datalink.h"

#include "subsystems/abi.h"

bool attitude_optitrack_status(void)
{
	return use_attitude_optitrack;
}

void get_attitude_optitrack_init(void){

	use_attitude_optitrack = false;
}

void get_attitude_optitrack_periodic(void)
{
	use_attitude_optitrack = true;
}

void get_attitude_optitrack(void) {

	//printf("%d 	%d\n", DL_OPTITRACK_ATT_EULER_ac_id(dl_buffer), AC_ID);
	if (DL_OPTITRACK_ATT_EULER_ac_id(dl_buffer) != AC_ID) { return; } // not for this aircraft

	attitude_optitrack.phi 	= 	DL_OPTITRACK_ATT_EULER_phi_optitrack(dl_buffer);
	attitude_optitrack.theta 	= 	DL_OPTITRACK_ATT_EULER_theta_optitrack(dl_buffer);
	attitude_optitrack.psi 		=   DL_OPTITRACK_ATT_EULER_psi_optitrack(dl_buffer);

	//printf("%6.3f	%6.3f	%6.3f\n", phi*57.3, theta*57.3, psi*57.3);
}






