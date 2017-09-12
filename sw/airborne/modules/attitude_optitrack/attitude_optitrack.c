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

#include "state.h"
#include "modules/attitude_optitrack/attitude_optitrack.h"
#include "subsystems/datalink/datalink.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"
#include <stdio.h>

float time_last_optitrack_attitude;
struct FirstOrderLowPass attitude_optitrack_filter_theta;
struct FirstOrderLowPass attitude_optitrack_filter_psi;

bool attitude_optitrack_status(void)
{
	return use_attitude_optitrack;
}

void get_attitude_optitrack_init(void){

	use_attitude_optitrack = false;

	float tau = 0.01;
	float sample_time = 1.0 / DATALINK_NATNET_FREQUENCY;
	init_first_order_low_pass(&attitude_optitrack_filter_theta,tau,sample_time,0);
	init_first_order_low_pass(&attitude_optitrack_filter_psi  ,tau,sample_time,0);	
}

void get_attitude_optitrack_periodic(void)
{
	use_attitude_optitrack = true;
}

void get_attitude_optitrack(void) {

	if (DL_OPTITRACK_ATT_EULER_ac_id(dl_buffer) != AC_ID) { return; } // not for this aircraft

	float now = get_sys_time_float();

	float theta_last =  attitude_optitrack.theta; 
	float psi_last   =	attitude_optitrack.psi;

	attitude_optitrack.phi 		= 	DL_OPTITRACK_ATT_EULER_phi_optitrack(dl_buffer);
	attitude_optitrack.theta 	= 	DL_OPTITRACK_ATT_EULER_theta_optitrack(dl_buffer);
	attitude_optitrack.psi 		=   DL_OPTITRACK_ATT_EULER_psi_optitrack(dl_buffer);

	//float theta_last = get_first_order_low_pass(&attitude_optitrack_filter_theta);
	//float psi_last = get_first_order_low_pass(&attitude_optitrack_filter_psi);

	update_first_order_low_pass(&attitude_optitrack_filter_theta,attitude_optitrack.theta);
	update_first_order_low_pass(&attitude_optitrack_filter_psi,attitude_optitrack.psi);

	float dtime = now - time_last_optitrack_attitude;
	time_last_optitrack_attitude = now;

	//float d_theta = (get_first_order_low_pass(&attitude_optitrack_filter_theta) - theta_last) / dtime;
	//float d_psi   = (get_first_order_low_pass(&attitude_optitrack_filter_psi) - psi_last) / dtime;
	float d_theta 	= (attitude_optitrack.theta - theta_last) / dtime;
	float d_psi 	= (attitude_optitrack.psi 	- psi_last)	/ dtime;
	angular_rate_optitrack.r = -sin(attitude_optitrack.phi)*d_theta 
							   + cos(attitude_optitrack.phi)*cos(attitude_optitrack.theta)*d_psi;

	struct FloatRates *body_rates = stateGetBodyRates_f();
	//printf("%f 	%f\n", angular_rate_optitrack.r, body_rates->r);
	//printf("%f 	%f\n", now,dtime);	
	printf("%f 	%f\n", d_psi, d_theta);		
}






