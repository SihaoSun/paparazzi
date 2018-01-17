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

#define R_MAX 35 //rad/s

float time_last_optitrack_attitude;
struct FirstOrderLowPass attitude_optitrack_filter_theta;
struct FirstOrderLowPass attitude_optitrack_filter_psi;
struct FirstOrderLowPass attitude_optitrack_filter_r;

int32_t	counter_attitude_optitrack = 0;

float theta_last_attitude_optitrack = 0;
float psi_last_attitude_optitrack = 0;  
float psi_last_step = 0;
float r_last = 0;

int32_t counter_for_psi = 0;
bool constant_r_flag = false;

bool attitude_optitrack_status(void)
{
	return use_attitude_optitrack;
}

void get_attitude_optitrack_init(void){

	use_attitude_optitrack = false;

	float tau = 0.1;
	float sample_time = 1.0 / DATALINK_NATNET_FREQUENCY;
	init_first_order_low_pass(&attitude_optitrack_filter_theta,tau,sample_time,0);
	init_first_order_low_pass(&attitude_optitrack_filter_psi  ,tau,sample_time,0);
	init_first_order_low_pass(&attitude_optitrack_filter_r ,tau,sample_time,0);	
}

void get_attitude_optitrack_periodic(void)
{
	use_attitude_optitrack = true;
}

void get_attitude_optitrack(void) {

	if (DL_OPTITRACK_ATT_EULER_ac_id(dl_buffer) != AC_ID) { return; } // not for this aircraft

	float now = get_sys_time_float();
	
	attitude_optitrack.phi 		= 	DL_OPTITRACK_ATT_EULER_phi_optitrack(dl_buffer);
	attitude_optitrack.theta 	= 	DL_OPTITRACK_ATT_EULER_theta_optitrack(dl_buffer);
	attitude_optitrack.psi 		=   DL_OPTITRACK_ATT_EULER_psi_optitrack(dl_buffer) + OPTITRACK_PSI_SHIFT_CYBERZOO;


	//update_first_order_low_pass(&attitude_optitrack_filter_theta,attitude_optitrack.theta);
	//update_first_order_low_pass(&attitude_optitrack_filter_psi,attitude_optitrack.psi);

	if (psi_last_step*attitude_optitrack.psi < -2500/(57.3*57.3))
	{
		if (psi_last_step >=0 )	 counter_for_psi ++;
		else	counter_for_psi --;
	}

	psi_last_step = attitude_optitrack.psi;
	float psi =  attitude_optitrack.psi + counter_for_psi * 2 * 3.14159;

	// Computation for r
	if (counter_attitude_optitrack % 1 == 0){
		float dtime = now - time_last_optitrack_attitude;
		float d_theta 	= (attitude_optitrack.theta - theta_last_attitude_optitrack) / dtime;
		float d_psi 	= (psi- psi_last_attitude_optitrack)/ dtime;
		
		time_last_optitrack_attitude = now;
		theta_last_attitude_optitrack =  attitude_optitrack.theta; 
		psi_last_attitude_optitrack   =	psi;

		if (dtime == 0 || dtime >= 5.0/DATALINK_NATNET_FREQUENCY){
			return;
	    }


		float r =  -sin(attitude_optitrack.phi)*d_theta 
				  	 + cos(attitude_optitrack.phi)*cos(attitude_optitrack.theta)*d_psi;

		if (r-r_last > 1.0){
			r = r_last + 1.0;
		}else if (r-r_last<-1.0){
			r = r_last - 1.0;
		}
		update_first_order_low_pass(&attitude_optitrack_filter_r,r);	
		angular_rate_optitrack.r = get_first_order_low_pass(&attitude_optitrack_filter_r);	
		r_last = r;
		//printf("%f\t%f\t%f\n", dtime, d_theta, d_psi);	
	}
 
	counter_attitude_optitrack ++;
	constant_r_flag = false;		
}






