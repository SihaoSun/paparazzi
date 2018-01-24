/*
 * Copyright (C) Sihao SUn
 *
 * This file is part of paparazzi
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
 * @file "modules/sliding_mode_observer/sliding_mode_observer.c"
 * @author Sihao Sun
 * Sliding mode observer for observing model uncertainty and disturbance in the equation of motion
 */

#include "modules/sliding_mode_observer/sliding_mode_observer.h"
#include "math/pprz_algebra_float.h"
#include "filters/low_pass_filter.h"
#include "stdio.h"
#include "state.h"
#include "generated/airframe.h"

//float SMDO_t[4] = {0.10,0.10,0.10,0.05};
float SMDO_t[4] = {0.30,0.10,0.30,0.05};

float signf(float x);

struct FirstOrderLowPass nu_est0;
struct FirstOrderLowPass nu_est1;
struct FirstOrderLowPass nu_est2;
struct FirstOrderLowPass nu_est3;

bool sliding_mode_observer_status(void){
	return SMDO_status;
}

void init_sliding_mode_observer(void) {

 	SMDO_status = false;

 	float sample_time = 1.0 / PERIODIC_FREQUENCY;
	init_first_order_low_pass(&nu_est0,SMDO_t[0],sample_time,0);
	init_first_order_low_pass(&nu_est1,SMDO_t[1],sample_time,0);
	init_first_order_low_pass(&nu_est2,SMDO_t[2],sample_time,0);
	init_first_order_low_pass(&nu_est3,SMDO_t[3],sample_time,0);
	
	printf("%f\n", SMDO_t[0]);

	for (int i = 0; i < 4; i++)
	{
		SMDO_sigma_integral[i] = 0;
		SMDO_z[i] = 0;
		SMDO_nu0[i] = 0;
		SMDO_nu_est[i] = 0;
	}
	return;
}

void periodic_sliding_mode_observer(void){

	SMDO_status = true;
	return;
}

void call_sliding_mode_observer(float* z_dot,
									 float* e, float* k, float* ks, int output_num) {

	for (int i = 0; i < output_num; i++)
	{	
		SMDO_sigma_integral[i] += k[i]*e[i]/PERIODIC_FREQUENCY;
		if (i == 3){
			SMDO_sigma[i] = SMDO_sigma_integral[i];
			//printf("%f\t%f\n", SMDO_sigma_integral[i], e[i]);
		}
		else
			SMDO_sigma[i] = e[i] + SMDO_sigma_integral[i];
			
		SMDO_z[i] += z_dot[i]/PERIODIC_FREQUENCY;
		SMDO_s[i] = SMDO_z[i] + SMDO_sigma[i];
		SMDO_nu0[i] = signf(SMDO_s[i])*ks[i];
	}

	update_first_order_low_pass(&nu_est0,SMDO_nu0[0]);
	update_first_order_low_pass(&nu_est1,SMDO_nu0[1]);
	update_first_order_low_pass(&nu_est2,SMDO_nu0[2]);
	update_first_order_low_pass(&nu_est3,SMDO_nu0[3]);

	SMDO_nu_est[0] = get_first_order_low_pass(&nu_est0);
	SMDO_nu_est[1] = get_first_order_low_pass(&nu_est1);
	SMDO_nu_est[2] = get_first_order_low_pass(&nu_est2);
	SMDO_nu_est[3] = get_first_order_low_pass(&nu_est3);
	
    //printf("%f,%f,%f,%f\n",nu_est0.last_out
    //					  ,SMDO_nu_est[1]
    //					  ,SMDO_nu_est[2]
    //					  ,SMDO_nu_est[3]);
}

float signf(float x){
	if (x>=0)
 		return 1.0;
 	else 
 		return -1.0;
}

