/*
 * Copyright (C) Sihao SUn
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

float signf(float x);

struct FirstOrderLowPass nu_est0,nu_est1,nu_est2,nu_est3;

bool sliding_mode_observer_status(void){
	return SMDO_status;
}

void init_sliding_mode_observer(void) {

 	SMDO_status = false;

 	float sample_time = 1.0 / PERIODIC_FREQUENCY;
	init_first_order_low_pass(&nu_est0,1.0/(2.0 * M_PI * SMDO_fc[0]),sample_time,0);
	init_first_order_low_pass(&nu_est1,1.0/(2.0 * M_PI * SMDO_fc[1]),sample_time,0);
	init_first_order_low_pass(&nu_est2,1.0/(2.0 * M_PI * SMDO_fc[2]),sample_time,0);
	init_first_order_low_pass(&nu_est3,1.0/(2.0 * M_PI * SMDO_fc[3]),sample_time,0);

	for (int i = 0; i < 4; i++)
	{
		SMDO_sigma_integral[i] = 0;
		SMDO_z[i] = 0;
	}
	return;
}

void periodic_sliding_mode_observer(void){

	SMDO_status = true;
	return;
}

void call_sliding_mode_observer(float* nu0, float* nu_est, float* z_dot,
									 float* e, float* k, float* ks, int output_num) {

	for (int i = 0; i < output_num; i++)
	{	
		SMDO_sigma_integral[i] += k[i]*e[i]/PERIODIC_FREQUENCY;
		SMDO_sigma[i] = e[i] + SMDO_sigma_integral[i];
		SMDO_z[i] += z_dot[i]/PERIODIC_FREQUENCY;
		SMDO_s[i] = SMDO_z[i] + SMDO_sigma[i];
		nu0[i] = signf(SMDO_s[i])*ks[i];
	}

	update_first_order_low_pass(&nu_est0,nu0[0]);
	update_first_order_low_pass(&nu_est1,nu0[1]);
	update_first_order_low_pass(&nu_est2,nu0[2]);
	update_first_order_low_pass(&nu_est3,nu0[3]);

	nu_est[0] = get_first_order_low_pass(&nu_est0);
	nu_est[1] = get_first_order_low_pass(&nu_est1);
	nu_est[2] = get_first_order_low_pass(&nu_est2);
	nu_est[3] = get_first_order_low_pass(&nu_est3);
	
}

float signf(float x){
 	return x/fabsf(x);
}

