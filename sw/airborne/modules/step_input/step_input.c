/*
 * Copyright (C) SihaoSun
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
 * @file "modules/step_input/step_input.c"
 * @author SihaoSun
 * Module containing step input functions with user defined magnitude and frequency
 */

#include "modules/step_input/step_input.h"
#include "stdio.h"
#include "state.h"
#include "generated/airframe.h"

bool step_input_flag;
float temp;
int8_t step_num;

void init_step_input(void){
	step_num = 0;
 	step_input_flag = false;
 	temp = 0;
}

bool step_input_status(void){
 	return step_input_flag;
}

void periodic_step_input(void){
	if (step_num <= 0)
		step_input_flag = true;
}

void call_step_input(float *output, float magnitude, float t1, float t2, float t3){

	temp += 1.0/PERIODIC_FREQUENCY;
	//printf("%f\n", temp);
	if (temp<=t1)
		*output = 0;
	else if (temp<t2)
		*output = magnitude;
	else if (temp<t3)
		*output = - magnitude;
	else
	{
		*output = 0;
		step_num++;
		temp = 0;
		step_input_flag = false;
		//init_step_input();
	}
}
