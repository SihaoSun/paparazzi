/*
 * Copyright (C) sihao sun
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
 * @file "modules/actuator_terminator/actuator_terminator.c"
 * @author sihao sun
 * Simple triger to terminate one actuator for damage flight test
 */

#include "modules/actuator_terminator/actuator_terminator.h"
#include "stdio.h"

bool damage_status(){
	return damage_flag;
}

void actuator_terminator_init(){
	damage_flag = 0;
	fault_limitation = 1;
	fault_factor = 0.0;	
}

void actuator_terminator(){

	//printf("this is a trial!\n");
	//actuator_pprz[3] = 5000;
	damage_flag = 1;
}

void actuator_terminator_print(){

	int a = (int) damage_status();
	printf("%d\n",a);

}





