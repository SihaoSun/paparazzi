/*
 * Copyright (C) LeonSijbers
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
 * @file "modules/double_failure/double_failure.c"
 * @author LeonSijbers
 * Introduce a double rotor failure on opposite rotors
 */
#include "modules/double_failure/double_failure.h"
#include "stdio.h"

bool double_failure_status(){
	return double_failure_flag;
}
void double_failure_init(){
	double_failure_flag = 0;
    double_failure_running = FALSE;
}

void double_failure(){

	// set flag to ON
	double_failure_flag = 1;
}
// void double_failure_init() {}
// void double_failure() {}
