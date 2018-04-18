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
 * @file "modules/single_failure/single_failure.c"
 * @author LeonSijbers
 * Introduce a single rotor failure
 */

#include "modules/single_failure/single_failure.h"
#include "stdio.h"


bool single_failure_status(){
	return single_failure_flag;
}

void single_failure_init(){
	single_failure_flag = 0;
    single_failure_running = FALSE;
}


void single_failure(){
	// set flag to ON
	single_failure_flag = 1;
}
// void single_failure_init() {}
// void single_failure() {}


