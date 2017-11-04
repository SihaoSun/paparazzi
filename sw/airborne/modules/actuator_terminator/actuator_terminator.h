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
 * @file "modules/actuator_terminator/actuator_terminator.h"
 * @author sihao sun
 * Simple triger to terminate one actuator for damage flight test
 */

#ifndef ACTUATOR_TERMINATOR_H
#define ACTUATOR_TERMINATOR_H

#include "std.h"

//#define DAMAGED_ROTOR_INDEX 3 //0 1 2 3

bool damage_flag;
bool damage_flag2;

bool fault_limitation;
float fault_factor;

bool actuator_terminator_running;

extern void actuator_terminator(void);
extern void actuator_terminator_init(void);
extern bool damage_status(void);
extern void actuator_terminator_print(void);
#endif

