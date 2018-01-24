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
 * @file "modules/step_input/step_input.h"
 * @author SihaoSun
 * Module containing step input functions with user defined magnitude and frequency
 */

#ifndef STEP_INPUT_H
#define STEP_INPUT_H

#include "std.h"

extern void init_step_input(void);
extern bool step_input_status(void);
extern void periodic_step_input(void);
extern void call_step_input(float *output, float magnitude, float t1, float t2, float t3, float t4);


#endif

