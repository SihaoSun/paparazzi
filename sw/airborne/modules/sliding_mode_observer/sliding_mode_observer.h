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
 * @file "modules/sliding_mode_observer/sliding_mode_observer.h"
 * @author Sihao SUn
 * Sliding mode observer for observing model uncertainty and disturbance in the equation of motion
 */

#ifndef SLIDING_MODE_OBSERVER_H
#define SLIDING_MODE_OBSERVER_H

#include "std.h"

extern bool sliding_mode_observer_status(void);
extern void init_sliding_mode_observer(void);

extern void periodic_sliding_mode_observer(void);

extern void call_sliding_mode_observer(float* z_dot,
									 float* e, float* k, float* ks, int output_num);

bool SMDO_status;
float SMDO_sigma[4];
float SMDO_s[4];
float SMDO_sigma_integral[4];
float SMDO_z[4];
float SMDO_fc[4];
float SMDO_nu0[4];
float SMDO_nu_est[4];
#endif

