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
 * @file "modules/attitude_optitrack/attitude_optitrack.h"
 * @author Sihao Sun
 * Get attitude information from Optitrack
 */

#include "std.h"
#include "math/pprz_algebra_float.h"

#ifndef ATTITUDE_OPTITRACK_H
#define ATTITUDE_OPTITRACK_H

struct FloatEulers attitude_optitrack;

extern void get_attitude_optitrack(void);
extern void get_attitude_optitrack_init(void);
extern void get_attitude_optitrack_periodic(void);
extern bool attitude_optitrack_status(void);

bool use_attitude_optitrack;
struct FloatVect3 nd_state;

#endif

