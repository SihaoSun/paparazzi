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
 * @file "modules/guidance_primary_axis/guidance_primary_axis.h"
 * @author Sihao Sun
 * guidance using primary axf euler angles
 */



#ifndef GUIDANCE_PRIMARY_AXIS_H
#define GUIDANCE_PRIMARY_AXIS_H

#include "std.h"

bool primary_axis_status;
float rate_cmd_primary_axis[3];
float thrust_primary_axis;

extern bool guidance_primary_axis_status(void);
extern void guidance_primary_axis_init(void);
extern void guidance_primary_axis_end(void);
extern void guidance_primary_axis_run(void);
void low_pass_filter_init(void);
#endif

