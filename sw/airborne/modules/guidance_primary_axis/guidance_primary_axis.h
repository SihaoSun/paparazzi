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

float primary_axis_n_gain_x;
float primary_axis_n_gain_y;
float primary_axis_n_abs;

float guidance_pa_att_gain;
float guidance_pa_pos_gain;

float extra_gain_att_gain;
float extra_gain_pos_gain;

// position act, ref, err
float pos_x_act;
float pos_y_act;
float pos_z_act;

float pos_x_ref;
float pos_y_ref;
float pos_z_ref;

float pos_x_err;
float pos_y_err;
float pos_z_err;

// velocity act, ref, err
float speed_x_act;
float speed_y_act;
float speed_z_act;

float speed_sp_x; //ref
float speed_sp_y;
float speed_sp_z;

float guidance_pa_pos_gain_int;

// acceleration act, ref, err

extern bool guidance_primary_axis_status(void);
extern void guidance_primary_axis_init(void);
extern void guidance_primary_axis_end(void);
extern void guidance_primary_axis_run(void);
extern void guidance_primary_axis_take_off(void);

void low_pass_filter_init(void);

struct FloatVect3 nd_state;
struct FloatVect3 nd_i_state;
struct FloatVect3 sp_accel_primary_axis;
struct FloatVect3 sp_accel_primary_axis_filter;

extern float minimum_threshold;
extern float maximum_threshold;
extern bool protect_inner_loop;

#endif

