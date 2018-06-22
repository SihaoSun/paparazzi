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
float speed_sp_x;
float speed_sp_y;
float speed_sp_z;
extern bool guidance_primary_axis_status(void);
extern void guidance_primary_axis_init(void);
extern void guidance_primary_axis_end(void);
extern void guidance_primary_axis_run(void);

extern void guidance_primary_axis_take_off(void);
extern float guidance_pa_pos_gain;
extern float guidance_pa_speed_gain;
extern float guidance_pa_pos_gain_int;
extern float guidance_pa_att_gain;
extern float guidance_pa_acc_filter_fc;
extern float guidance_pa_z_gain;
extern float guidance_pa_z_gain_int;

void low_pass_filter_init(void);

struct FloatVect3 nd_state;
struct FloatVect3 nd_i_state;
struct FloatVect3 sp_accel_primary_axis;
struct FloatVect3 sp_accel_primary_axis_filter;

float nx_desire_step,ny_desire_step;
float thrust_specific_raw;
extern float vz_err_integral;

#endif
