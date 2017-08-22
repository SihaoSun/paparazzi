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
 * @file "modules/guidance_primary_axis/guidance_primary_axis.c"
 * @author Sihao Sun
 * guidance using primary axf euler angles
 */

#include "modules/guidance_primary_axis/guidance_primary_axis.h"
#include "generated/airframe.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"
#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

bool guidance_primary_axis_status(void)
{
	return primary_axis_status;
}

void guidance_primary_axis_init(void)
{
	primary_axis_status = 0;
	return;
}
void guidance_primary_axis_run(void)
{
	primary_axis_status = 1;

	rate_cmd_primary_axis[0] = 0;
	rate_cmd_primary_axis[1] = 0;
	rate_cmd_primary_axis[2] = 0;

	return;
}
