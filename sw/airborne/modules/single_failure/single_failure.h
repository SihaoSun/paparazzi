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
 * @file "modules/single_failure/single_failure.h"
 * @author LeonSijbers
 * Introduce a single rotor failure
 */
#ifndef SINGLE_FAILURE_H
#define SINGLE_FAILURE_H

#include "std.h"
// extern void single_failure_init();
// extern void single_failure();

// variables used
bool 	single_failure_flag;
bool    single_failure_running;

extern void single_failure_init(void);
extern void single_failure(void);
extern bool single_failure_status(void);
#endif

