/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"
#include "boards/bebop/actuators.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "modules/attitude_optitrack/attitude_optitrack.h"
#include "modules/guidance_primary_axis/guidance_primary_axis.h"
#include "modules/sliding_mode_observer/sliding_mode_observer.h"
/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,"
      "mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz,"
      "w1obs,w2obs,w3obs,w4obs,w1ref,w2ref,w3ref,w4ref,w1obs_indi,w2obs_indi,w3obs_indi,w4obs_indi,"
      "p,q,r,phi,theta,psi,Acc^b_x,Acc^b_y,Acc^b_z,phi_ot,theta_ot,psi_ot,r_ot,"
      "p_des,q_des,r_des,h1,h2,ndi_x,ndi_y,ndi_z,acc_des_x,acc_des_y,acc_des_z,acc_des_x_filter,acc_des_y_filter,acc_des_z_filter,"
      "p_des_dot, q_des_dot, r_des_dot, p_des_filter, q_des_filter, r_des_filter,"
      "nu0_1,nu0_2,nu0_3,nu0_4,nu_est1,nu_est2,nu_est3,nu_est4,PSI0_1,PSI0_2,PSI0_3,PSI0_4,z_dot1,z_dot2,z_dot3,z_dot4,sigma1,sigma2,sigma3,sigma4,s1,s2,s3,s4,nx_des,ny_des,e1,e2,e3,e4"
      "z_ref,z,du1,du2,du3,du4,Vx,Vy,Vz,Vx_des,Vy_des,Vz_des,nu1,nu2,nu3,x0_dot1,x0_dot2,x0_dot3\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
          counter,
          imu.gyro_unscaled.p,
          imu.gyro_unscaled.q,
          imu.gyro_unscaled.r,
          imu.accel_unscaled.x,
          imu.accel_unscaled.y,
          imu.accel_unscaled.z,
          imu.mag_unscaled.x,
          imu.mag_unscaled.y,
          imu.mag_unscaled.z,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz,
          (int)actuators_bebop.rpm_obs[0],
          (int)actuators_bebop.rpm_obs[1],
          (int)actuators_bebop.rpm_obs[2],
          (int)actuators_bebop.rpm_obs[3],
          (int)actuators_bebop.rpm_ref[0],
          (int)actuators_bebop.rpm_ref[1],
          (int)actuators_bebop.rpm_ref[2],
          (int)actuators_bebop.rpm_ref[3],
          act_obs[0],
          act_obs[1],
          act_obs[2],
          act_obs[3],
          stateGetBodyRates_f()->p,
          stateGetBodyRates_f()->q,
          stateGetBodyRates_f()->r,
          stateGetNedToBodyEulers_f()->phi,
          stateGetNedToBodyEulers_f()->theta,
          stateGetNedToBodyEulers_f()->psi,
          stateGetAccelBody_i()->x,
          stateGetAccelBody_i()->y,
          stateGetAccelBody_i()->z,
          attitude_optitrack.phi,
          attitude_optitrack.theta,
          attitude_optitrack.psi,
          angular_rate_optitrack.r,
          rate_cmd_primary_axis[0],
          rate_cmd_primary_axis[1],
          rate_cmd_primary_axis[2],
          nd_state.x,
          nd_state.y,
          nd_i_state.x,
          nd_i_state.y,
          nd_i_state.z,
          sp_accel_primary_axis.x,
          sp_accel_primary_axis.y,
          sp_accel_primary_axis.z,
          sp_accel_primary_axis_filter.x,
          sp_accel_primary_axis_filter.y,
          sp_accel_primary_axis_filter.z,
          p_des_dot_logger,
          q_des_dot_logger,
          r_des_dot_logger,
          p_des_filter_logger,
          q_des_filter_logger,
          r_des_filter_logger,
          SMDO_nu0[0],
          SMDO_nu0[1],
          SMDO_nu0[2],
          SMDO_nu0[3],
          SMDO_nu_est[0],
          SMDO_nu_est[1],
          SMDO_nu_est[2],
          SMDO_nu_est[3],
          NDI_PSI0[0],
          NDI_PSI0[1],
          NDI_PSI0[2],
          NDI_PSI0[3],
          SMDO_z_dot[0],
          SMDO_z_dot[1],
          SMDO_z_dot[2],
          SMDO_z_dot[3],
          SMDO_sigma[0],
          SMDO_sigma[1],
          SMDO_sigma[2],
          SMDO_sigma[3],
          SMDO_s[0],
          SMDO_s[1],
          SMDO_s[2],
          SMDO_s[3],
          nx_desire_step,
          ny_desire_step,
          Error[0],
          Error[1],
          Error[2],
          Error[3],
          POS_FLOAT_OF_BFP(guidance_v_z_ref),
          POS_FLOAT_OF_BFP(stateGetPositionNed_i()->z),
          du_log[0],
          du_log[1],
          du_log[2],
          du_log[3],
          stateGetSpeedNed_f()->x,
          stateGetSpeedNed_f()->y,
          stateGetSpeedNed_f()->z,
          speed_sp_x,
          speed_sp_y,
          speed_sp_z,
          angular_accel_ref.p,
          angular_accel_ref.q,
          angular_accel_ref.r,
          angular_acceleration[0],
          angular_acceleration[1],
          angular_acceleration[2]
         );
  counter++;
}
