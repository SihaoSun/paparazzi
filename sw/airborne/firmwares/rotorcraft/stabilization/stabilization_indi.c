/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
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
 */

/** @file stabilization_attitude_quat_indi.c
 * @brief MAVLab Delft University of Technology
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/radio_control.h"
#include "subsystems/actuators.h"
#include "subsystems/abi.h"
#include "wls/wls_alloc.h"
#include "modules/actuator_terminator/actuator_terminator.h"
#include "modules/sliding_mode_observer/sliding_mode_observer.h"
#include "modules/attitude_optitrack/attitude_optitrack.h"
#include <stdio.h>

//only 4 actuators supported for now
#define INDI_NUM_ACT 4
// outputs: roll, pitch, yaw, thrust
#define INDI_OUTPUTS 4
// Factor that the estimated G matrix is allowed to deviate from initial one
#define INDI_ALLOWED_G_FACTOR 2.0
// Scaling for the control effectiveness to make it readible
#define INDI_G_SCALING 1000.0

float du_min[INDI_NUM_ACT];
float du_max[INDI_NUM_ACT];
float du_pref[INDI_NUM_ACT];
float indi_v[INDI_OUTPUTS];
float *Bwls[INDI_OUTPUTS];
int num_iter = 0;

static void lms_estimation(void);
static void get_actuator_state(void);
static void calc_g1_element(float dx_error, int8_t i, int8_t j, float mu_extra);
static void calc_g2_element(float dx_error, int8_t j, float mu_extra);
static void calc_g1g2_pseudo_inv(void);
static void bound_g_mat(void);
static void calc_g1_inv_damage(void);

int32_t stabilization_att_indi_cmd[COMMANDS_NB];
struct ReferenceSystem reference_acceleration = {
  STABILIZATION_INDI_REF_ERR_P,
  STABILIZATION_INDI_REF_ERR_Q,
  STABILIZATION_INDI_REF_ERR_R,
  STABILIZATION_INDI_REF_RATE_P,
  STABILIZATION_INDI_REF_RATE_Q,
  STABILIZATION_INDI_REF_RATE_R,
};

#if STABILIZATION_INDI_USE_ADAPTIVE
bool indi_use_adaptive = true;
#else
bool indi_use_adaptive = false;
#endif

bool indi_use_ndi = false;

#ifdef STABILIZATION_INDI_ACT_RATE_LIMIT
float act_rate_limit[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_RATE_LIMIT;
#endif

#ifdef STABILIZATION_INDI_ACT_IS_SERVO
bool act_is_servo[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_IS_SERVO;
#else
bool act_is_servo[INDI_NUM_ACT] = {0};
#endif

#ifdef STABILIZATION_INDI_ACT_PREF
// Preferred (neutral, least energy) actuator value
float act_pref[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_PREF;
#else
// Assume 0 is neutral
float act_pref[INDI_NUM_ACT] = {0.0};
#endif

float act_dyn[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_DYN;

/** Maximum rate you can request in RC rate mode (rad/s)*/
#ifndef STABILIZATION_INDI_MAX_RATE
#define STABILIZATION_INDI_MAX_RATE 6.0
#endif

// variables needed for control
float actuator_state_filt_vect[INDI_NUM_ACT];
struct FloatRates angular_accel_ref = {0., 0., 0.};
float angular_acceleration[3] = {0., 0., 0.};
float actuator_state[INDI_NUM_ACT];
float indi_u[INDI_NUM_ACT];
float indi_du[INDI_NUM_ACT];
float g2_times_du;

// variables needed for estimation
float g1g2_trans_mult[INDI_OUTPUTS][INDI_OUTPUTS];
float g1g2inv[INDI_OUTPUTS][INDI_OUTPUTS];
float actuator_state_filt_vectd[INDI_NUM_ACT];
float actuator_state_filt_vectdd[INDI_NUM_ACT];
float estimation_rate_d[INDI_NUM_ACT];
float estimation_rate_dd[INDI_NUM_ACT];
float du_estimation[INDI_NUM_ACT];
float ddu_estimation[INDI_NUM_ACT];

// The learning rate per axis (roll, pitch, yaw, thrust)
float mu1[INDI_OUTPUTS] = {0.00001, 0.00001, 0.000003, 0.000002};
// The learning rate for the propeller inertia (scaled by 512 wrt mu1)
float mu2 = 0.002;

// Number of actuators used to provide thrust
int32_t num_thrusters;

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_sp_quat;

abi_event rpm_ev;
static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t num_act);

abi_event thrust_ev;
static void thrust_cb(uint8_t sender_id, float thrust_increment);
float indi_thrust_increment;
bool indi_thrust_increment_set = false;

float g1g2_pseudo_inv[INDI_NUM_ACT][INDI_OUTPUTS];
float g2[INDI_NUM_ACT] = STABILIZATION_INDI_G2; //scaled by INDI_G_SCALING
float g1[INDI_OUTPUTS][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL,
                                        STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST
                                       };
float g1g2[INDI_OUTPUTS][INDI_NUM_ACT];
float g1_est[INDI_OUTPUTS][INDI_NUM_ACT];
float g2_est[INDI_NUM_ACT];
float g1_init[INDI_OUTPUTS][INDI_NUM_ACT];
float g2_init[INDI_NUM_ACT];
float g1_damage[3][3];
float g1_damage_inv[3][3];

float p_des_dot;
float q_des_dot;
float r_des_dot;
double ET_integral;

Butterworth2LowPass actuator_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass estimation_input_lowpass_filters[INDI_NUM_ACT];
Butterworth2LowPass measurement_lowpass_filters[3];
Butterworth2LowPass estimation_output_lowpass_filters[3];
Butterworth2LowPass acceleration_lowpass_filter;
Butterworth2LowPass az_lowpass_filter;
Butterworth2LowPass p_des_filter;
Butterworth2LowPass q_des_filter;
Butterworth2LowPass r_des_filter;
Butterworth2LowPass rate_lowpass_filters[3];

struct FloatVect3 body_accel_f;

void init_filters(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_indi_g(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INDI_G(trans, dev, AC_ID, INDI_NUM_ACT, g1_est[0],
                       INDI_NUM_ACT, g1_est[1],
                       INDI_NUM_ACT, g1_est[2],
                       INDI_NUM_ACT, g1_est[3],
                       INDI_NUM_ACT, g2_est);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_sp_quat.qi,
                              &stab_att_sp_quat.qx,
                              &stab_att_sp_quat.qy,
                              &stab_att_sp_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}
#endif

/**
 * Function that initializes important values upon engaging INDI
 */
void stabilization_indi_init(void)
{
  // Initialize filters
  init_filters();

  AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_ev, rpm_cb);
  AbiBindMsgTHRUST(THRUST_INCREMENT_ID, &thrust_ev, thrust_cb);

  float_vect_zero(actuator_state_filt_vectd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vectdd, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_d, INDI_NUM_ACT);
  float_vect_zero(estimation_rate_dd, INDI_NUM_ACT);
  float_vect_zero(actuator_state_filt_vect, INDI_NUM_ACT);

  //Calculate G1G2_PSEUDO_INVERSE
  calc_g1g2_pseudo_inv();
  calc_g1_inv_damage();

  // Initialize the array of pointers to the rows of g1g2
  uint8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // Initialize the estimator matrices
  float_vect_copy(g1_est[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2_est, g2, INDI_NUM_ACT);
  // Remember the initial matrices
  float_vect_copy(g1_init[0], g1[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2_init, g2, INDI_NUM_ACT);

  // Assume all non-servos are delivering thrust
  num_thrusters = INDI_NUM_ACT;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    num_thrusters -= act_is_servo[i];
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INDI_G, send_indi_g);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_REF_QUAT, send_ahrs_ref_quat);
#endif

  ET_integral = 0;
}

/**
 * Function that resets important values upon engaging INDI.
 *
 * Don't reset inputs and filters, because it is unlikely to switch stabilization in flight,
 * and there are multiple modes that use (the same) stabilization. Resetting the controller
 * is not so nice when you are flying.
 * FIXME: Ideally we should detect when coming from something that is not INDI
 */
void stabilization_indi_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  float_vect_zero(du_estimation, INDI_NUM_ACT);
  float_vect_zero(ddu_estimation, INDI_NUM_ACT);
}          

/**
 * Function that resets the filters to zeros
 */
void init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_FILT_CUTOFF);
  float tau_est = 1.0 / (2.0 * M_PI * STABILIZATION_INDI_ESTIMATION_FILT_CUTOFF);
  float tau_pq_des = 1.0 / (2.0 * M_PI *20.0);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  float tau_pqr_mea = 1.0 / (2.0 * M_PI * 30.0);
  // Filtering of the gyroscope
  int8_t i;
  for (i = 0; i < 3; i++) {
    init_butterworth_2_low_pass(&measurement_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
    init_butterworth_2_low_pass(&actuator_lowpass_filters[i], tau, sample_time, 0.0);
    init_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], tau_est, sample_time, 0.0);
  }

  // Filtering of the accel body z
  init_butterworth_2_low_pass(&acceleration_lowpass_filter, tau_est, sample_time, 0.0); 
  init_butterworth_2_low_pass(&az_lowpass_filter, tau_est*1.2, sample_time, 0.0);

  // Filtering of the p q designed value from Praimary Guidance
  init_butterworth_2_low_pass(&p_des_filter, tau_pq_des, sample_time, 0.0);
  init_butterworth_2_low_pass(&q_des_filter, tau_pq_des, sample_time, 0.0);
  init_butterworth_2_low_pass(&r_des_filter, tau_pq_des, sample_time, 0.0);

  // Filtering of the p q r measured value
  for (int i = 0; i < 3; ++i)
  {
    init_butterworth_2_low_pass(&rate_lowpass_filters[i], tau_pqr_mea, sample_time, 0.0);
  }
}

/**
 * Function that calculates the failsafe setpoint
 */
void stabilization_indi_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

/**
 * @param rpy rpy from which to calculate quaternion setpoint
 *
 * Function that calculates the setpoint quaternion from rpy
 */
void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

/**
 * @param cmd 2D command in North East axes
 * @param heading Heading of the setpoint
 *
 * Function that calculates the setpoint quaternion from a command in earth axes
 */
void stabilization_indi_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

/**
 * @param att_err attitude error
 * @param rate_control boolean that states if we are in rate control or attitude control
 * @param in_flight boolean that states if the UAV is in flight or not
 *
 * Function that calculates the INDI commands
 */
static void stabilization_indi_calc_cmd(struct Int32Quat *att_err, bool rate_control, bool in_flight)
{

  struct FloatRates rate_ref;
  if (rate_control) { //Check if we are running the rate controller
    rate_ref.p = (float)radio_control.values[RADIO_ROLL]  / MAX_PPRZ * STABILIZATION_INDI_MAX_RATE;
    rate_ref.q = (float)radio_control.values[RADIO_PITCH] / MAX_PPRZ * STABILIZATION_INDI_MAX_RATE;
    rate_ref.r = (float)radio_control.values[RADIO_YAW]   / MAX_PPRZ * STABILIZATION_INDI_MAX_RATE;
  } else if(guidance_primary_axis_status()==true){
    rate_ref.p = rate_cmd_primary_axis[0];
    rate_ref.q = rate_cmd_primary_axis[1];
    rate_ref.r = rate_cmd_primary_axis[2];
//    rate_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
//                 / reference_acceleration.rate_r;
  }else{//calculate the virtual control (reference acceleration) based on a PD controller
          rate_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                   / reference_acceleration.rate_p;
      rate_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                   / reference_acceleration.rate_q;
      rate_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                 / reference_acceleration.rate_r;      
    // Possibly we can use some bounding here
    /*BoundAbs(rate_ref.r, 5.0);*/
  }

  struct FloatRates *body_rates = stateGetBodyRates_f();
  float p_filter,q_filter,r_filter;
  p_filter = rate_lowpass_filters[0].o[0];
  q_filter = rate_lowpass_filters[1].o[0];
  r_filter = rate_lowpass_filters[2].o[0];

  //float Error[4];
  if (abs(r_filter)>=10)
  {
    Error[0] = rate_ref.p - (p_filter+0.4);
    Error[1] = rate_ref.q - (q_filter+0.5);
    Error[2] = rate_ref.r - (r_filter+0.2);
  }
  else{
    Error[0] = rate_ref.p - p_filter;
    Error[1] = rate_ref.q - q_filter;
    Error[2] = rate_ref.r - r_filter;
  }
  
  //Error[0] = rate_ref.p-body_rates->p;
  //Error[1] = rate_ref.q-body_rates->q;
  Error[2] = rate_ref.r-body_rates->r;
  
  //calculate the virtual control (reference acceleration) based on a PD controller
  if(guidance_primary_axis_status()==true) {
      update_butterworth_2_low_pass(&p_des_filter,rate_ref.p);
      update_butterworth_2_low_pass(&q_des_filter,rate_ref.q);
      update_butterworth_2_low_pass(&r_des_filter,rate_ref.r);      
      p_des_dot = (p_des_filter.o[0] - p_des_filter.o[1])*PERIODIC_FREQUENCY;
      q_des_dot = (q_des_filter.o[0] - q_des_filter.o[1])*PERIODIC_FREQUENCY;
      r_des_dot = (r_des_filter.o[0] - r_des_filter.o[1])*PERIODIC_FREQUENCY;

      //angular_accel_ref.p = (rate_ref.p - body_rates->p) * reference_acceleration.rate_p + p_des_dot ;
      //angular_accel_ref.q = (rate_ref.q - body_rates->q) * reference_acceleration.rate_q + q_des_dot ;    

      angular_accel_ref.p = Error[0] * reference_acceleration.rate_p + p_des_dot ;
      angular_accel_ref.q = Error[1] * reference_acceleration.rate_q + q_des_dot ;    

      p_des_dot_logger = p_des_dot;
      q_des_dot_logger = q_des_dot;
      r_des_dot_logger = r_des_dot;

      p_des_filter_logger = p_des_filter.o[0];
      q_des_filter_logger = q_des_filter.o[0]; 
      r_des_filter_logger = r_des_filter.o[0];  
  }
  else {
      angular_accel_ref.p = (rate_ref.p - body_rates->p) * reference_acceleration.rate_p;
      angular_accel_ref.q = (rate_ref.q - body_rates->q) * reference_acceleration.rate_q;
  }

  if (attitude_optitrack_status() == false) 
    angular_accel_ref.r = (rate_ref.r - body_rates->r) * reference_acceleration.rate_r;
  else{
    if (body_rates->r < 35.0) // gyroscope limitation on Bebop2, 2000deg/sec
      angular_accel_ref.r = (rate_ref.r - body_rates->r) * reference_acceleration.rate_r;
    else
      angular_accel_ref.r = (rate_ref.r - angular_rate_optitrack.r) * reference_acceleration.rate_r;
  }

  int8_t i,j;
  g2_times_du = 0.0;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    g2_times_du += g2[i] * indi_du[i];
  }
  //G2 is scaled by INDI_G_SCALING to make it readable
  g2_times_du = g2_times_du / INDI_G_SCALING;

  float rpm_cmd_pa;
  float thrust_pprz_cmd;
//////////////////////////////// INDI ////////////////////////////////////////
  float v_thrust = 0.0;
  if (indi_thrust_increment_set*0) {
//  if (indi_thrust_increment_set) {    
    v_thrust = indi_thrust_increment;

    //update thrust command such that the current is correctly estimated
    stabilization_cmd[COMMAND_THRUST] = 0;
    for (i = 0; i < INDI_NUM_ACT; i++) {
      stabilization_cmd[COMMAND_THRUST] += actuator_state[i] * -((int32_t) act_is_servo[i] - 1);

    }
    stabilization_cmd[COMMAND_THRUST] /= num_thrusters;

  }
#if PRIMARY_AXIS_THRUST_COMMAND
#warning "Thrust command from primary axis guidance, which may cause oscilation"   
  else if (guidance_primary_axis_status() == true){
      // Get the acceleration in body axes
      struct Int32Vect3 *body_accel_i;
      body_accel_i = stateGetAccelBody_i();
      ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

      // Filter the acceleration in z axis
      update_butterworth_2_low_pass(&az_lowpass_filter, body_accel_f.z);

      if (thrust_primary_axis<=0)
        thrust_primary_axis = 0;
      //rpm_cmd_pa = sqrtf(thrust_primary_axis * 4.7197e+06/0.8);
      rpm_cmd_pa = sqrtf(thrust_primary_axis * 4.7197e+06/0.5);
      //rpm_cmd_pa_fb = -3000*(POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z));
      float rpm_cmd_pa_fb = -1000 * vz_err_integral;
      
      if (autopilot.mode != AP_MODE_ATTITUDE_DIRECT)
      {
        rpm_cmd_pa += rpm_cmd_pa_fb;
      }
      thrust_pprz_cmd = (rpm_cmd_pa- get_servo_min(0));
      thrust_pprz_cmd *= (MAX_PPRZ / (float)(get_servo_max(0) - get_servo_min(0)));  

      //printf("%f\t%f\t%f\n",rpm_cmd_pa,rpm_cmd_pa_fb,vz_err_integral);

      for (i = 0; i < 4; ++i)
      {
        v_thrust += (thrust_pprz_cmd - actuator_state_filt_vect[i]) * Bwls[3][i];
      }

    }
#endif  
  else {
    // incremental thrust
    for (i = 0; i < INDI_NUM_ACT; i++) {
      v_thrust +=
        (stabilization_cmd[COMMAND_THRUST] - actuator_state_filt_vect[i]) * Bwls[3][i];
    }
  }

  //printf("%f\n", v_thrust);
  // Calculate the min and max increments
  for (i = 0; i < INDI_NUM_ACT; i++) {
     du_min[i] = -MAX_PPRZ * act_is_servo[i] - actuator_state_filt_vect[i];
     du_max[i] = MAX_PPRZ - actuator_state_filt_vect[i];
     du_pref[i] = act_pref[i] - actuator_state_filt_vect[i];
  }

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100};


  // The control objective in array format
  indi_v[0] = (angular_accel_ref.p - angular_acceleration[0]);
  indi_v[1] = (angular_accel_ref.q - angular_acceleration[1]);
  indi_v[2] = (angular_accel_ref.r - angular_acceleration[2] + g2_times_du);
  indi_v[3] = v_thrust;

#if SMC_TEST_NOMINAL
  #if SMC_TEST_NDI_USED
  float k_sigma[4]  = {1.0,1.0,1.0,1.5}; //ndi nominal
  indi_use_ndi =   1-autopilot_mode_status;
  #else
//  float k_sigma[4] = {0.5,0.5,0.5,1.0}; //indi nominal
  float k_sigma[4] = {0.5,0.5,0.5,1.0}; //indi nominal
  #endif
#else
  #if SMC_TEST_NDI_USED
  float k_sigma[4]  = {1.2,1.2,1.2,1.6}; //ndi
  indi_use_ndi =   1-autopilot_mode_status;
  #else
  float k_sigma[4] = {0.5,0.5,0.5,1.0}; //indi
  #endif
#endif
  
  if(autopilot_mode_status == 1)
    SMDO_nu_est[3] = 0;

  if (sliding_mode_observer_status() == true)
  {
    for (i = 0; i < 4; ++i)
    {
      indi_v[i] += SMDO_nu_est[i] + k_sigma[i]*SMDO_sigma[i];
    }    
  }


#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
  // Calculate the increment for each actuator
  for (i = 0; i < INDI_NUM_ACT; i++) {
    indi_du[i] = (g1g2_pseudo_inv[i][0] * indi_v[0])
                 + (g1g2_pseudo_inv[i][1] * indi_v[1])
                 + (g1g2_pseudo_inv[i][2] * indi_v[2])
                 + (g1g2_pseudo_inv[i][3] * indi_v[3]);
  }

  if (damage_status())
  {
    indi_du[DAMAGED_ROTOR_INDEX] = 0;
    int8_t i0 = 0;
    for (i = 0; i < INDI_NUM_ACT; i++){
      if (i != DAMAGED_ROTOR_INDEX) {
        indi_du[i] = (g1_damage_inv[i0][0] * indi_v[0])
                    +(g1_damage_inv[i0][1] * indi_v[1])
                    +(g1_damage_inv[i0][2] * indi_v[3]);
      i0++;
      }
    }
  }
#else
  // WLS Control Allocator
  num_iter =
    wls_alloc(indi_du, indi_v, du_min, du_max, Bwls, INDI_NUM_ACT, INDI_OUTPUTS, 0, 0, Wv, 0, du_min, 10000, 10);
#endif


  // Compile this part if omega^2 is used as system input. The g1 matrix should be
  // set the same as NDI and g2 matrix should be set as zero. 
#ifdef INDI_USE_OMEGA_SQUARE
    for (int i = 0; i < 4; ++i)
    {
      if (indi_du > 0)
        indi_du = sqrtf(indi_du)*(MAX_PPRZ / (float)(get_servo_max(i) - get_servo_min(i)));
      else
        indi_du = sqrtf(-indi_du)*(MAX_PPRZ / (float)(get_servo_max(i) - get_servo_min(i)));
    }
#endif

  // Add the increments to the actuators
  float_vect_sum(indi_u, actuator_state_filt_vect, indi_du, INDI_NUM_ACT);

  for (i = 0; i < 4; ++i)
  {
    du_log[i] = indi_du[i];
  }

  // Bound the inputs to the actuators
  for (i = 0; i < INDI_NUM_ACT; i++) {
       if (act_is_servo[i]) {
         BoundAbs(indi_u[i], MAX_PPRZ);
       } else {
         Bound(indi_u[i], 0, MAX_PPRZ);
       }
  }
  //Don't increment if not flying (not armed)
  if (!in_flight) {
    float_vect_zero(indi_u, INDI_NUM_ACT);
    float_vect_zero(indi_du, INDI_NUM_ACT);
  }

  // Propagate actuator filters
  get_actuator_state();

  for (i = 0; i < INDI_NUM_ACT; i++) {
    update_butterworth_2_low_pass(&actuator_lowpass_filters[i], actuator_state[i]);
    update_butterworth_2_low_pass(&estimation_input_lowpass_filters[i], actuator_state[i]);
    actuator_state_filt_vect[i] = actuator_lowpass_filters[i].o[0];

    // calculate derivatives for estimation
    float actuator_state_filt_vectd_prev = actuator_state_filt_vectd[i];
    actuator_state_filt_vectd[i] = (estimation_input_lowpass_filters[i].o[0] - estimation_input_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;
    actuator_state_filt_vectdd[i] = (actuator_state_filt_vectd[i] - actuator_state_filt_vectd_prev) * PERIODIC_FREQUENCY;
  }

  // Use online effectiveness estimation only when flying
  if (in_flight && indi_use_adaptive) {
    lms_estimation();
  }

  /*Commit the actuator command*/
  for (i = 0; i < INDI_NUM_ACT; i++) {
    actuators_pprz[i] = (int16_t) indi_u[i];
    //actuators_pprz[i] = -MAX_PPRZ;
    if ((i == DAMAGED_ROTOR_INDEX ) && damage_status()){
      actuators_pprz[i] = -MAX_PPRZ;
    }
  }

  
//////////////////////////////// NDI ////////////////////////////////////////


    if(indi_use_ndi == true){
      float nu[4],omega_ref;
      //float kp,kq,kr;
      //kp = 30.0;
      //kq = 30.0;
      //kr = 20.0;
      float Ix, Iy,Iz;
      Ix = 0.0016616;
      Iy = 0.0013659;
      Iz = 0.0028079;

      //printf("%f  %f\n", (float)stabilization_cmd[COMMAND_THRUST], omega_ref);
      nu[0] = reference_acceleration.rate_p*Error[0] + p_des_dot;
      nu[1] = reference_acceleration.rate_q*Error[1] + q_des_dot;
      nu[2] = reference_acceleration.rate_r*Error[2] + r_des_dot;
      nu[3] = rpm_cmd_pa * rpm_cmd_pa/1e6;

      nu[0] -= body_rates->q*body_rates->r*(Iz-Iy)/Ix;
      nu[1] -= body_rates->q*body_rates->r*(Iz-Iy)/Ix;
      nu[2] -= body_rates->p*body_rates->q*(Iy-Ix)/Iz;

    //  float G_inv[4][4]={  0.3333,    0.3289,   8.5997,    1.0000,
    //                      -0.3333,    0.3289,  -8.5997,    1.0000,
    //                      -0.3333,   -0.3289,   8.5997,    1.0000,
    //                       0.3333,   -0.3289,  -8.5997,    1.0000};
    //  float G[4][4] = {       0.7501,   -0.7501,   -0.7501,    0.7501,
    //                          0.7601,    0.7601,   -0.7601,   -0.7601,
    //                          0.0291,   -0.0291,    0.0291,   -0.0291,
    //                          0.2500,    0.2500,    0.2500,    0.2500}; //G is scaled by 1e6
      //w0 = 9500
//      float G_inv[4][4]={     0.3111,    0.3333,    1.7781,    1.0000,
//                             -0.3111,    0.3333,   -1.7781,    1.0000,
//                             -0.3111,   -0.3333,    1.7781,    1.0000,
//                              0.3111,   -0.3333,   -1.7781,    1.0000};                              
//      float G[4][4] = {       0.8035,   -0.8035,   -0.8035,    0.8035,
//                              0.7501,    0.7501,   -0.7501,   -0.7501,
//                              0.1406,   -0.1406,    0.1406,   -0.1406,
//                              0.2500,    0.2500,    0.2500,    0.2500}; //G is scaled by 1e6

      //w0 = 8000
      float G_inv[4][4]={    0.2620,    0.2807,    1.4973,    1.0000,
                            -0.2620,    0.2807,   -1.4973,    1.0000,
                            -0.2620,   -0.2807,    1.4973,    1.0000,
                             0.2620,   -0.2807,   -1.4973,    1.0000}; 

      float G[4][4] = {      0.9542,   -0.9542,   -0.9542,    0.9542,
                             0.8907,    0.8907,   -0.8907,   -0.8907,
                             0.1670,   -0.1670,    0.1670,   -0.1670,
                             0.2500,    0.2500,    0.2500,    0.2500}; //G is scaled by 1e6


      //w0 = 7000
//      float G_inv[4][4]={  0.2333,    0.2500,   1.3998,    1.0000,
//                          -0.2333,    0.2500,  -1.3998,    1.0000,
//                          -0.2333,   -0.2500,   1.3998,    1.0000,
//                           0.2333,   -0.2500,  -1.3998,    1.0000};
//      float G[4][4] = {       1.0714,   -1.0714,   -1.0714,    1.0714,
//                              1.0,       1.0,      -1.0,      -1.0,
//                              0.1786,   -0.1786,    0.1786,   -0.1786,
//                              0.2500,    0.2500,    0.2500,    0.2500}; //G is scaled by 1e6
  
      float w2[4] = {0.0,0.0,0.0,0.0}; //rpm^2
      float w[4] = {0.0,0.0,0.0,0.0}; //rpm

      for (i = 0; i < 4; i++)
      {
        NDI_PSI0[i] = nu[i];
        for (j = 0; j < 4; j++)
        { 
          if (sliding_mode_observer_status() == false)
            w2[i] += G_inv[i][j]*(nu[j])*1e6;
          else
            w2[i] += G_inv[i][j]*(nu[j] + SMDO_nu_est[j] + k_sigma[j]*SMDO_sigma[j])*1e6;
        }

//        if (w2[i] <= (float)(get_servo_min(i))*(float)(get_servo_min(i)))
//          w2[i] = (float)(get_servo_min(i))*(float)(get_servo_min(i));
//        w[i] = sqrtf(w2[i]);
        if (w2[i] <= 2500*2500)
          w2[i] = 2500*2500;
        w[i] = sqrtf(w2[i]);        
      }
      
      float act_cmd[4];
      for (i = 0; i < 4; i++) {
        act_cmd[i] = (w[i] - (float)get_servo_min(i));
        act_cmd[i] *= (MAX_PPRZ / (float)(get_servo_max(i) - get_servo_min(i)));
        Bound(act_cmd[i], 0, MAX_PPRZ);
        actuators_pprz[i] = (int16_t)act_cmd[i];
      }

        //Call SMDC. Now psi0 = nu;
        //float Ks[4] = {5.0,5.0,5.0,10.0};
        for (i = 0; i < 4; ++i)
        {
          SMDO_z_dot[i] = -nu[i] - SMDO_nu0[i]; 
          for (j = 0; j < 4; ++j)
          {
            SMDO_z_dot[i] += G[i][j]*w2[j]*1e-6;
          } 
        }

        float X_thrust = 0;
        for (int i = 0; i < 4; ++i)
        {
          float X = actuator_state_filt_vect[i]*((float)(get_servo_max(i) - get_servo_min(i)))/(float)MAX_PPRZ
                      + (float)get_servo_min(i);
          X_thrust += X*X;
        }
        Error[3] = rpm_cmd_pa*rpm_cmd_pa/1e6 - X_thrust/4e6;
#if SMC_TEST_NOMINAL
        //float Ks[4] = {30,30,30,20}; // nominal
        float Ks[4] = {60,60,20,20};
#else
        float Ks[4] = {180,180,30,30}; // damage_status
#endif
        float K[4] = {reference_acceleration.rate_p,reference_acceleration.rate_q,reference_acceleration.rate_r, 1.0};
        if (sliding_mode_observer_status() == true){
          call_sliding_mode_observer(SMDO_z_dot, Error, K, Ks, 4);
        }
    }
    else{
        // call sliding mode observer
        for (i = 0; i < 4; ++i)
        {
          NDI_PSI0[i] = indi_v[i];
          SMDO_z_dot[i] = -indi_v[i]- SMDO_nu0[i];
          for (j = 0; j < 4; ++j)
          {
             SMDO_z_dot[i] += g1[i][j]*indi_du[j]/INDI_G_SCALING;
             if (i == 2)
              SMDO_z_dot[i] += g2[j]*indi_du[j]/INDI_G_SCALING;
          } 
        }

        Error[3] = 0;
        for (i = 0; i < 4; ++i)
        {
            Error[3] += (thrust_pprz_cmd - actuator_state[i]) * Bwls[3][i];
        }


        float Ks[4] = {20,20,1.0,1.0};
        float K[4] = {reference_acceleration.rate_p,reference_acceleration.rate_q,reference_acceleration.rate_r, 1.0};
        if (sliding_mode_observer_status() == true){
          call_sliding_mode_observer(SMDO_z_dot, Error, K, Ks, 4);
        }      
    }
}

/**
 * @param enable_integrator
 * @param rate_control boolean that determines if we are in rate control or attitude control
 *
 * Function that should be called to run the INDI controller
 */
void stabilization_indi_run(bool in_flight, bool rate_control)
{

  /* Propagate the filter on the gyroscopes */
  struct FloatRates *body_rates = stateGetBodyRates_f();
  float rate_vect[3] = {body_rates->p, body_rates->q, body_rates->r};
  int8_t i;
  for (i = 0; i < 3; i++) {
    update_butterworth_2_low_pass(&measurement_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&estimation_output_lowpass_filters[i], rate_vect[i]);
    update_butterworth_2_low_pass(&rate_lowpass_filters[i], rate_vect[i]);

    //Calculate the angular acceleration via finite difference
    angular_acceleration[i] = (measurement_lowpass_filters[i].o[0]
                               - measurement_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;

    // Calculate derivatives for estimation
    float estimation_rate_d_prev = estimation_rate_d[i];
    estimation_rate_d[i] = (estimation_output_lowpass_filters[i].o[0] - estimation_output_lowpass_filters[i].o[1]) * PERIODIC_FREQUENCY;
    estimation_rate_dd[i] = (estimation_rate_d[i] - estimation_rate_d_prev) * PERIODIC_FREQUENCY;
  }

  /* attitude error                          */
  struct Int32Quat att_err;
  if (0/*attitude_optitrack_status()==true*/)
  {  //incorrect, still need to debug
    struct FloatQuat att_err_float;
    struct FloatQuat att_quat_optitrack;
    float_quat_of_eulers(&att_quat_optitrack, &attitude_optitrack);
    float_quat_inv_comp(&att_err_float, &att_quat_optitrack, &stab_att_sp_quat);
    //printf("%f,%f,%f\n", att_quat_optitrack.qx, att_quat_optitrack.qy, att_quat_optitrack.qz);
  }
  else{
    struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
    int32_quat_inv_comp(&att_err, att_quat, &stab_att_sp_quat);   
  }

  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);

  /* compute the INDI command */
  stabilization_indi_calc_cmd(&att_err, rate_control, in_flight);

  // Set the stab_cmd to 42 to indicate that it is not used
  stabilization_cmd[COMMAND_ROLL] = 42;
  stabilization_cmd[COMMAND_PITCH] = 42;
  stabilization_cmd[COMMAND_YAW] = 42;

  // Reset thrust increment boolean
  indi_thrust_increment_set = false;
}

// This function reads rc commands
void stabilization_indi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif

  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);

//   printf("%d  %d  %d  %d\n", q_sp.qi, q_sp.qx, q_sp.qy, q_sp.qz);
}

/**
 * Function that tries to get actuator feedback.
 *
 * If this is not available it will use a first order filter to approximate the actuator state.
 * It is also possible to model rate limits (unit: PPRZ/loop cycle)
 */
void get_actuator_state(void)
{
#if INDI_RPM_FEEDBACK
  float_vect_copy(actuator_state, act_obs, INDI_NUM_ACT);
#else
  //actuator dynamics
  int8_t i;
  float UNUSED prev_actuator_state;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    prev_actuator_state = actuator_state[i];

    actuator_state[i] = actuator_state[i]
                        + act_dyn[i] * (indi_u[i] - actuator_state[i]);

#ifdef STABILIZATION_INDI_ACT_RATE_LIMIT
    if ((actuator_state[i] - prev_actuator_state) > act_rate_limit[i]) {
      actuator_state[i] = prev_actuator_state + act_rate_limit[i];
    } else if ((actuator_state[i] - prev_actuator_state) < -act_rate_limit[i]) {
      actuator_state[i] = prev_actuator_state - act_rate_limit[i];
    }
#endif
  }

#endif
}

/**
 * @param ddx_error error in output change
 * @param i row of the matrix element
 * @param j column of the matrix element
 * @param mu learning rate
 *
 * Function that calculates an element of the G1 matrix.
 * The elements are stored in a different matrix,
 * because the old matrix is necessary to caclulate more elements.
 */
void calc_g1_element(float ddx_error, int8_t i, int8_t j, float mu)
{
  g1_est[i][j] = g1_est[i][j] - du_estimation[j] * mu * ddx_error;
}

/**
 * @param ddx_error error in output change
 * @param j column of the matrix element
 * @param mu learning rate
 *
 * Function that calculates an element of the G2 matrix.
 * The elements are stored in a different matrix,
 * because the old matrix is necessary to caclulate more elements.
 */
void calc_g2_element(float ddx_error, int8_t j, float mu)
{
  g2_est[j] = g2_est[j] - ddu_estimation[j] * mu * ddx_error;
}

/**
 * Function that estimates the control effectiveness of each actuator online.
 * It is assumed that disturbances do not play a large role.
 * All elements of the G1 and G2 matrices are be estimated.
 */
void lms_estimation(void)
{

  // Get the acceleration in body axes
  struct Int32Vect3 *body_accel_i;
  body_accel_i = stateGetAccelBody_i();
  ACCELS_FLOAT_OF_BFP(body_accel_f, *body_accel_i);

  // Filter the acceleration in z axis
  update_butterworth_2_low_pass(&acceleration_lowpass_filter, body_accel_f.z);

  // Calculate the derivative of the acceleration via finite difference
  float indi_accel_d = (acceleration_lowpass_filter.o[0]
                        - acceleration_lowpass_filter.o[1]) * PERIODIC_FREQUENCY;

  // scale the inputs to avoid numerical errors
  float_vect_smul(du_estimation, actuator_state_filt_vectd, 0.001, INDI_NUM_ACT);
  float_vect_smul(ddu_estimation, actuator_state_filt_vectdd, 0.001 / PERIODIC_FREQUENCY, INDI_NUM_ACT);

  float ddx_estimation[INDI_OUTPUTS] = {estimation_rate_dd[0], estimation_rate_dd[1], estimation_rate_dd[2], indi_accel_d};

  //Estimation of G
  // TODO: only estimate when du_norm2 is large enough (enough input)
  /*float du_norm2 = du_estimation[0]*du_estimation[0] + du_estimation[1]*du_estimation[1] +du_estimation[2]*du_estimation[2] + du_estimation[3]*du_estimation[3];*/
  int8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    // Calculate the error between prediction and measurement
    float ddx_error = - ddx_estimation[i];
    int8_t j;
    for (j = 0; j < INDI_NUM_ACT; j++) {
      ddx_error += g1_est[i][j] * du_estimation[j];
      if (i == 2) {
        // Changing the momentum of the rotors gives a counter torque
        ddx_error += g2_est[j] * ddu_estimation[j];
      }
    }

    // when doing the yaw axis, also use G2
    if (i == 2) {
      for (j = 0; j < INDI_NUM_ACT; j++) {
        calc_g2_element(ddx_error, j, mu2);
      }
    } else if (i == 3) {
      // If the acceleration change is very large (rough landing), don't adapt
      if (fabs(indi_accel_d) > 60.0) {
        ddx_error = 0.0;
      }
    }

    // Calculate the row of the G1 matrix corresponding to this axis
    for (j = 0; j < INDI_NUM_ACT; j++) {
      calc_g1_element(ddx_error, i, j, mu1[i]);
    }
  }

  bound_g_mat();

  // Save the calculated matrix to G1 and G2
  // until thrust is included, first part of the array
  float_vect_copy(g1[0], g1_est[0], INDI_OUTPUTS * INDI_NUM_ACT);
  float_vect_copy(g2, g2_est, INDI_NUM_ACT);

  // Calculate the inverse of (G1+G2)
  calc_g1g2_pseudo_inv();
}


void calc_g1_inv_damage(void)
{
    int i0 = 0,j0 = 0;
    for (int i = 0; i < 4; i++)
    {
      if (i != 2)
      {
        j0 = 0;
        for (int j = 0; j < 4; j++)
        {
          if (j != DAMAGED_ROTOR_INDEX)
          { 
            g1_damage[i0][j0] = g1[i][j]/INDI_G_SCALING;
            j0++;
          }
        }
        i0++;
      }
    }
    MAT_INV33(g1_damage_inv,g1_damage);
//    printf("%2.1f %2.1f %2.1f\n%2.1f %2.1f %2.1f\n%2.1f %2.1f %2.1f\n"
//                    , g1_damage[0][0], g1_damage[0][1], g1_damage[0][2]
//                    , g1_damage[1][0], g1_damage[1][1], g1_damage[1][2]
//                    , g1_damage[2][0], g1_damage[2][1], g1_damage[2][2]);
//    printf("%6.5f %6.5f %6.5f\n%6.5f %6.5f %6.5f\n%6.5f %6.5f %6.5f\n"
//                    , g1_damage_inv[0][0], g1_damage_inv[0][1], g1_damage_inv[0][2]
//                    , g1_damage_inv[1][0], g1_damage_inv[1][1], g1_damage_inv[1][2]
//                    , g1_damage_inv[2][0], g1_damage_inv[2][1], g1_damage_inv[2][2]);

}

/**
 * Function that calculates the pseudo-inverse of (G1+G2).
 */
void calc_g1g2_pseudo_inv(void)
{

  //sum of G1 and G2
  int8_t i;
  int8_t j;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    for (j = 0; j < INDI_NUM_ACT; j++) {
      if (i != 2) {
        g1g2[i][j] = g1[i][j] / INDI_G_SCALING;
      } else {
        g1g2[i][j] = (g1[i][j] + g2[j]) / INDI_G_SCALING;
      }
    }
  }

  //G1G2*transpose(G1G2)
  //calculate matrix multiplication of its transpose INDI_OUTPUTSxnum_act x num_actxINDI_OUTPUTS
  float element = 0;
  int8_t row;
  int8_t col;
  for (row = 0; row < INDI_OUTPUTS; row++) {
    for (col = 0; col < INDI_OUTPUTS; col++) {
      element = 0;
      for (i = 0; i < INDI_NUM_ACT; i++) {
        element = element + g1g2[row][i] * g1g2[col][i];
      }
      g1g2_trans_mult[row][col] = element;
    }
  }

  //there are numerical errors if the scaling is not right.
  float_vect_scale(g1g2_trans_mult[0], 100.0, INDI_OUTPUTS * INDI_OUTPUTS);

  //inverse of 4x4 matrix
  float_mat_inv_4d(g1g2inv[0], g1g2_trans_mult[0]);

  //scale back
  float_vect_scale(g1g2inv[0], 100.0, INDI_OUTPUTS * INDI_OUTPUTS);

  //G1G2'*G1G2inv
  //calculate matrix multiplication INDI_NUM_ACTxINDI_OUTPUTS x INDI_OUTPUTSxINDI_OUTPUTS
  for (row = 0; row < INDI_NUM_ACT; row++) {
    for (col = 0; col < INDI_OUTPUTS; col++) {
      element = 0;
      for (i = 0; i < INDI_OUTPUTS; i++) {
        element = element + g1g2[i][row] * g1g2inv[col][i];
      }
      g1g2_pseudo_inv[row][col] = element;
    }
  }
}

static void rpm_cb(uint8_t __attribute__((unused)) sender_id, uint16_t UNUSED *rpm, uint8_t UNUSED num_act)
{
#if INDI_RPM_FEEDBACK
  int8_t i;
  for (i = 0; i < num_act; i++) {
    act_obs[i] = (rpm[i] - get_servo_min(i));
    act_obs[i] *= (MAX_PPRZ / (float)(get_servo_max(i) - get_servo_min(i)));
    Bound(act_obs[i], 0, MAX_PPRZ);
  }
#endif
}

/**
 * ABI callback that obtains the thrust increment from guidance INDI
 */
static void thrust_cb(uint8_t UNUSED sender_id, float thrust_increment)
{
  indi_thrust_increment = thrust_increment;
  indi_thrust_increment_set = true;
}

static void bound_g_mat(void)
{
  int8_t i;
  int8_t j;
  for (j = 0; j < INDI_NUM_ACT; j++) {
    float max_limit;
    float min_limit;

    // Limit the values of the estimated G1 matrix
    for (i = 0; i < INDI_OUTPUTS; i++) {
      if (g1_init[i][j] > 0.0) {
        max_limit = g1_init[i][j] * INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j] / INDI_ALLOWED_G_FACTOR;
      } else {
        max_limit = g1_init[i][j] / INDI_ALLOWED_G_FACTOR;
        min_limit = g1_init[i][j] * INDI_ALLOWED_G_FACTOR;
      }

      if (g1_est[i][j] > max_limit) {
        g1_est[i][j] = max_limit;
      }
      if (g1_est[i][j] < min_limit) {
        g1_est[i][j] = min_limit;
      }
    }

    // Do the same for the G2 matrix
    if (g2_init[j] > 0.0) {
      max_limit = g2_init[j] * INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j] / INDI_ALLOWED_G_FACTOR;
    } else {
      max_limit = g2_init[j] / INDI_ALLOWED_G_FACTOR;
      min_limit = g2_init[j] * INDI_ALLOWED_G_FACTOR;
    }

    if (g2_est[j] > max_limit) {
      g2_est[j] = max_limit;
    }
    if (g2_est[j] < min_limit) {
      g2_est[j] = min_limit;
    }
  }
}
