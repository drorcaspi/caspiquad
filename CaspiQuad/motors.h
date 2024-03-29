#ifndef __MOTORS_H__
#define __MOTORS_H__

//=============================================================================
//
// Quad Motors Control API
//
//=============================================================================

/*-----------------------------------------------------------------------------
  CaspiQuad 1
  Copyright (c) 2009 Dror Caspi.  All rights reserved.

  Based on AeroQuad (http://www.AeroQuad.info)
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
-----------------------------------------------------------------------------*/

#define MOTORS_PWM_FREQ_HZ     488
#define MOTORS_PWM_CYCLE_USEC 2049  // (1000000 / MOTORS_PWM_FREQ_HZ)

//-----------------------------------------------------------------------------
//
// Motor Control Ranges
//
//-----------------------------------------------------------------------------

// Maximum and minimim motors control pulse widths

#define MOTORS_PW_MAX_USEC    1799
#define MOTORS_PW_MIN_USEC     800
#define MOTORS_PW_RANGE_USEC  (MOTORS_PW_MAX_USEC - MOTORS_PW_MIN_USEC + 1)

// Maximum, minimum and range values of motor command, used as AnalogWrite()
// parameter.  In brushless mode, AnalogWrite() controls the ESCs, with pulse
// width range as defined above

#define MOTOR_COMMAND_RANGE   256
#define MOTOR_COMMAND_MAX     224  // ((uint32_t)((uint32_t)MOTORS_PW_MAX_USEC * MOTOR_COMMAND_RANGE) / MOTORS_PWM_CYCLE_USEC)
#define MOTOR_COMMAND_MIN      99  // ((uint32_t)((uint32_t)MOTORS_PW_MIN_USEC * MOTOR_COMMAND_RANGE) / MOTORS_PWM_CYCLE_USEC)

// Top and idle value define a range with some guard bands allowing room for
// varying the actual motor command to control rotation in 3 axis

#define MOTOR_COMMAND_TOP     (MOTOR_COMMAND_MAX - 20)  // 204
#define MOTOR_COMMAND_IDLE    (MOTOR_COMMAND_MIN + 10)  // 109

// Maximum, minimum and range values of throttle input to motors_command().
// A shift factor of 2 (multiply by 4) is used to reduce rounding errors,
// since actual motor command is calculated by adding 4 variable.

#define MOTOR_THROTTLE_FACTOR 2
#define MOTOR_THROTTLE_MAX    (MOTOR_COMMAND_MAX  << MOTOR_THROTTLE_FACTOR)  // 896
#define MOTOR_THROTTLE_MIN    (MOTOR_COMMAND_MIN  << MOTOR_THROTTLE_FACTOR)  // 396

// Top and idle value define a range with some guard bands allowing room for
// varying the actual motor command to control rotation in 3 axis.  Throttle
// input to motors_command() should be within this range

#define MOTOR_THROTTLE_TOP    (MOTOR_COMMAND_TOP  << MOTOR_THROTTLE_FACTOR)  // 816
#define MOTOR_THROTTLE_IDLE   (MOTOR_COMMAND_IDLE << MOTOR_THROTTLE_FACTOR)  // 436
#define MOTOR_THROTTLE_RANGE  (MOTOR_THROTTLE_TOP - MOTOR_THROTTLE_IDLE + 1) // 381

// Maximum, minimum and range values of rotation (pitch, roll, yaw) rate input

#define MOTOR_ROTATION_RATE_MAX   (MOTOR_THROTTLE_RANGE / 2)                 // 190
#define MOTOR_ROTATION_RATE_MIN   (-MOTOR_THROTTLE_RANGE / 2)                // -190
#define MOTOR_ROTATION_RATE_RANGE MOTOR_THROTTLE_RANGE                       // 381


//============================== motors_init() ================================
//
// Initialize the motors module
// Should be called on system initalization

void motors_init(void);


//============================= motors_enable() ===============================
//
// Enable the motors
// Until enabled, motors_command() has no effect

void motors_enable(void);


//============================ motors_disable() ===============================
//
// Disable the motors
// After enabled, motors_command() has no effect

void motors_disable(void);


//============================ motors_command() ===============================
//
// Command all 4 motors, given desired throttle and rotation (pitch, roll & 
// yaw) rates.
//
// Rotation is defined as follows:
//
//   Pitch - nose down rotation about Y Body Axis
//   Roll  - left wing down rotation about X Body Axis
//   Yaw   - nose left rotation about Z Body Axis
//
// Front & rear motors turn clockwise, right & left motors turn counter-
// clockwise.
//
// Thus, the simplified calulation of motor commands is as follows:
//
//   front = throttle - pitch - yaw
//   back  = throttle + pitch - yaw
//   right = throttle + roll  + yaw
//   left  = throttle - roll  + yaw
//
// However, internally the function also takes into account the following:
//
// - Enable or disable status

void
motors_command(
  int16_t  throttle,    // In:  Throttle command
  int16_t  pitch_rate,  // In:  Pitch rate (centered at 0)
  int16_t  roll_rate,   // In:  Roll rate (centered at 0)
  int16_t  yaw_rate);   // In:  Yaw rate  (centered at 0)


//====================== motors_get_current_command() =========================
//
// Retrieve the motor command, as sent to AnalogWrite()

uint8_t motors_get_current_command(uint8_t dir);


//====================== motors_get_current_pw_usec() =========================
//
// Retrieve the motor command pulse width, as sent to the ESC

uint16_t motors_get_current_pw_usec(uint8_t dir);


//========================== motors_print_stats() =============================
//
// Print some statistics (for debug)

void motors_print_stats(void);

#endif
