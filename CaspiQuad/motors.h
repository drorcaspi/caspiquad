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

#define MOTORS_PWM_FREQ_HZ    488
#define MOTORS_PWM_CYCLE_USEC (1000000 / MOTORS_PWM_FREQ_HZ)

// Maximum, minimum and range values of motor command, used as AnalogWrite()
// parameter

#define MOTOR_COMMAND_MAX   255
#define MOTOR_COMMAND_MIN   0
#define MOTOR_COMMAND_RANGE (MOTOR_COMMAND_MAX - MOTOR_COMMAND_MIN + 1)

//-----------------------------------------------------------------------------
//
// Motor Input Ranges
//
//-----------------------------------------------------------------------------

// Maximum and minimim motors control pulse widths

#define MOTORS_PW_MAX_USEC    1799
#define MOTORS_PW_MIN_USEC     800
#define MOTORS_PW_RANGE_USEC  (MOTORS_PW_MAX_USEC - MOTORS_PW_MIN_USEC + 1)

// Maximum, minimum and range values of throttle input

// In brushless mode, AnalogWrite() controls the ESCs, with pulse width range
// as defined above

#define MOTOR_THROTTLE_MAX    ((uint32_t)((uint32_t)MOTORS_PW_MAX_USEC * MOTOR_COMMAND_RANGE) / MOTORS_PWM_CYCLE_USEC)
#define MOTOR_THROTTLE_MIN    ((uint32_t)((uint32_t)MOTORS_PW_MIN_USEC * MOTOR_COMMAND_RANGE) / MOTORS_PWM_CYCLE_USEC)
#define MOTOR_THROTTLE_RANGE  (MOTOR_THROTTLE_MAX - MOTOR_THROTTLE_MIN + 1)

// Maximum, minimum and range values of rotation (pitch, roll, yaw) rate input

#define MOTOR_ROTATION_RATE_MAX   (MOTOR_THROTTLE_RANGE / 2)
#define MOTOR_ROTATION_RATE_MIN   (-MOTOR_THROTTLE_RANGE / 2)
#define MOTOR_ROTATION_RATE_RANGE MOTOR_THROTTLE_RANGE


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
// TODO: check whether the above is also how the transmitter works
//
// Front & rear motors turn clockwise, right & left motors turn counter-
// clockwise.
//
// Thus, the simplified calulation of motor commands is as follows:
//
//   front = throttle + pitch - yaw
//   back  = throttle - pitch - yaw
//   right = throttle - roll  + yaw
//   left  = throttle + roll  + yaw
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


//========================= motors_get_current_*() ============================
//
// Retrieve the internal command, short & long averages of a single motor
// These functions are intended for telemetry & printing.

uint8_t motors_get_current_command(uint8_t dir);


//========================== motors_print_stats() =============================
//
// Print some statistics (for debug)

void motors_print_stats(void);

#endif
