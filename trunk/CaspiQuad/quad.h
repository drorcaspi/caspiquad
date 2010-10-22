#ifndef __QUAD_H__
#define __QUAD_H__

//=============================================================================
//
// Quad Definitions
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

// Note the order of includes.  hardware_serial2.h actually overrides
// the inclusion of hardware_serial.h within WProgram.h

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "wiring.h"

#include "hardware_serial2.h"

#include "WProgram.h"

#include "quad_hw.h"


//=============================================================================
//
//  Feature Switches
//
//=============================================================================

//#define AUTO_ZERO_RECEIVER_ROLL_PITCH  0 // Auto-zero the received roll & pitch
//                                         // controls
#define SUPPORT_ACCEL                  1 // Usage of accelerometers
#define SUPPORT_ACCEL_CALIBRATION      0 // Accelerometer calibration
#define SUPPORT_ACCEL_ROT_INDICATION   0 // Indicate rotation measurement
                                         // status (are we using the accel.?)
#define ESTIMATE_EARTH_ACCEL           0
#define SUPPORT_BARO                   0 // Usage of barometric pressure sensor
#define SUPPORT_TELEMENTRY             1 // Support serial control protocol


//=============================================================================
//
//  Receiver Channel Assignments
//
//  NOTE: Never assign the same channel to more than one usage!
//
//=============================================================================

// Switch between acrobatic mode and stable mode
// 0: stable mode, stick controls rotation angle
// 1: acrobatic mode, no accelerometer usage, stick controls rotation rate

#define SUPPORT_ACRO_MODE_SWITCH       0
#define ENABLE_ACRO_MODE_CH            GEAR_CH

// Switch between pitch/roll stick control of rotation angle or rotation rate
// 0: stick controls rotation angle
// 1: stick controls rotation rate

#define SUPPORT_ROT_RATE_SWITCH        1
#define ENABLE_ROT_RATE_CH             GEAR_CH

// Switch on/off derivative calculation of pitch/roll stick

#define SUPPORT_ROT_DERIVATIVE_SWITCH  0
#define ENABLE_ROT_DERIVATIVE_CH       AUX1_CH

// Switch on/off altitude hold

#define SUPPORT_ALT_HOLD_SWITCH        0
#define ENABLE_ALT_HOLD_CH             GEAR_CH

// Switch on/off accelerometers measurement of pitch/roll rotation angles

#define SUPPORT_ACCEL_ROT_SWITCH       0
#define DISABLE_ACCEL_ROT_CH           AUX1_CH


//=============================================================================
//
//  Debug Print Switches
//
//=============================================================================

#define PRINT_STATE               0
#define PRINT_CYCLE_TIME          0
#define PRINT_EEPROM              0
#define PRINT_RECEIVER            0
#define PRINT_RECEIVER_ROT        0
#define PRINT_RECEIVER_THROTTLE   0
#define PRINT_ACCEL               0
#define PRINT_TRANSLATED_ACCEL    0
#define PRINT_GYRO                0
#define PRINT_BARO                0
#define PRINT_ROT_ERROR           0
#define PRINT_ROT_ESTIMATE        0
#define PRINT_ROT_CORRECTION      0
#define PRINT_MOTOR_ROT_COMMAND   0
#define PRINT_PID                 0
#define PRINT_MOTOR_COMMAND       0
#define PRINT_BAT_SENSOR          0
#define PRINT_INDICATORS          0
#define PRINT_INDICATORS_TEXT     0
#define PRINT_TELEMETRY           0


//=============================================================================
//
// General Definitions
//
//=============================================================================

#define PI              3.1415927
#define DEG_IN_RAD      (180.0 / PI)   // Degress in 1 radian

#define constrain_abs(__x, __r) constrain((__x), (__r), -(__r))


//=============================================================================
//
// Coordinate Systems and Attitudes
//
// See http://en.wikipedia.org/wiki/Flight_dynamics
//
//=============================================================================

// Directions

typedef enum
{
  FRONT            = 0,
  REAR             = 1,
  RIGHT            = 2,
  LEFT             = 3,
  NUM_DIRECTIONS   = 4
} Directions;

#define FIRST_DIRECTION  FRONT

// Axis

typedef enum
{
  X_AXIS           = 0,   // Positive forward, through nose of aircraft
  Y_AXIS           = 1,   // Positive to Right of X Axis
  Z_AXIS           = 2,   // Positive downwards
  NUM_AXES         = 3
} Axes;

#define FIRST_AXIS       X_AXIS

// Rotations

typedef enum
{
  ROLL             = 0,   // Angle of Y Body Axis (wing) relative to horizon.
                          // Also a positive (right wing down) rotation about
                          // X Axis.
  PITCH            = 1,   // Angle of X Body Axis (nose) relative to horizon.
                          // Also a positive (nose up) rotation about Y Axis.
  YAW              = 2,   // Angle of X Body Axis (nose) relative to North.
                          // Also a positive (nose right) rotation about Z Axis.
  NUM_ROTATIONS    = 3
} Rotations;

#define FIRST_ROTATION   ROLL


//=============================================================================
//
// Rotation Scale Definitions
//
//=============================================================================

#define ROT_SCALE_PI  (1u << 15)   // Representation of PI radians (180 degrees)
                                   // Selected so it will wrap nicely when used
                                   // in a signed 16-bit variable.
#define ROT_SCALE_RAD 10430        // Representation of 1 radian
#define ROT_NONE      ((int16_t)ROT_SCALE_PI)
                                   // Represents no valid rotation measurement

// Convert a rotation value to degrees

#define ROT_TO_DEG(_rot) ((int16_t)(_rot) / (int16_t)((uint16_t)ROT_SCALE_PI / 180))

// Convert a rotation value to radians

#define ROT_TO_RAD(_rot) ((float)(_rot) / (float)ROT_SCALE_RAD)


//=============================================================================
//
// Control Loop Definitions
//
//=============================================================================

#define CONTROL_LOOP_CYCLE_MSEC 10
#define CONTROL_LOOP_CYCLE_SEC  (CONTROL_LOOP_CYCLE_MSEC * 0.001)


#endif
