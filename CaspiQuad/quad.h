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

#include "WProgram.h"
#include "quad_hw.h"

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
#define PRINT_GYRO                0
#define PRINT_ROT_ERROR           0
#define PRINT_ROT_ESTIMATE        0
#define PRINT_ROT_CORRECTION      0
#define PRINT_MOTOR_ROT_COMMAND   0
#define PRINT_PID                 0
#define PRINT_MOTOR_COMMAND       0
#define PRINT_BAT_SENSOR          0
#define PRINT_INDICATORS          0
#define SUPPORT_TELEMENTRY        1


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

#define FRONT            0
#define REAR             1
#define RIGHT            2
#define LEFT             3
#define NUM_DIRECTIONS   4
#define FIRST_DIRECTION  FRONT

// Axis

#define X_AXIS           0   // Positive forward, through nose of aircraft
#define Y_AXIS           1   // Positive to Right of X Axis
#define Z_AXIS           2   // Positive downwards
#define NUM_AXIS         3
#define FIRST_AXIS       X_AXIS

// Rotations

#define ROLL             0   // Angle of Y Body Axis (wing) relative to horizon.
                             // Also a positive (right wing down) rotation about
                             // X Axis.
#define PITCH            1   // Angle of X Body Axis (nose) relative to horizon.
                             // Also a positive (nose up) rotation about Y Axis.
#define YAW              2   // Angle of X Body Axis (nose) relative to North.
                             // Also a positive (nose right) rotation about Z Axis.
#define NUM_ROTATIONS    3
#define FIRST_ROTATION   ROLL


//=============================================================================
//
// Control Loop Definitions
//
//=============================================================================

#define CONTROL_LOOP_CYCLE_MSEC 10
#define CONTROL_LOOP_CYCLE_SEC  (CONTROL_LOOP_CYCLE_MSEC * 0.001)


#endif
