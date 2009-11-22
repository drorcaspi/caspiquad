#ifndef __FLIGHT_CONTROL_H__
#define __FLIGHT_CONTROL_H__

//=============================================================================
//
// Flight Control API
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

//=============================================================================
//
// Public Definitions
//
//=============================================================================

                      

//=============================================================================
//
// Public Variables
//
//=============================================================================

// TODO: these variables are accessed by serial_telemetry.cpp, should be
// TODO: replaced by accessor functions

extern uint16_t           avg_cycle_msec;
extern uint8_t            max_cycle_msec;

extern Gyro               gyro[NUM_ROTATIONS];
extern RotationEstimator  rot_estimator[2];
extern RotationIntegrator yaw_estimator;

extern PID                rot_rate_pid[NUM_ROTATIONS];
extern PID                rot_pid[NUM_ROTATIONS];

extern float              receiver_rot_rate_gain;  // (rad/sec)
extern uint8_t            receiver_rot_gain;
extern uint16_t           receiver_rot_limit;

//=============================================================================
//
// Public Functions
//
//=============================================================================



#endif
