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

// Main Flight State

typedef enum
{
  FLIGHT_ERROR,                 // Error, stop motors and wait for reset
  FLIGHT_SETUP,                 // Setup before flight
  FLIGHT_READY                  // Flying
} FlightState;
                      

//=============================================================================
//
// Public Variables
//
//=============================================================================

// TODO: these variables are accessed by serial_telemetry.cpp, should be
// TODO: replaced by accessor functions

extern FlightState        flight_state;

extern uint16_t           avg_cycle_msec;
extern uint8_t            max_cycle_msec;

extern Gyro               gyro[NUM_ROTATIONS];
extern RotationEstimator  rot_estimator[2];
extern YawEstimator       yaw_estimator;

extern PID                rot_rate_pid[NUM_ROTATIONS];
extern PID                rot_pid[NUM_ROTATIONS];

extern float              receiver_rot_rate_gain;  // (rad/sec)
extern uint8_t            receiver_rot_gain;
extern uint16_t           receiver_rot_limit;

extern ReceiverRotation   receiver_rot[NUM_ROTATIONS];

extern int16_t            motor_throttle_command;
extern int16_t            motor_rot_command[NUM_ROTATIONS];

//=============================================================================
//
// Public Functions
//
//=============================================================================

//====================== flight_control_write_eeprom() ========================
//
// Write the configuration parameters to EEPROM

void flight_control_write_eeprom(void);


#endif
