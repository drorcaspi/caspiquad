//=============================================================================
//
// Flight Control Module
//
//=============================================================================

/*-----------------------------------------------------------------------------
  CaspiQuad 1
  Copyright (c) 2010 Dror Caspi.  All rights reserved.

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

#include "quad.h"
#include "adc.h"
#include "gyro.h"
#include "accel.h"
#if SUPPORT_BARO
#include "baro.h"
#endif
#include "receiver.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "flight_control.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

// Thresholds for initiating altitude hold mode

#define ALT_HOLD_INIT_MOTOR_THROTTLE_MAX     850
#define ALT_HOLD_INIT_MOTOR_THROTTLE_MIN     600

// Threshold for controlling vertical speed

#define ALT_HOLD_MOTOR_THROTTLE_RANGE        800

// Limits on altitude difference as input to altitude hold PID

#define ALT_HOLD_ALT_DIFF_MIN_CM            -300
#define ALT_HOLD_ALT_DIFF_MAX_CM             200

// Range of throttle automatic control in altitude hold mode
// (from center to edge in either direction)

#define ALT_HOLD_MOTOR_THROTTLE_CHANGE_MAX   300
#define ALT_HOLD_MOTOR_THROTTLE_CHANGE_MIN  -100


//=============================================================================
//
// Static Variables
//
//=============================================================================



//=============================================================================
//
// Public Variables
//
//=============================================================================

PID                alt_pid;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=========================== flight_control_init() ===========================
//
// Initialize the flight control

int                             // Ret: Next address in EEPROM
flight_control_init(
  int   eeprom_addr)            // In: Base address in EEPROM

{
  eeprom_addr = alt_pid.read_eeprom(eeprom_addr,
                                    0,   // P
                                    0,   // I
                                    0,   // D
                                    0);  // windup_guard (rad/sec)

  return eeprom_addr;
}


#if SUPPORT_BARO
//=========================== flight_control_alt() ============================
//
// Altitude control using barometer input

int16_t                              // Ret: Updated throttle command
flight_control_alt(
  int16_t motor_throttle_command)    // In : Throttle command from thr receiver

{
  // Static Local Variables
  
  static boolean     is_alt_hold                     = false;
                          // State of altitude hold
  static boolean     was_alt_hold_sw                 = true;
                          // State of the altitude hold switch in the last cycle
  static int16_t     alt_hold_motor_throttle_command = 0;
                          // Throttle command for altitude hold.  Set on
                          // initiation of altitude hold mode.
  static int16_t     target_alt_8cm;
                          // Target altitude, in cm

  // Local Variables
                          
  boolean            is_alt_hold_sw;
                          // Current state of the altitude hold switch
  int16_t            motor_throttle_command_diff;
                          // How far the throttle is from the altitude hold point
  int16_t            alt_8cm;
                          // Altitude from the zero point, in cm
  int16_t            vert_speed;
                          // Vertical speed etimate, in 1/843 cm/sec
  float              motor_alt_command;
                          // Output of altitude hold PID


  // Called each cycle, required for correct averaging

  alt_8cm = baro_alt_estimate_get(&vert_speed);
                          
  // Altitude hold in initiated when BOTH conditions exist:
  // - Receiver altitude hold channel is toggled from OFF to ON, and
  // - Throttle is within initiation limits
  //
  // Altitude hold is terminated when EITHER of the conditions exist:
  // - Receiver altitude hold channel is OFF, or
  // - Throttle is outside vertical speed control limits
                          
  is_alt_hold_sw = receiver_get_boolean(ENABLE_ALT_HOLD_CH);
  
  if (is_alt_hold)
  {
    motor_throttle_command_diff = motor_throttle_command -
                                  alt_hold_motor_throttle_command;

    // Verify if altitude hold is still in effect
    
    if (is_alt_hold_sw                                                                  &&
        (motor_throttle_command_diff <=  (int16_t)(ALT_HOLD_MOTOR_THROTTLE_RANGE / 2))  &&
        (motor_throttle_command_diff >= -(int16_t)(ALT_HOLD_MOTOR_THROTTLE_RANGE / 2)))
    {
      // Scale motor_throttle_command_diff to the same units of vert_speed (1/843 cm/sec)

      motor_throttle_command_diff *= GAIN????;
      
      // Update the target altitude

      target_alt_8cm += (motor_throttle_command_diff * FACTOR);  // TODO: maybe usint of 8CM is too coarse

      alt_8cm -= target_alt_8cm;

      // Constrain the altitude error before calculating the PID
      // TODO: This should really be part of the PID, the windup guard
      
      alt_8cm = constrain(alt_8cm,
                          (int16_t)ALT_HOLD_ALT_DIFF_MIN_CM,
                          (int16_t)ALT_HOLD_ALT_DIFF_MAX_CM);
                           
      // Do the PID for the altitude
    
      motor_alt_command = alt_pid.update_pid(alt_8cm,
                                             vert_speed - (motor_throttle_command_diff * GAIN)??,
                                             0);
      
      // Constrain the altitude command to the minimum and maximum legal
      // values and add to the stored initial throttle command
    
      motor_throttle_command = alt_hold_motor_throttle_command +
                               (int16_t)constrain(motor_alt_command,
                                                  (float)ALT_HOLD_MOTOR_THROTTLE_CHANGE_MIN,
                                                  (float)ALT_HOLD_MOTOR_THROTTLE_CHANGE_MAX);
    }

    else
    {
      // Terminate altitude hold
      
      is_alt_hold = false;
    }
  }

  else
  {
    // Altitude hold is currently off, check initiation conditions
    
    if (is_alt_hold_sw                                                        &&
        (! was_alt_hold_sw)                                                   &&
        (motor_throttle_command >= (int16_t)ALT_HOLD_INIT_MOTOR_THROTTLE_MIN) &&
        (motor_throttle_command <= (int16_t)ALT_HOLD_INIT_MOTOR_THROTTLE_MAX)
    {
      // Set the starting point and zero the barometer
    
      alt_hold_motor_throttle_command = motor_throttle_command;
      target_alt_8cm = alt_8cm;
      is_alt_hold = true;
    }
  }

  // Save the state of the altitude hold switch for the next cycle
  
  was_alt_hold_sw = is_alt_hold_sw;
}
#endif

