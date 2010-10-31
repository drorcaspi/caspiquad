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
#include "indicators.h"
#include "flight_control.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

// Thresholds for initiating altitude hold mode

#define ALT_HOLD_INIT_MOTOR_THROTTLE_MAX     850
#define ALT_HOLD_INIT_MOTOR_THROTTLE_MIN     550

// Limits on altitude difference as input to altitude hold PID

#define ALT_HOLD_ALT_DIFF_MIN_CM            -300
#define ALT_HOLD_ALT_DIFF_MAX_CM             200

// Range of throttle automatic control in altitude hold mode
// (from center to edge in either direction)

#define ALT_HOLD_MOTOR_THROTTLE_CHANGE_MAX   400
#define ALT_HOLD_MOTOR_THROTTLE_CHANGE_MIN  -150


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
//
// Algorithm Description:
//
// Start:
// - Altitude hold starts when toggling a receiver channel
// - At that point we sample the barometer and set the target altitude
// - We also sample the throttle channel ans set it as the reference point
//
// Ongoing:
// - Throttle channel input vs. the reference point is used to move the target
//   altitude up and down.  I.e., it is used as the desired vertical speed.
// - Barometer input is used as the actual altitude.
// - PID on the altitude error (as P)
// - Do not take vertical speed (P) into account, as its measurement resolution
//   is very rough (about 1m/sec)

int16_t                              // Ret: Updated throttle command
flight_control_alt(
  int16_t motor_throttle_command)    // In : Throttle command from thr receiver

{
  // Static Local Variables
  
  static boolean     is_alt_hold                     = false;
                          // State of altitude hold
  static boolean     was_alt_hold_sw                 = true;
                          // State of the altitude hold switch in the last cycle
  static int16_t     ref_motor_throttle_command      = 0;
                          // Throttle command value when altitude hold was initiated
  static int32_t     target_alt_1_512cm;
                          // Target altitude, in 1/512 cm units

  // Local Variables
                          
  boolean            is_alt_hold_sw;
                          // Current state of the altitude hold switch
  int32_t            alt_cm;
                          // Altitude from the zero point, in 1 cm units
  int32_t            alt_err_cm;
                          // Altitude error
  float              motor_alt_command;
                          // Output of altitude hold PID


  // Called each cycle, required for correct averaging

  alt_cm = baro_alt_estimate_get();
                          
  // Altitude hold in initiated when BOTH conditions exist:
  // 1. Receiver altitude hold channel is toggled from OFF to ON, and
  // 2. Throttle is within initiation limits
  //
  // Altitude hold is terminated when EITHER of the conditions exist:
  // 1. Receiver altitude hold channel is OFF, or
  // 2. Throttle is outside vertical speed control limits
                          
  is_alt_hold_sw = receiver_get_boolean(ENABLE_ALT_HOLD_CH);
  
  if (is_alt_hold)
  {
    // Altitute hold is currently ON
    // Verify if altitude hold is still in effect
    
    if (is_alt_hold_sw)
    {
      // Vertical speed command is controlled by the distance of the throttle command
      // from its reference point.  Unit is 1/5 cm/sec.
      // Update the target altitude.  Range is about +/- 1 M/sec.

      target_alt_1_512cm += motor_throttle_command - ref_motor_throttle_command;

      // Calculate the altitude error and constrain
      
      alt_err_cm = (target_alt_1_512cm >> 9) - alt_cm;
      alt_err_cm = constrain(alt_err_cm,
                             (int32_t)ALT_HOLD_ALT_DIFF_MIN_CM,
                             (int32_t)ALT_HOLD_ALT_DIFF_MAX_CM);
      
      // Calculate the altitude error and do the PID
      // Note that we do not take the vertical speed error into account.  This is
      // because the granularity of the barometer measurement would be very rough
      // (about 1m / sec).
    
      motor_alt_command = alt_pid.update_p((int16_t)alt_err_cm);
      
      // Constrain the altitude command to the minimum and maximum legal
      // values and add to the stored initial throttle command
    
      motor_throttle_command = ref_motor_throttle_command +
                               (int16_t)constrain(motor_alt_command,
                                                  (float)ALT_HOLD_MOTOR_THROTTLE_CHANGE_MIN,
                                                  (float)ALT_HOLD_MOTOR_THROTTLE_CHANGE_MAX);
    }

    else
    {
      // Terminate altitude hold
      
      is_alt_hold = false;
      indicators_set(IND_FLIGHT);
    }
  }

  else
  {
    // Altitude hold is currently OFF
    // Check initiation conditions
    
    if (is_alt_hold_sw                                                        &&
        (! was_alt_hold_sw)                                                   &&
        (motor_throttle_command >= (int16_t)ALT_HOLD_INIT_MOTOR_THROTTLE_MIN) &&
        (motor_throttle_command <= (int16_t)ALT_HOLD_INIT_MOTOR_THROTTLE_MAX))
    {
      // Turn altitude hold ON for the next cycle
      // Set the starting point and zero the barometer
    
      ref_motor_throttle_command = motor_throttle_command;
      target_alt_1_512cm = alt_cm << 9;
      is_alt_hold = true;
      indicators_set(IND_FLIGHT_ALT_HOLD);
    }
  }

  // Save the state of the altitude hold switch for the next cycle
  
  was_alt_hold_sw = is_alt_hold_sw;

  // Return the updated throttle command
  
  return motor_throttle_command;
}
#endif

