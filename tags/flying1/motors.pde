//=============================================================================
//
// Quad Motors Control
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

#include "quad.h"
#include "motors.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

#define MOTOR_THROTTLE_IDLE_RANGE (10 << MOTOR_THROTTLE_FACTOR)


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Current motor commands

static uint8_t motors_current_commands[NUM_DIRECTIONS] = {MOTOR_COMMAND_MIN};

// Global motors enable flag

static boolean motors_enabled = false;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================== motors_init() ================================
//
// See the description in motors.h

void motors_init(void)

{
  motors_disable();
}


//============================= motors_enable() ===============================
//
// See the description in motors.h

void motors_enable(void)

{
  motors_enabled = true;
}


//============================ motors_disable() ===============================
//
// See the description in motors.h

void motors_disable(void)

{
  motors_enabled = false;

  motors_command(MOTOR_THROTTLE_MIN, 0, 0, 0);
};


//============================ motors_command() ===============================
//
// See the description in motors.h

void
motors_command(
  int16_t  throttle,    // In:  Throttle command
  int16_t  pitch_rate,  // In:  Pitch rate (centered at 0)
  int16_t  roll_rate,   // In:  Roll rate (centered at 0)
  int16_t  yaw_rate)    // In:  Yaw rate  (centered at 0)

{
  uint8_t   dir;
  int16_t   rotation_rate_max;
  int16_t   motor_corrections[NUM_DIRECTIONS];
  int16_t   temp_motor_throttle;
  boolean   is_overflow;

  
  if (! motors_enabled)
  {
    // Set motor commands and averages to the minimum
    
    for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
      motors_current_commands[dir] = (uint8_t)MOTOR_COMMAND_MIN;
  }
  
  else
  {
    // Limit the inputs to their legal values

    // In idle mode, set the throttle input the idle value, so the props
    // rotate at a low speed as a visual indication.

    throttle = constrain(throttle,
                         (int16_t)MOTOR_THROTTLE_IDLE,
                         (int16_t)MOTOR_THROTTLE_TOP);
    
    // Find the available throttle range
    
    rotation_rate_max = throttle - (int16_t)MOTOR_THROTTLE_MIN;
    if (rotation_rate_max > ((int16_t)MOTOR_THROTTLE_MAX - throttle))
      rotation_rate_max = (int16_t)MOTOR_THROTTLE_MAX - throttle;
      
    // Limit roll & pitch to 75% of available throttle range
    
    rotation_rate_max -= (rotation_rate_max >> 2);
    
    roll_rate  = constrain(roll_rate, -rotation_rate_max, rotation_rate_max);
    pitch_rate = constrain(pitch_rate, -rotation_rate_max, rotation_rate_max);
    
    // Limit yaw to 37.5% of the available throttle range
    
    rotation_rate_max >>= 1;
    
    yaw_rate   = constrain(yaw_rate, -rotation_rate_max, rotation_rate_max);
      
    // Calculate Motor Commands:
    // ------------------------
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
    // Note the scaling down to command range (shift by MOTOR_THROTTLE_FACTOR) is
    // done only after the summing of trottle and rotations.  This reduces the
    // roundoff errors.

    motor_corrections[FRONT] = - pitch_rate + yaw_rate;
    motor_corrections[REAR ] = + pitch_rate + yaw_rate;
    motor_corrections[RIGHT] = + roll_rate  - yaw_rate;
    motor_corrections[LEFT ] = - roll_rate  - yaw_rate;

    // Repeat the motor commands calculation until there's no overflow
    
    do
    {
      // Calculate the motor command for each motor, while checking for overflow
      
      is_overflow = false;
      for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
      {
        temp_motor_throttle = throttle + motor_corrections[dir];
        
        if ((temp_motor_throttle > (int16_t)MOTOR_THROTTLE_MAX) ||
            (temp_motor_throttle < (int16_t)MOTOR_THROTTLE_MIN))
        {
          // The command is to high or too low
          
          is_overflow = true;
        }
        
        else
        {
          // Set the motor command for the current direction.
          // Scale down to the motor command range.
          
          motors_current_commands[dir] =
            (uint16_t)temp_motor_throttle >> MOTOR_THROTTLE_FACTOR;
        }
      }

      if (is_overflow)
      {
        // There was an overflow in at least one of the motors.  Scale down all
        // the corrections (to keep the quad balanced) and try again
        
        for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
        {
          motor_corrections[dir] >>= 1;
        };
      };
    } while (is_overflow);
  };

  // Now send the commands to the motors
  
  analogWrite(FRONT_MOTOR_PIN, motors_current_commands[FRONT]);
  analogWrite(REAR_MOTOR_PIN,  motors_current_commands[REAR ]);
  analogWrite(RIGHT_MOTOR_PIN, motors_current_commands[RIGHT]);
  analogWrite(LEFT_MOTOR_PIN,  motors_current_commands[LEFT ]);
};


//====================== motors_get_current_command() =========================
//
// Retrieve the motor command, as sent to AnalogWrite()

uint8_t motors_get_current_command(uint8_t dir)

{
  return motors_current_commands[dir];
};


//====================== motors_get_current_pw_usec() =========================
//
// Retrieve the motor command pulse width, as sent to the ESC

uint16_t motors_get_current_pw_usec(uint8_t dir)

{
  return ((uint32_t)motors_current_commands[dir] * (uint32_t)MOTORS_PWM_CYCLE_USEC) /
                                                  (uint16_t)MOTOR_COMMAND_RANGE;
};


#if PRINT_MOTOR_COMMAND
//========================== motors_print_stats() =============================
//
// Print some statistics (for debug)

void motors_print_stats(void)

{
  Serial.print(motors_get_current_command(FRONT), DEC);
  Serial.print("\t");
  Serial.print(motors_get_current_command(REAR), DEC);
  Serial.print("\t");
  Serial.print(motors_get_current_command(RIGHT), DEC);
  Serial.print("\t");
  Serial.println(motors_get_current_command(LEFT), DEC);
};
#endif

