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

#define MOTOR_THROTTLE_IDLE       (MOTOR_THROTTLE_MIN  +            \
                                   (10 << MOTOR_THROTTLE_FACTOR))
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
  int16_t   throttle_limit;

  
  if (! motors_enabled)
  {
    // Set motor commands and averages to the minimum
    
    for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
      motors_current_commands[dir] = (uint8_t)MOTOR_COMMAND_MIN;
  }
  
  else
  {
    // Limit the inputs to their legal values
    
    if (throttle < (int16_t)MOTOR_THROTTLE_IDLE)
    {
      // In idle mode, set the throttle input the idle value, and limit
      // the motor command
      
      throttle = MOTOR_THROTTLE_IDLE;
      throttle_limit = MOTOR_THROTTLE_IDLE + MOTOR_THROTTLE_IDLE_RANGE;
    }

    else
      throttle_limit = (int16_t)MOTOR_THROTTLE_MAX;
    
    throttle   = constrain(throttle,
                           (int16_t)MOTOR_THROTTLE_MIN,
                           (int16_t)MOTOR_THROTTLE_MAX);
    roll_rate  = constrain(roll_rate,
                           (int16_t)MOTOR_ROTATION_RATE_MIN,
                           (int16_t)MOTOR_ROTATION_RATE_MAX);
    pitch_rate = constrain(pitch_rate,
                           (int16_t)MOTOR_ROTATION_RATE_MIN,
                           (int16_t)MOTOR_ROTATION_RATE_MAX);
    yaw_rate   = constrain(yaw_rate,
                           -throttle_limit / 2,
                           throttle_limit / 2);
      
    // Calculate motor commands.
    // Note the scaling down to command range (shift by MOTOR_THROTTLE_FACTOR) is
    // done only after the summing of trottle and rotations.  This reduces the
    // roundoff errors.

    throttle_limit >>= MOTOR_THROTTLE_FACTOR;
    
    motors_current_commands[FRONT] =
      constrain((int16_t)(throttle + pitch_rate + yaw_rate) >> MOTOR_THROTTLE_FACTOR,
                (int16_t)MOTOR_COMMAND_MIN,
                throttle_limit);
    motors_current_commands[REAR ] =
      constrain((int16_t)(throttle - pitch_rate + yaw_rate) >> MOTOR_THROTTLE_FACTOR,
                (int16_t)MOTOR_COMMAND_MIN,
                throttle_limit);
    motors_current_commands[RIGHT] =
      constrain((int16_t)(throttle + roll_rate  - yaw_rate) >> MOTOR_THROTTLE_FACTOR,
                (int16_t)MOTOR_COMMAND_MIN,
                throttle_limit);
    motors_current_commands[LEFT ] =
      constrain((int16_t)(throttle - roll_rate  - yaw_rate) >> MOTOR_THROTTLE_FACTOR,
                (int16_t)MOTOR_COMMAND_MIN,
                throttle_limit);
  };

  // Now send the commands to the motors
  
  analogWrite(FRONT_MOTOR_PIN, motors_current_commands[FRONT]);
  analogWrite(REAR_MOTOR_PIN,  motors_current_commands[REAR ]);
  analogWrite(RIGHT_MOTOR_PIN, motors_current_commands[RIGHT]);
  analogWrite(LEFT_MOTOR_PIN,  motors_current_commands[LEFT ]);
};


//============================= motors_get_*() ================================
//
// See the description in motors.h

uint8_t motors_get_current_command(uint8_t dir)

{
  return motors_current_commands[dir];
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

