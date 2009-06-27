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

#ifdef MOTORS_BRUSHED

//-----------------------------------------------------------------------------
//
// Overheating Prevention Definitions
//
//-----------------------------------------------------------------------------

// Command cycle time in mSec
// *** MUST BE EQUAL TO THE TRUE CYCLE TO KEEP THE CALCULATIONS CORRECT ***

#define MOTORS_COMMAND_CYCLE    20   // mSec

// Short averaging over a period of ~320mSec

#define MOTORS_SHORT_AVG_FACTOR 4
#define MOTORS_SHORT_AVG_TIME   (MOTORS_COMMAND_CYCLE << MOTORS_SHORT_AVG_FACTOR)

// Long averaging over a period of ~10Sec

#define MOTORS_LONG_AVG_FACTOR  4
#define MOTORS_LONG_AVG_TIME    (MOTORS_SHORT_AVG_TIME << MOTORS_LONG_AVG_FACTOR)

// Overheating Limits:
// If motor average is above threshold, motor is limited to the limit value

#define SHORT_OVERHEAT_THRESHOLD 160 
#define SHORT_OVERHEAT_LIMIT     160

#define LONG_OVERHEAT_THRESHOLD  120 
#define LONG_OVERHEAT_LIMIT      120

#endif  // MOTORS_BRUSHED


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Current motor commands

static uint8_t motors_current_commands[NUM_DIRECTIONS] = {MOTOR_THROTTLE_MIN};

#ifdef MOTORS_BRUSHED
// Motors averages over a short time and long time

static uint16_t motors_short_avg[NUM_DIRECTIONS] = {MOTOR_THROTTLE_MIN};
static uint16_t motors_long_avg [NUM_DIRECTIONS] = {MOTOR_THROTTLE_MIN};

// Cycle counter

static uint8_t cycle_counter = 0;
#endif  // MOTORS_BRUSHED

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
}


#ifdef MOTORS_BRUSHED
//============================ motors_command() ===============================
//
// See the description in motors.h

boolean                // Ret: true if motor commands were limited due to, e.g
                       //      overheating.
motors_command(
  int16_t  throttle,    // In:  Throttle command
  int16_t  pitch_rate,  // In:  Pitch rate (centered at 0)
  int16_t  roll_rate,   // In:  Roll rate (centered at 0)
  int16_t  yaw_rate)    // In:  Yaw rate  (centered at 0)

{
  uint8_t   dir;
  boolean   motors_limited = false;
  int16_t   limits[NUM_DIRECTIONS];

  
  if (! motors_enabled)
  {
    // Set motor commands and averages to the minimum
    
    for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
    {
      motors_current_commands[dir] = MOTOR_COMMAND_MIN;
      motors_short_avg[dir] = MOTOR_COMMAND_MIN;
      motors_long_avg[dir] = MOTOR_COMMAND_MIN;
    }
  }
  
  else
  {
    // Update averaging based on the last motor commands, and set limits to
    // protect against overheating
    
    cycle_counter++;
    
    for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
    {
      // Calculate short average
      
      motors_short_avg[dir] -= motors_short_avg[dir] >> MOTORS_SHORT_AVG_FACTOR;
      motors_short_avg[dir] += motors_current_commands[dir];
      
      // Calculate long average once every MOTORS_SHORT_AVG_TIME
      
      if ((cycle_counter & ((1 << MOTORS_SHORT_AVG_FACTOR) - 1)) == 0)
      {
        motors_long_avg[dir] -= motors_long_avg[dir] >> MOTORS_LONG_AVG_FACTOR;
        motors_long_avg[dir] += motors_short_avg[dir] >> MOTORS_SHORT_AVG_FACTOR;
      }

      if (motors_long_avg[dir] > (LONG_OVERHEAT_THRESHOLD << MOTORS_LONG_AVG_FACTOR))
      {
        limits[dir] = LONG_OVERHEAT_LIMIT;
        motors_limited = true;
      }
      
      else if (motors_short_avg[dir] > SHORT_OVERHEAT_THRESHOLD << MOTORS_SHORT_AVG_FACTOR)
      {
        limits[dir] = SHORT_OVERHEAT_LIMIT;
        motors_limited = true;
      }
      
      else
        limits[dir] = MOTOR_THROTTLE_MAX;
    };
    
    // Limit the inputs to legal values
    
    throttle   = constrain(throttle,   MOTOR_THROTTLE_MIN,      MOTOR_THROTTLE_MAX);
    roll_rate  = constrain(roll_rate,  MOTOR_ROTATION_RATE_MIN, MOTOR_ROTATION_RATE_MAX);
    pitch_rate = constrain(pitch_rate, MOTOR_ROTATION_RATE_MIN, MOTOR_ROTATION_RATE_MAX);
    yaw_rate   = constrain(yaw_rate,   MOTOR_ROTATION_RATE_MIN, MOTOR_ROTATION_RATE_MAX);
      
    // Calculate motor commands.  Do this in signed 16 bits to avoid overflow
    // or underflow.
    // Currently we limit each motor separately.  We assume stability control
    // will compensate for the resulting assymetry.
    
    motors_current_commands[FRONT] = constrain(throttle + pitch_rate + yaw_rate,
                                               0,
                                               limits[FRONT]);
    motors_current_commands[REAR ] = constrain(throttle - pitch_rate + yaw_rate,
                                               0,
                                               limits[REAR]);
    motors_current_commands[RIGHT] = constrain(throttle + roll_rate  - yaw_rate,
                                               0,
                                               limits[RIGHT]);
    motors_current_commands[LEFT ] = constrain(throttle - roll_rate  - yaw_rate,
                                               0,
                                               limits[LEFT]);
  };

  // Now send the commands to the motors
  
  analogWrite(FRONT_MOTOR_PIN, motors_current_commands[FRONT]);
  analogWrite(REAR_MOTOR_PIN,  motors_current_commands[REAR ]);
  analogWrite(RIGHT_MOTOR_PIN, motors_current_commands[RIGHT]);
  analogWrite(LEFT_MOTOR_PIN,  motors_current_commands[LEFT ]);

  
  return motors_limited;
}

#else  // #ifdef MOTORS_BRUSHED

//============================ motors_command() ===============================
//
// See the description in motors.h

boolean                // Ret: true if motor commands were limited due to, e.g
                       //      overheating.
motors_command(
  int16_t  throttle,    // In:  Throttle command
  int16_t  pitch_rate,  // In:  Pitch rate (centered at 0)
  int16_t  roll_rate,   // In:  Roll rate (centered at 0)
  int16_t  yaw_rate)    // In:  Yaw rate  (centered at 0)

{
  uint8_t   dir;

  
  if (! motors_enabled)
  {
    // Set motor commands and averages to the minimum
    
    for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
      motors_current_commands[dir] = (int16_t)MOTOR_THROTTLE_MIN;
  }
  
  else
  {
    // Limit the inputs to legal values
    
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
                           (int16_t)MOTOR_ROTATION_RATE_MIN,
                           (int16_t)MOTOR_ROTATION_RATE_MAX);
      
    // Calculate motor commands.  Do this in signed 16 bits to avoid overflow
    // or underflow.
    
    motors_current_commands[FRONT] = constrain(throttle + pitch_rate + yaw_rate,
                                               (int16_t)MOTOR_THROTTLE_MIN,
                                               (int16_t)MOTOR_THROTTLE_MAX);
    motors_current_commands[REAR ] = constrain(throttle - pitch_rate + yaw_rate,
                                               (int16_t)MOTOR_THROTTLE_MIN,
                                               (int16_t)MOTOR_THROTTLE_MAX);
    motors_current_commands[RIGHT] = constrain(throttle + roll_rate  - yaw_rate,
                                               (int16_t)MOTOR_THROTTLE_MIN,
                                               (int16_t)MOTOR_THROTTLE_MAX);
    motors_current_commands[LEFT ] = constrain(throttle - roll_rate  - yaw_rate,
                                               (int16_t)MOTOR_THROTTLE_MIN,
                                               (int16_t)MOTOR_THROTTLE_MAX);
  };

  // Now send the commands to the motors
  
  analogWrite(FRONT_MOTOR_PIN, motors_current_commands[FRONT]);
  analogWrite(REAR_MOTOR_PIN,  motors_current_commands[REAR ]);
  analogWrite(RIGHT_MOTOR_PIN, motors_current_commands[RIGHT]);
  analogWrite(LEFT_MOTOR_PIN,  motors_current_commands[LEFT ]);

  
  return false;
}
#endif


//============================= motors_get_*() ================================
//
// See the description in motors.h

uint8_t motors_get_current_command(uint8_t dir)

{
  return motors_current_commands[dir];
};


#ifdef MOTORS_BRUSHED
uint8_t motors_get_short_avg(uint8_t dir)

{
  return motors_short_avg[dir] >> MOTORS_SHORT_AVG_FACTOR;
};


uint8_t motors_get_long_avg(uint8_t dir)

{
  return motors_long_avg[dir] >> MOTORS_LONG_AVG_FACTOR;
};
#endif  // MOTORS_BRUSHED


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

