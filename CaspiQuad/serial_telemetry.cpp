//=============================================================================
//
// AeroQuad Controller Telemetry Interface
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
#include "eeprom_utils.h"
#include "receiver.h"
#include "adc.h"
#include "gyro.h"
#include "accel.h"
#include "motors.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "flight_control.h"
#include "serial_telemetry.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

// Maximum number of command parameters

#define MAX_PARAMS                6

// Maximum length of parameter string

#define MAX_PARAM_STRING_LENGTH   8

// Timeout after which a command line is considered complete

#define COMMAND_TIMEOUT_MSEC    100

// Cycle period for continuous queries

#define CONT_QUERY_CYCLE_MSEC    50

// AeroQuad Configurator Definitions

#define SENSOR_DISPLAY_RANGE                  1000

#define RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR  100

#define I_DISPLAY_FACTOR                      1000


//=============================================================================
//
// Static Variables
//
//=============================================================================

static char    query                   = '\0'; // Command opcode
static float   params[MAX_PARAMS];
static uint8_t num_params              = 0;
static uint8_t expected_params         = 0;
static char    param_string[MAX_PARAM_STRING_LENGTH];
                                               // Command line buffer
static uint8_t i_param_string          = 0;    // Index to the command line
static uint8_t query_cycle_counter     = 0;    // Continuous commands cycle counter
static uint8_t cycle_counter;                  // Status query cycle counter
static uint8_t command_timeout_counter = 0;    // Counts timeout for command completion


//=============================================================================
//
// Static Functions
//
//=============================================================================

//=============================== print_semicolon() ===========================
//
// Prints a ';'

static void print_semicolon(void)

{
  Serial.print(';');
}


//=============================== print_comma() ===============================
//
// Prints a ','

static void print_comma()

{
  Serial.print(',');
}


//============================= command_reset() ===============================
//
// Reset the command buffer and associated variables

void command_reset(void)
{
  query = '\0';
  expected_params = 0;
  num_params = 0;
  i_param_string = 0;
  command_timeout_counter = 0;
}


//=============================================================================
//
// Public Functions
//
//=============================================================================

//=============================================================================
//
// Handles a serial telemetry query

void
handle_serial_telemetry(void)

{
  char           new_char;                       // Character read from serial
  boolean        is_continuous_query;
  uint8_t        rot;
  uint8_t        dir;
  uint8_t        axis;
  float          dummy;
  float          windup_guard;
  int8_t         accel_data[NUM_AXES];
  
  
  is_continuous_query = false;
  if ((query != '\0') && (num_params == expected_params))
  {
    // This is a contiuous query, process every CONT_QUERY_CYCLE_MSEC
    
    if (++query_cycle_counter >=
                     (uint8_t)(CONT_QUERY_CYCLE_MSEC / CONTROL_LOOP_CYCLE_MSEC))
    {
      // It is time to re-issue the query
      
      query_cycle_counter = 0;
      is_continuous_query = true;
    }
  };

  // Repeat while there is serial input, and at least one time if there's
  // an ongoing continuous query
  
  while (Serial.is_available() || is_continuous_query)
  {
    is_continuous_query = false;  // Only do this loop once for continuos query
    
    if (Serial.is_available())
    {
      new_char = Serial.read();
      
      // Any character reset the command timeout
      
      command_timeout_counter = 0;

      if (expected_params == 0)
      {
        // This is the first character of a new command
        
        command_reset();
        query = new_char;
      }

      else
      {
        // This is a character that belong to a parameter
        
        if (new_char == ';')
        {
          // ';' marks the end of parameter
          
          param_string[i_param_string] = '\0';
          i_param_string = 0;
          params[num_params++] = atof(param_string);
        }

        else if (((new_char >= '0') && (new_char <= '9')) ||
                 (new_char == '-') ||
                 (new_char == '+') ||
                 (new_char == '.'))
        {
          // Append the new character to the parameter string
          
          param_string[i_param_string++] = new_char;

          if (i_param_string >= sizeof(param_string))
          {
            // Parameter buffer overflow, abort

            command_reset();
          }
        }

        else
        {
          // Unexpected character. Abort, starting a new command

          command_reset();
          query = new_char;
        }
      }
    };

    if ((query != '\0') && (num_params == expected_params))
    {
      // A command has been received:
      // - Either this is the first char of a new command
      // - Or a complete command with all its paramters
      
      if (expected_params == 0)
      {
        // Set the number of expected params per the query
        
        switch (query)
        {
          case 'A':
          case 'E':
            expected_params = 6;
            break;

          case 'C':
          case 'K':
            expected_params = 3;
            break;

          case 'G':
          case 'I':
            expected_params = 2;
            break;

          default:
            expected_params = 0;
            break;
        }
      }

      if (num_params >= expected_params)
      {
        // The command is ready for processing
        
        switch (query)
        {
        case 'A':
          // Receive roll and pitch rotation rate (gyro) PID setting

          rot_rate_pid[ROLL].set_p(params[0]);
          rot_rate_pid[ROLL].set_i(params[1] / I_DISPLAY_FACTOR);
          rot_rate_pid[ROLL].set_d(params[2]);
          rot_rate_pid[ROLL].reset();
          
          rot_rate_pid[PITCH].set_p(params[3]);
          rot_rate_pid[PITCH].set_i(params[4] / I_DISPLAY_FACTOR);
          rot_rate_pid[PITCH].set_d(params[5]);
          rot_rate_pid[PITCH].reset();

          command_reset();
          
          break;
        
        case 'C':
          // Receive yaw rotation rate (gyro) PID settings
          
          rot_rate_pid[YAW].set_p(params[0]);
          rot_rate_pid[YAW].set_i(params[1] / I_DISPLAY_FACTOR);
          rot_rate_pid[YAW].set_d(params[2]);
          rot_rate_pid[YAW].reset();

          command_reset();
          
          break;
        
        case 'E':
          // Receive roll and pitch rotation (auto level) PID settings
          
          alt_pid.set_p(params[0]);
          alt_pid.set_i(0.0);
          alt_pid.set_d(params[2]);
          alt_pid.reset();
          
          //TBD_pid[PITCH].set_p(params[3]);
          //TBD_pid[PITCH].set_i(params[4]);
          //TBD_pid[PITCH].set_d(params[5]);
          //TBD_pid[PITCH].reset();

          command_reset();
          
          break;
       
        case 'G':
          // Receive auto level configuration values
          
          receiver_rot_gain = (uint8_t)params[0];  // levelLimit
          
          receiver_rot_limit = (uint16_t)params[1];  // levelOff

          command_reset();
          
          break;
         
        case 'I':
          // Receive flight control configuration values

          // The configurator only supports a single windup guard
        
          windup_guard = params[0];
          
          for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            rot_rate_pid[rot].set_windup_guard(windup_guard);
          };
          
          alt_pid.set_windup_guard(windup_guard);
          
          receiver_rot_rate_gain = params[1] /
                                   RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR;  // xmitFactor

          command_reset();
          
          break;
        
        case 'K':
          // Receive data filtering values
          
          dummy = params[0];  // *** NOT IMPLEMENTED *** smoothFactor[GYRO]
          dummy = params[1];  // *** NOT IMPLEMENTED *** smoothFactor[ACCEL]
          
          RotationEstimator::set_bw(params[2]);

          command_reset();
          
          break;
        
        case 'W':
          // Write EEPROM

          flight_control_write_eeprom();
          Gyro::write_eeprom();
          RotationEstimator::write_eeprom();

          for (uint8_t rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            rot_rate_pid[rot].write_eeprom();
          };
          alt_pid.write_eeprom();

          eeprom_write_ver();

          command_reset();
          
          break;

        case 'B':
          // Send roll and pitch rotation rate (gyro) PID settings

          Serial.print(rot_rate_pid[ROLL].get_p());
          print_comma();
          Serial.print(rot_rate_pid[ROLL].get_i() * I_DISPLAY_FACTOR);
          print_comma();
          Serial.print(rot_rate_pid[ROLL].get_d());
          print_comma();
          Serial.print(rot_rate_pid[PITCH].get_p());
          print_comma();
          Serial.print(rot_rate_pid[PITCH].get_i() * I_DISPLAY_FACTOR);
          print_comma();
          Serial.println(rot_rate_pid[PITCH].get_d());

          command_reset();
          
          break;

        case 'D':
          // Send yaw rotation rate (gyro) PID settings
          
          Serial.print(rot_rate_pid[YAW].get_p());
          print_comma();
          Serial.print(rot_rate_pid[YAW].get_i() * I_DISPLAY_FACTOR);
          print_comma();
          Serial.println(rot_rate_pid[YAW].get_d());  

          command_reset();
          
          break;
          
        case 'F':
          // Send roll and pitch rotation (auto level) PID settings
          
          Serial.print(alt_pid.get_p());
          print_comma();
          Serial.print(0.0);
          print_comma();
          Serial.print(alt_pid.get_d());
          print_comma();
          Serial.print(0.99);
          print_comma();
          Serial.print(0.99);
          print_comma();
          Serial.println(0.99);

          command_reset();
          
          break;
          
        case 'H':
          // Send auto level configuration values
          
          Serial.print(receiver_rot_gain, DEC);  // levelLimit
          
          print_comma();
          Serial.println(receiver_rot_limit);  // levelOff

          command_reset();
          
          break;
          
        case 'J':
          // Send flight control configuration values

          // The configurator only supports a single windup guard value
          
          Serial.print(rot_rate_pid[ROLL].get_windup_guard());
          
          print_comma();
          Serial.println(receiver_rot_rate_gain * RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR);  // xmitFactor

          command_reset();
          
          break;
          
        case 'L':
          // Send data filtering values
          
          Serial.print(9.99); // *** NOT IMPLEMENTED *** smoothFactor[GYRO]
          print_comma();
          
          Serial.print(9.99); // *** NOT IMPLEMENTED *** smoothFactor[ACCEL]
          print_comma();
          
          Serial.println(RotationEstimator::get_bw());

          command_reset();
          
          break;
          
        case 'Q':
        // Send sensor data
        
          for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            Serial.print((int)(gyro[rot].get_rad_per_sec() * DEG_IN_RAD));
            print_comma();
          };

          accel_get_current(accel_data);
          for (axis = FIRST_AXIS; axis < NUM_AXES; axis++)
          {
            Serial.print(accel_data[axis]);  // Not sure about the order
            print_comma();
          };

          Serial.print(0, DEC);   // levelAdjust - not used
          print_comma();
          Serial.print(0, DEC);   // levelAdjust -not used
          print_comma();

          Serial.print(rot_estimator[ROLL].get_estimate() * DEG_IN_RAD);
          print_comma();
          Serial.println(rot_estimator[PITCH].get_estimate() * DEG_IN_RAD);

          // This is a continuous query
          
          break;
          
        case 'R':
          // Send raw sensor data

          for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            Serial.print((int16_t)(gyro[rot].get_raw() - gyro[rot].get_raw_zero()));
            print_comma();
          };

          accel_get_current(accel_data);

          Serial.print(accel_data[Y_AXIS], DEC);  // Not sure about the order
          print_comma();
          Serial.print(accel_data[X_AXIS], DEC);  // Not sure about the order
          print_comma();
          Serial.println(accel_data[Z_AXIS], DEC);  // Not sure about the order
        
          // This is a continuous query
        
          break;
          
        case 'S':
          // Send all flight data

          if ((cycle_counter++ & 0x20) != 0)    
            Serial.print((int)(max_cycle_msec));
          else
            Serial.print((int)(avg_cycle_msec >> 8));
          
          for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            print_comma();
            Serial.print((int)(gyro[rot].get_rad_per_sec() * DEG_IN_RAD));
          }
          
          print_comma();    
          Serial.print(motor_throttle_command);
          
          for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            print_comma();
            Serial.print(motor_rot_command[rot], DEC);
          };

          // Print motor commands (pulse width in usec)
          
          for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
          {
            print_comma();
            Serial.print(motors_get_current_pw_usec(dir), DEC);
          };
          
          print_comma();

          Serial.print(flight_state == FLIGHT_READY ? 1 : 0, BIN);  // Armed
          print_comma();
          
          Serial.println(receiver_get_boolean(GEAR_CH) ? 1 : 0, BIN);   // Mode
        
          // This is a continuous query
        
          break;
          
        case 'T':
          // Send processed transmitter values
          
          Serial.print(receiver_rot_rate_gain * RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR);  // xmitFactor
          print_comma();

          Serial.print(0); // transmitterCommand[ROLL] - not supported
          print_comma();
          Serial.print(0); // transmitterCommand[PITCH] - not supported
          print_comma();
          Serial.print(0); // transmitterCommand[YAW] - not supported

          print_comma();
          Serial.print(0, DEC);   // levelAdjust - not supported
          print_comma();
          Serial.print(0, DEC);   // levelAdjust - not supported

          for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
          {
            print_comma();
            Serial.print(motor_rot_command[rot], DEC);
          };

          // This is a continuous query
          
          Serial.println();

          break;
           
        case 'U':
          // Send receiver values
          
          Serial.print(receiver_get_current_raw(ROLL_CH));
          print_comma();
          Serial.print(receiver_get_current_raw(PITCH_CH));
          print_comma();
          Serial.print(receiver_get_current_raw(YAW_CH));
          print_comma();
          Serial.print(receiver_get_current_raw(THROTTLE_CH));
          print_comma();
          Serial.print(receiver_get_current_raw(GEAR_CH));
          print_comma();
          Serial.println(receiver_get_current_raw(AUX1_CH));

          // This is a continuous query
          
          break;
           
        case 'X':
          // Stop continuous commands
          
          command_reset();
          
          break;
           
        default:
          command_reset();
          
          break;
        }
      }
    }

    // Check for command timeout
    
    if ((expected_params > 0) &&
        (++command_timeout_counter >=
                      (uint8_t)(COMMAND_TIMEOUT_MSEC / CONTROL_LOOP_CYCLE_MSEC)))
    {
      // Abort

      command_reset();
    }
  }
}
