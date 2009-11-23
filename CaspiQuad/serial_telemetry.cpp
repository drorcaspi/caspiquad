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

// Cycle period for continuous queries

#define CONT_QUERY_CYCLE_MSEC 100

// AeroQuad Configurator Definitions

#define SENSOR_DISPLAY_RANGE                  1000

#define RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR  100

#define I_DISPLAY_FACTOR                      1000

#define MOTOR_COMMAND_DISPLAY_MIN             1000
#define MOTOR_COMMAND_DISPLAY_MAX             2000
#define MOTOR_COMMAND_DISPLAY_RANGE           (MOTOR_COMMAND_DISPLAY_MAX - MOTOR_COMMAND_DISPLAY_MIN)


//=============================================================================
//
// Static Functions
//
//=============================================================================

//=============================== parse_float() ===============================
//
// Parse a floating point value from the command string

static
float
parse_float(char **pp_char)   // I/O: On input, pointer to first char
                              //      On output, pointer to the first char of
                              //      next parameter
{
  char *p_start;
  char *p_end;


  p_start = *pp_char;
  p_end = p_start;

  // Find the end of parameter
  
  while ((*p_end != '\0') && (*p_end != ';'))
    p_end++;

  if (*p_end != '\0')
  {
    // We're not at end of string.  Mark the end of paramter and advance the
    // pointer to the next one.
    
    *p_end++ = '\0';
  }
  
  *pp_char = p_end;
  
  return atof(p_start);
}


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
  static char    query               = '\0'; // Command opcode
  static char    command_line[64];           // Command line buffer
  static uint8_t i_command_line      = 0;    // Index to the command line
  static uint8_t query_cycle_counter = 0;    // Continuous commands cycle counter
  static uint8_t cycle_counter;              // Status query cycle counter
  
  boolean        is_command;                 // Flags a command is available
  char           new_char;                   // Character read from serial
  char          *p_command_line;             // Pointer to the command line
  uint8_t        rot;
  uint8_t        dir;
  uint8_t        axis;
  float          dummy;
  float          windup_guard;
  int8_t         accel_data[NUM_AXES];
  

  // Read any available characters into the command line, until LF character
  // or full buffer.

  is_command = false;
  while ((Serial.available() > 0) && (! is_command))
  {
    new_char = Serial.read();
    command_line[i_command_line++] = new_char;
    is_command = (i_command_line >= sizeof(command_line)) || (new_char == '\n');

    // Whenever any character is read, previous outstanding commands are cancelled
    
    query = '\0';
    query_cycle_counter = 0;
  };

  if (is_command)
  {
    // A new command line has been recieved
    
    query = command_line[0];
    p_command_line = &command_line[1];

    // Put a null character at the end of the command, to make sure parameter
    // parsing has an end mark.
    
    command_line[i_command_line - 1] = '\0';

    // Reset the index to the beginning of buffer, to prepare for the next command
    
    i_command_line = 0;
  }

  else if (query != '\0')
  {
    // This is a contiuous query
    
    if (++query_cycle_counter >= (uint8_t)(CONT_QUERY_CYCLE_MSEC / CONTROL_LOOP_CYCLE_MSEC))
    {
      // It is time to re-issue the query
      
      query_cycle_counter = 0;
      is_command = true;
    };
  };

  if (is_command)
  {
    // A command has been received, either a new one or a contnuous one
    
    switch (query)
    {
    case 'A':
      // Receive roll and pitch rotation rate (gyro) PID setting

      rot_rate_pid[ROLL].set_p(parse_float(&p_command_line));
      rot_rate_pid[ROLL].set_i(parse_float(&p_command_line) / I_DISPLAY_FACTOR);
      rot_rate_pid[ROLL].set_d(parse_float(&p_command_line));
      rot_rate_pid[ROLL].reset();
      
      rot_rate_pid[PITCH].set_p(parse_float(&p_command_line));
      rot_rate_pid[PITCH].set_i(parse_float(&p_command_line) / I_DISPLAY_FACTOR);
      rot_rate_pid[PITCH].set_d(parse_float(&p_command_line));
      rot_rate_pid[PITCH].reset();

      query = '\0';
      break;
    
    case 'C':
      // Receive yaw rotation rate (gyro) PID settings
      
      rot_rate_pid[YAW].set_p(parse_float(&p_command_line));
      rot_rate_pid[YAW].set_i(parse_float(&p_command_line) / I_DISPLAY_FACTOR);
      rot_rate_pid[YAW].set_d(parse_float(&p_command_line));
      rot_rate_pid[YAW].reset();

      query = '\0';
      break;
    
    case 'E':
      // Receive roll and pitch rotation (auto level) PID settings
      
      rot_pid[ROLL].set_p(parse_float(&p_command_line));
      rot_pid[ROLL].set_i(parse_float(&p_command_line));
      rot_pid[ROLL].set_d(parse_float(&p_command_line));
      rot_pid[ROLL].reset();
      
      rot_pid[PITCH].set_p(parse_float(&p_command_line));
      rot_pid[PITCH].set_i(parse_float(&p_command_line));
      rot_pid[PITCH].set_d(parse_float(&p_command_line));
      rot_pid[PITCH].reset();

      query = '\0';
      break;
   
    case 'G':
      // Receive auto level configuration values
      
      receiver_rot_gain = (uint8_t)parse_float(&p_command_line);  // levelLimit
      
      receiver_rot_limit = (uint16_t)parse_float(&p_command_line);  // levelOff

      query = '\0';
      break;
     
    case 'I':
      // Receive flight control configuration values

      // The configurator only supports a single windup guard
    
      windup_guard = parse_float(&p_command_line);
      
      for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
      {
        rot_rate_pid[rot].set_windup_guard(windup_guard);
      };
      
      for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
      {
        rot_pid[rot].set_windup_guard(windup_guard);
      };
      
      receiver_rot_rate_gain = parse_float(&p_command_line) /
                               RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR;  // xmitFactor

      query = '\0';
      break;
    
    case 'K':
      // Receive data filtering values
      
      dummy = parse_float(&p_command_line);  // *** NOT IMPLEMENTED *** smoothFactor[GYRO]
      dummy = parse_float(&p_command_line);  // *** NOT IMPLEMENTED *** smoothFactor[ACCEL]
      
      RotationEstimator::set_bw(parse_float(&p_command_line));

      query = '\0';
      break;
    
    case 'W':
      // Write EEPROM

      flight_control_write_eeprom();
      Gyro::write_eeprom();
      RotationEstimator::write_eeprom();

      for (uint8_t rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
      {
        rot_rate_pid[rot].write_eeprom();
        rot_pid[rot].write_eeprom();
      };

      eeprom_write_ver();

      query = '\0';
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

      query = '\0';
      break;

    case 'D':
      // Send yaw rotation rate (gyro) PID settings
      
      Serial.print(rot_rate_pid[YAW].get_p());
      print_comma();
      Serial.print(rot_rate_pid[YAW].get_i() * I_DISPLAY_FACTOR);
      print_comma();
      Serial.println(rot_rate_pid[YAW].get_d());  

      query = '\0';
      break;
      
    case 'F':
      // Send roll and pitch rotation (auto level) PID settings
      
      Serial.print(rot_pid[ROLL].get_p());
      print_comma();
      Serial.print(rot_pid[ROLL].get_i());
      print_comma();
      Serial.print(rot_pid[ROLL].get_d());
      print_comma();
      Serial.print(rot_pid[PITCH].get_p());
      print_comma();
      Serial.print(rot_pid[PITCH].get_i());
      print_comma();
      Serial.println(rot_pid[PITCH].get_d());

      query = '\0';
      break;
      
    case 'H':
      // Send auto level configuration values
      
      Serial.print(receiver_rot_gain, DEC);  // levelLimit
      
      print_comma();
      Serial.println(receiver_rot_limit);  // levelOff

      query = '\0';
      break;
      
    case 'J':
      // Send flight control configuration values

      // The configurator only supports a single windup guard value
      
      Serial.print(rot_rate_pid[ROLL].get_windup_guard());
      
      print_comma();
      Serial.println(receiver_rot_rate_gain * RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR);  // xmitFactor

      query = '\0';
      break;
      
    case 'L':
      // Send data filtering values
      
      Serial.print(1.0 /* *** NOT IMPLEMENTED *** smoothFactor[GYRO] */);
      print_comma();
      
      Serial.print(1.0 /* *** NOT IMPLEMENTED *** smoothFactor[ACCEL] */);
      print_comma();
      
      Serial.println(RotationEstimator::get_bw());

      query = '\0';
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
      
      Serial.println(receiver_get_boolean(GEAR_CH)  ? 1 : 0, BIN);   // Mode
    
      break;
      
    case 'T':
      // Send processed transmitter values
      
      Serial.print(receiver_rot_rate_gain * RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR);  // xmitFactor
      print_comma();

      Serial.print(0 /* transmitterCommand[ROLL] */);
      print_comma();
      Serial.print(0 /* transmitterCommand[PITCH] */);
      print_comma();
      Serial.print(0 /* transmitterCommand[YAW] */);

      print_comma();
      Serial.print(0, DEC);   // levelAdjust - not supported
      print_comma();
      Serial.print(0, DEC);   // levelAdjust - not supported

      for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
      {
        print_comma();
        Serial.print(motor_rot_command[rot], DEC);
      };

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
      
      break;
       
    case 'X':
      // Stop continuous commands
      
      query = '\0';
      break;
       
    default:
      query = '\0';
      break;
    }
  }
}
