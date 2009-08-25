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

#include "gyro.h"
#include "accel.h"
#include "motors.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "serial_telemetry.h"


//=============================================================================
//
// Private Definitions
//
//=============================================================================

// AeroQuad Configurator Definitions

#define SENSOR_DISPLAY_RANGE                  1000

#define RECEIVER_ROT_GAIN_DISPLAY_FACTOR      1000
#define RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR  100

#define MOTOR_COMMAND_DISPLAY_MIN             1000
#define MOTOR_COMMAND_DISPLAY_MAX             2000
#define MOTOR_COMMAND_DISPLAY_RANGE           (MOTOR_COMMAND_DISPLAY_MAX - MOTOR_COMMAND_DISPLAY_MIN)


//=============================================================================
//
// Static Functions
//
//=============================================================================

//=============================== serial_read_float() =========================
//
// Used to read floating point values from the serial port

static float serial_read_float()
{
  byte        index     = 0;
  byte        timeout   = 0;
  char        last_char = '\0';
  static char data[128];


  do
  {
    if (Serial.available() == 0)
    {
      delay(10);
      timeout++;
    }
    
    else
    {
      last_char = Serial.read();
      data[index++] = last_char;
      timeout = 0;
    }
  }
  while ((last_char != ';') && (timeout < 5) && (index < 127));

  if (last_char == ';')
    index--;
  data[index] = '\0';
  
  return atof(data);
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

boolean                                // Ret: If true, the query was continous
handle_serial_telemetry(char query)    // In:  Query

{
  uint8_t rot;
  uint8_t dir;
  uint8_t axis;
  boolean cont = false;
  float   dummy;
  float   windup_guard;
  
  
  switch (query)
  {
  case 'A':
    // Receive roll and pitch rotation rate (gyro) PID setting

    rot_rate_pid[ROLL].set_p(serial_read_float());
    rot_rate_pid[ROLL].set_i(serial_read_float());
    rot_rate_pid[ROLL].set_d(serial_read_float());
    rot_rate_pid[ROLL].reset();
    
    rot_rate_pid[PITCH].set_p(serial_read_float());
    rot_rate_pid[PITCH].set_i(serial_read_float());
    rot_rate_pid[PITCH].set_d(serial_read_float());
    rot_rate_pid[PITCH].reset();

    break;
  
  case 'C':
    // Receive yaw rotation rate (gyro) PID settings
    
    rot_rate_pid[YAW].set_p(serial_read_float());
    rot_rate_pid[YAW].set_i(serial_read_float());
    rot_rate_pid[YAW].set_d(serial_read_float());
    rot_rate_pid[YAW].reset();

    break;
  
  case 'E':
    // Receive roll and pitch rotation (auto level) PID settings
    
    rot_pid[ROLL].set_p(serial_read_float());
    rot_pid[ROLL].set_i(serial_read_float());
    rot_pid[ROLL].set_d(serial_read_float());
    rot_pid[ROLL].reset();
    
    rot_pid[PITCH].set_p(serial_read_float());
    rot_pid[PITCH].set_i(serial_read_float());
    rot_pid[PITCH].set_d(serial_read_float());
    rot_pid[PITCH].reset();

    break;
 
  case 'G':
    // Receive auto level configuration values
    
    receiver_rot_gain = serial_read_float() / RECEIVER_ROT_GAIN_DISPLAY_FACTOR;  // levelLimit
    
    receiver_rot_limit = serial_read_float();  // levelOff

    break;
   
  case 'I':
    // Receive flight control configuration values

    // The configurator only supports a single windup guard
  
    windup_guard = serial_read_float();
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      rot_rate_pid[rot].set_windup_guard(windup_guard);
    };
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      rot_pid[rot].set_windup_guard(windup_guard);
    };
    
    receiver_rot_rate_gain = serial_read_float() /
                             RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR;  // xmitFactor

    break;
  
  case 'K':
    // Receive data filtering values
    
    Gyro::set_smooth_factor(serial_read_float());
    
    dummy = serial_read_float();  // *** NOT IMPLEMENTED *** smoothFactor[ACCEL]
    
    RotationEstimator::set_bw(serial_read_float());

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

    break;

  case 'B':
    // Send roll and pitch rotation rate (gyro) PID settings

    Serial.print(rot_rate_pid[ROLL].get_p());
    print_comma();
    Serial.print(rot_rate_pid[ROLL].get_i());
    print_comma();
    Serial.print(rot_rate_pid[ROLL].get_d());
    print_comma();
    Serial.print(rot_rate_pid[PITCH].get_p());
    print_comma();
    Serial.print(rot_rate_pid[PITCH].get_i());
    print_comma();
    Serial.println(rot_rate_pid[PITCH].get_d());

    break;

  case 'D':
    // Send yaw rotation rate (gyro) PID settings
    
    Serial.print(rot_rate_pid[YAW].get_p());
    print_comma();
    Serial.print(rot_rate_pid[YAW].get_i());
    print_comma();
    Serial.println(rot_rate_pid[YAW].get_d());  
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
    break;
    
  case 'H':
    // Send auto level configuration values
    
    Serial.print((int16_t)(receiver_rot_gain * RECEIVER_ROT_GAIN_DISPLAY_FACTOR), DEC);  // levelLimit
    
    print_comma();
    Serial.println(receiver_rot_limit);  // levelOff
    break;
    
  case 'J':
    // Send flight control configuration values

    // The configurator only supports a single windup guard value
    
    Serial.print(rot_rate_pid[ROLL].get_windup_guard());
    
    print_comma();
    Serial.println(receiver_rot_rate_gain * RECEIVER_ROT_RATE_GAIN_DISPLAY_FACTOR);  // xmitFactor
    break;
    
  case 'L':
    // Send data filtering values
    
    Serial.print(Gyro::get_smooth_factor());
    print_comma();
    
    Serial.print(0.0 /* *** NOT IMPLEMENTED *** smoothFactor[ACCEL] */);
    print_comma();
    
    Serial.println(RotationEstimator::get_bw());
    break;
    
  case 'Q':
  // Send sensor data
  
    {
      int8_t   accel_data[NUM_AXIS];

      
      for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
      {
        Serial.print((int)(gyro[rot].get_rad_per_sec() * DEG_IN_RAD));
        print_comma();
      };

      accel_get_current(accel_data);
      for (axis = FIRST_AXIS; axis < NUM_AXIS; axis++)
      {
        Serial.print(accel_data[axis]);  // Not sure about the order
        print_comma();
      };
    }

    Serial.print((int)rot_correction[ROLL]);   // levelAdjust
    print_comma();
    Serial.print((int)rot_correction[PITCH]);   // levelAdjust
    print_comma();

    Serial.print(rot_estimator[ROLL].get_estimate() * DEG_IN_RAD);
    print_comma();
    Serial.println(rot_estimator[PITCH].get_estimate() * DEG_IN_RAD);

    cont = true;
    break;
    
  case 'R':
    // Send raw sensor data

    {
      int8_t   accel_data[NUM_AXIS];

      
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
    }
  
    cont = true;
    break;
    
  case 'S':
  {
    static uint8_t cycle_counter;

    
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

    cont = true;
  };
  
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
    Serial.print((int)rot_correction[ROLL]);   // levelAdjust
    print_comma();
    Serial.print((int)rot_correction[PITCH]);   // levelAdjust

    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      print_comma();
      Serial.print(motor_rot_command[rot], DEC);
    };

    Serial.println();
    cont = true;
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
    
    cont = true;
    break;
     
  case 'X':
    // Stop sending messages
    
    cont = false;
    break;
     
  default:
    cont = false;
    break;
  }

  return cont;
}
