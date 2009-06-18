#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>

#include "quad.h"
#include "motors.h"
#include "gyro.h"
#include "accel.h"
#include "receiver.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "indicator.h"
#include "serial_telemetry.h"
#include "eeprom_utils.h"


//=============================================================================
//
// Global Definitions
//
//=============================================================================

#define CONTROL_LOOP_CYCLE_SEC 0.02 
#define MAX_SENSORS_SETUP_SEC  4

#include "WProgram.h"
void setup();
void loop ();
boolean              
accel_init(void);
void
accel_read(int8_t accel_data[NUM_AXIS]);
void
accel_get_current(int8_t accel_data[NUM_AXIS]);
void accel_print_stats(void);
void eeprom_init(void);
boolean              
                     
                     
eeprom_is_ok(void);
void eeprom_write_ver(void);
float eeprom_read_float(int address);
void eeprom_write_float(int   address,
                        float data);
void motors_init(void);
void motors_enable(void);
void motors_disable(void);
boolean                 
                        
motors_command(
  int16_t  throttle,     
  int16_t  pitch_rate,   
  int16_t  roll_rate,    
  int16_t  yaw_rate);
boolean                 
                        
motors_command(
  int16_t  throttle,     
  int16_t  pitch_rate,   
  int16_t  roll_rate,    
  int16_t  yaw_rate);
uint8_t motors_get_current_command(uint8_t dir);
uint8_t motors_get_short_avg(uint8_t dir);
uint8_t motors_get_long_avg(uint8_t dir);
void motors_print_stats(void);
void pcint_detach(uint8_t pin);
static void pcint_isr_common(uint8_t port);
void pcint_print_stats();
static void receiver_pci_handler(void     *p_usr,
                                 uint8_t   masked_in,
                                 uint32_t  curr_ticks);
boolean              
receiver_init(void);
boolean                     
receiver_get_status(void);
uint16_t                              
receiver_get_current_raw(uint8_t ch);
void receiver_print_stats(void);
static float serial_read_float();
static void print_semicolon(void);
static void print_comma();
boolean                                 
handle_serial_telemetry(char query);
typedef enum

{
  FLIGHT_SETUP,
  FLIGHT_READY
} FlightState;


//=============================================================================
//
// Global Variables
//
//=============================================================================

// EEPROM Variables


// Cycle Timing Variables

uint32_t          last_msec = 0;
uint16_t          avg_cycle_msec = 0;
uint8_t           max_cycle_msec = 0;

// Flight Control Variables

FlightState       flight_state = FLIGHT_SETUP;
uint8_t           sensors_setup_cycles = 0;

Gyro              gyro[NUM_ROTATIONS];
RotationEstimator rot_estimator[NUM_ROTATIONS];

float             g_windup_guard = 3.0;

PID               rot_rate_pid[NUM_ROTATIONS];
PID               rot_pid[NUM_ROTATIONS];

float             rot_correction[NUM_ROTATIONS];   // (rad/sec)

//float             gyro_rad_per_sec[NUM_ROTATIONS];

float             receiver_rot_rate_gain = 0.002;  // (rad/sec)
float             receiver_rot_gain      = 0.002;  // (rad)
uint16_t          receiver_rot_limit     = 100;

int16_t           motor_throttle_command;
int16_t           motor_rot_command[NUM_ROTATIONS];

// Telemetry Variables

char              query;
boolean           cont_query = false;


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  uint8_t  rot;
  int      eeprom_addr = EEPROM_FLIGHT_CONTROL_BASE_ADDR;


  analogReference(ANALOG_REFERENCE);
  Serial.begin(115200);

  Indicator::init();
  eeprom_init();
  motors_init();
  accel_init();
  
  // Gyro
  
  eeprom_addr = Gyro::read_eeprom(eeprom_addr, 0.2);  // Default smooting factor
  gyro[PITCH].init(PITCH_RATE_PIN);
  gyro[ROLL].init(ROLL_RATE_PIN);
  gyro[YAW].init(YAW_RATE_PIN);

  // Receiver
  
  receiver_init();

  // Rotation Estimators
  
  RotationEstimator::set_cycle(CONTROL_LOOP_CYCLE_SEC);
  eeprom_addr = RotationEstimator::read_eeprom(eeprom_addr, 1);

  // PIDs
  
  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    eeprom_addr = rot_rate_pid[rot].read_eeprom(eeprom_addr,
                                                1,               // P
                                                0,               // I
                                               -50,              // D
                                                g_windup_guard); // windup_guard (rad/sec/sec)
  };

  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    eeprom_addr = rot_pid[rot].read_eeprom(eeprom_addr,
                                           1.5,                  // P
                                           0,                    // I
                                          -30,                   // D
                                           g_windup_guard);      // windup_guard (rad/sec)
  };

  Indicator::indicate(Indicator::SETUP);
  last_msec = millis();
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop ()
{
  uint32_t current_msec;
  uint8_t  cycle_msec;
  int8_t   accel_raw[NUM_AXIS];
  //uint16_t gyro_raw[NUM_ROTATIONS];
  float    rotation_raw[NUM_ROTATIONS];
  float    pitch_estimate;                     // (rad)
  float    roll_estimate;                      // (rad)
  int16_t  temp_receiver_raw;
  float    target_rot[NUM_ROTATIONS];        // (rad)
  float    target_rot_rate[NUM_ROTATIONS];   // (rad/sec)
  float    rot_error[NUM_ROTATIONS];        // (rad)
  float    rot_rate_error[NUM_ROTATIONS];   // (rad/sec)
  //uint16_t throttle_command;
  uint8_t  rot;
  boolean  receiver_ok;
  

  //------------------------------
  // Wait for start of next cycle
  //------------------------------
  
  current_msec = millis();
  cycle_msec = (uint8_t)(current_msec- last_msec);
  
#if 1
  if (cycle_msec > (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
    Indicator::indicate(Indicator::SW_WARN);
#endif

  if (cycle_msec > max_cycle_msec)
    max_cycle_msec = cycle_msec;

  avg_cycle_msec -= avg_cycle_msec >> 8;
  avg_cycle_msec += cycle_msec;

  while (cycle_msec < (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
  {
    current_msec = millis();
    cycle_msec = (uint8_t)(current_msec- last_msec);
  };

  last_msec = current_msec;

#if PRINT_CYCLE_TIME
  Serial.print(cycle_msec, DEC);
  Serial.print("\t");
  Serial.print(max_cycle_msec, DEC);
  Serial.print("\t");
  Serial.println(avg_cycle_msec >> 8, DEC);
#endif

  //-------------------------------- 
  // Now do the flight control work
  //--------------------------------

  // Read the accelerometers and calculate raw rotations
  
  accel_read(accel_raw);
  
#if PRINT_ACCEL
  accel_print_stats();
#endif

  // Read gyros, center and scale

  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    gyro[rot].update();

#if PRINT_GYRO
    gyro[rot].print_stats();
#endif
  };

#if PRINT_GYRO
  Serial.println();
#endif

  pitch_estimate = rot_estimator[PITCH].estimate(gyro[PITCH].get_rad_per_sec(),
                                                 accel_raw[X_AXIS],
                                                 accel_raw[Z_AXIS]);
  roll_estimate = rot_estimator[ROLL].estimate(gyro[ROLL].get_rad_per_sec(),
                                               accel_raw[Y_AXIS],
                                               accel_raw[Z_AXIS]);

#if PRINT_ROT_ESTIMATE
  rot_estimator[ROLL].print_stats();
#endif

  if (flight_state == FLIGHT_SETUP)
  {
    //-----------------
    // Setup Phase
    //-----------------

    if (gyro[PITCH].is_stable() &&
        gyro[ROLL].is_stable()  &&
        gyro[YAW].is_stable())
    {
      gyro[PITCH].zero();
      gyro[ROLL].zero();
      gyro[YAW].zero();
      
      flight_state = FLIGHT_READY;
      Indicator::indicate(Indicator::FLIGHT);  // TODO: indicate warning first!
      
      motors_enable();
    }

    else if (sensors_setup_cycles <= (uint8_t)(MAX_SENSORS_SETUP_SEC / CONTROL_LOOP_CYCLE_SEC))
      sensors_setup_cycles++;

    else
    {
      sensors_setup_cycles = 0;
      Indicator::indicate(Indicator::SETUP_ERR);
    }
  }
    

  else
  {
    //-----------------
    // Flight Phase
    //-----------------

    receiver_ok = receiver_get_status();
    temp_receiver_raw = 0;

    // Read pitch command and calculate error
    
    if (receiver_ok)
      temp_receiver_raw = (int16_t)(receiver_get_current_raw(PITCH_CH) -
                                    (uint16_t)RECEIVER_NOM_MID);

    if ((temp_receiver_raw > (int16_t)receiver_rot_limit)  ||
        (temp_receiver_raw < (int16_t)-receiver_rot_limit))
    {
      // Target is pitch rate
      
      rot_rate_error[PITCH] = ((float)temp_receiver_raw * receiver_rot_rate_gain) -
                              gyro[PITCH].get_smoothed_rad_per_sec();
      rot_error[PITCH] = 0.0;
    }

    else
    {
      // Target is pitch 
      
      rot_rate_error[PITCH] = -gyro[PITCH].get_smoothed_rad_per_sec();
      rot_error[PITCH] = ((float)temp_receiver_raw * receiver_rot_gain) -
                         pitch_estimate;
    };

    // Read roll command and calculate error
      
    if (receiver_ok)
      temp_receiver_raw = (int16_t)(receiver_get_current_raw(ROLL_CH) -
                                    (uint16_t)RECEIVER_NOM_MID);

    if ((temp_receiver_raw > (int16_t)receiver_rot_limit)  ||
        (temp_receiver_raw < (int16_t)-receiver_rot_limit))
    {
      // Target is roll rate
      
      rot_rate_error[ROLL] = ((float)temp_receiver_raw * receiver_rot_rate_gain) -
                              gyro[ROLL].get_smoothed_rad_per_sec();
      rot_error[ROLL] = 0.0;
    }
    
    else
    {
      // Target is roll
      
      rot_rate_error[ROLL] = -gyro[ROLL].get_smoothed_rad_per_sec();
      rot_error[ROLL] = ((float)temp_receiver_raw * receiver_rot_gain) -
                        roll_estimate;
    };

    // Read yaw command and calculate error
        
    if (receiver_ok)
      temp_receiver_raw = (int16_t)(receiver_get_current_raw(YAW_CH) -
                                    (uint16_t)RECEIVER_NOM_MID);

    // Target is yaw rate
    
    rot_rate_error[YAW] = ((float)temp_receiver_raw * receiver_rot_rate_gain) - 
                          gyro[YAW].get_smoothed_rad_per_sec();
    target_rot[YAW] = 0.0;

#if PRINT_RECEIVER_ROT
  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    Serial.print(rot_rate_error[rot]);
    Serial.print("\t");
    Serial.print(rot_error[rot]);
    Serial.print("\t");
  };
  Serial.println("\t");
#endif

#if 0
    rot_correction[PITCH] = rot_pid[PITCH].update(0.0,              // target_position (rad)
                                                  pitch_estimate);  // current_position (rad)
    rot_correction[ROLL] = rot_pid[ROLL].update(0.0,                // target_position (rad)
                                                roll_estimate);     // current_position (rad)
#if PRINT_ROT_CORRECTION
    Serial.print(rot_correction[PITCH]);
    Serial.print("\t");
    Serial.println(rot_correction[ROLL]);
#endif
#endif

#if 1  // Two PIDs using both rotation and gyro
  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    motor_rot_command[rot] = rot_pid[rot].update(rot_error[rot]) +
                             rot_rate_pid[rot].update(rot_rate_error[rot]);
  };
#endif
#if 0  // PID using both rotation and gyro
  motor_rot_command[PITCH] = 
    rot_rate_pid[PITCH].update(target_rot[PITCH],           // target position (rad)
                               pitch_estimate,                // current position (rad)
                               target_rot_rate[PITCH],      // target_rate (rad/sec)
                               gyro[PITCH].get_smoothed_rad_per_sec());
                                                              // current_rate (rad/sec)
  motor_rot_command[ROLL] = 
    rot_rate_pid[ROLL].update(target_rot[ROLL],             // target position (rad)
                              roll_estimate,                  // current position (rad)
                              target_rot_rate[ROLL],        // target_rate (rad/sec)
                              gyro[ROLL].get_smoothed_rad_per_sec());
                                                              // current_rate (rad/sec)
  motor_rot_command[YAW] = 
    rot_rate_pid[YAW].update(0.0,                             // target position (rad)
                             0.0,                             // current position (rad)
                             target_rot_rate[YAW],          // target_rate (rad/sec)
                             gyro[YAW].get_smoothed_rad_per_sec());
                                                              // current_rate (rad/sec)
#endif
#if 0  
    motor_rot_command[PITCH] = 
      rot_rate_pid[PITCH].update(rot_correction[PITCH],         // target_position (rad/sec)
                                 gyro[PITCH].get_smoothed_rad_per_sec());
                                                                // current_position (rad/sec)
    motor_rot_command[ROLL] = 
      rot_rate_pid[ROLL].update(rot_correction[ROLL],           // target_position (rad/sec)
                                gyro[ROLL].get_smoothed_rad_per_sec());
                                                                // current_position (rad/sec)
    motor_rot_command[YAW] = 
      rot_rate_pid[YAW].update(0.0,                             // target_position (rad/sec)
                               gyro[YAW].get_smoothed_rad_per_sec());
                                                                // current_position (rad/sec)
#endif
#if 0
    motor_rot_command[PITCH] = motor_rot_gain * rot_correction[PITCH];
    motor_rot_command[ROLL] = motor_rot_gain * rot_correction[ROLL];
    motor_rot_command[YAW] = 0.0;
#endif

#if PRINT_MOTOR_ROT_COMMAND           
    Serial.print(motor_rot_command[PITCH]);
    Serial.print("\t");
    Serial.print(motor_rot_command[ROLL]);
    Serial.print("\t");
    Serial.print(motor_rot_command[YAW]);
    Serial.println("\t");
#endif

#if PRINT_RECEIVER
    receiver_print_stats();
#endif

    if (receiver_get_status())
      motor_throttle_command =
                     (receiver_get_current_raw(THROTTLE_CH)  - RECEIVER_NOM_MIN) /
                     ((RECEIVER_NOM_MAX - RECEIVER_NOM_MIN) / MOTOR_THROTTLE_RANGE);
    else
      motor_throttle_command = MOTOR_THROTTLE_MIN;
    
    motors_command(motor_throttle_command,
                   0 /*motor_rot_command[PITCH]*/,
                   motor_rot_command[ROLL],
                   0 /*motor_rot_command[YAW]*/);
  } 
  
#if PRINT_MOTOR_COMMAND
  motors_print_stats();
#endif

  //-----------
  // Telemetry
  //-----------
  
#if SUPPORT_TELEMENTRY 
  // Check for a new serial command
  
  if (Serial.available())
  {
    query = Serial.read();
    cont_query = true;
  }

  // New or continuous command
  
  if (cont_query)
  {
    cont_query = handle_serial_telemetry(query);
  }
#endif

  Indicator::update();
}

//=============================================================================
//
// PID Class Implementation
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
#include "pid.h"


//=============================================================================
//
// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
//
//=============================================================================


//================================ Constructors ===============================
//
// Constructors
//
// Init a PID object with the given configuration parameters (see their
// description above)

PID::PID(void)

{
  reset();
  // TODO: what about last position
};

PID::PID(float p_in, float i_in, float d_in, float windup_guard_in)

{
  set_params(p_in, i_in, d_in, windup_guard_in);
  reset();
  // TODO: what about last position
};


//============================== set_params() =================================
//
// Set PID object's configuration parameters

void PID::set_params(float p_in, float i_in, float d_in, float windup_guard_in)

{
  p = p_in;
  i = i_in;
  d = d_in;
  windup_guard = windup_guard_in;
};

void PID::set_p(float p_in)

{
  p = p_in;
};

void PID::set_i(float i_in)

{
  i = i_in;
};

void PID::set_d(float d_in)

{
  d = d_in;
};

void PID::set_windup_guard(float windup_guard_in)

{
  windup_guard = windup_guard_in;
};


//============================== get_*() ======================================
//
// Get PID object's configuration parameters

float PID::get_p(void)

{
  return p;
};

float PID::get_i(void)

{
  return i;
};

float PID::get_d(void)

{
  return d;
};

float PID::get_windup_guard(void)
 
{
 return windup_guard;
};
 

//============================== update() =====================================
//
// Perform the PID algorithm and calulate the correction value

float PID::update(float error)

{
  float d_term;


  // Calculate error integral and limit integrated_error to +/-windup_guard
  
  integrated_error += error;
  integrated_error = constrain(integrated_error, -windup_guard, windup_guard);

  // Calculate error derivative 
  
  d_term = d * (error - last_error);
  last_error = error;
  
  return (p * error) + (i * integrated_error) + d_term;
};


//============================== update() =====================================
//
// Perform the PID algorithm and calulate the correction value

float PID::update(float target_position, float current_position)

{
  float error;
  float d_term;


  error = target_position - current_position;

  // Calculate error integral and limit integrated_error to +/-windup_guard
  
  integrated_error += error;
  integrated_error = constrain(integrated_error, -windup_guard, windup_guard);

  // Calculate error derivative 
  
  d_term = d * (error - last_error);
  last_error = error;
  
  return (p * error) + (i * integrated_error) + d_term;
};


//============================== update() =====================================
//
// Perform the PID algorithm and calulate the correction value, given target
// rate and current rate

float PID::update(float target_position,
                  float current_position,
                  float target_rate,
                  float current_rate)

{
  float error;
  float d_term;


  error = target_position - current_position;
  
  // Calculate error integral and limit integrated_error to +/-windup_guard
  
  integrated_error += error;
  integrated_error = constrain(integrated_error, -windup_guard, windup_guard);
  
  last_error = error;
  
  return (p * error) + (i * integrated_error) + (d * (target_rate - current_rate));
};


//============================== reset() ======================================
//
// Zero the integrated error part of the PID

void PID::reset(void)

{
  integrated_error = 0;
};


//============================== read_eeprom() ==============================
//
// Read the configuration parameters from EEPROM, if valid.  If not, set
// defaults.

int                             // Ret: Next address in EEPROM
PID::read_eeprom(
  int   eeprom_base_addr_in,    // In: Base address in EEPROM
  float default_p,              // In:  Proportional factor
  float default_i,              // In:  Integral factor
  float default_d,              // In:  Derivative factor
  float default_windup_guard)   // In:  Limit on the abs. value of integrated_error

{
  int eeprom_addr = eeprom_base_addr_in;


  eeprom_base_addr = eeprom_addr;  // Save for later write

  if (eeprom_is_ok())
  {
    p = eeprom_read_float(eeprom_addr);
    eeprom_addr += sizeof(p);
    i = eeprom_read_float(eeprom_addr);
    eeprom_addr += sizeof(i);
    d = eeprom_read_float(eeprom_addr);
    eeprom_addr += sizeof(d);
    windup_guard = eeprom_read_float(eeprom_addr);
    eeprom_addr += sizeof(windup_guard);
  }

  else
  {
    set_params(default_p, default_i, default_d, default_windup_guard);
    eeprom_addr += sizeof(p) + sizeof(i) + sizeof(d) + sizeof(windup_guard);
  };

  return eeprom_addr;
};


//============================== write_eeprom() =============================
//
// Write the configuration parameters to EEPROM

void PID::write_eeprom(void)

{
  int eeprom_addr = eeprom_base_addr;

  
  eeprom_write_float(eeprom_addr, p);
  eeprom_addr += sizeof(p);
  eeprom_write_float(eeprom_addr, i);
  eeprom_addr += sizeof(i);
  eeprom_write_float(eeprom_addr, d);
  eeprom_addr += sizeof(d);
  eeprom_write_float(eeprom_addr, windup_guard);
};


//=============================================================================
//
// LIS302DL Accelerometers Handler
//
// Uses TWI (I2C) communicate with the accelerometer
//
// Using the Wire library (created by Nicholas Zambetti)
// http://wiring.org.co/reference/libraries/Wire/index.html
// On the Arduino board, Analog In 4 is SDA, Analog In 5 is SCL
// These correspond to pin 27 (PC4/ADC4/SDA) and pin 28 (PC5/ADC5/SCL) on the Atmega8
// The Wire class handles the TWI transactions, abstracting the nitty-gritty to make
// prototyping easy.
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
#include "accel.h"

#include <Wire.h>

//-----------------------------------------------------------------------------
//
// LIS302DL Accelerometer Definitions
//
//-----------------------------------------------------------------------------

#define LIS302DL_0_ADDRESS      0x1C   // Device Addresss

// Device Registers

#define LIS302DL_WHO_AM_I       0x0F
#define LIS302DL_WHO_AM_I_VALUE 0x3B   // Constant value of WHO_AM_I

#define LIS302DL_CTRL_REG1      0x20

#define LIS302DL_OUT_X          0x29
#define LIS302DL_OUT_Y          0x2B
#define LIS302DL_OUT_Z          0x2D


//=============================================================================
//
// Static Variables
//
//=============================================================================

static int8_t current_accel_data[NUM_AXIS];


//=============================== accel_init() ================================
//
// Initialize the accelerometers module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
accel_init(void)
{
  uint8_t who_am_i_val;
  

  current_accel_data[X_AXIS] = 0;
  current_accel_data[Y_AXIS] = 0;
  current_accel_data[Z_AXIS] = ACCEL_1G;
      
  Wire.begin();   // join i2c bus (address optional for master)

  // Read the WHO_AM_I register to make sure it's there
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_WHO_AM_I); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    who_am_i_val = Wire.receive();
  };

  if (who_am_i_val != LIS302DL_WHO_AM_I_VALUE)
    return false;

  // Write CTRL_REG1    
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_CTRL_REG1);
  Wire.send(0x47);    // Device on, 100hz, normal mode, all axis\u2019s enabled,
                      // +/- 2.3G range
  Wire.endTransmission();

  return true;
};


//=============================== accel_read() ================================
//
// Read the accelerometers

void
accel_read(int8_t accel_data[NUM_AXIS])   // Out: 3 axis data

{
  // Read X
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_OUT_X); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    current_accel_data[X_AXIS] = (int8_t)Wire.receive();
  }

  // Read Y
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_OUT_Y); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    current_accel_data[Y_AXIS] = (int8_t)Wire.receive();
  }

  // Read Z
  
  Wire.beginTransmission(LIS302DL_0_ADDRESS);
  Wire.send(LIS302DL_OUT_Z); 
  Wire.endTransmission();
  Wire.requestFrom(LIS302DL_0_ADDRESS, 1);
  while(Wire.available())
  {
    current_accel_data[Z_AXIS] = (int8_t)Wire.receive();
  }
  accel_get_current(accel_data);
};


//========================== accel_get_current() ==============================
//
// Get the current accelerometers data (that has been read before from the h/w)
// This function is intended for reading of telemetry.

void
accel_get_current(int8_t accel_data[NUM_AXIS])   // Out: 3 axis data

{
  accel_data[X_AXIS] = current_accel_data[X_AXIS];
  accel_data[Y_AXIS] = current_accel_data[Y_AXIS];
  accel_data[Z_AXIS] = current_accel_data[Z_AXIS];
};


#if PRINT_ACCEL
//========================== accel_print_stats() ==============================
//
// Print some statistics (for debug)

void accel_print_stats(void)

{
  Serial.print(current_accel_data[X_AXIS], DEC);
  Serial.print("\t");
  Serial.print(current_accel_data[Y_AXIS], DEC);
  Serial.print("\t");
  Serial.println(current_accel_data[Z_AXIS], DEC);
};
#endif


//=============================================================================
//
// EEPROM Utilities
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


//=============================================================================
//
// Private Definitions
//
//=============================================================================

#define EEPROM_VER_ADDR     (EEPROM_UTILS_BASE_ADDR + 0)  // Version #
#define EEPROM_VER_INV_ADDR (EEPROM_UTILS_BASE_ADDR + 1)  // Inverted version #


//=============================================================================
//
// Static Variables
//
//=============================================================================

static boolean eeprom_ok;


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================= eeprom_init() =================================
//
// Initializes the EEPROM handler.  Compares the version number read from the
// EEPROM with the current number.

void eeprom_init(void)

{
  eeprom_ok = (EEPROM.read(EEPROM_VER_ADDR)     == (uint8_t)EEPROM_VER) &&
              (EEPROM.read(EEPROM_VER_INV_ADDR) == (uint8_t)~EEPROM_VER);
#if PRINT_EEPROM
  Serial.print("EEPROM Read Version: ");
  Serial.print(EEPROM.read(EEPROM_VER_ADDR), HEX);
  Serial.print("  EEPROM Read ~Version: ");
  Serial.print(EEPROM.read(EEPROM_VER_INV_ADDR), HEX);
  Serial.print("  EEPROM OK: ");
  Serial.println(eeprom_ok, DEC);
#endif
};


//============================= eeprom_is_ok() ================================
//
// Returns the EEPROM data validity status

boolean             // Ret: status, true if OK, false if EEPROM is
                    //      does not contain valid data.  In this case it must
                    //      be written first.
eeprom_is_ok(void)

{
  return eeprom_ok;
};


//============================= eeprom_write_ver() ============================
//
// Write the current EEPROM version number

void eeprom_write_ver(void)

{
  EEPROM.write(EEPROM_VER_ADDR, (uint8_t)EEPROM_VER);
  EEPROM.write(EEPROM_VER_INV_ADDR, (uint8_t)~EEPROM_VER);

  eeprom_ok = true;

#if PRINT_EEPROM
  Serial.println("Wrote EEPROM version");
#endif
};


//============================= eeprom_read_float() ===========================
//
// Read a floating point number (4 bytes) from the given EEPROM address

float eeprom_read_float(int address)

{
  float    data;
  uint8_t *p_byte;
  uint8_t  i;


  // Read the data byte-by-byte
  
  p_byte = (uint8_t *)&data;
  
  for (int i = 0; i < sizeof(data); i++) 
    *p_byte++ = EEPROM.read(address++);

#if PRINT_EEPROM
  Serial.print("Read EEPROM @ ");
  Serial.print(address - sizeof(data), HEX);
  Serial.print(": ");
  Serial.println(data);
#endif
  return data;
};


//============================ eeprom_write_float() ===========================
//
// Write a floating point number (4 bytes) to the given EEPROM address

void eeprom_write_float(int   address,
                        float data)

{
  uint8_t *p_byte;
  uint8_t  i;


  // Read the data byte-by-byte
  
  p_byte = (uint8_t *)&data;
  
#if PRINT_EEPROM
  Serial.print("Write EEPROM @ ");
  Serial.print(address, HEX);
  Serial.print(": ");
  Serial.println(data);
#endif

  for (int i = 0; i < sizeof(data); i++) 
    EEPROM.write(address++, *p_byte++);

};


//=============================================================================
//
// LIS300AL Gyroscopes Handler
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
#include "gyro.h"


//=============================================================================
//
// Gyro Device Definitions
//
// The following defintions are based on the LISY300AL datasheet
//
//=============================================================================

#define GYRO_RANGE_DEG_PER_SEC      600.0     // Measurement range (deg/sec)
#define GYRO_SENS_V_PER_DEG_PER_SEC   0.0033  // Sensitivity (volts / deg/sec)
#define GYRO_ZERO_V                   1.65    // Zero-rate level (volts)
#define GYRO_BW_HZ                   85.0     // Bandwidth (Hz)

// Derived Definitions in Volts

#define GYRO_RANGE_V (GYRO_RANGE_DEG_PER_SEC * GYRO_SENS_V_PER_DEG_PER_SEC)

// Derived Definitions in Radians

#define GYRO_RANGE_RAD_PER_SEC      (GYRO_RANGE_DEG_PER_SEC / DEG_IN_RAD)
#define GYRO_SENS_V_PER_RAD_PER_SEC (GYRO_SENS_V_PER_DEG_PER_SEC * DEG_IN_RAD)

// Derived Definitions in Numbers

#define GYRO_RANGE                  (GYRO_RANGE_V / ADC_SENS_V)
#define GYRO_SENS_PER_DEG_PER_SEC   (GYRO_SENS_V_PER_DEG_PER_SEC / ADC_SENS_V)
#define GYRO_SENS_PER_RAD_PER_SEC   (GYRO_SENS_PER_DEG_PER_SEC * DEG_IN_RAD)
#define GYRO_ZERO                   (GYRO_ZERO_V / ADC_SENS_V)

//-----------------------------------------------------------------------------
//
// Gyro rest state determination

#define GYRO_REST_AVG_DEV_MAX       2    // How much the gyro reading can
                                         // deviate from the long-term average
                                         // and still be considered stable.
#define GYRO_REST_ZERO_DEV_MAX    100    // How much the gyro reading can
                                         // deviate from zero and still be
                                         // zero if stable.
#define GYRO_LONG_AVG_FACTOR        6    // Determines the averaging period. For
                                         // 20 msec rate this is 2^6 * 20 which
                                         // is a little more than 1 sec.
#define GYRO_REST_CYCLES_MIN      100    // Number of cycles the gyro must be
                                         // at rest to flag a stable condition
                                         

//=============================================================================
//
// Static Variables
//
//=============================================================================

float Gyro::smooth_factor;       // Smooting factor, used in 1-pole IIR LPF
float Gyro::one_minus_smooth_factor;
                                 // 1 - smooth_factor
int   Gyro::eeprom_base_addr;    // Base address in EEPROM


//=================================== set_*() =================================
//

void Gyro::set_smooth_factor(float smooth_factor_in)

{
  smooth_factor = smooth_factor_in;
  one_minus_smooth_factor = 1 - smooth_factor_in;
};


//============================= Constructor ===================================

Gyro::Gyro(void)

{
  cycle_counter = 0;
  raw_zero = (uint16_t)GYRO_ZERO;
  raw = (uint16_t)GYRO_ZERO;
  raw_avg = (uint16_t)GYRO_ZERO << GYRO_LONG_AVG_FACTOR;
  last_unstable_cycle = 0;
  stable_flag = false;
  rad_per_sec = 0;
};


//================================= init() ====================================
//
// Initialize a gyro object

void
Gyro::init(uint8_t ain_in)

{
  ain = ain_in;
  pinMode(ain_in, INPUT);
};


//=============================== update() ====================================
//
// Read the gyroscope's raw data from the h/w and perform all the required
// calculations.  Called periodically.

void
Gyro::update(void)

{
  int16_t avg_diff;
  int16_t zero_diff;

  
  raw = analogRead(ain);

  // Calculate long-term average, in units of (1 << GYRO_LONG_AVG_FACTOR)

  avg_diff = (int16_t)(raw - (raw_avg >> GYRO_LONG_AVG_FACTOR));
  raw_avg += avg_diff;

  zero_diff = (int16_t)(raw - raw_zero);
  
  // To be stable, the current reading must not deviate from average too much.
  // It also must not deviate from zero too much.
  
  if ((avg_diff > (int16_t)GYRO_REST_AVG_DEV_MAX)    ||
      (avg_diff < (int16_t)-GYRO_REST_AVG_DEV_MAX)   ||
      (zero_diff > (int16_t)GYRO_REST_ZERO_DEV_MAX)  ||
      (zero_diff < (int16_t)-GYRO_REST_ZERO_DEV_MAX)
     )
  {
    // Not stable
    
    stable_flag = false;
    last_unstable_cycle = cycle_counter;
  }

  else if ((uint8_t)(cycle_counter - last_unstable_cycle) >= (uint8_t)GYRO_REST_CYCLES_MIN)
  {
    // We have been at rest for long enough, flag a stable condition

    stable_flag = true;
  }
  
  // Convert to rad/sec
  
  rad_per_sec = (float)zero_diff / (float)GYRO_SENS_PER_RAD_PER_SEC;

  smoothed_rad_per_sec = (one_minus_smooth_factor * smoothed_rad_per_sec) +
                         (smooth_factor * rad_per_sec);
  

  cycle_counter++;
};


//================================= zero() ====================================
//
// Set the gyroscope's zero point to the current long-time average.

void Gyro::zero(void)

{
  raw_zero = raw_avg >> GYRO_LONG_AVG_FACTOR;
};


#if PRINT_GYRO
//============================= print_stats() =================================
//
// Print some statistics (for debug)

void
Gyro::print_stats(void)

{
   Serial.print(raw, DEC);
   Serial.print("\t");
   Serial.print(raw_avg >> GYRO_LONG_AVG_FACTOR, DEC);
   Serial.print("\t");
   Serial.print(raw_zero, DEC);
   Serial.print("\t");
   Serial.print(rad_per_sec);
   Serial.print("\t");
   Serial.print(stable_flag, BIN);
   Serial.print("\t");
};
#endif


//============================== read_eeprom() ==============================
//
// Read the configuration parameters from EEPROM, if valid.  If not, set
// defaults.

int                         // Ret: Next address in EEPROM
Gyro::read_eeprom(
  int   eeprom_base_addr_in,// In: Base address in EEPROM
  float smooth_factor)      // Smooting factor, used in 1-pole IIR LPF

{
  eeprom_base_addr = eeprom_base_addr_in;
  
  if (eeprom_is_ok())
  {
    set_smooth_factor(eeprom_read_float(eeprom_base_addr_in));
  }
  
  else
    set_smooth_factor(smooth_factor);
  
  return eeprom_base_addr_in + sizeof(float);
};


//============================== write_eeprom() =============================
//
// Write the configuration parameters to EEPROM

void Gyro::write_eeprom(void)

{
  eeprom_write_float(eeprom_base_addr, smooth_factor);
};



//=============================================================================
//
// Status Indicators
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
#include "indicator.h"


//=============================================================================
//
// Indicator Definitions
//
//=============================================================================

#define PATTERN_REPEAT ((uint8_t)-1)
#define PATTERN_ONCE   ((uint8_t)-2)
#define PATTERN_END    ((uint8_t) 0)


//=============================================================================
//
// Static Variables
//
//=============================================================================

Indicator::Status Indicator::status        = Indicator::NONE;
Indicator::Status Indicator::temp_status   = Indicator::NONE;
uint8_t           Indicator::led_cycle_counter;
uint8_t           Indicator::led_pattern_counter;

const uint8_t     Indicator::none_led_pattern[]        = {PATTERN_ONCE, PATTERN_END};
const uint8_t     Indicator::setup_led_pattern[]       = {PATTERN_REPEAT,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::setup_next_led_pattern[]  = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::setup_err_led_pattern[]   = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::arming_led_pattern[]      = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          PATTERN_END};
const uint8_t     Indicator::flight_led_pattern[]      = {PATTERN_REPEAT,
                                                          50, 50,
                                                          PATTERN_END};
const uint8_t     Indicator::bat_warning_led_pattern[] = {PATTERN_REPEAT,
                                                          25, 25,
                                                          PATTERN_END};
const uint8_t     Indicator::bat_empty_led_pattern[]   = {PATTERN_REPEAT,
                                                          5, 25,
                                                          PATTERN_END};
const uint8_t     Indicator::hw_err_led_pattern[]      = {PATTERN_REPEAT,
                                                          5, 5,
                                                          PATTERN_END};
const uint8_t     Indicator::sw_warn_led_pattern[]     = {PATTERN_ONCE,
                                                          5, 5,
                                                          5, 5,
                                                          5, 5,
                                                          5, 35,
                                                          PATTERN_END};
const uint8_t     Indicator::sw_err_led_pattern[]      = {PATTERN_REPEAT,
                                                          10, 10,
                                                          PATTERN_END};

const uint8_t *const Indicator::p_patterns[] =
                     {
                       Indicator::none_led_pattern,
                       Indicator::setup_led_pattern,
                       Indicator::setup_next_led_pattern,
                       Indicator::setup_err_led_pattern,
                       Indicator::arming_led_pattern,
                       Indicator::flight_led_pattern,
                       Indicator::bat_warning_led_pattern,
                       Indicator::bat_empty_led_pattern,
                       Indicator::hw_err_led_pattern,
                       Indicator::sw_warn_led_pattern,
                       Indicator::sw_err_led_pattern
                     };


//=============================== init() ====================================
//
// Initialize the statues indicators

void Indicator::init(void)

{
  digitalWrite(LED_PIN, 0);
};


//============================== update() ===================================
//
// Called periodically

Indicator::Status         // Ret: Indicared status indicated.
Indicator::update(void)   // In:  Status to indicate

{
  Status         current_status;
  const uint8_t *p_pattern;

  
  if (temp_status != NONE)
    current_status = temp_status;

  else
    current_status = status;

#if 0    
  Serial.print(status, DEC);
  Serial.print("\t");
  Serial.print(temp_status, DEC);
  Serial.print("\t");
  Serial.print(current_status, DEC);
  Serial.print("\t");
  Serial.print(led_cycle_counter, DEC);
  Serial.print("\t");
  Serial.println(led_pattern_counter, DEC);
#endif  

  if (current_status != NONE)
  {
    p_pattern = p_patterns[current_status];

    if (--led_cycle_counter == 0)
    {
      if (p_pattern[++led_pattern_counter] == PATTERN_END)
      {
        // End of pattern.

        temp_status = NONE;  // Even if we we're displaying temp

        indicate(status);
      }

      else
      {
        // Get next item in pattern
        
        led_cycle_counter = p_pattern[led_pattern_counter];
        //Serial.println(led_cycle_counter, DEC);
        digitalWrite(LED_PIN, led_pattern_counter & 1);
      }
    }
  }

  return current_status;
};


//============================= indicate() ==================================

void
Indicator::indicate(Status status_in)   // In:  Status to indicate

{
  const uint8_t *p_pattern;


  if (status_in == NONE)
  {
    status = NONE;
    temp_status = NONE;
  }

  else
  {
    p_pattern = p_patterns[status_in];

    if (p_pattern[0] == PATTERN_ONCE)
      temp_status = status_in;

    else
      status = status_in;
#if 0
    Serial.print(p_pattern[0], DEC);
    Serial.print("\t");
    Serial.print(status, DEC);
    Serial.print("\t");
    Serial.println(temp_status, DEC);
#endif
    led_pattern_counter = 1;
    led_cycle_counter = p_pattern[1];
    //Serial.println(led_cycle_counter, DEC);
    digitalWrite(LED_PIN, 1);
  }
};


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

#define SHORT_OVERHEAT_THRESHOLD 180 
#define SHORT_OVERHEAT_LIMIT     180

#define LONG_OVERHEAT_THRESHOLD  150 
#define LONG_OVERHEAT_LIMIT      150

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
      motors_current_commands[dir] = MOTOR_COMMAND_MIN;
  }
  
  else
  {
    // Limit the inputs to legal values
    
    throttle   = constrain(throttle,   MOTOR_THROTTLE_MIN,      MOTOR_THROTTLE_MAX);
    roll_rate  = constrain(roll_rate,  MOTOR_ROTATION_RATE_MIN, MOTOR_ROTATION_RATE_MAX);
    pitch_rate = constrain(pitch_rate, MOTOR_ROTATION_RATE_MIN, MOTOR_ROTATION_RATE_MAX);
    yaw_rate   = constrain(yaw_rate,   MOTOR_ROTATION_RATE_MIN, MOTOR_ROTATION_RATE_MAX);
      
    // Calculate motor commands.  Do this in signed 16 bits to avoid overflow
    // or underflow.
    
    motors_current_commands[FRONT] = constrain(throttle + pitch_rate + yaw_rate,
                                               MOTOR_COMMAND_MIN,
                                               MOTOR_COMMAND_MAX);
    motors_current_commands[REAR ] = constrain(throttle - pitch_rate + yaw_rate,
                                               MOTOR_COMMAND_MIN,
                                               MOTOR_COMMAND_MAX);
    motors_current_commands[RIGHT] = constrain(throttle + roll_rate  - yaw_rate,
                                               MOTOR_COMMAND_MIN,
                                               MOTOR_COMMAND_MAX);
    motors_current_commands[LEFT ] = constrain(throttle - roll_rate  - yaw_rate,
                                               MOTOR_COMMAND_MIN,
                                               MOTOR_COMMAND_MAX);
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


//=============================================================================
//
// Pin Change Interrupt Handler
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

/*---------------------------------------------------------------------------
 * The code in this file originated in:
 *    http://www.arduino.cc/playground/Main/PcInt
 *
 * This is an extension to the interrupt support for arduino.
 * add pin change interrupts to the external interrupts, giving a way
 * for users to have interrupts drive off of any pin.
 * Refer to avr-gcc header files, arduino source and atmega datasheet.
 *
 *
 * Theory: all IO pins on Atmega168/328 are covered by Pin Change Interrupts.
 * The PCINT corresponding to the pin must be enabled and masked, and
 * an ISR routine provided.  Since PCINTs are per port, not per pin, the ISR
 * must use some logic to actually implement a per-pin interrupt service.
 *
 *
 * Pin to interrupt map:
 * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
 *---------------------------------------------------------------------------*/


#include <pins_arduino.h>
#include "pcint.h"


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Array of PCMSKn register address per port

static volatile uint8_t *const  port_to_p_pcmask[] =
{
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

// Handler function pointers, per pin

static p_pcintHandler           pcint_handler[24] = {NULL};

// User data passed to handler function, per pin

static void                    *pcint_usr[24] = {NULL};

// Last value read from input register

static volatile uint8_t         pcint_last[3];

#ifdef PCINT_DEBUG
static uint8_t                  num_changed_while_processing = 0;
#endif


//=============================== pcint_attach() ==============================
//
// attach an interrupt to a specific pin using pin change interrupts.

void pcint_attach(uint8_t   pin,    // Pin to attach
                  void    (*handler)(void *, uint8_t, uint32_t),
                                    // Pointer to handler function
                  void     *usr)    // User data to pass to handler

{
  uint8_t           bit;        // Bit mask for bit in port
  uint8_t           port;       // 8-bit AVR port (0..2)
  uint8_t           slot;       // 0..23
  volatile uint8_t *p_pcmask;   // Pointer to Pin Change Mask register


  bit  = digitalPinToBitMask(pin);
  port = digitalPinToPort(pin);

  // map pin to PCIR register

  if (port == NOT_A_PORT)
  {
    return;
  } 

  else
  {
    port -= 2;
    p_pcmask = port_to_p_pcmask[port];
  }

  slot = port * 8 + (pin % 8);
  pcint_handler[slot] = handler;
  pcint_usr[slot] = usr;

  // set the mask

  *p_pcmask |= bit;

  // enable the interrupt

  PCICR |= 0x01 << port;
}


//=============================== pcint_detach() ==============================
//
// dettach an interrupt from a specific pin using pin change interrupts.

void pcint_detach(uint8_t pin)

{
  uint8_t           bit;        // Bit mask for bit in port
  uint8_t           port;       // 8-bit AVR port (0..2)
  volatile uint8_t *p_pcmask;   // Pointer to Pin Change Mask register


  bit = digitalPinToBitMask(pin);
  port = digitalPinToPort(pin);
  
  // map pin to PCIR register
  
  if (port == NOT_A_PORT)
  {
    return;
  } 

  else
  {
    port -= 2;
    p_pcmask = port_to_p_pcmask[port];
  }

  // disable the mask.

  *p_pcmask &= ~bit;

  // if that's the last one, disable the interrupt.

  if (*p_pcmask == 0)
  {
    PCICR &= ~(0x01 << port);
  }
}


//=========================== pcint_isr_common() ==============================
//
// common code for isr handler. "port" is the PCINT number.
// there isn't really a good way to back-map ports and masks to pins.

static void pcint_isr_common(uint8_t port)
{
  uint8_t           bit;
  volatile uint8_t *p_reg;              // Pointer to input register
  uint8_t           curr;               // Current value read from input
  uint8_t           last;               // Last value read from input
  uint8_t           pin_changed_mask;   // pcint pins that have changed.
  uint8_t           slot;               // Arduino pin (0..23)
  uint32_t          curr_time;


  curr_time = micros();

  // get the pin states for the indicated port and set pin_changed_mask to the pins that
  // have changed. screen out non pcint pins.

  p_reg = portInputRegister(port + 2);

  curr = *p_reg;
  pin_changed_mask = (curr ^ pcint_last[port]) & *port_to_p_pcmask[port];
  last = curr;

  while (pin_changed_mask != 0)
  {
    slot = port * 8;
#if 1
    for (bit = 0x01; bit != 0; bit <<= 1)
    {
      if ((bit & pin_changed_mask) && (pcint_handler[slot] != NULL))
      {
        pcint_handler[slot](pcint_usr[slot], curr & pin_changed_mask, curr_time);
      }

      slot++;
    }
#endif
#if 0   // This code should be better but for some reason it doesn't work :(
    while (pin_changed_mask != 0)
    {
      uint8_t temp_mask = pin_changed_mask & 0x01;
      
    
      if (temp_mask && (pcint_handler[slot] != NULL))
      {
        pcint_handler[slot](pcint_usr[slot], temp_mask, curr_time);
      }

      pin_changed_mask >>= 1;
      slot++;
    }
#endif
    // Inputs may have changed while we were processing
    
    curr = *p_reg;
    pin_changed_mask = (curr ^ last) & *port_to_p_pcmask[port];
    last = curr;
    
    if (pin_changed_mask != 0)
      num_changed_while_processing++;
  }

  pcint_last[port] = last;
}


//=============================================================================
//
// ISRs for PC interrupts on 3 port registers

ISR(PCINT0_vect)

{
  pcint_isr_common(0);
}


ISR(PCINT1_vect)

{
  pcint_isr_common(1);
}


ISR(PCINT2_vect)

{
  pcint_isr_common(2);
}


#ifdef PCINT_DEBUG
//=============================== pcint_print_stats() =========================
//
// Print some internal statistics (for debug)

void pcint_print_stats()

{
  Serial.print(num_changed_while_processing, DEC);
}
#endif

//=============================================================================
//
// Receiver Handler Module
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
#include "receiver.h"
#include "pcint.h"


//=============================================================================
//
// Local Definitions
//
//=============================================================================


// Absolute minimum and maximum values of raw receiver data.
// Beyond them the receiver is considered to be non-functional.

#define RECEIVER_LOW_MIN  (16000 / RECEIVER_TICK)
#define RECEIVER_LOW_MAX  (24000 / RECEIVER_TICK)
#define RECEIVER_HIGH_MIN (  900 / RECEIVER_TICK)
#define RECEIVER_HIGH_MAX ( 2100 / RECEIVER_TICK)

// Maximum number of consecutive errors before we decalre a fault.
// Experience has shown that from time to time we get too-short or too-long
// pulses from the reciver.  This does not seem to be a s/w bug but either a
// receiver mis-behavior of a h/w problem.  The current solution is to ignore
// illegal-width pulses, if their consecutive number is small.

#define RECEIVER_MAX_ERRORS 4


//=============================================================================
//
// Static Variables
//
//=============================================================================

// Channel data

typedef struct
{
  boolean  last_was_high;   // true if last time channel input was high
  uint8_t  error_count;     // Counts error to detect receiver faults
  uint32_t last_ticks;      // Time (number of ticks) of last pin change
  uint32_t ticks_high;      // Pulse width (number of ticks) last measured
} ReceiverChData;

static ReceiverChData ch_data[NUM_CH];


//=============================================================================
//
// Static Functions
//
//=============================================================================

//======================== receiver_pci_handler() =============================
//
// Handles PCI for receiver pins

static void receiver_pci_handler(void     *p_usr,
                                 uint8_t   masked_in,
                                 uint32_t  curr_ticks)

{
  ReceiverChData *p_ch_data;
  uint32_t        ticks_diff;   // Time diff (# of ticks) from last input change
  boolean         error_flag;   // Flags a receiver error


  p_ch_data = (ReceiverChData *)p_usr;
  
  if (masked_in == 0)
  {
    // high-to-low transition

    if (! p_ch_data->last_was_high)
    {
      // Sanity check failed, last time was low
      
      error_flag = true;
    }

    else
    {
      ticks_diff = curr_ticks - p_ch_data->last_ticks;
      
      if ((ticks_diff < RECEIVER_HIGH_MIN) || 
          (ticks_diff > RECEIVER_HIGH_MAX))
        error_flag = true;

      else
      {
        p_ch_data->ticks_high = ticks_diff;
        p_ch_data->error_count = 0;   // Only successful high resets the counter
        error_flag = false;
      }
    }
    
    p_ch_data->last_was_high = false;
    p_ch_data->last_ticks = curr_ticks;
  }

  else
  {
    // low-to-high transition

    if (p_ch_data->last_was_high)
    {
      // Sanity check failed, last time was high
      
      error_flag = true;
    }

    else
    {

      ticks_diff = curr_ticks - p_ch_data->last_ticks;

      error_flag = ((ticks_diff < RECEIVER_LOW_MIN) || 
                    (ticks_diff > RECEIVER_LOW_MAX));
    }
    
    p_ch_data->last_was_high = true;
    p_ch_data->last_ticks = curr_ticks;
  }

  if (error_flag)
  {
    if (p_ch_data->error_count < RECEIVER_MAX_ERRORS)
      p_ch_data->error_count++;
  }
}


//=============================================================================
//
// Public Functions
//
//=============================================================================

//============================ receiver_init() ================================
//
// Initialize the receiver module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
receiver_init(void)

{
  uint8_t ch;
  
 
  for (ch = FIRST_CH; ch < NUM_CH; ch++)
  {
    ch_data[ch].last_was_high = false;
    ch_data[ch].error_count = RECEIVER_MAX_ERRORS;   // Error until proven otherwise
    ch_data[ch].last_ticks = 0;
    ch_data[ch].ticks_high = 0;
    // ch_data[ch].ticks_low = 0;
  }
  
  pinMode(THROTTLE_CH_PIN, INPUT);
  pinMode(ROLL_CH_PIN,     INPUT);
  pinMode(PITCH_CH_PIN,    INPUT);
  pinMode(YAW_CH_PIN,      INPUT);
  pinMode(GEAR_CH_PIN,     INPUT);
  pinMode(AUX1_CH_PIN,     INPUT);

  pcint_attach(THROTTLE_CH_PIN, receiver_pci_handler, &ch_data[THROTTLE_CH]);
  pcint_attach(ROLL_CH_PIN,     receiver_pci_handler, &ch_data[ROLL_CH]);
  pcint_attach(PITCH_CH_PIN,    receiver_pci_handler, &ch_data[PITCH_CH]);
  pcint_attach(YAW_CH_PIN,      receiver_pci_handler, &ch_data[YAW_CH]);
  pcint_attach(GEAR_CH_PIN,     receiver_pci_handler, &ch_data[GEAR_CH]);
  pcint_attach(AUX1_CH_PIN,     receiver_pci_handler, &ch_data[AUX1_CH]);

  return true;
}


//======================== receiver_get_status() ==============================
//
// Get the current status of the receiver

boolean                    // Ret: true if OK, false if not OK
receiver_get_status(void)

{
  boolean status = true;
  uint8_t ch;


  for (ch = FIRST_CH; ch < NUM_CH; ch++)
  {
    if (ch_data[ch].error_count >= RECEIVER_MAX_ERRORS)
       status = false;
  }

  return status;
}


//====================== receiver_get_current_raw() ===========================
//
// Get the current raw receiver data (that has been read before from the h/w)

uint16_t                             // Ret: raw data, in units of  RECEIVER_TICK
receiver_get_current_raw(uint8_t ch) // In:  channel

{
  uint16_t data;
  uint8_t  old_sreg;


  // Save the interrupt status and disable interrupts.
  // This is required to assure consistent reading of the data.
  
  old_sreg = SREG;
  cli();

  data = ch_data[ch].ticks_high;

  // Restore the interrupt status
  
  SREG = old_sreg;

  return data;
}


#if PRINT_RECEIVER
//========================== receiver_print_stats() ===========================
//
// Print some statistics (for debug)

void receiver_print_stats(void)

{
  uint8_t ch;

  
  if (receiver_get_status())
    Serial.print("GOOD\t");
  else
    Serial.print("BAD\t");
  
  for (ch = FIRST_CH; ch < NUM_CH; ch++)
  { 
    Serial.print(receiver_get_current_raw(ch), DEC);
    Serial.print("\t");
  }
  
  pcint_print_stats();
  Serial.println();
}
#endif


//=============================================================================
//
// Flight Rotation Estimator
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
// Original code written by RoyLB at:
// http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
//=============================================================================

#include "quad.h"
#include "accel.h"
#include "rotation_estimator.h"


//=============================================================================
//
// Local Definitions
//
//=============================================================================

// Minimum accelerometer reading, below which the reading is not considered
// reiable for rotation calculation

#define ROTATION_ESTIMATOR_ACCEL_RAW_MIN (ACCEL_1G / 4)


//=============================================================================
//
// Static Members
//
//=============================================================================

float RotationEstimator::bw          = 1;
float RotationEstimator::cycle       = 0.02;
float RotationEstimator::bw_2;
float RotationEstimator::cycle_bw_sq;

int   RotationEstimator::eeprom_base_addr;   // Base address in EEPROM


//============================== Constructor ==================================
//
// Initializes a RotationEstimator object

RotationEstimator::RotationEstimator()

{
  init(0.0, 0.0);
};
  

//============================== set_*() ======================================
//
// Set the estimator's configurable parameters

void
RotationEstimator::set_bw(
  float bw_in)      // In: Bandwidth of the estimator filter (1/sec). Tune
                    //     this to match sensor performance.
{
  bw = bw_in;
  bw_2 = 2 * bw_in;
  cycle_bw_sq = cycle * bw_in * bw_in;
};
  

void
RotationEstimator::set_cycle(
  float cycle_in)   // In: Iteration cycle of the estimator filter (sec)

{
  cycle = cycle_in;
  cycle_bw_sq = cycle_in * bw * bw;
};
  

//============================== get_*() ======================================
//
// Get the estimator's configurable parameters and state variables

float                       // Ret: Bandwidth of the estimator filter (1/sec).
RotationEstimator::get_bw(void)

{
  return bw;
};

float          // Ret: Rotation estimation (rad).
RotationEstimator::get_estimate(void)

{
  return rotation_estimate;
};


//========================== print_stats() ====================================
//
// Print some statistics (for debug)

void
RotationEstimator::print_stats(void)

{
  Serial.print(bw);
  Serial.print("\t");
  Serial.print(cycle);
  Serial.print("\t");
  Serial.print(integ1_out);
  Serial.print("\t");
  Serial.println(rotation_estimate);
};


//============================== init() =======================================
//
// Initialize the estimator's state variables (integrator outputs)

void
RotationEstimator::init(
  float  rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                             //      scaled from gyro reading
  float  rotation_in)        // In:  Rotation angle measurement, in rad,
                             //      calculated from accelerometer readings

{
  // Set the integrator outputs so we would get exactly 0 at their inputs if
  // we call the estimator with the same values (see estimate()).
  
  rotation_estimate = rotation_in;
  integ1_out = -rotation_rate_in;
};
  

//============================== estimate() ===================================
//
// Estimate rotation angle for one rotation axis, based on rotation rate and
// raw accelerator measurements.

float                        // Ret: New rotation estimate
RotationEstimator::estimate(
  float  rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                             //      scaled from gyro reading
  int8_t accel_raw_base,     // In:  Raw accelerometer reading, base
  int8_t accel_raw_perp)     // In:  Raw accelerometer reading, perpendicular          

{
  float rotation_diff;


  if ((accel_raw_base >=  ROTATION_ESTIMATOR_ACCEL_RAW_MIN)  ||
      (accel_raw_base <= -ROTATION_ESTIMATOR_ACCEL_RAW_MIN)  ||
      (accel_raw_perp >=  ROTATION_ESTIMATOR_ACCEL_RAW_MIN)  ||
      (accel_raw_perp <= -ROTATION_ESTIMATOR_ACCEL_RAW_MIN))
  {
    // Accelerometer readings can be used as raw rotation measurement

#if 0    
    Serial.print(rotation_rate_in);
    Serial.print("\t");
    Serial.println(atan2(accel_raw_base, accel_raw_perp));
#endif

    rotation_diff = atan2(accel_raw_base, accel_raw_perp) - rotation_estimate;

    // First integration
  
    integ1_out += cycle_bw_sq * rotation_diff;

    // Second integration
  
    rotation_estimate += cycle * (integ1_out + (bw_2 * rotation_diff) + rotation_rate_in);
  }

  else
  {
    // Accelerometer readings are not reliable, use only rotation rate input
    
    rotation_estimate += cycle * (integ1_out + rotation_rate_in);
  };
  
  return rotation_estimate;
};
  

#if 0
//============================== estimate() ===================================
//
// Estimate rotation angle for one rotation axis, based on rotation rate and
// rotation angle measurements.

float                        // Ret: New rotation estimate
RotationEstimator::estimate(
  float  rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                             //      scaled from gyro reading
  float  rotation_in)        // In:  Rotation angle measurement, in rad,
                             //      calculated from accelerometer readings          

{
  float rotation_diff;


  rotation_diff = rotation_in - rotation_estimate;

  // First integration
  
  integ1_out += cycle_bw_sq * rotation_diff;

  // Second integration
  
  rotation_estimate += cycle * (integ1_out + (bw_2 * rotation_diff) + rotation_rate_in);

  return rotation_estimate;
};
  

//============================== estimate() ===================================
//
// Estimate rotation angle for one rotation axis, based on rotation rate only.
// This is used in case rotation inputs are not reliable (when accelerometer
// reading on botx axis are close to 0).

float                        // Ret: New rotation estimate
RotationEstimator::estimate(
  float  rotation_rate_in)   // In:  Rotation rate measurement, in rad/sec,
                             //      scaled from gyro reading

{
  // Second integration
  
  rotation_estimate += cycle * (integ1_out + rotation_rate_in);

  return rotation_estimate;
};
#endif


//============================== read_eeprom() ==============================
//
// Read the configuration parameters from EEPROM, if valid.  If not, set
// defaults.

int                         // Ret: Next address in EEPROM
RotationEstimator::read_eeprom(
  int   eeprom_base_addr_in,// In: Base address in EEPROM
  float bw_in)              // In: Bandwidth of the estimator filter (1/sec).
                            //     Tune this to match sensor performance.

{
  eeprom_base_addr = eeprom_base_addr_in;
  
  if (eeprom_is_ok())
  {
    set_bw(eeprom_read_float(eeprom_base_addr_in));
  }
  
  else
    set_bw(bw_in);
  
  return eeprom_base_addr_in + sizeof(float);
};


//============================== write_eeprom() =============================
//
// Write the configuration parameters to EEPROM

void RotationEstimator::write_eeprom(void)

{
  eeprom_write_float(eeprom_base_addr, bw);
};



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

#define SENSOR_DISPLAY_RANGE        1000

#define MOTOR_COMMAND_DISPLAY_MIN   1000
#define MOTOR_COMMAND_DISPLAY_MAX   2000
#define MOTOR_COMMAND_DISPLAY_RANGE (MOTOR_COMMAND_DISPLAY_MAX - MOTOR_COMMAND_DISPLAY_MIN)


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
    // *** NOT IMPLEMENTED YET ***
    
    dummy = serial_read_float();  // levelLimit
    dummy = serial_read_float();  // levelOff

    break;
   
  case 'I':
    // Receive flight control configuration values
  
    g_windup_guard = serial_read_float();
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      rot_rate_pid[rot].set_windup_guard(g_windup_guard);
    };
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      rot_pid[rot].set_windup_guard(g_windup_guard);
    };
    
    dummy = serial_read_float();  // *** NOT IMPLEMENTED *** xmitFactor

    break;
  
  case 'K':
    // Receive data filtering values
    
    Gyro::set_smooth_factor(serial_read_float());
    dummy = serial_read_float();  // *** NOT IMPLEMENTED *** smoothFactor[ACCEL]
    
    RotationEstimator::set_bw(serial_read_float());

    break;
  
  case 'W':
    // Write EEPROM

    Gyro::write_eeprom();
    RotationEstimator::write_eeprom();

    for (uint8_t rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      rot_rate_pid[rot].write_eeprom();
      rot_pid[rot].write_eeprom();
    };

    eeprom_write_ver();
    
#if 0      
    writeFloat(levelLimit, LEVELLIMIT_ADR);   
    writeFloat(levelOff, LEVELOFF_ADR); 
    writeFloat(xmitFactor, XMITFACTOR_ADR);
    writeFloat(smoothFactor[GYRO], GYROSMOOTH_ADR);
    writeFloat(smoothFactor[ACCEL], ACCSMOOTH_ADR);
    zeroIntegralError();
#endif
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
    // *** NOT IMPLEMENTED YET ***
    
    Serial.print(0 /* levelLimit */);
    print_comma();
    Serial.println(0 /* levelOff */);
    break;
    
  case 'J':
    // Send flight control configuration values

    Serial.print(g_windup_guard);
    print_comma();
    Serial.println(0.0 /* *** NOT IMPLEMENTED *** xmitFactor */); 
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
    
      // Scale to display range and print
      
      Serial.print((int16_t)(motor_rot_command[rot] *
                   (int16_t)(SENSOR_DISPLAY_RANGE / MOTOR_ROTATION_RATE_RANGE)));
    };

    for (dir = FIRST_DIRECTION; dir < NUM_DIRECTIONS; dir++)
    {
      print_comma();
      
      // Scale to display range and print
      
      Serial.print((uint16_t)(((motors_get_current_command(dir) - MOTOR_THROTTLE_MIN) *
                               (MOTOR_COMMAND_DISPLAY_RANGE / MOTOR_THROTTLE_RANGE)) +
                              MOTOR_COMMAND_DISPLAY_MIN));
    };
    
    print_comma();

    Serial.print(flight_state == FLIGHT_READY ? 1 : 0, BIN);  // Armed
    print_comma();
    
    Serial.println(0 /* transmitterData[MODE] */);

    cont = true;
    };
  
    break;
    
  case 'T':
    // Send processed transmitter values
    
    Serial.print(0.0 /* xmitFactor */);
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
    
      // Scale to display range and print
      
      Serial.print((int16_t)(motor_rot_command[rot] *
                   (int16_t)(SENSOR_DISPLAY_RANGE / MOTOR_ROTATION_RATE_RANGE)));
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

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

