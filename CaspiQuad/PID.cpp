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
 

//============================== update_pd() ==================================
//
// Perform the PID algorithm, calulate the correction value given measured
// error, using only P and D terms.

float                       // Ret: Correction value
PID::update_pd(float error) // In:  Measured error

{
  float d_term;


#if PRINT_PID
  Serial.print(p);
  Serial.print("\t");
  Serial.print(i);
  Serial.print("\t");
  Serial.print(d);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
#endif

  // Calculate error derivative 
  
  d_term = d * (error - last_error);
  last_error = error;
  
  return (p * error) + d_term;
};


//============================== update_pd_i() ================================
//
// Perform the PID algorithm, calulate the correction value given measured
// error (using P and D terms) and externally integrated error (using I term).

float                            // Ret: Correction value
PID::update_pd_i(float error,    // In:  Measured error
                 float i_error)  // In:  Measured integrated error

{
  float d_term;


#if PRINT_PID
  Serial.print(p);
  Serial.print("\t");
  Serial.print(i);
  Serial.print("\t");
  Serial.print(d);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(i_error);
  Serial.print("\t");
#endif

  // Calculate error derivative 
  
  d_term = d * (error - last_error);
  last_error = error;
  
  return (p * error) + (i * i_error) + d_term;
};


//============================== update_pid() =================================
//
// Perform the PID algorithm, calulate the correction value given measured
// errors (proportional, integral and derivative).

float                              // Ret: Correction value
PID::update_pid(int16_t p_error,   // In:  Measured error
                int16_t i_error,   // In:  Measured integrated error
                int16_t d_error)   // In:  Measured derivative error

{
#if PRINT_PID
  Serial.print(p);
  Serial.print("\t");
  Serial.print(i);
  Serial.print("\t");
  Serial.print(d);
  Serial.print("\t");
  Serial.print(p_error);
  Serial.print("\t");
  Serial.print(i_error);
  Serial.print("\t");
  Serial.print(d_error);
  Serial.print("\t");
#endif

  return (p * p_error) + (i * i_error) + (d * d_error);
};


#if 0
//============================== update() =====================================
//
// Perform the PID algorithm and calulate the correction value

float PID::update(float error)

{
  float d_term;


#if PRINT_PID
  Serial.print(p);
  Serial.print("\t");
  Serial.print(i);
  Serial.print("\t");
  Serial.print(d);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
#endif

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
#endif

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

