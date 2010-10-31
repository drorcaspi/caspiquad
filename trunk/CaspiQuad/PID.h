#ifndef __PID_H__
#define __PID_H__

//=============================================================================
//
// PID Class Declaration
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
//
// The PID class implement a Proportinal-Integral-Derivative control algorithm.
//
// For a description of PID see:
//   http://en.wikipedia.org/wiki/PID_controller
//
//                                              +---+   +---+
//                       +--------------------->|* P|-->|   |
// target    + +---+     |                      +---+   |   |
// position -->|   |error|   +--------+         +---+   |   |correction
//             | + |-----+-->|integral|-------->|* I|-->| + |------->
// current  -->|   |     |   +--------+         +---+   |   |
// position  - +---+     |   +--------+         +---+   |   | 
//                       +-->|deriv.  |-------->|* D|-->|   |
//                           +--------+         +---+   +---+
//
//
//=============================================================================

class PID

{
private:
  // Configuration Parameters
  
  float p;                  // Proportional factor
  float i;                  // Integral factor
  float d;                  // Derivative factor
  float windup_guard;       // Limit on the absolute value of integrated_error

  int   eeprom_base_addr;   // Base address in EEPROM

  // State Variables
  
  float last_error;         // Error stored from last cycle
  float integrated_error;

public:
  //============================== Constructors ===============================
  //
  // Constructors
  //
  // Init a PID object with the given configuration parameters (see their
  // description above)
  
  PID(void);
  
  PID(float p, float i, float d, float windup_guard);
  
  //============================== set_params() ===============================
  //
  // Set PID object's configuration parameters

  void set_params(float p_in, float i_in, float d_in, float windup_guard_in);
  void set_p(float p_in);
  void set_i(float i_in);
  void set_d(float d_in);
  void set_windup_guard(float windup_guard_in);
  
  //============================== get_*() ====================================
  //
  // Get PID object's configuration parameters
  
  float get_p(void);
  float get_i(void);
  float get_d(void);
  float get_windup_guard(void);

  //============================== update_p() ===================================
  //
  // Perform the PID algorithm, calulate the correction value given measured
  // error (only proportional, both integral and derivative are 0).
  
  float                        // Ret: Correction value
  update_p(int16_t p_error);   // In:  Measured error

  //============================== update_pd() ==================================
  //
  // Perform the PID algorithm, calulate the correction value given measured
  // error, using only P and D terms.
  
  float                       // Ret: Correction value
  update_pd(float error);     // In:  Measured error

  //============================== update_pd_i() ================================
  //
  // Perform the PID algorithm, calulate the correction value given measured
  // error (using P and D terms) and externally integrated error (using I term).
  
  float                       // Ret: Correction value
  update_pd_i(float error,    // In:  Measured error
              float i_error); // In:  Measured integrated error

  //============================== update_pid() =================================
  //
  // Perform the PID algorithm, calulate the correction value given measured
  // errors (proportional, integral and derivative).
  
  float                              // Ret: Correction value
  update_pid(int16_t p_error,   // In:  Measured error
             int16_t i_error,   // In:  Measured integrated error
             int16_t d_error);  // In:  Measured derivative error

  //============================== update() ===================================
  //
  // Perform the PID algorithm and calulate the correction value
  
  float update(float error);

  //============================== update() ===================================
  //
  // Perform the PID algorithm and calulate the correction value
  
  float update(float target_position, float current_position);

  //============================== update() =====================================
  //
  // Perform the PID algorithm and calulate the correction value, given target
  // rate and current rate
  
  float update(float target_position,
               float current_position,
               float target_rate,
               float current_rate);

  //================================= reset() =================================
  //
  // Zero the integrated error part of the PID

  void reset(void);

#if 0
  //============================== set_i_err() ================================
  //
  // Set the integrated error part of the PID to an initial value

  void set_i_err(float initial_i_err);
#endif

  //============================== read_eeprom() ==============================
  //
  // Read the configuration parameters from EEPROM, if valid.  If not, set
  // defaults.

  int                             // Ret: Next address in EEPROM
  read_eeprom(
    int   eeprom_base_addr_in,    // In: Base address in EEPROM
    float default_p,              // In:  Proportional factor
    float default_i,              // In:  Integral factor
    float default_d,              // In:  Derivative factor
    float default_windup_guard);  // In:  Limit on the abs. value of integrated_error

  //============================== write_eeprom() =============================
  //
  // Write the configuration parameters to EEPROM

  void write_eeprom(void);
};


#endif
