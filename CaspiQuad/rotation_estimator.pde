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

// None


//=============================================================================
//
// Static Members
//
//=============================================================================

float RotationEstimator::bw          = 1;
float RotationEstimator::cycle       = CONTROL_LOOP_CYCLE_SEC;
float RotationEstimator::bw_2;
float RotationEstimator::cycle_bw_sq;

int   RotationEstimator::eeprom_base_addr;   // Base address in EEPROM


//============================== Constructor ==================================
//
// Initializes a RotationEstimator object

RotationEstimator::RotationEstimator()

{
  init(0.0, 0);
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
  bw_2 = 2 * bw_in / (float)ROT_SCALE_RAD;
  cycle_bw_sq = cycle * bw_in * bw_in / (float)ROT_SCALE_RAD;
};
  

void
RotationEstimator::set_cycle(
  float cycle_in)   // In: Iteration cycle of the estimator filter (sec)

{
  cycle = cycle_in;
  cycle_bw_sq = cycle_in * bw * bw / (float)ROT_SCALE_RAD;
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
  return (float)rotation_estimate / (float)ROT_SCALE_RAD;
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
  float   rotation_rate_in,     // In:  Rotation rate measurement, in rad/sec,
                                //      scaled from gyro reading
  int16_t rotation_in)          // In:  Rotation angle measurement, in units of
                                //      (1 / ROT_SCALE_RAD) radians, calculated
                                //      from accelerometer readings.

{
  // Set the integrator outputs so we would get exactly 0 at their inputs if
  // we call the estimator with the same values (see estimate()).
  
  rotation_estimate = rotation_in;
  integ1_out = -rotation_rate_in;
};
  

//============================== estimate() ===================================
//
// Estimate rotation angle for one rotation axis, based on rotation rate and
// rotation angle measurements.

int16_t                         // Ret: New rotation estimate
RotationEstimator::estimate(
  float   rotation_rate_in,     // In:  Rotation rate measurement, in rad/sec,
                                //      scaled from gyro reading
  int16_t rotation_in)          // In:  Rotation angle measurement, in units of
                                //      (1 / ROT_SCALE_RAD) radians, calculated
                                //      from accelerometer readings.
                                //      ROT_NONE means no valid measurement.
  
{
  int16_t rotation_diff;


  if (rotation_in != ROT_NONE)
  {
    rotation_diff = rotation_in - rotation_estimate;

    // First integration
    // TODO: convert to fixed point
  
    integ1_out += cycle_bw_sq * (float)rotation_diff;

    // Second integration
  
    rotation_estimate += (int16_t)(cycle * (integ1_out +
                                            (bw_2 * (float)rotation_diff) +
                                            rotation_rate_in));

#if PRINT_ROT_ESTIMATE
    Serial.print(rotation_diff, DEC);
    Serial.print("\t");
#endif
  }

  else
    rotation_estimate += (int16_t)(cycle * (integ1_out + rotation_rate_in));

#if PRINT_ROT_ESTIMATE
  Serial.print(integ1_out);
  Serial.print("\t");
  Serial.println(rotation_estimate, DEC);
#endif

  // Rotation estimate is cyclic, never above +/- PI (180 degrees)
  // This happens natively if the scale is set to 2^15 per rad
  // The following is a sanity check

#if (ROT_SCALE_PI != (1u << 15))
  #error ROT_SCALE_PI assumed to be 2^15
#endif
  
  return rotation_estimate;
};


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


