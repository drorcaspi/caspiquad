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

#define ROTATION_ESTIMATOR_ACCEL_RAW_MIN (ACCEL_TYP_1G / 4)

// Absolute acceleration must be close to 1G for taking into account

#define ROTATION_ESTIMATOR_ACCEL_SQ_MAX  ((ACCEL_MAX_1G * 1.1) * (ACCEL_MAX_1G * 1.1))
#define ROTATION_ESTIMATOR_ACCEL_SQ_MIN  ((ACCEL_MIN_1G * 0.9) * (ACCEL_MIN_1G * 0.9))

// Limit on the time estimation is based on gyro input only

#define ROTATION_ESTIMATOR_INAVLID_ACC_CYCLE_LIMIT 25


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
  invalid_acc_cycle_counter = 0;
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
  float    rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                               //      scaled from gyro reading
  int8_t   accel_raw_base,     // In:  Raw accelerometer reading, base
  int8_t   accel_raw_perp,     // In:  Raw accelerometer reading, perpendicular
  uint16_t accel_abs_sq)       // In:  Sum of accel readings squared     

{
  float rotation_diff;


  if (((accel_raw_base >= (int8_t) ROTATION_ESTIMATOR_ACCEL_RAW_MIN)  ||
       (accel_raw_base <= (int8_t)-ROTATION_ESTIMATOR_ACCEL_RAW_MIN)  ||
       (accel_raw_perp >= (int8_t) ROTATION_ESTIMATOR_ACCEL_RAW_MIN)  ||
       (accel_raw_perp <= (int8_t)-ROTATION_ESTIMATOR_ACCEL_RAW_MIN)
      )  &&
      (
       (accel_abs_sq   <= (uint16_t)ROTATION_ESTIMATOR_ACCEL_SQ_MAX)  &&
       (accel_abs_sq   >= (uint16_t)ROTATION_ESTIMATOR_ACCEL_SQ_MIN)
      )
     )
  {
    // Accelerometer readings can be used as raw rotation measurement

    invalid_acc_cycle_counter = 0;

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
    // Accelerometer readings are not reliable, use only rotation rate input.
    // We limit the number of cycles this is done, since integration error accumulate
    // over time.

    if (invalid_acc_cycle_counter < ROTATION_ESTIMATOR_INAVLID_ACC_CYCLE_LIMIT)
    {
      invalid_acc_cycle_counter++;
      rotation_estimate += cycle * (integ1_out + rotation_rate_in);
    };
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


