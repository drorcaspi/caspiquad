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
#include "eeprom_utils.h"
#include "rotation_estimator.h"


//=============================================================================
//
// Local Definitions
//
//=============================================================================

// None


//========================= class RotationIntegrator ==========================
//
// This class implements a rotation angle estimator based on rotation rate
// input, using an integrator
//
//  rotation rate
//  measure.                 +--------+  
//  (rad/sec) -------------->|integral|----->   rotation estimate (rad)
//                           +--------+  
//
//=============================================================================


//============================== Constructor ==================================
//
// Initializes a RotationIntegrator object

RotationIntegrator::RotationIntegrator()

{
  reset();
};


//============================== get_*() ======================================
//
// Get the estimator's configurable parameters and state variables

float          // Ret: Rotation estimation (rad).
RotationIntegrator::get_estimate(void)

{
  return (float)rotation_estimate / (float)ROT_SCALE_RAD;
};


//============================== reset() ======================================
//
// Reset the estimator's state variable (integrator output)

void
RotationIntegrator::reset(void)

{
  rotation_estimate = 0;
};


//============================== estimate() ===================================
//
// Estimate rotation angle for one rotation axis, based on rotation rate and
// rotation angle measurements.

int16_t                         // Ret: New rotation estimate
RotationIntegrator::estimate(
  float   rotation_rate_in)    // In:  Rotation rate measurement, in rad/sec,
                                //      scaled from gyro reading

{
  rotation_estimate +=
    (int16_t)((float)CONTROL_LOOP_CYCLE_SEC * (float)ROT_SCALE_RAD * rotation_rate_in);

#if PRINT_ROT_ESTIMATE
  Serial.print(rotation_rate_in);
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

                                
//========================= class RotationEstimator ===========================
//
// This class implements a rotation angle estimator using a complementary
// filter.
//
//  rotation rate
//  measure.                                        +---+
//  (rad/sec) ------------------------------------->|   |              rotation
//                             (rad/sec^2)          |   |              estimate
//  rotation    +---+      +-----+    +--------+    |   |   +--------+  (rad)
//  measure. -->| + |--+-->|*bw^2|--->|integral|--->| + |-->|integral|--+-->
//  (rad)       +---+  |   +-----+    +--------+    |   |   +--------+  |
//                ^    |   +-----+         (rad/sec)|   |               |
//              - |    +-->|*2bw |----------------->|   |               |
//                |        +-----+ (rad/sec)        +---+               |
//                +-----------------------------------------------------+
//
// Original code written by RoyLB at:
// http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
//
//=============================================================================

//=============================================================================
//
// Static Members
//
//=============================================================================

float RotationEstimator::bw          = 1;
float RotationEstimator::bw_2;
float RotationEstimator::cycle_bw_sq;

int   RotationEstimator::eeprom_base_addr;   // Base address in EEPROM


//============================== Constructor ==================================
//
// Initializes a RotationEstimator object

RotationEstimator::RotationEstimator()

{
  reset();
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
  cycle_bw_sq = (float)CONTROL_LOOP_CYCLE_SEC * bw_in * bw_in;
};
  

//============================== get_*() ======================================
//
// Get the estimator's configurable parameters and state variables

float                       // Ret: Bandwidth of the estimator filter (1/sec).
RotationEstimator::get_bw(void)

{
  return bw;
};


//============================== reset() ======================================
//
// Initialize the estimator's state variables (integrator outputs)

void
RotationEstimator::reset(void)

{
  rotation_estimate = 0;
  integ1_out = 0;
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


#if PRINT_ROT_ESTIMATE
  Serial.print(rotation_rate_in);
  Serial.print("\t");
#endif

  if (rotation_in != ROT_NONE)
  {
    rotation_diff = rotation_in - rotation_estimate;

    // First integration
    // TODO: convert to fixed point
  
    integ1_out += cycle_bw_sq * (float)rotation_diff;

    // Second integration
  
    rotation_estimate += 
      (int16_t)(((float)CONTROL_LOOP_CYCLE_SEC) * (integ1_out +
                                                   (bw_2 * (float)rotation_diff) +
                                                   ROT_SCALE_RAD * rotation_rate_in));

#if PRINT_ROT_ESTIMATE
    Serial.print(rotation_in, DEC);
    Serial.print("\t");
    Serial.print(rotation_diff, DEC);
    Serial.print("\t");
#endif
  }

  else
  {
    rotation_estimate +=
      (int16_t)(((float)CONTROL_LOOP_CYCLE_SEC) * (integ1_out +
                                                   (ROT_SCALE_RAD * rotation_rate_in)));
#if PRINT_ROT_ESTIMATE
    Serial.print("-\t-\t");
#endif
  };

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


