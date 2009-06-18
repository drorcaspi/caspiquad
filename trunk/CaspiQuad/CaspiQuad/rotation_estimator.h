#ifndef __ROTATION_ESTIMATOR_H__
#define __ROTATION_ESTIMATOR_H__

//=============================================================================
//
// RotationEstimator Class Declaration
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

class RotationEstimator

{
private:
  // Configuration Parameters
  // The following parameters are static (per-class) as there's no need for
  // separate settings per rotation.
  
  static float bw;           // Bandwidth of the estimator filter (1/sec). Tune
                             // this to match sensor performance.
  static float cycle;        // Iteration cycle of the estimator filter (sec)
  static float bw_2;         // Calculated as 2 * bw at setup time
  static float cycle_bw_sq;  // Calculated as dt * bw * bw at setup time

  static int   eeprom_base_addr;   // Base address in EEPROM

  // State Variables
  
  float integ1_out;        // Output of the first integrator (rad/sec)
  float rotation_estimate; // Output of the second integrator (rad)
  
public:
  //============================== Constructor ==================================
  //
  // Initializes a RotationEstimator object
  
  RotationEstimator(void);
  
  //============================== set_*() ======================================
  //
  // Set the estimator's configurable parameters
  
  static
  void
  set_bw(
    float bw_in);        // In: Bandwidth of the estimator filter (1/sec).
                         //     Tune this to match sensor performance.

  static
  void
  set_cycle(
    float cycle_in);     // In: Iteration cycle of the estimator filter (sec)
                       
  //============================== get_*() ======================================
  //
  // Get the estimator's configurable parameters and state variable
  
  static
  float          // Ret: Bandwidth of the estimator filter (1/sec).
  get_bw(void);

  float          // Ret: Rotation estimation (rad).
  get_estimate(void);

  //========================== print_stats() ====================================
  //
  // Print some statistics (for debug)

  void print_stats(void);
  
  //============================== init() =======================================
  //
  // Initialize the estimator's state variables (integrator outputs)
  
  void
  init(
    float  rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                               //      scaled from gyro reading
    float  rotation_in);       // In:  Rotation angle measurement, in rad,
                               //      calculated from accelerometer readings

  //============================== estimate() ===================================
  //
  // Estimate rotation angle for one rotation axis, based on rotation rate and
  // raw accelerator measurements.

  float                        // Ret: New rotation estimate
  estimate(
    float  rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                               //      scaled from gyro reading
    int8_t accel_raw_base,     // In:  Raw accelerometer reading, base
    int8_t accel_raw_perp);    // In:  Raw accelerometer reading, perpendicular          

  //============================== estimate() ===================================
  //
  // Estimate rotation angle for one rotation axis, based on rotation rate and
  // rotation angle measurements.

  float                        // Ret: New rotation estimate
  estimate(
    float  rotation_rate_in,   // In:  Rotation rate measurement, in rad/sec,
                               //      scaled from gyro reading
    float  rotation_in);       // In:  Rotation angle measurement, in rad,
                               //      calculated from accelerometer readings          

  //============================== estimate() ===================================
  //
  // Estimate rotation angle for one rotation axis, based on rotation rate only.
  // This is used in case rotation inputs are not reliable (when accelerometer
  // reading on botx axis are close to 0).
  
  float                        // Ret: New rotation estimate
  estimate(
    float  rotation_rate_in);  // In:  Rotation rate measurement, in rad/sec,
                               //      scaled from gyro reading

  //============================== read_eeprom() ==============================
  //
  // Read the configuration parameters from EEPROM, if valid.  If not, set
  // defaults.

  static
  int                         // Ret: Next address in EEPROM
  read_eeprom(
    int   eeprom_base_addr,   // In: Base address in EEPROM
    float bw_in);             // In: Bandwidth of the estimator filter (1/sec).
                              //     Tune this to match sensor performance.

  //============================== write_eeprom() =============================
  //
  // Write the configuration parameters to EEPROM

  static
  void
  write_eeprom(void);
};


#endif
