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

class RotationIntegrator

{
protected:
  int16_t      rotation_estimate; // Output of the second integrator
                                  // ((1 / ROT_SCALE_RAD) radians)
  
public:
  //============================== Constructor ==================================
  //
  // Initializes a RotationIntegrator object
  
  RotationIntegrator(void);
  
  //============================== get_*() ======================================
  //
  // Get the estimator's configurable parameters and state variable
  
  float          // Ret: Rotation estimation (rad).
  get_estimate(void);

  //============================== reset() ======================================
  //
  // Reset the estimator's state variable (integrator output)
  
  void
  reset(void);

  //============================== estimate() ===================================
  //
  // Estimate rotation angle for one rotation axis, based on rotation rate and
  // rotation angle measurements.
  
  int16_t                         // Ret: New rotation estimate
  estimate(
    float   rotation_rate_in);    // In:  Rotation rate measurement, in rad/sec,
                                  //      scaled from gyro reading
};


//========================= class RotationManualEstimator =====================
//
// This class implements a rotation angle estimator based on rotation rate
// input, using an integrator, and control input based on user yaw stick
// command 
//
//  rotation rate
//  measure.                                      + +---+
//  (rad/sec) ------------------------------------->|   |              rotation
//                                                  |   |              estimate
//  rotation                                        |   |   +--------+  (rad)
//  control  --+------+                             | + |-->|integral|--+-->
//  (boolean)  |      |/                            |   |   +--------+  |
//             |      /      +-----+              - |   |       ^       |
//             |  +--*   *-->| * K |--------------->|   |       |reset  |
//             |  |          +-----+                +---+       |       |
//             |  +---------------------------------------------|-------+
//             |                                                |
//             +------------------------------------------------+
//
//=============================================================================

class RotationManualEstimator: public RotationIntegrator

{
private:
  boolean was_user_command;  // Stores is_user_command parameter to estimate()
                             // from the last cycle
                             
public:
  //============================== Constructor ==================================
  //
  // Initializes a RotationManualEstimator object
  
  RotationManualEstimator(void);

  //============================== estimate() ===================================
  //
  // Estimate rotation angle for one rotation axis, based on rotation rate and
  // rotation angle measurements.

  int16_t                         // Ret: New rotation estimate
  estimate(
    float   rotation_rate_in,     // In:  Rotation rate measurement, in rad/sec,
                                  //      scaled from gyro reading
    boolean is_user_command);     // In:  Indicates if the user is commanding
                                  //      yaw (i.e., stick not at 0).
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

class RotationEstimator: public RotationIntegrator

{
private:
  // Configuration Parameters
  // The following parameters are static (per-class) as there's no need for
  // separate settings per rotation.
  
  static float bw;           // Bandwidth of the estimator filter (1/sec). Tune
                             // this to match sensor performance.
  static float bw_2;         // Calculated as 2 * bw at setup time
  static float cycle_bw_sq;  // Calculated as dt * bw * bw at setup time

  static int   eeprom_base_addr;   // Base address in EEPROM

  // State Variables
  
  float        integ1_out;        // Output of the first integrator (rad/sec)
  
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
                      
  //============================== get_*() ======================================
  //
  // Get the estimator's configurable parameters and state variable
  
  static
  float          // Ret: Bandwidth of the estimator filter (1/sec).
  get_bw(void);

  //============================== reset() ======================================
  //
  // Initialize the estimator's state variables (integrator outputs)
  
  void
  reset(void);
 
  //============================== estimate() ===================================
  //
  // Estimate rotation angle for one rotation axis, based on rotation rate and
  // rotation angle measurements.
  
  int16_t                         // Ret: New rotation estimate
  estimate(
    float   rotation_rate_in,     // In:  Rotation rate measurement, in rad/sec,
                                  //      scaled from gyro reading
    int16_t rotation_in);         // In:  Rotation angle measurement, in units of
                                  //      (1 / ROT_SCALE_RAD) radians, calculated
                                  //      from accelerometer readings.
                                  //      ROT_NONE means no valid measurement.
                                
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
