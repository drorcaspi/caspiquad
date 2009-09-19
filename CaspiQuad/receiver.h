#ifndef __RECEIVER_H__
#define __RECEIVER_H__

//=============================================================================
//
// Receiver API
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


// Receiver Channels

#define THROTTLE_CH   0
#define ROLL_CH       1   // Aileron
#define PITCH_CH      2   // Elevator
#define YAW_CH        3   // Rudder
#define GEAR_CH       4
#define AUX1_CH       5
#define NUM_CH        6   // Number of channels
#define FIRST_CH      THROTTLE_CH

// Receiver Tick defines the measuement units of raw receiver data.  Currently
// this is 1 uSec but it may change for the sake of optimization.

#define RECEIVER_TICK 1   // uSec

// Nominal minimum, middle and maximum values of raw receiver data

#define RECEIVER_NOM_MIN   (1000 / RECEIVER_TICK)
#define RECEIVER_NOM_MID   (1500 / RECEIVER_TICK)
#define RECEIVER_NOM_MAX   (2000 / RECEIVER_TICK)
#define RECEIVER_NOM_SWING ( 500 / RECEIVER_TICK)


//============================ receiver_init() ================================
//
// Initialize the receiver module
// Should be called on system initalization

boolean             // Ret: true if OK, false if failed
receiver_init(void);


//======================== receiver_update_status() ===========================
//
// Update the current status of the receiver.  Should be called periodically.

boolean                    // Ret: true if OK, false if not OK
receiver_update_status(void);


//======================== receiver_get_status() ==============================
//
// Get the current status of the receiver

boolean                    // Ret: true if OK, false if not OK
receiver_get_status(void);


//====================== receiver_get_current_raw() ===========================
//
// Get the current raw receiver data (that has been read before from the h/w)

uint16_t                              // Ret: raw data, in units of RECEIVER_TICK
receiver_get_current_raw(uint8_t ch); // In:  channel


//========================== receiver_get_boolean() ===========================
//
// Get a receiver channel data in a boolean (0 or 1) format.  This is used for,
// e.g., the Gear channel.

boolean                           // Ret: boolean channel data
receiver_get_boolean(uint8_t ch); // In:  channel


//======================== receiver_is_at_extreme() ===========================
//
// Check if receiver channel is near minimum or maximum.

int8_t                              // Ret: -1 if near minimum
                                    //       1 if near maximum
                                    //       0 otherwise
receiver_is_at_extreme(uint8_t ch); // In:  channel


//========================== receiver_print_stats() ===========================
//
// Print some statistics (for debug)

void receiver_print_stats(void);


//========================== Class ReceiverRotation ===========================
//
// Handle rotation commands (roll, pitch, yaw): find zero point and convert to
// signed number.
//
//=============================================================================

class ReceiverRotation

{
private:
  uint8_t      ch;                  // Receiver channel
  uint16_t     raw_zero;            // Raw receiver reading at middle
  uint16_t     raw_avg;             // Average of raw readings
  uint8_t      cycle_counter;       // Count the number of cycles

public:
  //============================= Constructor ===================================

  ReceiverRotation(void) {};
  ReceiverRotation(uint8_t ch);     // In:  Receiver channel
  
  
  //============================ init_zero() ====================================
  //
  // initialize the zero finding algorithm

  void init_zero(void);


  //=========================== find_zero() =====================================
  //
  // Calculate an average and check whether the raw receiver input is stable
  // around the zero point.  Should be called periodically.
  
  volatile
  boolean                        // Ret: true if stable aroud zero
  find_zero(void);
  
  
  //=========================== get_rotation() ==================================
  //
  // Get the centered rotation value.  Should be called periodically.
  
  int16_t                        // Ret: centered data, in units of RECEIVER_TICK
  get_rotation(void);
  
  
  //============================= print_stats() =================================
  //
  // Print some statistics (for debug)

  void print_stats(void);
};



//========================== Class ReceiverThrottle ===========================
//
// Handle throttle command: find minimum and maximum
//
//=============================================================================

class ReceiverThrottle

{
private:
  uint16_t     raw_min;             // Raw receiver reading at minimum
  uint16_t     raw_max;             // Raw receiver reading at maximum
  uint16_t     raw_stable;          // Candidate for stable throttle reading

  // For converting receiver reading to motor throttle command, we calculate
  // 3 ranges.  Thus, we need 2 thresholds and 3 slopes.
  
  uint16_t     raw_threshold1;
  uint16_t     raw_threshold2;

  uint16_t     motor_throttle_slope1;
  uint16_t     motor_throttle_slope2;
  uint16_t     motor_throttle_slope3;
  
  uint8_t      cycle_counter;       // Count the number of cycles


  //=========================== find_stable() ===================================
  //
  // Calculate an average and check whether the raw receiver input is stable.
  // Should be called periodically.

  volatile
  int16_t                        // Ret: stable point if stable, -1 otherwise
  find_stable(void);

public:
  //============================= Constructor ===================================

  ReceiverThrottle(void);
  
  
  //============================ init_stable() ==================================
  //
  // initialize the stable point finding algorithm
  
  void init_stable(void);

  
  //=========================== find_min() ======================================
  //
  // Calculate an average and check whether the raw receiver input is stable
  // around the minimum point.  Should be called periodically.
  
  volatile boolean                        // Ret: true if stable at minimum
  find_min(void);
  

  //=========================== find_max() ======================================
  //
  // Calculate an average and check whether the raw receiver input is stable
  // around the maximum point.  Should be called periodically.

  volatile boolean                        // Ret: true if stable at maximum
  find_max(void);


  //===================== calculate_throttle_motor_factor() =====================
  //
  // Calculate the factor used in get_throttle(), based on the detected minimum
  // and maximum values.  Should be called after find_min() and find_max() have
  // done their job.

  void
  calculate_throttle_motor_factor(void);


  //=========================== get_throttle() ==================================
  //
  // Get the throttle value, normalized to motor command range
  // The translation is not linear; to get better control in the useful range, we
  // divide the throttle range into 3 sections.

  int16_t                        // Ret: normalized data
  get_throttle(void);
};
#endif
