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

//-----------------------------------------------------------------------------
//
// Receiver rest state determination

#define RECEIVER_REST_AVG_DEV_MAX    2    // How much the receiver reading can
                                          // deviate from the long-term average
                                          // and still be considered stable.
#define RECEIVER_REST_ZERO_DEV_MAX 100    // How much the receiver reading can
                                          // deviate from zero and still be
                                          // zero if stable.
#define RECEIVER_LONG_AVG_FACTOR     5    // Determines the averaging period. For
                                          // 20 msec rate this is 2^5 * 20 which
                                          // is a little more than 0.5 sec.
#define RECEIVER_REST_CYCLES_MIN    50    // Number of cycles the gyro must be
                                          // at rest to flag a stable condition

//-----------------------------------------------------------------------------
//
// Minimum and maximum thresholds, beyond which the receiver input is considered
// to be at minimum or maximum respectively

#define RECEIVER_LOW_THRESHOLD  (1150 / RECEIVER_TICK)
#define RECEIVER_HIGH_THRESHOLD (1850 / RECEIVER_TICK)

// Range and factor used in throttle range to motor range conversion.
// Intended to avoid rounding errors in calculations

#define RECEIVER_THROTTLE_RANGE_MAX    1023  // 2^10 - 1
#define RECEIVER_THROTTLE_RANGE_FACTOR    6  // Shift of 6, still fits in uint16_t


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

static boolean receiver_status = false;


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
        p_ch_data->ticks_high -= (p_ch_data->ticks_high >> 2);
        p_ch_data->ticks_high += (ticks_diff >> 2);
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


//======================== receiver_update_status() ===========================
//
// Update the current status of the receiver.  Should be called periodically.

boolean                    // Ret: true if OK, false if not OK
receiver_update_status(void)

{
  uint8_t ch;


  receiver_status = true;
  for (ch = FIRST_CH; ch < NUM_CH; ch++)
  {
    if (ch_data[ch].error_count >= RECEIVER_MAX_ERRORS)
       receiver_status = false;
  }

  return receiver_status;
}


//======================== receiver_get_status() ==============================
//
// Get the current status of the receiver

boolean                    // Ret: true if OK, false if not OK
receiver_get_status(void)

{
  return receiver_status;
}


//====================== receiver_get_current_raw() ===========================
//
// Get the current raw receiver data (that has been read before from the h/w)

uint16_t                             // Ret: raw data, in units of RECEIVER_TICK
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


//========================== receiver_get_boolean() ===========================
//
// Get a receiver channel data in a boolean (0 or 1) format.  This is used for,
// e.g., the Gear channel.

boolean                          // Ret: boolean channel data
receiver_get_boolean(uint8_t ch) // In:  channel

{
  return (receiver_get_current_raw(ch) > RECEIVER_NOM_MID);
}


//======================== receiver_is_at_extreme() ===========================
//
// Get a receiver channel data in a boolean (0 or 1) format.  This is used for,
// e.g., the Gear channel.

int8_t                             // Ret: -1 if near minimum, 1 if near
                                   //      maximum, 0 otherwise
receiver_is_at_extreme(uint8_t ch) // In:  channel

{
  uint16_t raw;
  uint8_t  result;

  
  result = 0;
  
  if (receiver_get_status())
  {
    raw = receiver_get_current_raw(ch);
    
    if (raw > RECEIVER_HIGH_THRESHOLD)
      result = 1;
    
    else if (raw < RECEIVER_LOW_THRESHOLD)
      result = -1;
  }

  return result;
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


//========================== Class ReceiverRotation ===========================
//
// Handle rotation commands (roll, pitch, yaw): find zero point and convert to
// signed number.
//
//=============================================================================

//============================= Constructor ===================================

ReceiverRotation::ReceiverRotation(uint8_t ch_in)     // In:  Receiver channel

{
  ch = ch_in;
}


//============================ init_zero() ====================================
//
// initialize the zero finding algorithm

void 
ReceiverRotation::init_zero(void)

{
  cycle_counter = 0;
  raw_zero = (uint16_t)RECEIVER_NOM_MID;
  if (receiver_get_status())
    raw_avg = receiver_get_current_raw(ch) << RECEIVER_LONG_AVG_FACTOR;
  else  
    raw_avg = (uint16_t)RECEIVER_NOM_MID << RECEIVER_LONG_AVG_FACTOR;
}


//=========================== find_zero() =====================================
//
// Calculate an average and check whether the raw receiver input is stable
// around the zero point.  Should be called periodically.

boolean                        // Ret: true if stable aroud zero
ReceiverRotation::find_zero(void)

{
  int16_t  avg_diff;
  int16_t  zero_diff;
  uint16_t raw;


  if (! receiver_get_status())
  {
    // Receiver is not OK
    
    cycle_counter = 0;
    
    return false;
  };
  
  raw = receiver_get_current_raw(ch);
  
  // Calculate long-term average, in units of (1 << RECEIVER_LONG_AVG_FACTOR)

  avg_diff = (int16_t)(raw - (raw_avg >> RECEIVER_LONG_AVG_FACTOR));
  raw_avg += avg_diff;

  zero_diff = (int16_t)(raw - (uint16_t)RECEIVER_NOM_MID);
  
  // To be stable, the current reading must not deviate from average too much.
  // It also must not deviate from zero too much.
  
  if ((avg_diff > (int16_t)RECEIVER_REST_AVG_DEV_MAX)    ||
      (avg_diff < (int16_t)-RECEIVER_REST_AVG_DEV_MAX)   ||
      (zero_diff > (int16_t)RECEIVER_REST_ZERO_DEV_MAX)  ||
      (zero_diff < (int16_t)-RECEIVER_REST_ZERO_DEV_MAX)
     )
  {
    // Not stable
    
    cycle_counter = 0;
    
    return false;
  }

  else if (cycle_counter < (uint8_t)RECEIVER_REST_CYCLES_MIN)
  {
    // Stable, but not for long enough
    
    cycle_counter++;
    
    return false;
  }

  else
  {
    // We have been stable long enough, flag a stable condition
    
    raw_zero = raw_avg >> RECEIVER_LONG_AVG_FACTOR;

    return true;
  };
}


//=========================== get_rotation() ==================================
//
// Get the centered rotation value.  Should be called periodically.

int16_t                        // Ret: centered data, in units of RECEIVER_TICK
ReceiverRotation::get_rotation(void)

{
  if (receiver_get_status())
    return receiver_get_current_raw(ch) - raw_zero;

  else
    return 0;
}


//========================== Class ReceiverThrottle ===========================
//
// Handle throttle command: find minimum and maximum
//
//=============================================================================

//============================= Constructor ===================================

ReceiverThrottle::ReceiverThrottle(void)

{
  init_stable();
}


//============================ init_stable() ==================================
//
// initialize the stable point finding algorithm

void
ReceiverThrottle::init_stable(void)

{
  cycle_counter = 0;
}


//=========================== find_stable() ===================================
//
// Calculate an average and check whether the raw receiver input is stable.
// Should be called periodically.

int16_t                        // Ret: stable point if stable, -1 otherwise
ReceiverThrottle::find_stable(void)

{
  int16_t  avg_diff;
  int16_t  zero_diff;
  uint16_t raw;


  if (! receiver_get_status())
  {
    // Receiver is not OK
    
    cycle_counter = 0;
    
    return -1;
  };
  
  raw = receiver_get_current_raw(THROTTLE_CH);
  
  // Calculate long-term average, in units of (1 << RECEIVER_LONG_AVG_FACTOR)

  avg_diff = (int16_t)(raw - (raw_avg >> RECEIVER_LONG_AVG_FACTOR));
  raw_avg += avg_diff;

  zero_diff = (int16_t)(raw - (uint16_t)RECEIVER_NOM_MID);
  
  // To be stable, the current reading must not deviate from average too much.
  // It also must not deviate from zero too much.
  
  if ((avg_diff > (int16_t)RECEIVER_REST_AVG_DEV_MAX)    ||
      (avg_diff < (int16_t)-RECEIVER_REST_AVG_DEV_MAX)
     )
  {
    // Not stable
    
    cycle_counter = 0;
    
    return -1;
  }

  else if (cycle_counter < (uint8_t)RECEIVER_REST_CYCLES_MIN)
  {
    // Stable, but not for long enough
    
    cycle_counter++;
    
    return -1;
  }

  else
  {
    // We have been stable long enough, flag a stable condition
    
    return raw_avg >> RECEIVER_LONG_AVG_FACTOR;
  };
}


//=========================== find_min() ======================================
//
// Calculate an average and check whether the raw receiver input is stable
// around the minimum point.  Should be called periodically.

int16_t                        // Ret: stable point if stable, -1 otherwise
ReceiverThrottle::find_min(void)

{
  int16_t raw_stable;


  raw_stable = find_stable();
  
  if ((raw_stable > (int16_t)RECEIVER_LOW_THRESHOLD) || (raw_stable < 0))
    return false;

  raw_min = raw_stable;

  return true;
}


//=========================== find_max() ======================================
//
// Calculate an average and check whether the raw receiver input is stable
// around the maximum point.  Should be called periodically.

int16_t                        // Ret: stable point if stable, -1 otherwise
ReceiverThrottle::find_max(void)

{
  int16_t raw_stable;


  raw_stable = find_stable();
  
  if (raw_stable < (int16_t)RECEIVER_HIGH_THRESHOLD)
    return false;

  raw_max = raw_stable;

  return true;
}


//===================== calculate_throttle_motor_factor() =====================
//
// Calculate the factor used in get_throttle(), based on the detected minimum
// and maximum values.  Should be called after find_min() and find_max() have
// done their job.

void
ReceiverThrottle::calculate_throttle_motor_factor(void)

{
  uint16_t throttle_range;


  throttle_range = raw_max - raw_min + 1;
  
  // Make sure the diff is within the nominal range

  if (throttle_range > (uint16_t)RECEIVER_THROTTLE_RANGE_MAX)
    throttle_range = RECEIVER_THROTTLE_RANGE_MAX;
  throttle_motor_range_factor =
    (uint16_t)(throttle_range << RECEIVER_THROTTLE_RANGE_FACTOR) /
                                                (uint16_t)MOTOR_THROTTLE_RANGE;
}


//=========================== get_throttle() ==================================
//
// Get the throttle value, normalized to motor command range

int16_t                        // Ret: normalized data
ReceiverThrottle::get_throttle(void)

{
  int16_t throttle_diff;

  
  if (receiver_get_status())
  {
    throttle_diff = receiver_get_current_raw(THROTTLE_CH)  - raw_min;

    // Make sure the diff is within the nominal range
    
    if (throttle_diff < 0)
      throttle_diff = 0;
    else if (throttle_diff > (int16_t)RECEIVER_THROTTLE_RANGE_MAX)
      throttle_diff = RECEIVER_THROTTLE_RANGE_MAX;
  
    return ((uint16_t)((uint16_t)throttle_diff << RECEIVER_THROTTLE_RANGE_FACTOR) / 
                                                        throttle_motor_range_factor) +
           (uint16_t)MOTOR_THROTTLE_MIN;
  }
  
  else
    return MOTOR_THROTTLE_MIN;
}


