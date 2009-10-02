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
// receiver mis-behavior or a h/w problem.  The current solution is to ignore
// illegal-width pulses, if their consecutive number is small.

#define RECEIVER_MAX_ERRORS                  4

// Shift factor for averaging receiver pulse width readings

#define RECEIVER_AVG_SHIFT                   2

//-----------------------------------------------------------------------------
// Receiver rest state determination
//-----------------------------------------------------------------------------

// How much the receiver reading can deviate from the long-term average
// and still be considered stable.

#define RECEIVER_REST_AVG_DEV_MAX            2

// How much the receiver reading can deviate from zero and still be
// zero if stable.

#define RECEIVER_REST_ZERO_DEV_MAX         100

// Shift factor, determines the averaging period. For 20 msec rate this is
// 2^5 * 20 which is a little more than 0.5 sec.

#define RECEIVER_LONG_AVG_SHIFT              5

// Number of cycles the receiver must be at rest to flag a stable condition

#define RECEIVER_REST_CYCLES_MIN            50

//-----------------------------------------------------------------------------
// Throttle stable state determination
// Accuracy is less important than above, so we allow greater deviation and
// shorter time.
//-----------------------------------------------------------------------------

// How much the throttle reading can deviate from the stable candidate value
// and still be considered stable.

#define RECEIVER_THROTTLE_STABLE_DEV_MAX     4

// Number of cycles the throttle must be at rest to flag a stable condition

#define RECEIVER_THROTTLE_STABLE_CYCLES_MIN 25

//-----------------------------------------------------------------------------
// Minimum and maximum thresholds, beyond which the receiver input is considered
// to be at minimum or maximum respectively
//-----------------------------------------------------------------------------

#define RECEIVER_LOW_THRESHOLD  (1200 / RECEIVER_TICK)
#define RECEIVER_HIGH_THRESHOLD (1800 / RECEIVER_TICK)

//-----------------------------------------------------------------------------
//
// Receiver Rotation Definitions
// =============================
//
// Rotation input from the receiver is translated to rotation rate command,
// by centering and using 2 ranges to provide finer control in the middle.
//
//-----------------------------------------------------------------------------

#define RECEIVER_ROTATION_THRESHOLD0   16
#define RECEIVER_ROTATION_THRESHOLD1  200

//-----------------------------------------------------------------------------
//
// Receiver Throttle Definitions
// =============================
//
// Throttle input from the receiver is translated to motor throttle command,
// using 3 ranges to provide finer control in the middle.
//
//-----------------------------------------------------------------------------

// Range and factor used in throttle range to motor range conversion.
// Intended to avoid rounding errors in calculations

#define RECEIVER_THROTTLE_RANGE_MAX   1023  // 2^10 - 1

// Divide the throttle command, as input to the motor control, to 3 ranges.
// Thresholds at 40% and 70%
// TODO: the above should be EEPROM parameters

#define MOTOR_THROTTLE_THRESHOLD1     (MOTOR_THROTTLE_IDLE + ((MOTOR_THROTTLE_RANGE * 40) / 100))
#define MOTOR_THROTTLE_THRESHOLD2     (MOTOR_THROTTLE_IDLE + ((MOTOR_THROTTLE_RANGE * 70) / 100))

#define MOTOR_THROTTLE_RANGE1         (MOTOR_THROTTLE_THRESHOLD1 - MOTOR_THROTTLE_IDLE)
#define MOTOR_THROTTLE_RANGE2         (MOTOR_THROTTLE_THRESHOLD2 - MOTOR_THROTTLE_THRESHOLD1)
#define MOTOR_THROTTLE_RANGE3         (MOTOR_THROTTLE_TOP        - MOTOR_THROTTLE_THRESHOLD2)

// Shift factor used in calculations, to minimize roundoff errors

#define RECEIVER_THROTTLE_SLOPE_SHIFT 5  // Result still fits in uint16_t


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
  uint16_t ticks_high;      // Average of last measured pulse widths, in units
                            // of (number of ticks << RECEIVER_AVG_SHIFT)
} ReceiverChData;

static ReceiverChData ch_data[NUM_CH];

// Receiver status: true if OK, false if not OK

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
        // We have a valid reading. Update the stored ticks_high, averaging
        // with a simple single-pole IIR filter.  Unsigned 16 bits are enough
        // here.
        
        p_ch_data->ticks_high -= ((uint16_t)p_ch_data->ticks_high >> RECEIVER_AVG_SHIFT);
        p_ch_data->ticks_high += (uint16_t)ticks_diff;
        
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
  // This is required to assure consistent reading of the data (that may change
  // during multi-byte reads)
  
  old_sreg = SREG;
  cli();

  data = ch_data[ch].ticks_high >> RECEIVER_AVG_SHIFT;

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
// Check if receiver channel is near minimum or maximum.

int8_t                              // Ret: -1 if near minimum
                                    //       1 if near maximum
                                    //       0 otherwise
receiver_is_at_extreme(uint8_t ch) // In:  channel

{
  uint16_t raw;
  uint8_t  result;

  
  result = 0;
  
  if (receiver_get_status())
  {
    raw = receiver_get_current_raw(ch);
    
    if (raw > (uint16_t)RECEIVER_HIGH_THRESHOLD)
      result = 1;
    
    else if (raw < (uint16_t)RECEIVER_LOW_THRESHOLD)
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
    raw_avg = receiver_get_current_raw(ch) << RECEIVER_LONG_AVG_SHIFT;
  else  
    raw_avg = (uint16_t)RECEIVER_NOM_MID << RECEIVER_LONG_AVG_SHIFT;
}


//=========================== find_zero() =====================================
//
// Calculate an average and check whether the raw receiver input is stable
// around the zero point.  Should be called periodically.

volatile
boolean                        // Ret: true if stable aroud zero
ReceiverRotation::find_zero(void)

{
  int16_t  avg_diff;
  int16_t  zero_diff;
  uint16_t raw;
  boolean  status;


  if (! receiver_get_status())
  {
    // Receiver is not OK
    
    cycle_counter = 0;
    
    status = false;
  }

  else
  {
    raw = receiver_get_current_raw(ch);
    
    // Calculate long-term average, in units of (1 << RECEIVER_LONG_AVG_SHIFT)

    avg_diff = (int16_t)(raw - (raw_avg >> RECEIVER_LONG_AVG_SHIFT));
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
      
      status = false;
    }

    else if (cycle_counter < (uint8_t)RECEIVER_REST_CYCLES_MIN)
    {
      // Stable, but not for long enough
      
      cycle_counter++;
      
      status = false;
    }

    else
    {
      // We have been stable long enough, flag a stable condition
      
      raw_zero = raw_avg >> RECEIVER_LONG_AVG_SHIFT;

      status = true;
    };
  };
  
#if PRINT_RECEIVER_ROT
  Serial.print(ch, DEC);
  Serial.print("\t");
  Serial.print(raw_avg >> RECEIVER_LONG_AVG_SHIFT, DEC);
  Serial.print("\t");
  Serial.print(raw_zero, DEC);
  Serial.print("\t");
  Serial.print(raw, DEC);
  Serial.print("\t");
  Serial.println(cycle_counter, DEC);
#endif

  return status;
}


//=========================== get_rotation() ==================================
//
// Get the centered rotation value.  Should be called periodically.

int16_t                        // Ret: centered data, in units of RECEIVER_TICK
ReceiverRotation::get_rotation(void)

{
  int16_t diff;
  int16_t ret_val;

  
  if (receiver_get_status())
  {
    // Receiver works correctly
    
    ret_val = receiver_get_current_raw(ch) - raw_zero;

    if ((ret_val <= (int16_t) RECEIVER_ROTATION_THRESHOLD0)   &&
        (ret_val >= (int16_t)-RECEIVER_ROTATION_THRESHOLD0))
    {
      // A small margin around 0 is considered 0
      
      ret_val = 0;
    }

    else
    {
      diff = ret_val - (int16_t)RECEIVER_ROTATION_THRESHOLD1;
      if (diff > 0)
      {
        // Double the slope at the positive edge
        
        ret_val += diff;
      }

      else
      {
        diff = ret_val + (int16_t)RECEIVER_ROTATION_THRESHOLD1;
        if (diff < 0)
        {
          // Double the slope at the negative edge
          ret_val += diff;
        }
      }
    }
  }
  
  else
  {
    // Receiver does not operate correctly
    
    ret_val = 0;
  };

#if PRINT_RECEIVER_ROT
  Serial.print(ch, DEC);
  Serial.print("\t");
  Serial.println(ret_val, DEC);
#endif
    
  return ret_val;
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

volatile
int16_t                        // Ret: stable point if stable, -1 otherwise
ReceiverThrottle::find_stable(void)

{
  int16_t  diff;
  uint16_t raw;
  int16_t  ret_val = -1;


  if (! receiver_get_status())
  {
    // Receiver is not OK
    
    cycle_counter = 0;
  }

  else
  {
    raw = receiver_get_current_raw(THROTTLE_CH);
    
    // Calculate long-term average, in units of (1 << RECEIVER_LONG_AVG_SHIFT)

    diff = (int16_t)(raw - raw_stable);

    // To be stable, the current reading must not deviate from raw_stable too much.
    
    if ((diff > (int16_t)RECEIVER_THROTTLE_STABLE_DEV_MAX)    ||
        (diff < (int16_t)-RECEIVER_THROTTLE_STABLE_DEV_MAX)
       )
    {
      // Not stable. Reset the cycle counter and set a new raw_stable
      
      cycle_counter = 0;
      raw_stable = raw;
    }

    else if (cycle_counter < (uint8_t)RECEIVER_THROTTLE_STABLE_CYCLES_MIN)
    {
      // Stable, but not for long enough
      
      cycle_counter++;
    }

    else
    {
      // We have been stable long enough, flag a stable condition
      
      ret_val = raw_stable;
    };
  };
  
#if PRINT_RECEIVER_THROTTLE
  Serial.print(THROTTLE_CH, DEC);
  Serial.print("\t");
  Serial.print(ret_val, DEC);
  Serial.print("\t");
  Serial.print(raw_stable, DEC);
  Serial.print("\t");
  Serial.print(raw, DEC);
  Serial.print("\t");
  Serial.println(cycle_counter, DEC);
#endif

  return ret_val;
}


//=========================== find_min() ======================================
//
// Calculate an average and check whether the raw receiver input is stable
// around the minimum point.  Should be called periodically.

volatile boolean                        // Ret: true if stable at minimum
ReceiverThrottle::find_min(void)

{
  int16_t stable;
  boolean status = false;


  // Throttle has been at low for at least one sample
  
  stable = find_stable();
  
  if ((stable <= (int16_t)RECEIVER_LOW_THRESHOLD) && (stable >= 0))
  {
    // We have stabilized at low throttle
    
    status = true;
    raw_min = stable;
  };
  
  return status;
}


//=========================== find_max() ======================================
//
// Calculate an average and check whether the raw receiver input is stable
// around the maximum point.  Should be called periodically.

volatile boolean                        // Ret: true if stable at maximum
ReceiverThrottle::find_max(void)

{
  int16_t stable;
  boolean status = false;


  // Throttle has been at high for at least one sample
  
  stable = find_stable();
  
  if (stable >= (int16_t)RECEIVER_HIGH_THRESHOLD)
  {
    // We have stabilized at high throttle
    
    status = true;
    raw_max = stable;
  };
  
  return status;
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


  throttle_range = raw_max - raw_min;
  
  // Make sure the diff is within the nominal range

  if (throttle_range > (uint16_t)RECEIVER_THROTTLE_RANGE_MAX)
    throttle_range = RECEIVER_THROTTLE_RANGE_MAX;

  // Calculate the two thresholds of the received throttle
  
  raw_threshold1 = raw_min + (throttle_range >> 2);
  raw_threshold2 = raw_max - (throttle_range >> 2);

  // Calculate the slopes of the 3 ranges.

// Compile-time sanity check: make sure we do not have an overflow in the calculation

#if ((MOTOR_THROTTLE_RANGE1 >= (0x10000ul >> RECEIVER_THROTTLE_SLOPE_SHIFT))  || \
     (MOTOR_THROTTLE_RANGE2 >= (0x10000ul >> RECEIVER_THROTTLE_SLOPE_SHIFT))  || \
     (MOTOR_THROTTLE_RANGE3 >= (0x10000ul >> RECEIVER_THROTTLE_SLOPE_SHIFT)))
#error RECEIVER_THROTTLE_SLOPE_SHIFT is too large
#endif

  motor_throttle_slope1 =
    (uint16_t)(MOTOR_THROTTLE_RANGE1 << RECEIVER_THROTTLE_SLOPE_SHIFT) / (uint16_t)(raw_threshold1 - raw_min);
  motor_throttle_slope2 =
    (uint16_t)(MOTOR_THROTTLE_RANGE2 << RECEIVER_THROTTLE_SLOPE_SHIFT) / (uint16_t)(raw_threshold2 - raw_threshold1);
  motor_throttle_slope3 =
    (uint16_t)(MOTOR_THROTTLE_RANGE3 << RECEIVER_THROTTLE_SLOPE_SHIFT) / (uint16_t)(raw_max - raw_threshold2);

// The following definition are used for later compile-time checking of overflow

#if (MOTOR_THROTTLE_RANGE1 > MOTOR_THROTTLE_RANGE2)
#define MOTOR_THROTTLE_RANGE_MAX MOTOR_THROTTLE_RANGE1
#else
#define MOTOR_THROTTLE_RANGE_MAX MOTOR_THROTTLE_RANGE2
#endif
#if (MOTOR_THROTTLE_RANGE3 > MOTOR_THROTTLE_MAX)
#define MOTOR_THROTTLE_RANGE_MAX MOTOR_THROTTLE_RANGE3
#endif
#define RECEIVER_THROTTLE_RANGE_MIN ((RECEIVER_HIGH_THRESHOLD - RECEIVER_LOW_THRESHOLD) >> 2)
#define MOTOR_THROTTLE_SLOPE_MAX    (MOTOR_THROTTLE_RANGE_MAX / RECEIVER_THROTTLE_RANGE_MIN)

#if PRINT_RECEIVER_THROTTLE
  Serial.print("Thr. thresholds\t");
  Serial.print(raw_min, DEC);
  Serial.print("\t");
  Serial.print(raw_threshold1, DEC);
  Serial.print("\t");
  Serial.print(raw_threshold2, DEC);
  Serial.print("\t");
  Serial.println(raw_max, DEC);

  Serial.print("Thr. slopes\t");
  Serial.print(motor_throttle_slope1, DEC);
  Serial.print("\t");
  Serial.print(motor_throttle_slope2, DEC);
  Serial.print("\t");
  Serial.println(motor_throttle_slope3, DEC);
#endif
}


//=========================== get_throttle() ==================================
//
// Get the throttle value, normalized to motor throttle range
// The translation is not linear; to get better control in the useful range, we
// divide the throttle range into 3 sections.

int16_t                        // Ret: normalized data
ReceiverThrottle::get_throttle(void)

{
  int16_t   throttle_diff;
  uint16_t  motor_throttle;
  uint16_t  raw;


// Compile-time sanity check: make sure we do not have an overflow in the calculation

#if ((MOTOR_THROTTLE_SLOPE_MAX * (RECEIVER_THROTTLE_RANGE_MAX / 2)) > 0xFFFFul)
#error MOTOR_THROTTLE_SLOPE_MAX is too large
#endif

  if (receiver_get_status())
  {
    raw = receiver_get_current_raw(THROTTLE_CH);

    throttle_diff = raw - raw_threshold2;
    
    if (throttle_diff >= 0)
    {
      // We're in the upper range
      
      if (throttle_diff > (int16_t)RECEIVER_THROTTLE_RANGE_MAX)
        throttle_diff = RECEIVER_THROTTLE_RANGE_MAX;
      
      motor_throttle = motor_throttle_slope3 * (uint16_t)throttle_diff;
      motor_throttle >>= RECEIVER_THROTTLE_SLOPE_SHIFT;
      motor_throttle += (uint16_t)MOTOR_THROTTLE_THRESHOLD2;

      if (motor_throttle > (uint16_t)MOTOR_THROTTLE_TOP)
        motor_throttle = MOTOR_THROTTLE_TOP;
    }

    else
    {
      throttle_diff = raw - raw_threshold1;
      
      if (throttle_diff >= 0)
      {
        // We're in the middle range
        
        if (throttle_diff > (int16_t)RECEIVER_THROTTLE_RANGE_MAX)
          throttle_diff = RECEIVER_THROTTLE_RANGE_MAX;
        
        motor_throttle = motor_throttle_slope2 * (uint16_t)throttle_diff;
        motor_throttle >>= RECEIVER_THROTTLE_SLOPE_SHIFT;
        motor_throttle += (uint16_t)MOTOR_THROTTLE_THRESHOLD1;

        if (motor_throttle > (uint16_t)MOTOR_THROTTLE_THRESHOLD2)
          motor_throttle = MOTOR_THROTTLE_THRESHOLD2;
      }

      else
      {
        // We're in the lower range
        
        throttle_diff = raw - raw_min;
        
        if (throttle_diff > 0)
        {
          if (throttle_diff > (int16_t)RECEIVER_THROTTLE_RANGE_MAX)
            throttle_diff = RECEIVER_THROTTLE_RANGE_MAX;
          
          motor_throttle = motor_throttle_slope1 * (uint16_t)throttle_diff;
          motor_throttle >>= RECEIVER_THROTTLE_SLOPE_SHIFT;
        }
        
        else
          motor_throttle = 0;
        
        motor_throttle += (uint16_t)MOTOR_THROTTLE_IDLE;

        if (motor_throttle > (uint16_t)MOTOR_THROTTLE_THRESHOLD1)
          motor_throttle = MOTOR_THROTTLE_THRESHOLD1;
      }
    }
  }
  
  else
    motor_throttle = MOTOR_THROTTLE_IDLE;

  
#if PRINT_RECEIVER_THROTTLE
  Serial.print(THROTTLE_CH, DEC);
  Serial.print("\t");
  Serial.println(motor_throttle, DEC);
#endif

  return (int16_t)motor_throttle;
}


