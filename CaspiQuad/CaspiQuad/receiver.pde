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
        p_ch_data->ticks_high = ticks_diff;
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


//======================== receiver_get_status() ==============================
//
// Get the current status of the receiver

boolean                    // Ret: true if OK, false if not OK
receiver_get_status(void)

{
  boolean status = true;
  uint8_t ch;


  for (ch = FIRST_CH; ch < NUM_CH; ch++)
  {
    if (ch_data[ch].error_count >= RECEIVER_MAX_ERRORS)
       status = false;
  }

  return status;
}


//====================== receiver_get_current_raw() ===========================
//
// Get the current raw receiver data (that has been read before from the h/w)

uint16_t                             // Ret: raw data, in units of  RECEIVER_TICK
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

