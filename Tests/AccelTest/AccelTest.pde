#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>

#include "quad.h"
#include "adc.h"
#include "bat_sensor.h"
#include "motors.h"
#include "gyro.h"
#include "accel.h"
#include "receiver.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "indicators.h"
#include "serial_telemetry.h"
#include "eeprom_utils.h"


//=============================================================================
//
// Global Definitions
//
//=============================================================================

// Timeout values for the setup state

#define SENSORS_SETUP_MAX_SEC   5.0
#define SETUP_RECEIVER_MIN_SEC  1.5
#define SETUP_RECEIVER_MAX_SEC 10.0
#define SETUP_ARMING_MIN_SEC    2.0
#define SETUP_ERR_MIN_SEC       5.0

// Main Flight State

typedef enum
{
  FLIGHT_ERROR,                 // Error, stop motors and wait for reset
  FLIGHT_SETUP,                 // Setup before flight
  FLIGHT_READY                  // Flying
} FlightState;

// Sub-States of FLIGHT_SETUP

typedef enum
{
  SETUP_GYROS,                  // Waiting for gyros to stabilize and zero
  SETUP_RECEIVER_ROTATIONS,     // Waiting for receiver pitch, roll, yaw zero
  SETUP_RECEIVER_THROTTLE_MAX,  // Waiting for receiver throttle maximum
  SETUP_RECEIVER_THROTTLE_MIN,  // Waiting for receiver throttle minimum
  SETUP_ARMING,                 // Arming for flight
  SETUP_ERR                     // Setup error
} SetupState;

// Threshold for assuming yaw at 0.  Within this range, yaw heading is assumed
// correct (yaw is 0)

#define RECEIVER_YAW_ZERO_MAX  10


//=============================================================================
//
// Global Variables
//
//=============================================================================

// EEPROM Variables


// Cycle Timing Variables

uint8_t           last_msec      = 0;
uint16_t          avg_cycle_msec = 0;
uint8_t           max_cycle_msec = 0;

// Flight Control Variables

FlightState       flight_state = FLIGHT_SETUP;

Gyro              gyro[NUM_ROTATIONS];
RotationEstimator rot_estimator[NUM_ROTATIONS];

PID               rot_rate_pid[NUM_ROTATIONS];
PID               rot_pid[NUM_ROTATIONS];

float             rot_correction[NUM_ROTATIONS];   // (rad/sec)

float             receiver_rot_rate_gain = 0.002;  // (rad/sec)
                    // Multiplies the receiver rotation command (cenetered)
                    // to generate target rotation rate
float             receiver_rot_gain      = 0.002;  // (rad)
                    // Multiplies the receiver rotation command (cenetered)
                    // to generate target rotation
uint16_t          receiver_rot_limit     = 100;
                    // When the receiver rotation command is within +/- this
                    // limit from the center, the quad is stabilized for
                    // rotation.  Beyond this, it is stabilized for rotation
                    // rate.

// Objects for processing receiver inputs

ReceiverRotation receiver_rot[NUM_ROTATIONS] = {
                                                 ReceiverRotation(ROLL_CH),
                                                 ReceiverRotation(PITCH_CH),
                                                 ReceiverRotation(YAW_CH)
                                               };
ReceiverThrottle receiver_throttle;

// Calculated motor commands

int16_t           motor_throttle_command;
int16_t           motor_rot_command[NUM_ROTATIONS];

// Telemetry Variables

char              query;
boolean           cont_query = false;


//============================== read_eeprom() ==============================
//
// Read the configuration parameters from EEPROM, if valid.  If not, set
// defaults.

int                         // Ret: Next address in EEPROM
flight_control_read_eeprom(void)

{
  int eeprom_base_addr = EEPROM_FLIGHT_CONTROL_BASE_ADDR;
  
  if (eeprom_is_ok())
  {
    receiver_rot_rate_gain = eeprom_read_float(eeprom_base_addr);
    eeprom_base_addr += sizeof(float);
    receiver_rot_gain = eeprom_read_float(eeprom_base_addr);
    eeprom_base_addr += sizeof(float);
    receiver_rot_limit = eeprom_read_float(eeprom_base_addr);
    eeprom_base_addr += sizeof(float);
  }
  
  return eeprom_base_addr;
};


//============================== write_eeprom() =============================
//
// Write the configuration parameters to EEPROM

void flight_control_write_eeprom(void)

{
  eeprom_write_float(EEPROM_FLIGHT_CONTROL_BASE_ADDR,
                     receiver_rot_rate_gain);
  eeprom_write_float(EEPROM_FLIGHT_CONTROL_BASE_ADDR + sizeof(float),
                     receiver_rot_gain);
  eeprom_write_float(EEPROM_FLIGHT_CONTROL_BASE_ADDR + (2 * sizeof(float)),
                     receiver_rot_limit);
};


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  uint8_t        rot;
  int            eeprom_addr;


  Serial.begin(115200);

  adc_init();
  bat_sensor_init();
  indicators_init();
  eeprom_init();
  motors_init();
  accel_init();

  eeprom_addr = flight_control_read_eeprom();
  
  // Gyro
  
  eeprom_addr = Gyro::read_eeprom(eeprom_addr);
  gyro[PITCH].init(PITCH_RATE_PIN);
  gyro[ROLL].init(ROLL_RATE_PIN);
  gyro[YAW].init(YAW_RATE_PIN);

  // Receiver
  
  receiver_init();

  // Rotation Estimators
  
  RotationEstimator::set_cycle(CONTROL_LOOP_CYCLE_SEC);
  eeprom_addr = RotationEstimator::read_eeprom(eeprom_addr, 1);

  // PIDs
  
  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    eeprom_addr = rot_rate_pid[rot].read_eeprom(eeprom_addr,
                                                1,    // P
                                                0,    // I
                                               -50,   // D
                                                3.0); // windup_guard (rad/sec/sec)
  };

  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    eeprom_addr = rot_pid[rot].read_eeprom(eeprom_addr,
                                           1.5,   // P
                                           0,     // I
                                          -30,    // D
                                           3.0);  // windup_guard (rad/sec)
  };

  indicators_set(IND_SETUP);

  delay(1000);  // 1 second delay before we start.  Allows things such as
                // battery monitor, various filters to stabilize.
  
  //last_msec = adc_cycles;
  last_msec = millis();
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop()
{
  // Static Local Variables
  
  static uint16_t    setup_cycles                = 0;
  static SetupState  setup_state                 = SETUP_GYROS;
  static BatStatus   bat_status                  = BAT_OK;

  // Local Variables
  
  uint8_t            current_msec;
  uint8_t            cycle_msec;
  int16_t            receiver_rot_command[NUM_ROTATIONS];
                          // Rotation command input from the reciever
  float              rot_measurement[NUM_ROTATIONS];
                          // Rotations, as measured by the accelerometers (rad)
  float              rot_estimate;
                          // Rotation, as estimated based on gyro and
                          // accelerometer measurements (rad)
  //float              rot_error[NUM_ROTATIONS];        // (rad)
  float              rot_rate_error;   // (rad/sec)
  uint8_t            rot;                             // Rotation index
  BatStatus          new_bat_status;
  

  //---------------------------------------------------------------------------
  // Wait for start of next cycle
  //---------------------------------------------------------------------------
  
  //current_msec = adc_cycles;  // TODO: this is in 1.024msec in fact
  current_msec = millis();
  cycle_msec = (uint8_t)(current_msec - last_msec);
  
  if (cycle_msec > (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
    indicators_set(IND_SW_WARN);

  if (cycle_msec > max_cycle_msec)
    max_cycle_msec = cycle_msec;

  avg_cycle_msec -= avg_cycle_msec >> 8;
  avg_cycle_msec += cycle_msec;

  while (cycle_msec < (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
  {
    //current_msec = adc_cycles;
    current_msec = millis();
    cycle_msec = (uint8_t)(current_msec- last_msec);
  };

  last_msec = current_msec;

#if PRINT_CYCLE_TIME
  Serial.print(cycle_msec, DEC);
  Serial.print("\t");
  Serial.print(max_cycle_msec, DEC);
  Serial.print("\t");
  Serial.println(avg_cycle_msec >> 8, DEC);
#endif

  //---------------------------------------------------------------------------
  // Now do the flight control work
  //---------------------------------------------------------------------------

#if PRINT_STATE
  {
    static FlightState last_flight_state = (FlightState)-1;
    static SetupState  last_setup_state  = (SetupState)-1;
    
    if ((flight_state != last_flight_state) || (setup_state != last_setup_state))
    {
      Serial.print((int)flight_state, DEC);
      Serial.print("\t");
      Serial.println((int)setup_state, DEC);

      last_flight_state = flight_state;
      last_setup_state = setup_state;
    };
  }
#endif

  indicators_update();

  // Get the battery status and indicate
  // We only allow bat_status to get worse

  new_bat_status = bat_sensor_get();

  if (new_bat_status > bat_status)
  {
    bat_status = new_bat_status;
    if (new_bat_status == BAT_LOW)
      indicators_set(IND_BAT_LOW);
    else
      indicators_set(IND_BAT_WARN);
  }
  
  // Read the accelerometers
  
  accel_update();
    
#if PRINT_ACCEL
  accel_print_stats();
#endif

  // Read gyros, center and scale

  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    gyro[rot].update();

#if PRINT_GYRO
    gyro[rot].print_stats();
#endif
  };

#if PRINT_GYRO
  Serial.println();
#endif

  receiver_update_status();

#if PRINT_RECEIVER
  receiver_print_stats();
#endif
  
  if ((receiver_is_at_extreme(THROTTLE_CH) == -1) &&
      (receiver_is_at_extreme(YAW_CH)      ==  1) &&
      ((flight_state != FLIGHT_SETUP) || (setup_state != SETUP_GYROS)))
  {
    // TODO:  Same if Gyros are stable but only after flight.
    // Minimum throttle + yaw stick left indicates immediate stop.
    // Kill the motors and return to the setup phase.
  
    motors_disable();
    flight_state = FLIGHT_SETUP;
    setup_state = SETUP_GYROS;
    setup_cycles = 0;
    indicators_set(IND_SETUP);
  }

  else if (flight_state == FLIGHT_SETUP)
  {
    //-------------------------------------------------------------------------
    // Setup Phase
    // ===========
    //
    // Setup sequence is as follows:
    // 1. Wait for the gyros to stabilize
    // 2. Wait for the receiver to stabilize (roll, pitch, yaw at 0, throttle
    //    at minimum)
    // 3. Wait for throttle at maximum for 1/2 second
    // 4. Wait for throttle at minimum for 1/2 second
    // 5. Indicate "Arming"
    // 6. Start flight mode
    //-------------------------------------------------------------------------

    if (bat_status == BAT_LOW)
    {
      // Do not start the setup if the battery is low

      motors_disable();
      flight_state = FLIGHT_ERROR;
    }

    else
    {
      // Battery is not low
      
      switch (setup_state)
      {
        case SETUP_GYROS:
          // Wait until all the gyros are stable.  If this lasts more than
          // SENSORS_SETUP_MAX_SEC, indicate an error but continue waiting
          
          if (gyro[PITCH].is_stable() &&
              gyro[ROLL].is_stable()  &&
              gyro[YAW].is_stable())
          {
            // Zero all gyros
            
            gyro[PITCH].zero();
            gyro[ROLL].zero();
            gyro[YAW].zero();

            // Advance to next sub-state
            
            indicators_set(IND_SETUP_NEXT1);
            setup_state = SETUP_RECEIVER_ROTATIONS;
            setup_cycles = 0;

            // Initialize receiver rotations & throttle, in preparation for next
            // sub-state
            
            for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
            {
              receiver_rot[rot].init_zero();
            };
            receiver_throttle.init_stable();
          }
          
          else if (setup_cycles <= (uint16_t)(SENSORS_SETUP_MAX_SEC / CONTROL_LOOP_CYCLE_SEC))
            setup_cycles++;
          
          else
          {
            // The gyros should have stabilized by now.  Issue an error
            // indication
            
            indicators_set(IND_SETUP_ERR);
            setup_state = SETUP_ERR;
            setup_cycles = 0;
          };
  
          break;
  
        case SETUP_RECEIVER_ROTATIONS:
          // Wait until the 3 rototations are stable at 0, throttle stable at
          // minimum.  No wait timeout since it depends on the operator.

          // Note that bitwise-and (not logical and) is used in the expression
          // below. This is necessary; for logical expression the compiler would
          // do short-circuit evaluation, and would not continue the calculation
          // if one function returns false.  We need all the functions to by
          // called in every cycle.
  
          if (((receiver_rot[PITCH].find_zero() &
                receiver_rot[ROLL].find_zero()  &
                receiver_rot[YAW].find_zero()   &
                receiver_throttle.find_min()) != 0)    &&
              (setup_cycles > (uint16_t)(SETUP_RECEIVER_MIN_SEC / CONTROL_LOOP_CYCLE_SEC)))
          {
  
            indicators_set(IND_SETUP_NEXT2);
            setup_state = SETUP_RECEIVER_THROTTLE_MAX;
            setup_cycles = 0;
  
            // Initialize receiver throttle, in preparation for next
            // sub-state
            
            receiver_throttle.init_stable();
          }

          else if (setup_cycles < (uint16_t)-1)
            setup_cycles++;
  
          break;
  
        case SETUP_RECEIVER_THROTTLE_MAX:
          // Wait until the the throttle is stable at maximum.
          // No wait timeout since it depends on the operator.
  
          if ((receiver_throttle.find_max()) &&
              (setup_cycles > (uint16_t)(SETUP_RECEIVER_MIN_SEC / CONTROL_LOOP_CYCLE_SEC)))
          {
  
            indicators_set(IND_SETUP_NEXT3);
            setup_state = SETUP_RECEIVER_THROTTLE_MIN;
            setup_cycles = 0;
  
            // Initialize receiver throttle, in preparation for next
            // sub-state
            
            receiver_throttle.init_stable();
          }

          else if (setup_cycles < (uint16_t)-1)
            setup_cycles++;
  
          break;
  
        case SETUP_RECEIVER_THROTTLE_MIN:
          // Wait until the the throttle stable at minimum.
  
          if ((receiver_throttle.find_min()) &&
              (setup_cycles > (uint16_t)(SETUP_RECEIVER_MIN_SEC / CONTROL_LOOP_CYCLE_SEC)))
          {
            // Now we have the minimum and maximum, we can calculate the range factor
            
            receiver_throttle.calculate_throttle_motor_factor();
            
            indicators_set(IND_ARMING);
            setup_state = SETUP_ARMING;
            setup_cycles = 0;
          }

          else if (setup_cycles > (uint16_t)(SETUP_RECEIVER_MAX_SEC / CONTROL_LOOP_CYCLE_SEC))
          {
            // Too long time has passed, issue an error indication and start over
            
            indicators_set(IND_SETUP_ERR);
            setup_state = SETUP_ERR;
            setup_cycles = 0;
          }

          else
            setup_cycles++;
  
          break;
  
        case SETUP_ARMING:
          // Indicate that the motors are being enabled.
  
          if (setup_cycles <= (uint16_t)(SETUP_ARMING_MIN_SEC / CONTROL_LOOP_CYCLE_SEC))
            setup_cycles++;
          
          else
          {
            flight_state = FLIGHT_READY;
            indicators_set(IND_FLIGHT);
            motors_enable();
          };
          
          break;
  
        case SETUP_ERR:
          // Wait for a short time, then start the setup from the beginning
        
          if (setup_cycles <= (uint16_t)(SETUP_ERR_MIN_SEC / CONTROL_LOOP_CYCLE_SEC))
            setup_cycles++;
          
          else
          {
            setup_state = SETUP_GYROS;
            setup_cycles = 0;
          };
          
          break;

        default:
          indicators_set(IND_SW_ERR);
          flight_state = FLIGHT_ERROR;
      }
    }
  }
    
  else if (flight_state == FLIGHT_READY)
  {
    //-------------------------------------------------------------------------
    // Flight Phase
    //-------------------------------------------------------------------------

    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      receiver_rot_command[rot] = receiver_rot[rot].get_rotation();
    }
    
    // Get the roll & pitch measurements from the accelerators
    
    accel_get_rotations(rot_measurement);

    // For yaw, if the stick is in the middle assume a zero yaw rotation is
    // "measured" by the operator.  Else, assume no measurement.
    
    if ((receiver_rot_command[YAW] <= (int16_t)-RECEIVER_YAW_ZERO_MAX) &&
        (receiver_rot_command[YAW] >= (int16_t)-RECEIVER_YAW_ZERO_MAX))
      rot_measurement[YAW] = 0;
    else
      rot_measurement[YAW] = NAN;
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      // Estimate the rotation based on measurements
      
      rot_estimate = 
        rot_estimator[rot].estimate(gyro[rot].get_rad_per_sec(),
                                    rot_measurement[rot]);

#if PRINT_ROT_ESTIMATE
      rot_estimator[rot].print_stats();
#endif

      // Read rotation command and calculate error
      
      rot_rate_error = 
        ((float)receiver_rot_command[rot] * receiver_rot_rate_gain) -
        gyro[rot].get_rad_per_sec();

      #if PRINT_ROT_ERROR
            Serial.print(rot_rate_error);
            Serial.print("\t");
      #endif

      if (receiver_get_boolean(GEAR_CH)                                &&
          (receiver_rot_command[rot] <= (int16_t) receiver_rot_limit)  &&
          (receiver_rot_command[rot] >= (int16_t)-receiver_rot_limit))
      {
        // Stable mode

        // TODO: rot_error calculation based on receiver_rot_gain

        motor_rot_command[rot] = rot_rate_pid[rot].update_pd_i(rot_rate_error,
                                                               -rot_estimate);
      }

      else
        motor_rot_command[rot] = rot_rate_pid[rot].update_pd(rot_rate_error);
        
#if PRINT_MOTOR_ROT_COMMAND           
      Serial.print(motor_rot_command[rot]);
      Serial.print("\t");
#endif
    };

#if PRINT_MOTOR_ROT_COMMAND || PRINT_ROT_ERROR
    Serial.println();
#endif
    
    motor_throttle_command = receiver_throttle.get_throttle();

    motors_command(motor_throttle_command,
                   motor_rot_command[PITCH],
                   motor_rot_command[ROLL],
                   motor_rot_command[YAW]);
  }

  else
  {
    // flight_state == FLIGHT_ERROR

    motors_disable();
  };
  
#if PRINT_MOTOR_COMMAND
  motors_print_stats();
#endif

  //---------------------------------------------------------------------------
  // Telemetry
  //---------------------------------------------------------------------------
  
#if SUPPORT_TELEMENTRY 
  // Check for a new serial command
  
  if (Serial.available())
  {
    query = Serial.read();
    cont_query = true;
  }

  // New or continuous command
  
  if (cont_query)
  {
    cont_query = handle_serial_telemetry(query);
  }
#endif
}
