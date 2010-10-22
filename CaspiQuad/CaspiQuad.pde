#include <EEPROM.h>
#include <Wire.h>

#include "quad.h"
#include "i2c.h"
#include "adc.h"
#include "bat_sensor.h"
#include "motors.h"
#include "gyro.h"
#include "accel.h"
#include "baro.h"
#include "receiver.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "indicators.h"
#include "serial_telemetry.h"
#include "eeprom_utils.h"
#include "flight_control.h"


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

// Sub-States of FLIGHT_SETUP

typedef enum
{
  SETUP_GYROS,                      // Waiting for gyros to stabilize and zero
  SETUP_RECEIVER_ROTATIONS,         // Waiting for receiver pitch, roll, yaw zero
  SETUP_RECEIVER_THROTTLE_MAX,      // Waiting for receiver throttle maximum
  SETUP_RECEIVER_THROTTLE_MIN,      // Waiting for receiver throttle minimum
  SETUP_ARMING,                     // Arming for flight
  SETUP_RECEIVER_ROLL_CENTER_WAIT,  // Interim state
  SETUP_RECEIVER_ROLL_CENTER,       // Setting the roll trim
  SETUP_RECEIVER_PITCH_CENTER_WAIT, // Interim state
  SETUP_RECEIVER_PITCH_CENTER,      // Setting the pitch trim
  SETUP_ERR                         // Setup error
} SetupState;

// Threshold for yaw flutter filtering around 0.  Within this range,
// yaw control is assumed 0

#define RECEIVER_YAW_FLUTTER_MAX 16

// Threshold for assuming yaw at 0.  Within this range, yaw heading is assumed
// correct (yaw is 0), resetting the yaw integrator

#define RECEIVER_YAW_ZERO_MAX    32


//=============================================================================
//
// Global Variables
//
//=============================================================================

// EEPROM Variables


// Cycle Timing Variables

uint8_t            last_msec      = 0;
uint16_t           avg_cycle_msec = 0;
uint8_t            max_cycle_msec = 0;

// Hardware Status Variables

boolean            accel_ok       = false;
boolean            baro_ok        = false;

// Flight Control Variables

boolean            is_acro_mode        = false;
                    // false: stable mode, stick controls rotation angle
                    // true:  acrobatic mode, no accelerometer usage, stick
                    //        controls rotation rate
boolean            is_rot_rate_control = false;
                    // false: stick controls rotation angle
                    // true:  stick controls rotation rate
                    
FlightState        flight_state   = FLIGHT_SETUP;
static SetupState  setup_state    = SETUP_GYROS;
static uint16_t    setup_cycles   = 0;

Gyro               gyro[NUM_ROTATIONS];
RotationEstimator  rot_estimator[2];
RotationManualEstimator       yaw_estimator;

PID                rot_rate_pid[NUM_ROTATIONS];
 
float              receiver_rot_rate_gain = 0.003;  // (rad/sec)
                    // Multiplies the receiver rotation command (cenetered)
                    // to generate target rotation rate
uint8_t            receiver_rot_gain      = 9;
                    // Multiplies the receiver rotation command (cenetered)
                    // to generate target rotation
uint16_t           receiver_rot_limit     = 100;
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


//====================== flight_control_read_eeprom() =========================
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
    receiver_rot_gain = (uint8_t)eeprom_read_float(eeprom_base_addr);
    eeprom_base_addr += sizeof(float);
    receiver_rot_limit = (uint16_t)eeprom_read_float(eeprom_base_addr);
    eeprom_base_addr += sizeof(float);
  }
  
  return eeprom_base_addr;
};


//====================== flight_control_write_eeprom() ========================
//
// Write the configuration parameters to EEPROM

void flight_control_write_eeprom(void)

{
  eeprom_write_float(EEPROM_FLIGHT_CONTROL_BASE_ADDR,
                     receiver_rot_rate_gain);
  eeprom_write_float(EEPROM_FLIGHT_CONTROL_BASE_ADDR + sizeof(float),
                     (float)receiver_rot_gain);
  eeprom_write_float(EEPROM_FLIGHT_CONTROL_BASE_ADDR + (2 * sizeof(float)),
                     (float)receiver_rot_limit);
};


//=============================== flight_init() ===============================
//
// Initializations before entering FLIGHT_SETUP state
//

void flight_init(void)

{
  flight_state = FLIGHT_SETUP;
  setup_state = SETUP_GYROS;
  setup_cycles = 0;
  indicators_set(IND_SETUP);
}


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  uint8_t        rot;
  int            eeprom_addr;


  delay(500);  // 0.5 second delay before we start.  Seems that, e.g.,
               // barometric pressure sensor takes some time to reset

  Serial.begin(115200);

  i2c_init();

  adc_init();
  bat_sensor_init();
  indicators_init();
  eeprom_init();
  motors_init();
#if SUPPORT_ACCEL
  accel_ok = accel_init();
  if (! accel_ok)
    indicators_set(IND_HW_ERR_ACCEL_INIT);
#endif
#if SUPPORT_BARO
  baro_ok = baro_init();
  if (! baro_ok)
    indicators_set(IND_HW_ERR_BARO_INIT);
#endif

  eeprom_addr = flight_control_read_eeprom();
  
  // Gyro
  
  eeprom_addr = Gyro::read_eeprom(eeprom_addr);
  gyro[PITCH].init(PITCH_RATE_PIN);
  gyro[ROLL].init(ROLL_RATE_PIN);
  gyro[YAW].init(YAW_RATE_PIN);

  // Receiver
  
  receiver_init();

  // Rotation Estimators
  
  eeprom_addr = RotationEstimator::read_eeprom(eeprom_addr,
                                               0.5);   // bw

  // PIDs
  
  eeprom_addr = rot_rate_pid[ROLL ].read_eeprom(eeprom_addr,
                                                30,    // P
                                                 0.01, // I
                                                40,    // D
                                                 0);   // windup_guard - NOT USED
  eeprom_addr = rot_rate_pid[PITCH].read_eeprom(eeprom_addr,
                                                30,    // P
                                                 0.01, // I
                                                40,    // D
                                                 0);   // windup_guard - NOT USED
  eeprom_addr = rot_rate_pid[YAW  ].read_eeprom(eeprom_addr,
                                                80,    // P
                                                 0.003,// I
                                                 0,    // D
                                                 0);   // windup_guard - NOT USED

  eeprom_addr = flight_control_init(eeprom_addr);
  
  delay(1000);  // 1 second delay before we start.  Allows things such as
                // battery monitor, various filters to stabilize.
  
  flight_init();

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
  
  static BatStatus   bat_status                      = BAT_OK;
                          // Battery status
  static int16_t     last_receiver_rot_command[2]    = {ROT_NONE, ROT_NONE};
                          // Last cycle's rotation command input from the
                          // reciever (pitch/roll only)
  static boolean     was_alt_hold                    = false;
                          // Indicates if we had altitude hold in the last cycle
  static int16_t     alt_hold_motor_throttle_command = 0;
                          // Throttle command for altitude hold.  Set on
                          // initiation of altitude hold mode.
                          
  // Local Variables
  
  uint8_t            current_msec;
                          // Current time
  uint8_t            cycle_msec;
                          // Measured cycle time

  boolean            yaw_at_zero;
  boolean            pitch_at_zero;
  boolean            roll_at_zero;
  boolean            throttle_at_min;
                          // Used during setup phase to indicate stick status
                          
  int16_t            receiver_rot_command[NUM_ROTATIONS];
                          // Rotation command input from the reciever
  int16_t            temp_receiver_rot_command;
  int16_t            rot_measurement[NUM_ROTATIONS];
                          // Rotations, as measured by the accelerometers.
                          // Scale is defined by ROT_SCALE_RAD
  int16_t            rot_estimate[NUM_ROTATIONS];
                          // Rotation, as estimated based on gyro and
                          // accelerometer measurements.
                          // Scale is defined by ROT_SCALE_RAD
  int16_t            rot_error;
                          // Rotation error (command - measurement)
                          // Scale is defined by ROT_SCALE_RAD
  float              rot_rate_error;
                          // Rotation rate error (command - measurement) (rad/sec)
  float              temp_motor_rot_command;
                          // Temporarily holds the motor rotation command before
                          // converting to int16_t
  uint8_t            rot; // Rotation index
  boolean            is_alt_hold;
                          // Indicates altitude hold for the current cycle
  float              motor_alt_command;
                          // Output of altitude hold PID
  BatStatus          new_bat_status;
                          // New battery status, as read
  

  //---------------------------------------------------------------------------
  // Wait for start of next cycle
  //---------------------------------------------------------------------------
  
  //current_msec = adc_cycles;  // TODO: this is in 1.024msec in fact
  current_msec = millis();
  cycle_msec = (uint8_t)(current_msec - last_msec);
  
  if (cycle_msec > (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
    indicators_set(IND_SW_WARN_LOOP_CYCLE);

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
  
#if SUPPORT_ACCEL
  // Read the accelerometers
  
  accel_ok = accel_update();
  if (! accel_ok)
    indicators_set(IND_HW_ERR_ACCEL_INIT);
  
#if PRINT_ACCEL
  accel_print_stats();
#endif
#endif

#if SUPPORT_BARO
  baro_ok = baro_update();
  if (! baro_ok)
    indicators_set(IND_HW_ERR_BARO_INIT);
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

  // Check for "immediate stop" input
  
  if ((receiver_is_at_extreme(THROTTLE_CH) == -1) &&
      (receiver_is_at_extreme(YAW_CH)      ==  1) &&
      ((flight_state != FLIGHT_SETUP) || (setup_state != SETUP_GYROS)))
  {
    // TODO:  Same if Gyros are stable but only after flight.
    // Minimum throttle + yaw stick left indicates immediate stop.
    // Kill the motors and return to the setup phase.

    motors_disable();
    flight_init();
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
              gyro[ROLL ].is_stable()  &&
              gyro[YAW  ].is_stable())
          {
            // Zero all gyros
            
            gyro[PITCH].zero();
            gyro[ROLL ].zero();
            gyro[YAW  ].zero();

            // Reset the rotation estimators

            rot_estimator[ROLL ].reset();
            rot_estimator[PITCH].reset();
            yaw_estimator.reset();

            // Measure the rest earth G acceleration

            accel_zero_earth_z();
            
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
            
            indicators_set(IND_SETUP_ERR_SENSORS_SETUP_TIMEOUT);
            setup_state = SETUP_ERR;
            setup_cycles = 0;
          };
  
          break;
  
        case SETUP_RECEIVER_ROTATIONS:
          // Wait until the 3 rototations are stable at 0, throttle stable at
          // minimum.  No wait timeout since it depends on the operator.

          // Set the is_acro mode and is_rot_rate_control here; they are not
          // allowed to change during flight
          
#if SUPPORT_ACRO_MODE_SWITCH
          is_acro_mode = receiver_get_boolean(ENABLE_ACRO_MODE_CH);
#endif

#if SUPPORT_ROT_RATE_SWITCH
          is_rot_rate_control = receiver_get_boolean(ENABLE_ROT_RATE_CH);
#endif

          if (is_acro_mode)
            is_rot_rate_control = true;

          // Note that all the find*() functions must be called every cycle.
          // Thus, we call them and put the result into temporary variables, then
          // do the if statement.  This is necessary; if we were to call the
          // functions inside the if, the compiler would do short-circuit evaluation,
          // and would not continue the calculation if one function returns false.

          yaw_at_zero = receiver_rot[YAW].find_zero();
          pitch_at_zero = true;
          roll_at_zero  = true;
          if (is_rot_rate_control)
          {
            // Pitch/roll sticks control rate, zero the stick
            
            pitch_at_zero = receiver_rot[PITCH].find_zero();
            roll_at_zero  = receiver_rot[ROLL ].find_zero();
          }
          throttle_at_min = receiver_throttle.find_min();

          if (yaw_at_zero     &&
              pitch_at_zero   &&
              roll_at_zero    &&
              throttle_at_min &&
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

          if (receiver_is_at_extreme(YAW_CH) ==  -1) 
          {
            // Yaw stick @ right, on to roll center setup state
            
            indicators_set(IND_SETUP_ROLL_CENTER);
            setup_state = SETUP_RECEIVER_ROLL_CENTER_WAIT;
            setup_cycles = 0;
          }
          
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
          // Wait until the throttle is stable at minimum.
  
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
            
            indicators_set(IND_SETUP_ERR_THROTTLE_MIN_TIMEOUT);
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

        case SETUP_RECEIVER_ROLL_CENTER_WAIT:
          // Wait until yaw stick is back to center
          
          if (receiver_is_near_center(YAW_CH))
            setup_state = SETUP_RECEIVER_ROLL_CENTER;

          break;
          
        case SETUP_RECEIVER_ROLL_CENTER:
          
          if (receiver_is_at_extreme(YAW_CH) ==  -1) 
          {
            // Yaw stick @ right, on to pitch center setup state
            
            indicators_set(IND_SETUP_PITCH_CENTER);
            setup_state = SETUP_RECEIVER_PITCH_CENTER_WAIT;
            setup_cycles = 0;
          }

          // Indicate roll stick state
          
          temp_receiver_rot_command = receiver_rot[ROLL].get_rotation();
          if (temp_receiver_rot_command > 0)
            indicators_set(IND_ROT_POSITIVE);
          else if (temp_receiver_rot_command < 0)
            indicators_set(IND_ROT_NEGATIVE);
          else
            indicators_set(IND_SETUP);

          break;

        case SETUP_RECEIVER_PITCH_CENTER_WAIT:
          // Wait until yaw stick is back to center
          
          if (receiver_is_near_center(YAW_CH))
            setup_state = SETUP_RECEIVER_PITCH_CENTER;
        
          break;
            
        case SETUP_RECEIVER_PITCH_CENTER:
          
          if (receiver_is_at_extreme(YAW_CH) ==  -1) 
          {
            // Yaw stick @ right, backup to initial setup state
            
            indicators_set(IND_SETUP);
            setup_state = SETUP_GYROS;
            setup_cycles = 0;
          }

          // Indicate pitch stick state

          temp_receiver_rot_command = receiver_rot[PITCH].get_rotation();
          if (temp_receiver_rot_command > 0)
            indicators_set(IND_ROT_POSITIVE);
          else if (temp_receiver_rot_command < 0)
            indicators_set(IND_ROT_NEGATIVE);
          else
            indicators_set(IND_SETUP);
        
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

    // Get the receiver inputs
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      receiver_rot_command[rot] = receiver_rot[rot].get_rotation();
    }
    
    // Get the roll & pitch measurements from the accelerators

    if ((! accel_ok)
#if SUPPORT_ACCEL_ROT_SWITCH
        || receiver_get_boolean(DISABLE_ACCEL_ROT_CH)
#endif
       )
    {
      // Accelerometers failure or test mode - ignore the accelerometer inputs

      rot_measurement[ROLL ] = ROT_NONE;
      rot_measurement[PITCH] = ROT_NONE;
    }

    else
      accel_get_rotations(rot_measurement);

#if SUPPORT_ACCEL_ROT_INDICATION
    // Indicate whether we have legal measurements on both axes

    if ((rot_measurement[ROLL ] != ROT_NONE) &&
        (rot_measurement[PITCH] != ROT_NONE))
      indicators_set(IND_FLIGHT_WITH_ACCEL);
    else
      indicators_set(IND_FLIGHT_WITHOUT_ACCEL);
#endif

    // Estimate the roll & pitch rotations based on measurements
    
    for (rot = ROLL; rot <= PITCH; rot++)
    {
      if (is_acro_mode)
      {
        // In acrobatic mode we do not control pitch/roll rotation angles
        
        rot_estimate[rot] = 0;
      }

      else
      {
        // In stable mode we control pitch/roll rotation angles, so get their
        // estiamtes
        
        rot_estimate[rot] = 
          rot_estimator[rot].estimate(gyro[rot].get_rad_per_sec(),
                                      rot_measurement[rot]);
      }
    };

    // Eliminite small yaw flutter around 0
    
    if ((receiver_rot_command[YAW] <= (int16_t) RECEIVER_YAW_FLUTTER_MAX)   &&
        (receiver_rot_command[YAW] >= (int16_t)-RECEIVER_YAW_FLUTTER_MAX))
      receiver_rot_command[YAW] = 0;
    
    // For yaw, we don't have a real rotation angle measurement.  The estimator
    // gets a paramter that tells it id the operator is currently controlling yaw,
     // i.e., if the stick is not near the center.
    
    rot_estimate[YAW] = yaw_estimator.estimate(
                          gyro[YAW].get_rad_per_sec(),
                          ((receiver_rot_command[YAW] >= (int16_t) RECEIVER_YAW_ZERO_MAX) ||
                           (receiver_rot_command[YAW] <= (int16_t)-RECEIVER_YAW_ZERO_MAX)));
    
    // PID Control using Rotation & Rotation Rate
    // ==========================================
    //
    // Interpretation of Receiver Roll, Pitch and Yaw Inputs
    // -----------------------------------------------------
    //
    // For roll & pitch, if the receiver input is near the center (within
    // receiver_rot_limit) then it controls rotation angle.  Outside that
    // range, receiver input controls rotation rate.
    //
    // Receiver gear channel overrides this, so receiver input always controls
    // rotation rate.
    // 
    // For yaw, receiver input always controls yaw rate.
    //
    // Consideration of Rotation Estimation
    // ------------------------------------
    //
    // For all 3 rotations (roll, pitch & yaw), if the receiver input is near
    // the center (within receiver_rot_limit) then the rotation estimation is
    // fed into the I leg of the PID.  Outside that range, the I input is 0.
    
    for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
    {
      temp_receiver_rot_command = receiver_rot_command[rot];
      rot_error = -rot_estimate[rot];
      rot_rate_error = -gyro[rot].get_rad_per_sec();
      
      if ((rot != YAW) && (! is_rot_rate_control)
         )
      {
        // Receiver input controls rotation angle

        // Scale the receiver command to the rotation scale (as set by
        // ROT_SCAL_RAD) and calculate the rotation error

        temp_receiver_rot_command *= receiver_rot_gain;
        
        rot_error += temp_receiver_rot_command;

        // Calculate the receiver command's derivative,scale to the
        // rotation rate scale (rad/sec) and calculate the rotation rate
        // error

        if ((last_receiver_rot_command[rot] != ROT_NONE)
#if SUPPORT_ROT_DERIVATIVE_SWITCH
            && receiver_get_boolean(ENABLE_ROT_DERIVATIVE_CH)
#endif
           )
        {
          rot_rate_error += (float)(temp_receiver_rot_command - last_receiver_rot_command[rot]) *
                            (1 / (float)ROT_SCALE_RAD * (float)CONTROL_LOOP_CYCLE_SEC);

        }

        last_receiver_rot_command[rot] = temp_receiver_rot_command;
      }

      else
      {
        // Receiver input controls rotation rate
        
        rot_rate_error += (float)temp_receiver_rot_command *
                          receiver_rot_rate_gain;
      };

#if PRINT_ROT_ERROR
      Serial.print(rot_error, DEC);
      Serial.print("\t");
      Serial.print(rot_rate_error);
      Serial.print("\t");
#endif

      // Do the PID for this rotation axis

      temp_motor_rot_command = rot_rate_pid[rot].update_pd_i(rot_rate_error,
                                                             rot_error);

      // Constrain the rotation command to the minimum and maximum legal
      // values before converting to int16_t.  This ensures no overflow nor
      // underflow happens.
      // TODO: motor_rot_command() does this constrain again (but in int16_t)
      // TODO: eliminate the double work.

      motor_rot_command[rot] = constrain(temp_motor_rot_command,
                                         (float)MOTOR_ROTATION_RATE_MIN,
                                         (float)MOTOR_ROTATION_RATE_MAX);
      
#if PRINT_MOTOR_ROT_COMMAND           
      Serial.print(motor_rot_command[rot], DEC);
      Serial.print("\t");
#endif
    };

#if PRINT_MOTOR_ROT_COMMAND || PRINT_ROT_ERROR
    Serial.println();
#endif

    motor_throttle_command = receiver_throttle.get_throttle();

#if SUPPORT_BARO
    // Altitude control using barometer input

    if (baro_ok)
      motor_throttle_command = flight_control_alt(motor_throttle_command);
#endif

    motors_command(motor_throttle_command,
                   motor_rot_command[PITCH],
                   motor_rot_command[ROLL ],
                   motor_rot_command[YAW  ]);
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
  handle_serial_telemetry();
#endif
}
