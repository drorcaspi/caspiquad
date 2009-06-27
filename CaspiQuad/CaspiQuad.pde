#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>

#include "quad.h"
#include "motors.h"
#include "gyro.h"
#include "accel.h"
#include "receiver.h"
#include "pid.h"
#include "rotation_estimator.h"
#include "indicator.h"
#include "serial_telemetry.h"
#include "eeprom_utils.h"


//=============================================================================
//
// Global Definitions
//
//=============================================================================

#define CONTROL_LOOP_CYCLE_SEC 0.02 
#define MAX_SENSORS_SETUP_SEC  4

typedef enum

{
  FLIGHT_SETUP,
  FLIGHT_READY
} FlightState;


//=============================================================================
//
// Global Variables
//
//=============================================================================

// EEPROM Variables


// Cycle Timing Variables

uint32_t          last_msec = 0;
uint16_t          avg_cycle_msec = 0;
uint8_t           max_cycle_msec = 0;

// Flight Control Variables

FlightState       flight_state = FLIGHT_SETUP;
uint8_t           sensors_setup_cycles = 0;

Gyro              gyro[NUM_ROTATIONS];
RotationEstimator rot_estimator[NUM_ROTATIONS];

PID               rot_rate_pid[NUM_ROTATIONS];
PID               rot_pid[NUM_ROTATIONS];

float             rot_correction[NUM_ROTATIONS];   // (rad/sec)

//float             gyro_rad_per_sec[NUM_ROTATIONS];

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
  uint8_t  rot;
  int      eeprom_addr;


  analogReference(ANALOG_REFERENCE);
  Serial.begin(115200);

  Indicator::init();
  eeprom_init();
  motors_init();
  accel_init();

  eeprom_addr = flight_control_read_eeprom();
  
  // Gyro
  
  eeprom_addr = Gyro::read_eeprom(eeprom_addr, 0.2);  // Default smooting factor
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

  Indicator::indicate(Indicator::SETUP);
  last_msec = millis();
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop ()
{
  uint32_t current_msec;
  uint8_t  cycle_msec;
  int8_t   accel_raw[NUM_AXIS];
  //uint16_t gyro_raw[NUM_ROTATIONS];
  float    rotation_raw[NUM_ROTATIONS];
  float    pitch_estimate;                     // (rad)
  float    roll_estimate;                      // (rad)
  int16_t  temp_receiver_raw;
  float    target_rot[NUM_ROTATIONS];        // (rad)
  float    target_rot_rate[NUM_ROTATIONS];   // (rad/sec)
  float    rot_error[NUM_ROTATIONS];        // (rad)
  float    rot_rate_error[NUM_ROTATIONS];   // (rad/sec)
  //uint16_t throttle_command;
  uint8_t  rot;
  boolean  receiver_ok;
  

  //------------------------------
  // Wait for start of next cycle
  //------------------------------
  
  current_msec = millis();
  cycle_msec = (uint8_t)(current_msec- last_msec);
  
#if 1
  if (cycle_msec > (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
    Indicator::indicate(Indicator::SW_WARN);
#endif

  if (cycle_msec > max_cycle_msec)
    max_cycle_msec = cycle_msec;

  avg_cycle_msec -= avg_cycle_msec >> 8;
  avg_cycle_msec += cycle_msec;

  while (cycle_msec < (uint8_t)(CONTROL_LOOP_CYCLE_SEC * 1000))
  {
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

  //-------------------------------- 
  // Now do the flight control work
  //--------------------------------

  // Read the accelerometers and calculate raw rotations
  
  accel_read(accel_raw);
  
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

  pitch_estimate = rot_estimator[PITCH].estimate(gyro[PITCH].get_rad_per_sec(),
                                                 accel_raw[X_AXIS],
                                                 accel_raw[Z_AXIS]);
  roll_estimate = rot_estimator[ROLL].estimate(gyro[ROLL].get_rad_per_sec(),
                                               accel_raw[Y_AXIS],
                                               accel_raw[Z_AXIS]);

#if PRINT_ROT_ESTIMATE
  rot_estimator[ROLL].print_stats();
#endif

  if (flight_state == FLIGHT_SETUP)
  {
    //-----------------
    // Setup Phase
    //-----------------

    if (gyro[PITCH].is_stable() &&
        gyro[ROLL].is_stable()  &&
        gyro[YAW].is_stable())
    {
      gyro[PITCH].zero();
      gyro[ROLL].zero();
      gyro[YAW].zero();
      
      flight_state = FLIGHT_READY;
      Indicator::indicate(Indicator::FLIGHT);  // TODO: indicate warning first!
      
      motors_enable();
    }

    else if (sensors_setup_cycles <= (uint8_t)(MAX_SENSORS_SETUP_SEC / CONTROL_LOOP_CYCLE_SEC))
      sensors_setup_cycles++;

    else
    {
      sensors_setup_cycles = 0;
      Indicator::indicate(Indicator::SETUP_ERR);
    }
  }
    

  else
  {
    //-----------------
    // Flight Phase
    //-----------------

    receiver_ok = receiver_get_status();
    temp_receiver_raw = 0;   // Default value in case receiver is not OK

    // Read pitch command and calculate error
    
    if (receiver_ok)
      temp_receiver_raw = (int16_t)(receiver_get_current_raw(PITCH_CH) -
                                    (uint16_t)RECEIVER_NOM_MID);

    if ((temp_receiver_raw > (int16_t)receiver_rot_limit)  ||
        (temp_receiver_raw < (int16_t)-receiver_rot_limit))
    {
      // Target is pitch rate
      
      rot_rate_error[PITCH] = ((float)temp_receiver_raw * receiver_rot_rate_gain) -
                              gyro[PITCH].get_smoothed_rad_per_sec();
      rot_error[PITCH] = 0.0;
    }

    else
    {
      // Target is pitch 
      
      rot_rate_error[PITCH] = -gyro[PITCH].get_smoothed_rad_per_sec();
      rot_error[PITCH] = ((float)temp_receiver_raw * receiver_rot_gain) -
                         pitch_estimate;
    };

    // Read roll command and calculate error
      
    if (receiver_ok)
      temp_receiver_raw = (int16_t)(receiver_get_current_raw(ROLL_CH) -
                                    (uint16_t)RECEIVER_NOM_MID);

    if ((temp_receiver_raw > (int16_t)receiver_rot_limit)  ||
        (temp_receiver_raw < (int16_t)-receiver_rot_limit))
    {
      // Target is roll rate
      
      rot_rate_error[ROLL] = ((float)temp_receiver_raw * receiver_rot_rate_gain) -
                              gyro[ROLL].get_smoothed_rad_per_sec();
      rot_error[ROLL] = 0.0;
    }
    
    else
    {
      // Target is roll
      
      rot_rate_error[ROLL] = -gyro[ROLL].get_smoothed_rad_per_sec();
      rot_error[ROLL] = ((float)temp_receiver_raw * receiver_rot_gain) -
                        roll_estimate;
    };

    // Read yaw command and calculate error
        
    if (receiver_ok)
      temp_receiver_raw = (int16_t)(receiver_get_current_raw(YAW_CH) -
                                    (uint16_t)RECEIVER_NOM_MID);

    // Target is yaw rate
    
    rot_rate_error[YAW] = ((float)temp_receiver_raw * receiver_rot_rate_gain) - 
                          gyro[YAW].get_smoothed_rad_per_sec();
    target_rot[YAW] = 0.0;

#if PRINT_RECEIVER_ROT
  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    Serial.print(rot_rate_error[rot]);
    Serial.print("\t");
    Serial.print(rot_error[rot]);
    Serial.print("\t");
  };
  Serial.println("\t");
#endif

#if 0
    rot_correction[PITCH] = rot_pid[PITCH].update(0.0,              // target_position (rad)
                                                  pitch_estimate);  // current_position (rad)
    rot_correction[ROLL] = rot_pid[ROLL].update(0.0,                // target_position (rad)
                                                roll_estimate);     // current_position (rad)
#if PRINT_ROT_CORRECTION
    Serial.print(rot_correction[PITCH]);
    Serial.print("\t");
    Serial.println(rot_correction[ROLL]);
#endif
#endif

#if 1  // Two PIDs using both rotation and gyro
  for (rot = FIRST_ROTATION; rot < NUM_ROTATIONS; rot++)
  {
    motor_rot_command[rot] = rot_pid[rot].update(rot_error[rot]) +
                             rot_rate_pid[rot].update(rot_rate_error[rot]);
  };
#endif
#if 0  // PID using both rotation and gyro
  motor_rot_command[PITCH] = 
    rot_rate_pid[PITCH].update(target_rot[PITCH],           // target position (rad)
                               pitch_estimate,                // current position (rad)
                               target_rot_rate[PITCH],      // target_rate (rad/sec)
                               gyro[PITCH].get_smoothed_rad_per_sec());
                                                              // current_rate (rad/sec)
  motor_rot_command[ROLL] = 
    rot_rate_pid[ROLL].update(target_rot[ROLL],             // target position (rad)
                              roll_estimate,                  // current position (rad)
                              target_rot_rate[ROLL],        // target_rate (rad/sec)
                              gyro[ROLL].get_smoothed_rad_per_sec());
                                                              // current_rate (rad/sec)
  motor_rot_command[YAW] = 
    rot_rate_pid[YAW].update(0.0,                             // target position (rad)
                             0.0,                             // current position (rad)
                             target_rot_rate[YAW],          // target_rate (rad/sec)
                             gyro[YAW].get_smoothed_rad_per_sec());
                                                              // current_rate (rad/sec)
#endif
#if 0  
    motor_rot_command[PITCH] = 
      rot_rate_pid[PITCH].update(rot_correction[PITCH],         // target_position (rad/sec)
                                 gyro[PITCH].get_smoothed_rad_per_sec());
                                                                // current_position (rad/sec)
    motor_rot_command[ROLL] = 
      rot_rate_pid[ROLL].update(rot_correction[ROLL],           // target_position (rad/sec)
                                gyro[ROLL].get_smoothed_rad_per_sec());
                                                                // current_position (rad/sec)
    motor_rot_command[YAW] = 
      rot_rate_pid[YAW].update(0.0,                             // target_position (rad/sec)
                               gyro[YAW].get_smoothed_rad_per_sec());
                                                                // current_position (rad/sec)
#endif
#if 0
    motor_rot_command[PITCH] = motor_rot_gain * rot_correction[PITCH];
    motor_rot_command[ROLL] = motor_rot_gain * rot_correction[ROLL];
    motor_rot_command[YAW] = 0.0;
#endif

#if PRINT_MOTOR_ROT_COMMAND           
    Serial.print(motor_rot_command[PITCH]);
    Serial.print("\t");
    Serial.print(motor_rot_command[ROLL]);
    Serial.print("\t");
    Serial.print(motor_rot_command[YAW]);
    Serial.println("\t");
#endif

#if PRINT_RECEIVER
    receiver_print_stats();
#endif

    if (receiver_get_status())
      motor_throttle_command =
                     ((receiver_get_current_raw(THROTTLE_CH)  - RECEIVER_NOM_MIN) /
                     ((RECEIVER_NOM_MAX - RECEIVER_NOM_MIN) / MOTOR_THROTTLE_RANGE)) +
                     MOTOR_THROTTLE_MIN;
    else
      motor_throttle_command = MOTOR_THROTTLE_MIN;
    
    motors_command(motor_throttle_command,
                   0 /*motor_rot_command[PITCH]*/,
                   motor_rot_command[ROLL],
                   0 /*motor_rot_command[YAW]*/);
  } 
  
#if PRINT_MOTOR_COMMAND
  motors_print_stats();
#endif

  //-----------
  // Telemetry
  //-----------
  
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

  Indicator::update();
}
