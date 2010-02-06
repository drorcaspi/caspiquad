#include "quad.h"
#include "motors.h"


//=============================================================================
//
// Global Definitions
//
//=============================================================================


//=============================================================================
//
// Global Variables
//
//=============================================================================


//=============================== setup() =====================================
//
// Arduino Initialization
//
//=============================================================================

void setup()
{
  uint8_t  dir;
  char     c;
  uint8_t  throttle = MOTOR_THROTTLE_MIN;

  
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.println("ESC Throttle Range Setup");
  Serial.println("Setting Throttle High");
  Serial.print(MOTORS_PWM_CYCLE_USEC, DEC);
  Serial.print("\t");
  Serial.print((uint32_t)((uint32_t)MOTORS_PW_MAX_USEC * MOTOR_COMMAND_RANGE), DEC);
  Serial.print("\t");
  Serial.print(MOTOR_THROTTLE_MAX, DEC);
  Serial.print("\t");
  Serial.println(MOTOR_THROTTLE_MIN, DEC);
  
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);

  analogWrite(FRONT_MOTOR_PIN, MOTOR_THROTTLE_MAX);
  analogWrite(REAR_MOTOR_PIN,  MOTOR_THROTTLE_MAX);
  analogWrite(RIGHT_MOTOR_PIN, MOTOR_THROTTLE_MAX);
  analogWrite(LEFT_MOTOR_PIN,  MOTOR_THROTTLE_MAX);

  Serial.print("Connect battery to ESC, wait ~2sec until \"beep-beep\" > ");
  while (! Serial.available());
  c = Serial.read();

  Serial.println("Setting Throttle Low");

  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);

  analogWrite(FRONT_MOTOR_PIN, MOTOR_THROTTLE_MIN);
  analogWrite(REAR_MOTOR_PIN,  MOTOR_THROTTLE_MIN);
  analogWrite(RIGHT_MOTOR_PIN, MOTOR_THROTTLE_MIN);
  analogWrite(LEFT_MOTOR_PIN,  MOTOR_THROTTLE_MIN);

  Serial.print("Wait for 3 \"beeps\", then long \"beep\" > ");
  while (! Serial.available());
  c = Serial.read();

  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("Enter + to increase throttle, - to decrease throttle");
  while (true)
  {
    while (! Serial.available());
    c = Serial.read();
    if ((c == '+') && (throttle <= (MOTOR_THROTTLE_MAX - 10)))
      throttle += 10;
    else if ((c == '-') && (throttle >= (MOTOR_THROTTLE_MIN + 10)))
      throttle -= 10;
    Serial.println((int16_t)throttle, DEC);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);

    analogWrite(FRONT_MOTOR_PIN, throttle);
    analogWrite(REAR_MOTOR_PIN,  throttle);
    analogWrite(RIGHT_MOTOR_PIN, throttle);
    analogWrite(LEFT_MOTOR_PIN,  throttle);
  };
}


//=============================== loop() ======================================
//
// Arduino Main Loop Body
//
//=============================================================================

void loop ()
{
}
