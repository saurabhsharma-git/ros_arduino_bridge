/*********************************************************************
 *  ROSArduinoBridge
 *  Modified for Hiwonder 4-Channel I2C Encoder Driver
 *********************************************************************/

#define USE_BASE
#undef USE_SERVOS

#define BAUDRATE     57600
#define MAX_PWM        100  // Motor driver accepts -100 to +100

#include <Wire.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "commands.h"
#include "sensors.h"

#ifdef USE_BASE
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"

  #define PID_RATE           20
  const int PID_INTERVAL = 1000 / PID_RATE;
  unsigned long nextPID = PID_INTERVAL;
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif
#define MAX_TICKS_PER_FRAME 360  // or lower, depending on your encoder and motor
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    digitalWrite(arg1, arg2 ? HIGH : LOW);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    pinMode(arg1, arg2 ? OUTPUT : INPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(0));
    Serial.print(" ");
    Serial.println(readEncoder(1));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    } else {
      moving = 1;
      if (arg1 > 100) arg1 = 100;
      if (arg1 < -100) arg1 = -100;
      if (arg2 > 100) arg2 = 100;
      if (arg2 < -100) arg2 = -100;

    // Scale percent into ticks/frame
      leftPID.TargetTicksPerFrame = arg1 * MAX_TICKS_PER_FRAME / 100.0;
      rightPID.TargetTicksPerFrame = arg2 * MAX_TICKS_PER_FRAME / 100.0;
    }
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i++] = atoi(str);
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
  return 1;
}

void setup() {
  Serial.begin(BAUDRATE);
#ifdef USE_BASE
  initMotorController();
  resetPID();
#endif
}

void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) cmd = chr;
      else if (arg == 1) argv1[index++] = chr;
      else if (arg == 2) argv2[index++] = chr;
    }
  }
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif
}
