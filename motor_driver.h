#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Wire.h>

#define I2C_ADDR 0x34
#define MOTOR_TYPE_ADDR 0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_SPEED_ADDR 0x33

void clampAndWriteSpeeds(int leftSpeed, int rightSpeed) {
  int maxSpeed = 100;
  if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
  if (leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
  if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
  if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;

  int8_t speeds[4] = {
    (int8_t)leftSpeed,
    (int8_t)rightSpeed,
    0, 0
  };

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(MOTOR_FIXED_SPEED_ADDR);
  for (int i = 0; i < 4; i++) {
    Wire.write(speeds[i]);
  }
  Wire.endTransmission();
}

void initMotorController() {
  Wire.begin();

  uint8_t motorType = 3;  // JGB37-520 motor
  uint8_t polarity = 0;   // Try 0 first. Change to 1 if direction is reversed

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(MOTOR_TYPE_ADDR);
  Wire.write(motorType);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(MOTOR_ENCODER_POLARITY_ADDR);
  Wire.write(polarity);
  Wire.endTransmission();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  clampAndWriteSpeeds(leftSpeed, rightSpeed);
}

#endif