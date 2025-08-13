#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Wire.h>

#define I2C_ADDR 0x34
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C

int32_t encoder_counts[4];
int32_t encoder_offsets[4] = {0, 0, 0, 0};

long readEncoder(int i) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(MOTOR_ENCODER_TOTAL_ADDR);
  Wire.endTransmission();
  
  if (Wire.requestFrom(I2C_ADDR, 16) == 16) {
    for (int j = 0; j < 4; j++) {
      encoder_counts[j]  = Wire.read();
      encoder_counts[j] |= Wire.read() << 8;
      encoder_counts[j] |= Wire.read() << 16;
      encoder_counts[j] |= Wire.read() << 24;
    }
  }

  return encoder_counts[i] - encoder_offsets[i];
}

void resetEncoders() {
  for (int i = 0; i < 4; i++) {
    encoder_offsets[i] = encoder_counts[i];
  }
}

#endif
